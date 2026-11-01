"""Drive the robot with a Bluetooth gamepad (Xbox One pad, the same set-up as Open Duck).

Test the pad alone (no servos move):   .venv/bin/python gamepad.py
Drive the robot:                       .venv/bin/python run_brain.py --pad

Controls:
  left stick up/down     speed (forward up to 0.15 m/s, back up to 0.10 m/s)
  right stick left/right turn (up to 1.0 rad/s)
  D-pad up/down          taller/lower (1 cm for each press)
  A                      stand still (speed and turn 0)
  B                      STOP (the same as q: the legs go limp, be ready to catch the robot)

Safety:
  - The speed and the turn change in small steps (0.05 m/s, 0.25 rad/s), at most 5 steps a second,
    the same steps as the keys. The brain was trained with steps, not with a stick that moves all the time.
  - If the pad disconnects (Bluetooth lost, battery flat, pad off), the robot stands still in place and
    keeps its balance. Press q in the terminal to stop it, or connect the pad again.
  - After a connect, the sticks must go back to the centre once before the robot moves.

Button and axis numbers are the ones in Open Duck (mini_bdx_runtime/xbox_controller.py) for an Xbox One
pad over Bluetooth on a Raspberry Pi. If a stick or a button does the wrong thing, run this file and
tell Claude what it shows.
"""
import os, threading, time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")   # the Pi runs over SSH, with no window
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

AX_LY, AX_RX = 1, 2                 # left stick up/down, right stick left/right (Open Duck numbers)
BTN_A, BTN_B = 0, 1
DEAD = 0.20                         # stick dead zone (part of full travel)
SPEED_STEP, TURN_STEP = 0.05, 0.25  # m/s, rad/s
PAD_TURN_MAX = 1.0                  # rad/s; the brain accepts 1.5, but a turn on the spot made it fall once
STEP_EVERY = 0.2                    # s, at most one step of speed and one of turn in this time
HEIGHT_STEP = 0.01                  # m for each D-pad press


def _stick(x):
    """Dead zone, then scale so that the edge of the dead zone is 0 and full travel is 1."""
    if abs(x) < DEAD:
        return 0.0
    return (abs(x) - DEAD) / (1 - DEAD) * (1 if x > 0 else -1)


def _target(raw_steps, now_steps):
    """Snap the stick to whole steps. Hysteresis: change only when the stick is clearly in the next step,
    so stick noise does not make the command go up and down."""
    if abs(raw_steps - now_steps) > 0.7:
        return round(raw_steps)
    return now_steps


class Gamepad:
    def __init__(self, speed_max, reverse_max, turn_max, wait_s=10.0):
        self.speed_max, self.reverse_max = float(speed_max), float(reverse_max)
        self.turn_max = min(float(turn_max), PAD_TURN_MAX)
        self._lock = threading.Lock()
        self._ly = self._rx = 0.0
        self._events = []
        self.connected = False
        self.name = ""
        self._armed = False           # True after the sticks were at the centre once since the connect
        self._error = None
        self._t_step = 0.0
        self._sp = self._tu = 0       # targets, in whole steps
        self._started = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()
        t0 = time.time()
        while time.time() - t0 < wait_s and not self.connected and self._error is None:
            time.sleep(0.05)
        if self._error is not None:
            raise RuntimeError(f"gamepad: {self._error}")
        if not self.connected:
            raise RuntimeError("no gamepad found. Turn it on and check that it is connected (bluetoothctl).")
        self.events()                 # forget the first "connected" event

    # ---- reader thread (pygame lives only in this thread) ----
    def _worker(self):
        try:
            import pygame
        except ImportError:
            self._error = "pygame is not installed. In rl/pi: .venv/bin/pip install pygame"
            return
        try:
            pygame.display.init()
            pygame.joystick.init()
        except Exception as e:              # noqa: BLE001
            self._error = f"pygame did not start ({e})"
            return
        js = None
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.JOYDEVICEADDED and js is None:
                    js = pygame.joystick.Joystick(ev.device_index); js.init()
                    with self._lock:
                        self.name, self.connected, self._armed = js.get_name(), True, False
                        self._ly = self._rx = 0.0
                        self._events.append("connected")
                elif ev.type == pygame.JOYDEVICEREMOVED and js is not None and ev.instance_id == js.get_instance_id():
                    js = None
                    with self._lock:
                        self.connected, self._armed = False, False
                        self._ly = self._rx = 0.0
                        self._events.append("lost")
                elif js is not None and ev.type == pygame.JOYBUTTONDOWN:
                    with self._lock:
                        if ev.button == BTN_B: self._events.append("stop")
                        elif ev.button == BTN_A: self._events.append("still")
                elif js is not None and ev.type == pygame.JOYHATMOTION and ev.value[1] != 0:
                    with self._lock:
                        self._events.append("up" if ev.value[1] > 0 else "down")
            if js is not None:
                try:
                    ly, rx = -js.get_axis(AX_LY), -js.get_axis(AX_RX)   # up = +, left = + (Open Duck signs)
                except Exception:               # noqa: BLE001  (pad removed between two reads)
                    ly = rx = 0.0
                with self._lock:
                    self._ly, self._rx = ly, rx
            time.sleep(0.01)

    # ---- main loop side ----
    def events(self):
        """Button and connect events since the last call: 'stop', 'still', 'up', 'down', 'connected', 'lost'."""
        with self._lock:
            ev, self._events = self._events, []
        return ev

    def drive(self, speed, turn, now=None):
        """The next speed and turn command, from the sticks. Call once a step with the present command.
        Pad not connected (or sticks not yet back at the centre after a connect): stand still."""
        now = time.monotonic() if now is None else now
        with self._lock:
            ly, rx, ok = _stick(self._ly), _stick(self._rx), self.connected
            if not ok:
                self._armed = False
            if ok and not self._armed and ly == 0.0 and rx == 0.0:
                self._armed = True
            armed = self._armed
        if not (ok and armed):
            self._sp = self._tu = 0
            return 0.0, 0.0
        sp_max = self.speed_max if ly >= 0 else self.reverse_max
        self._sp = _target(ly * sp_max / SPEED_STEP, self._sp)
        self._tu = _target(rx * self.turn_max / TURN_STEP, self._tu)
        if now - self._t_step < STEP_EVERY:
            return speed, turn
        new_s = speed + max(-SPEED_STEP, min(SPEED_STEP, self._sp * SPEED_STEP - speed))
        new_t = turn + max(-TURN_STEP, min(TURN_STEP, self._tu * TURN_STEP - turn))
        new_s, new_t = round(new_s, 3), round(new_t, 3)
        if (new_s, new_t) != (round(speed, 3), round(turn, 3)):
            self._t_step = now
        return new_s, new_t

    def still(self):
        """A pressed: speed and turn 0, and the sticks must go back to the centre before the next move."""
        with self._lock:
            self._armed = False
        self._sp = self._tu = 0


if __name__ == "__main__":
    pad = Gamepad(0.15, 0.10, 1.5)
    print(f"gamepad: {pad.name}. Move the sticks, press A, B and the D-pad. Ctrl-C to end.")
    s = t = 0.0
    h = 0.12
    while True:
        for e in pad.events():
            if e == "up": h += HEIGHT_STEP
            elif e == "down": h -= HEIGHT_STEP
            elif e == "still": pad.still(); s = t = 0.0
            print(f"\n{e}")
        s, t = pad.drive(s, t)
        print(f"\rleft stick up/down {pad._ly:+.2f}  right stick left/right {pad._rx:+.2f}  ->  "
              f"speed {s:+.2f} m/s  turn {t:+.2f} rad/s  height {h * 1000:.0f} mm  ", end="", flush=True)
        time.sleep(0.02)
