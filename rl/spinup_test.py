"""Wheel servo spin-up test: measures how fast the ST3215-HS wheel servo speeds up.

The speed-up time tells us the motor's hidden spinning mass (rotor inertia, "armature" in the sim).

  1. Wheel ON the servo, servo held still, wheel in the air (it spins at full speed for 1 s).
  2. Servo -> Waveshare bus adapter -> USB to the Mac. Adapter power: the 3S battery (as on the robot).
  3. .venv/bin/python spinup_test.py            (asks before anything moves)

It changes the servo to PWM mode for the test and puts the old mode back at the end.
Results: printed, and saved to spinup_<id>.csv (time, position).
"""
import glob
import sys
import time

import numpy as np
import serial

BAUD = 1_000_000
# Feetech STS memory table (ST3215 uses the same)
TORQUE_ENABLE, LOCK, MODE, PWM_GOAL = 0x28, 0x37, 0x21, 0x2C
PRESENT_POS, PRESENT_VOLT = 0x38, 0x3E
STALL_TORQUE_12V = 2.0   # Nm, ST3215-HS data at 12 V (used only for the inertia estimate)


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, instr, params=()):
    body = [sid, len(params) + 2, instr, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


class Bus:
    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, timeout=0.02)

    def _reply(self, n):
        r = self.s.read(6 + n)
        if len(r) < 6 + n or r[0] != 0xFF or r[1] != 0xFF:
            return None
        if checksum(list(r[2:-1])) != r[-1]:
            return None
        return r[5:5 + n]

    def ping(self, sid):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, 0x01))
        return self._reply(0) is not None

    def read(self, sid, addr, n):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, 0x02, (addr, n)))
        return self._reply(n)

    def write(self, sid, addr, data):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, 0x03, (addr, *data)))
        return self._reply(0) is not None


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*") + glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("No USB serial port found. Is the bus adapter plugged in (and its switch on USB)?")
    return ports[0]


def log(bus, sid, seconds):
    """Read the position as fast as the bus allows. Returns (time s, position counts)."""
    t, p, t0 = [], [], time.perf_counter()
    while (now := time.perf_counter() - t0) < seconds:
        r = bus.read(sid, PRESENT_POS, 2)
        if r is not None:
            t.append(now)
            p.append(r[0] | (r[1] << 8))
    return np.array(t), np.array(p, dtype=float)


def unwrap_counts(p):
    """Position is 0..4095 per turn; undo the jumps at the wrap."""
    d = np.diff(p)
    d[d > 2048] -= 4096
    d[d < -2048] += 4096
    return np.concatenate([[0.0], np.cumsum(d)])


def main():
    port = find_port()
    bus = Bus(port)
    print("port:", port)
    ids = [i for i in range(1, 21) if bus.ping(i)]
    if not ids:
        sys.exit("No servo answered on IDs 1-20. Check power on the adapter and the cable.")
    sid = int(sys.argv[1]) if len(sys.argv) > 1 else ids[0]
    print("servos found:", ids, "-> testing ID", sid)
    volt = bus.read(sid, PRESENT_VOLT, 1)
    volt = volt[0] / 10 if volt else float("nan")
    old_mode = bus.read(sid, MODE, 1)
    old_mode = old_mode[0] if old_mode else 0
    print(f"supply voltage {volt:.1f} V, current mode {old_mode}")
    input("Wheel in the air and servo held still? Press Enter to spin it (Ctrl+C to cancel)... ")

    # PWM mode (open loop, like on the robot)
    bus.write(sid, TORQUE_ENABLE, [0])
    bus.write(sid, LOCK, [0])
    bus.write(sid, MODE, [2])
    bus.write(sid, LOCK, [1])
    bus.write(sid, TORQUE_ENABLE, [1])

    runs = []
    try:
        for k in range(3):
            bus.write(sid, PWM_GOAL, [0, 0])
            time.sleep(0.8)                                  # wheel at rest
            bus.write(sid, PWM_GOAL, [1000 & 0xFF, 1000 >> 8])   # full power forward
            t, p = log(bus, sid, 1.0)
            bus.write(sid, PWM_GOAL, [0, 0])                # power off: coast down
            tc, pc = log(bus, sid, 1.0)
            runs.append((t, p, tc, pc))
            print(f"run {k + 1}: {len(t)} readings in 1 s ({1000 / max(len(t), 1):.1f} ms each)")
    finally:
        bus.write(sid, PWM_GOAL, [0, 0])
        bus.write(sid, TORQUE_ENABLE, [0])
        bus.write(sid, LOCK, [0])
        bus.write(sid, MODE, [old_mode])
        bus.write(sid, LOCK, [1])
        print("servo stopped, mode restored to", old_mode)

    rows, results = [], []
    for k, (t, p, tc, pc) in enumerate(runs):
        ang = unwrap_counts(p) * 2 * np.pi / 4096           # rad
        rows += [(k, ti, ai) for ti, ai in zip(t, ang)]
        # speed from the angle, over ~5 ms windows
        w = []
        for i in range(len(t)):
            j = np.searchsorted(t, t[i] - 0.005)
            w.append((ang[i] - ang[j]) / (t[i] - t[j]) if t[i] > t[j] else 0.0)
        w = np.abs(np.array(w))
        top = np.median(w[t > 0.6])                          # top speed (last 0.4 s)
        i63 = np.argmax(w >= 0.63 * top)                     # time constant: 63 % of top speed
        T = t[i63]
        results.append((top, T))
        print(f"run {k + 1}: top speed {top:.1f} rad/s ({top * 60 / (2 * np.pi):.0f} rpm), time to 63 %: {T * 1000:.0f} ms")
    np.savetxt(f"spinup_{sid}.csv", np.array(rows), delimiter=",", header="run,time_s,angle_rad", comments="")

    top = np.median([r[0] for r in results])
    T = np.median([r[1] for r in results])
    stall = STALL_TORQUE_12V * volt / 12.0
    J = T * stall / top
    print("\nRESULT (paste this to Claude)")
    print(f"  voltage {volt:.1f} V | top speed {top:.2f} rad/s | time to 63 % {T * 1000:.1f} ms | readings every "
          f"{1000 / max(len(runs[0][0]), 1):.1f} ms")
    print(f"  estimated rotor inertia (armature) {J:.5f} kg m^2  (sim guess: 0.005; uses stall torque {stall:.2f} Nm)")


if __name__ == "__main__":
    main()
