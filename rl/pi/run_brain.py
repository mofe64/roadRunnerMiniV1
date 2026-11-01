"""Run the trained brain on the robot, 50 decisions a second.

  python3 run_brain.py --air       robot on a box, wheels in the air: check that everything moves the right way
  python3 run_brain.py --held      hold the robot upright on the floor with your hands near it
  python3 run_brain.py             normal run
  python3 run_brain.py --freeze-hips   test: hips held at the stand pose, the brain drives the wheels only
  python3 run_brain.py --freeze-hips --gyro-lp 6 --fast-imu   test step 0 (review 2026-11-01)
  python3 run_brain.py --pad       drive with the Bluetooth gamepad (controls in gamepad.py; q still stops)
  python3 run_brain.py --depth     obstacle stop (DEPTH.md): first start `python3 depth_cam.py --watch --robot` in a
                                   second terminal. Head 20 deg nose down. Forward speed falls below 60 cm, 0 below 30 cm
Keys (in this terminal): w/s faster/slower (0.05 m/s), a/d turn left/right (0.5 rad/s), r/f taller/lower (1 cm),
space = stand still, q = stop. The robot STOPS by itself if it tips more than 25 deg, if the battery is low,
or if a servo stops answering. When it stops, the legs go limp: be ready to catch it.
A fall (tilt more than 25 deg): wheels off, legs tucked to 65 mm for 1 s so the robot lands on a bump, then all
servos off. Stand it up by hand and start again. (A self get-up is parked: review 2026-10-30.)
Every run writes a log file logs/run_<time>.csv (sensor readings, actions and the time each step took).
"""
import argparse, os, select, sys, termios, time, tty
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
from brain import Brain
from controller import Controller
from robot import Robot

ap = argparse.ArgumentParser()
ap.add_argument("--brain", default=os.path.join(HERE, "brain_drive_026.npz"))   # working brain since 2026-11-01
ap.add_argument("--air", action="store_true", help="wheels in the air: no tip-over stop, no battery stop")
ap.add_argument("--held", action="store_true")
ap.add_argument("--freeze-hips", action="store_true", help="test (review 2026-11-01 T2): hips HELD at the stand pose, the brain drives the wheels only")
ap.add_argument("--gyro-lp", type=float, default=0.0, help="test (review 2026-11-01 step 0): low-pass the gyro the brain sees, cut-off in Hz (e.g. 6)")
ap.add_argument("--fast-imu", action="store_true", help="also log the raw IMU about 200 times a second (logs/imu_<time>.csv)")
ap.add_argument("--imu-tau", type=float, default=0.0, help="IMU filter time constant in s (default: the brain's, else 2.0)")
ap.add_argument("--pad", action="store_true", help="drive with the Bluetooth gamepad (gamepad.py): sticks, A stand still, B stop")
ap.add_argument("--depth", action="store_true", help="obstacle stop with the depth camera (DEPTH.md step 5)")
ap.add_argument("--head-pitch", type=float, default=None, help="head pitch in deg, + = nose down (default 0; 20 with --depth)")
ap.add_argument("--bench-power", action="store_true", help="wheels bus on the 6 V bench adapter: no battery stop")
args = ap.parse_args()

TIP_STOP = np.radians(25)       # the drive brain cannot save a lean past about 15 deg (sim, 2026-10-30)
TUCK_HEIGHT, TUCK_S = 0.065, 1.0  # on a fall: legs to 65 mm for 1 s (lands on a bump, not on the head)
BATTERY_WARN, BATTERY_STOP = 10.8, 10.5      # V, 3S LiPo: 3.6 and 3.5 V per cell
brain = Brain(args.brain)
if args.gyro_lp > 0 and float(brain.c.get("gyro_lp", 0)) > 0:
    raise SystemExit("This brain already low-passes the gyro (drive_022 and later): do not use --gyro-lp.")


class LowPass:
    """2-pole Butterworth low-pass, one per gyro axis, at the decision rate."""
    def __init__(self, fc, fs):
        k = np.tan(np.pi * fc / fs); n = 1.0 / (1 + np.sqrt(2) * k + k * k)
        self.b = np.array([k * k * n, 2 * k * k * n, k * k * n])
        self.a = np.array([2 * (k * k - 1) * n, (1 - np.sqrt(2) * k + k * k) * n])
        self.z = None
    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        if self.z is None:                      # start at rest on the first reading (no jump)
            self.z = np.stack([x * (1 - self.b[0]), x * (self.b[2] - self.a[1])])
        y = self.b[0] * x + self.z[0]
        self.z = np.stack([self.b[1] * x - self.a[0] * y + self.z[1], self.b[2] * x - self.a[1] * y])
        return y


pad = None
if args.pad:     # find the pad BEFORE any servo moves
    from gamepad import Gamepad
    try:
        pad = Gamepad(brain.c["speed_max"], brain.c["reverse_max"], brain.c["turn_max"])
    except RuntimeError as e:
        raise SystemExit(f"STOP: {e}")
    print(f"gamepad: {pad.name}. Left stick speed, right stick turn, D-pad height, A stand still, B STOP.")

guard = None
if args.depth:   # open the camera link BEFORE any servo moves
    from obstacle import DepthGuard
    try:
        guard = DepthGuard(float(brain.c["speed_max"]))
    except OSError as e:
        raise SystemExit(f"STOP: depth link: {e} (is another run_brain.py running?)")
    print("depth: forward speed limited by the camera. depth_cam.py --watch --robot must run in a second terminal.")
HEAD_PITCH = np.radians(args.head_pitch if args.head_pitch is not None else (20.0 if args.depth else 0.0))
HEAD = (0.0, HEAD_PITCH)      # (yaw, pitch); set once in the stand move, never changed while balancing

ctl = Controller(brain)
hz = float(brain.c["policy_hz"]); dt = 1.0 / hz
robot = Robot()
if float(brain.c.get("imu_tau", 0)) > 0:      # brains from drive_028 on were trained with the Pi IMU filter at this time constant
    robot.imu.tau = float(brain.c["imu_tau"])
else:   # older brains (true gravity in the sim): a slow filter holds the spot better (drive_026 test 2026-11-01, Mofe chose it)
    robot.imu.tau = 2.0
if args.imu_tau > 0:
    robot.imu.tau = args.imu_tau
print(f"IMU filter time constant {robot.imu.tau:.1f} s")
robot.prepare()
v0 = robot.battery()
print(f"battery {v0:.1f} V")
if not (args.air or args.bench_power) and (v0 is None or v0 < BATTERY_WARN):
    raise SystemExit("Battery low or not read. Charge it (or use --bench-power on the bench adapter).")

# 1. stand pose, slowly (2 s), from where the legs are now
robot.hold_here()
j = robot.read_joints()
if j is None:
    raise SystemExit("A servo did not answer.")
start, goal = j[0], brain.stand_hips(ctl.command[2])
input("Hold the robot. The legs will move to the stand pose in 2 s. Press Enter.")
for k in range(int(2.5 * hz)):
    want = start + (goal - start) * min(1.0, (k + 1) / (2 * hz))
    robot.write_hips(want, head=(0.0, HEAD_PITCH * min(1.0, (k + 1) / (2 * hz)))); time.sleep(dt)
    jj = robot.read_joints()
    if jj is None or (np.abs(np.asarray(jj[0]) - want) > np.radians(20)).any():
        robot.stop(hips_off=True)
        raise SystemExit("STOP: a hip did not follow the stand move (or did not answer). All limp. Tell Claude.")

# 2. IMU zero (the robot must be still)
input(("Wheels in the air, robot still. " if args.air else "Hold the robot upright on its wheels, STILL. ") + "Press Enter.")
robot.imu.calibrate(1.0)
for _ in range(25):
    down, _ = robot.imu.update(0.02); time.sleep(0.02)
tilt = np.arccos(np.clip(-down[2], -1, 1))
if not args.air and tilt > np.radians(10):
    raise SystemExit(f"Tilted {np.degrees(tilt):.0f} deg: hold it upright and start again.")
print("GO. Let go gently." if not (args.air or args.held) else "GO.")

os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
log_path = os.path.join(HERE, "logs", time.strftime("run_%Y%m%d_%H%M%S.csv"))
log = open(log_path, "w")
log.write("t,loop_ms,imu_ms,read_ms,write_ms,gx,gy,gz,wx,wy,wz,q1,q2,q3,q4,v1,v2,v3,v4,wl,wr,a1,a2,a3,a4,cmd_v,cmd_w,cmd_h,volts,bwx,bwy,bwz,ax,ay,az,pad,ahead,limit\n")
old_tty = termios.tcgetattr(sys.stdin); tty.setcbreak(sys.stdin.fileno())
speed, turn, height = 0.0, 0.0, float(brain.c["height_normal"])
reason, volts, t0 = "q pressed", v0, time.perf_counter()
ctl.reset(robot.read_joints()[0])
hip_bad = 0
lp = LowPass(args.gyro_lp, hz) if args.gyro_lp > 0 else None
if pad is not None:      # forget presses made during the stand-up; the sticks must be at the centre before a move
    pad.events(); pad.still()
fast, t_fast = [], 0.0
next_t = prev = time.perf_counter()
try:
    while True:
        t_start = time.perf_counter()
        down, gyro = robot.imu.update(dt)
        t_imu = time.perf_counter()
        j = robot.read_joints()
        t_read = time.perf_counter()
        if j is None:
            reason = "a servo did not answer"; break
        tilt = np.arccos(np.clip(-down[2], -1, 1))
        if tilt > TIP_STOP and not args.air:
            reason = f"tipped {np.degrees(tilt):.0f} deg (legs tucked, then servos off)"
            robot.write_wheels(np.zeros(2))
            t_tuck = time.perf_counter()
            while time.perf_counter() - t_tuck < TUCK_S:
                robot.write_hips(brain.stand_hips(TUCK_HEIGHT), head=HEAD)
                robot.write_wheels(np.zeros(2))
                time.sleep(dt)
            break
        # Hip watchdog: a hip more than 25 deg from its last target for 0.2 s = a servo fault -> stop.
        far = np.abs(np.asarray(j[0]) - ctl.hip_target) > np.radians(25)
        hip_bad = hip_bad + 1 if far.any() else 0
        if hip_bad >= 10:
            reason = "hip far from its target (" + ", ".join(f"{np.degrees(a):+.0f}" for a in j[0]) + " deg): servo fault"
            break
        gyro_b = lp(gyro) if lp is not None else gyro
        target, power, action = ctl.step(down, gyro_b, *j)
        if args.freeze_hips:
            target = ctl.hip_target = np.asarray(goal, dtype=float).copy()   # hips stay at the stand pose
        robot.write_hips(target, head=HEAD)
        robot.write_wheels(power)
        t_write = time.perf_counter()
        if ctl.steps % 50 == 0:
            volts = robot.battery() or volts
            if volts is not None and volts < BATTERY_STOP and not (args.air or args.bench_power):
                reason = f"battery {volts:.1f} V"; break
            if volts is not None and volts < BATTERY_WARN and not args.bench_power:
                print(f"\rbattery LOW {volts:.1f} V   ", end="")
        if pad is not None:      # gamepad: sticks set speed and turn in small steps; pad lost = stand still
            for e in pad.events():
                if e == "stop":
                    reason = "B pressed on the gamepad"; raise KeyboardInterrupt
                if e == "still": pad.still()
                if e == "up": height += 0.01
                if e == "down": height -= 0.01
                if e == "lost": print("\rGAMEPAD LOST: the robot stands still. Press q to stop, or connect the pad again.")
                if e == "connected": print("\rgamepad connected again. Centre the sticks, then drive.")
            speed, turn = pad.drive(speed, turn)
        while select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k == "q": raise KeyboardInterrupt
            speed += {"w": 0.05, "s": -0.05}.get(k, 0.0); turn += {"a": 0.5, "d": -0.5}.get(k, 0.0)
            height += {"r": 0.01, "f": -0.01}.get(k, 0.0)
            if k == " ": speed, turn = 0.0, 0.0
        # ONE place sets the brain's command (keys, pad and camera), only on a real change: each change restarts
        # the brain's lag signals. `speed` is what you asked for; the camera can only lower the FORWARD part.
        speed = float(np.clip(speed, -float(brain.c["reverse_max"]), float(brain.c["speed_max"])))
        turn = float(np.clip(turn, -float(brain.c["turn_max"]), float(brain.c["turn_max"])))
        height = float(np.clip(height, *[float(x) for x in brain.c["height_range"]]))
        v_cmd, limit = speed, -1.0
        if guard is not None:
            guard.send_pose(down, ctl.command[2], HEAD_PITCH)
            limit = guard.update(time.monotonic())
            if speed > 0:
                v_cmd = min(speed, limit)
        if not np.allclose([v_cmd, turn, height], ctl.command):
            ctl.set_command(v_cmd, turn, height)
            note = f" (camera: {guard.ahead * 100:.0f} cm)" if guard is not None and v_cmd < speed and guard.ahead is not None \
                else " (camera: no data)" if v_cmd < speed else ""
            print(f"\rcommand: speed {v_cmd:+.2f} m/s{note}, turn {turn:+.2f} rad/s, height {height * 1000:.0f} mm   ", end="")
        log.write(",".join(f"{x:.5g}" for x in [t_start - t0, (t_start - prev) * 1000, (t_imu - t_start) * 1000,
                  (t_read - t_imu) * 1000, (t_write - t_read) * 1000, *down, *gyro, *j[0], *j[1], *j[2], *action,
                  *ctl.command, volts or 0, *gyro_b, *robot.imu.last_acc,
                  -1 if pad is None else int(pad.connected),
                  -1 if guard is None or guard.ahead is None else guard.ahead, limit]) + "\n")
        prev = t_start
        next_t += dt
        if args.fast_imu:                         # use the idle part of the step: raw IMU every 5 ms
            while next_t - time.perf_counter() > 0.0015:
                if time.perf_counter() - t_fast >= 0.005:
                    t_fast = time.perf_counter(); g_, a_ = robot.imu.raw()
                    fast.append([t_fast - t0, *g_, *a_])
                time.sleep(0.0005)
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.perf_counter()          # running late: do not try to catch up
except KeyboardInterrupt:
    pass
finally:
    robot.stop(hips_off=True)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
    log.close()
    if fast:
        imu_path = log_path.replace("run_", "imu_")
        np.savetxt(imu_path, np.array(fast), delimiter=",", header="t,wx,wy,wz,ax,ay,az", comments="", fmt="%.6g")
        print(f"Fast IMU log: {imu_path} ({len(fast)} readings)")
    print(f"\nSTOPPED: {reason}. Log: {log_path}")
