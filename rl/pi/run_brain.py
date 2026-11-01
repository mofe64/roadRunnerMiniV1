"""Run the trained brain on the robot, 50 decisions a second.

  python3 run_brain.py --air       robot on a box, wheels in the air: check that everything moves the right way
  python3 run_brain.py --held      hold the robot upright on the floor with your hands near it
  python3 run_brain.py             normal run
  python3 run_brain.py --freeze-hips   test: hips held at the stand pose, the brain drives the wheels only
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
ap.add_argument("--brain", default=os.path.join(HERE, "brain_drive_020.npz"))
ap.add_argument("--air", action="store_true", help="wheels in the air: no tip-over stop, no battery stop")
ap.add_argument("--held", action="store_true")
ap.add_argument("--freeze-hips", action="store_true", help="test (review 2026-11-01 T2): hips HELD at the stand pose, the brain drives the wheels only")
ap.add_argument("--bench-power", action="store_true", help="wheels bus on the 6 V bench adapter: no battery stop")
args = ap.parse_args()

TIP_STOP = np.radians(25)       # the drive brain cannot save a lean past about 15 deg (sim, 2026-10-30)
TUCK_HEIGHT, TUCK_S = 0.065, 1.0  # on a fall: legs to 65 mm for 1 s (lands on a bump, not on the head)
BATTERY_WARN, BATTERY_STOP = 10.8, 10.5      # V, 3S LiPo: 3.6 and 3.5 V per cell
brain = Brain(args.brain)
ctl = Controller(brain)
hz = float(brain.c["policy_hz"]); dt = 1.0 / hz
robot = Robot()
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
    robot.write_hips(want); time.sleep(dt)
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
log.write("t,loop_ms,imu_ms,read_ms,write_ms,gx,gy,gz,wx,wy,wz,q1,q2,q3,q4,v1,v2,v3,v4,wl,wr,a1,a2,a3,a4,cmd_v,cmd_w,cmd_h,volts\n")
old_tty = termios.tcgetattr(sys.stdin); tty.setcbreak(sys.stdin.fileno())
speed, turn, height = 0.0, 0.0, float(brain.c["height_normal"])
reason, volts, t0 = "q pressed", v0, time.perf_counter()
ctl.reset(robot.read_joints()[0])
hip_bad = 0
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
                robot.write_hips(brain.stand_hips(TUCK_HEIGHT))
                robot.write_wheels(np.zeros(2))
                time.sleep(dt)
            break
        # Hip watchdog: a hip more than 25 deg from its last target for 0.2 s = a servo fault -> stop.
        far = np.abs(np.asarray(j[0]) - ctl.hip_target) > np.radians(25)
        hip_bad = hip_bad + 1 if far.any() else 0
        if hip_bad >= 10:
            reason = "hip far from its target (" + ", ".join(f"{np.degrees(a):+.0f}" for a in j[0]) + " deg): servo fault"
            break
        target, power, action = ctl.step(down, gyro, *j)
        if args.freeze_hips:
            target = ctl.hip_target = np.asarray(goal, dtype=float).copy()   # hips stay at the stand pose
        robot.write_hips(target)
        robot.write_wheels(power)
        t_write = time.perf_counter()
        if ctl.steps % 50 == 0:
            volts = robot.battery() or volts
            if volts is not None and volts < BATTERY_STOP and not (args.air or args.bench_power):
                reason = f"battery {volts:.1f} V"; break
            if volts is not None and volts < BATTERY_WARN and not args.bench_power:
                print(f"\rbattery LOW {volts:.1f} V   ", end="")
        while select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k == "q": raise KeyboardInterrupt
            speed += {"w": 0.05, "s": -0.05}.get(k, 0.0); turn += {"a": 0.5, "d": -0.5}.get(k, 0.0)
            height += {"r": 0.01, "f": -0.01}.get(k, 0.0)
            if k == " ": speed, turn = 0.0, 0.0
            ctl.set_command(speed, turn, height)
            speed, turn, height = ctl.command
            print(f"\rcommand: speed {speed:+.2f} m/s, turn {turn:+.1f} rad/s, height {height * 1000:.0f} mm   ", end="")
        log.write(",".join(f"{x:.5g}" for x in [t_start - t0, (t_start - prev) * 1000, (t_imu - t_start) * 1000,
                  (t_read - t_imu) * 1000, (t_write - t_read) * 1000, *down, *gyro, *j[0], *j[1], *j[2], *action,
                  *ctl.command, volts or 0]) + "\n")
        prev = t_start
        next_t += dt
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
    print(f"\nSTOPPED: {reason}. Log: {log_path}")
