"""Test T1 (review 2026-11-01): is the 11 Hz buzz real motion or vibration folded into the 50 Hz loop?
The brain does NOT run. The gyro is read as fast as it can (about 200 times a second).

  python3 gyro_fast.py
Put the robot on the two book stacks (tray ends on the stacks, wheels hang free in the gap).
1. The legs move slowly (2 s) to the stand pose and hold it (hip force capped at 30 %).
2. 10 s: wheels stopped. 3. 10 s: both wheels turn slowly (power 0.3). Do not touch the robot.
Then all servos go limp. Log: logs/gyro_fast_<time>.csv
"""
import os, time
import numpy as np
from brain import Brain
from robot import Robot, HIPS

HERE = os.path.dirname(os.path.abspath(__file__))
brain = Brain(os.path.join(HERE, "brain_drive_021.npz"))
robot = Robot()
robot.prepare()
robot.hold_here()
j = robot.read_joints()
if j is None:
    raise SystemExit("A servo did not answer.")
start, goal = j[0], brain.stand_hips(0.12)
input("Robot on the book stacks, wheels free. The legs move to the stand pose in 2 s. Press Enter.")
for n in HIPS:
    robot.legs.write2(robot.id[n], 0x30, 300)          # hip force capped at 30 %
try:
    for k in range(125):
        want = start + (goal - start) * min(1.0, (k + 1) / 100)
        robot.write_hips(want); time.sleep(0.02)
        jj = robot.read_joints()
        if jj is None or (np.abs(np.asarray(jj[0]) - want) > np.radians(20)).any():
            robot.stop(hips_off=True)
            raise SystemExit("STOP: a hip did not follow (or did not answer). All limp. Tell Claude.")
    input("Do not touch the robot now. Press Enter to start (20 s).")
    robot.imu.calibrate(1.0)
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    path = os.path.join(HERE, "logs", time.strftime("gyro_fast_%Y%m%d_%H%M%S.csv"))
    rows, t0 = [], time.perf_counter()
    for phase, power in [(0, 0.0), (1, 0.3)]:
        robot.write_wheels(np.array([power, power]))
        t_end = time.perf_counter() + 10.0
        while time.perf_counter() < t_end:
            gyro, acc = robot.imu.raw()
            rows.append([time.perf_counter() - t0, phase, *gyro, *acc])
            time.sleep(0.001)
        print(f"  part {phase + 1} done")
finally:
    robot.write_wheels(np.zeros(2))
    robot.stop(hips_off=True)
    for n in HIPS:
        robot.legs.write2(robot.id[n], 0x30, 1000)
np.savetxt(path, np.array(rows), delimiter=",", header="t,phase,wx,wy,wz,ax,ay,az", comments="", fmt="%.5g")
r = np.array(rows)
print(f"Limp. Saved {path}: {len(r)} readings, {len(r) / r[-1, 0]:.0f} per second.")
