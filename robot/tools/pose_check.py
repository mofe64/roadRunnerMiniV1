"""Check the calibration: move the legs to the stand pose and hold it. The brain does NOT run.

  python3 tools/pose_check.py            stand pose, leg height 120 mm
  python3 tools/pose_check.py 0.08       another leg height in m (0.065 .. 0.140)
Hold the robot in the air. The legs move slowly (2 s). In the stand pose, both legs must look the same:
front short link forward, rear short link back, by the same angle, and the wheel under the hips.
Press Enter to make the robot limp again.
"""
import sys
import time
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
from brain import Brain
from robot import Robot, HIPS

h = float(sys.argv[1]) if len(sys.argv) > 1 else 0.12
brain = Brain(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "brains", "brain_drive_026.npz"))   # leg geometry only
robot = Robot(use_imu=False)
robot.prepare()
robot.hold_here()
j = robot.read_joints()
if j is None:
    raise SystemExit("A servo did not answer.")
start, goal = j[0], brain.stand_hips(h)
input(f"Hold the robot in the air. The legs move to the {h * 1000:.0f} mm pose in 2 s. Press Enter.")
# Safety: hip force capped at 30 % and every hip checked 50 times a second. If a hip is more than
# 20 deg from where it should be, all servos go limp at once.
for n in HIPS:
    robot.legs.write2(robot.id[n], 0x30, 300)
try:
    for k in range(125):
        want = start + (goal - start) * min(1.0, (k + 1) / 100)
        robot.write_hips(want); time.sleep(0.02)
        j = robot.read_joints()
        if j is None:
            robot.stop(hips_off=True); raise SystemExit("A servo did not answer. Limp.")
        bad = [n for n, w, m in zip(HIPS, want, j[0]) if abs(m - w) > np.radians(20)]
        if bad:
            robot.stop(hips_off=True)
            raise SystemExit(f"STOP: {', '.join(bad)} more than 20 deg from target "
                             f"({', '.join(f'{np.degrees(m):+.0f}' for m in j[0])}). All limp. Tell Claude.")
finally:
    pass
q = robot.read_joints()[0]
print("\n  link         wanted   measured   (deg, + = swings back)")
for n, g, m in zip(HIPS, goal, q):
    print(f"  {n:10s} {np.degrees(g):+7.1f}   {np.degrees(m):+7.1f}")
input("\nTake a photo of each side. Press Enter to go limp.")
robot.stop(hips_off=True)
for n in HIPS:
    robot.legs.write2(robot.id[n], 0x30, 1000)
print("Limp. Done.")
