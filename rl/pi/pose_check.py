"""Check the calibration: move the legs to the stand pose and hold it. The brain does NOT run.

  python3 pose_check.py            stand pose, leg height 120 mm
  python3 pose_check.py 0.08       another leg height in m (0.065 .. 0.140)
Hold the robot in the air. The legs move slowly (2 s). In the stand pose, both legs must look the same:
front short link forward, rear short link back, by the same angle, and the wheel under the hips.
Press Enter to make the robot limp again.
"""
import sys
import time
import numpy as np
from brain import Brain
from robot import Robot, HIPS

h = float(sys.argv[1]) if len(sys.argv) > 1 else 0.12
brain = Brain("brain_drive_020.npz")
robot = Robot(use_imu=False)
robot.prepare()
robot.hold_here()
j = robot.read_joints()
if j is None:
    raise SystemExit("A servo did not answer.")
start, goal = j[0], brain.stand_hips(h)
input(f"Hold the robot in the air. The legs move to the {h * 1000:.0f} mm pose in 2 s. Press Enter.")
for k in range(100):
    robot.write_hips(start + (goal - start) * (k + 1) / 100); time.sleep(0.02)
time.sleep(0.5)
q = robot.read_joints()[0]
print("\n  link         wanted   measured   (deg, + = swings back)")
for n, g, m in zip(HIPS, goal, q):
    print(f"  {n:10s} {np.degrees(g):+7.1f}   {np.degrees(m):+7.1f}")
input("\nTake a photo of each side. Press Enter to go limp.")
robot.stop(hips_off=True)
print("Limp. Done.")
