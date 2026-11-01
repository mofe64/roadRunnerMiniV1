"""Show the 4 hip angles live (deg, + = swings back, 0 = short link straight down). Servos stay limp.

  python3 tools/live_hips.py        Ctrl+C to stop
Move a leg by hand and watch its numbers. In the stand pose the front link reads about -42, the rear +42.
"""
import time
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
import sts
from robot import Robot, HIPS

robot = Robot(use_imu=False)
robot.legs.torque([robot.id[n] for n in HIPS], False)
print("   ".join(f"{n:>8s}" for n in HIPS))
try:
    while True:
        q = robot.read_joints()
        if q is not None:
            print("\r" + "   ".join(f"{np.degrees(a):+8.1f}" for a in q[0]), end="", flush=True)
        time.sleep(0.2)
except KeyboardInterrupt:
    print()
