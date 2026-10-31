"""Find the real balance point: the tilt at which the robot stands on its wheels by itself for a moment.
The brain does NOT run. The wheels do not turn (they are held, as a brake).

  python3 balance_point.py
1. Hold the robot in the air. The legs move slowly (2 s) to the stand pose and hold it (50 % force).
2. Put it on the floor on its wheels and hold it by the top with two fingers.
3. Tilt it slowly nose down and nose up. Find the angle where it feels weightless: it does not
   pull forward or back. Hold it there for 3 s. The screen shows the tilt 5 times a second.
4. Press Ctrl+C. The robot goes limp. Send the last lines to Claude.
The simulator says this tilt is -0.4 deg. If the real robot is different, the simulator's centre of
weight (or the IMU zero) is wrong, and the brain learned to balance a different robot.
Safety: if a hip is more than 20 deg from the stand pose, all servos go limp at once.
"""
import time
import numpy as np
from brain import Brain
from robot import Robot, HIPS

brain = Brain("brain_drive_020.npz")
robot = Robot(use_imu=True)
robot.prepare()
robot.hold_here()
j = robot.read_joints()
if j is None:
    raise SystemExit("A servo did not answer.")
start, goal = j[0], brain.stand_hips(0.12)
input("Hold the robot in the air. The legs move to the stand pose in 2 s. Press Enter.")
for n in HIPS:
    robot.legs.write2(robot.id[n], 0x30, 500)
try:
    for k in range(100):
        robot.write_hips(start + (goal - start) * (k + 1) / 100)
        robot.write_wheels([0.0, 0.0]); time.sleep(0.02)
    print("Put it on its wheels. Find the weightless tilt (+ = nose down). Ctrl+C to stop.")
    while True:
        robot.write_hips(goal); robot.write_wheels([0.0, 0.0])
        j = robot.read_joints()
        if j is None:
            print("A servo did not answer."); break
        if np.any(np.abs(j[0] - goal) > np.radians(20)):
            print("STOP: a hip is more than 20 deg from the stand pose."); break
        acc = np.mean([robot.imu.raw()[1] for _ in range(10)], axis=0)
        down = -acc / np.linalg.norm(acc)
        pitch = np.degrees(np.arctan2(down[0], -down[2]))
        roll = np.degrees(np.arctan2(down[1], -down[2]))
        print(f"  tilt {pitch:+5.1f} deg   (side {roll:+5.1f})   hips " +
              " ".join(f"{np.degrees(a):+5.1f}" for a in j[0]))
        time.sleep(0.15)
except KeyboardInterrupt:
    pass
finally:
    robot.stop(hips_off=True)
    for n in HIPS:
        robot.legs.write2(robot.id[n], 0x30, 1000)
    print("Limp. Done.")
