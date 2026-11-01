"""Check the IMU: is it found, which way are its axes, how much noise. Nothing moves.

  python3 tools/check_imu.py
Put the robot upright and still (on a box or held still on the floor).
"""
import time
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
from imu import IMU
from robot import load_config, save_config

cfg = load_config()
imu = IMU(axes=np.array(cfg.get("imu_axes", np.eye(3).tolist())))
print("IMU found.")
input("Keep the robot upright and STILL. Press Enter (measures for 2 s).")
zero, jitter = imu.calibrate(2.0)
g, t = [], time.perf_counter()
for i in range(100):
    gyro, acc = imu.raw(); g.append(np.concatenate([gyro, acc])); time.sleep(0.01)
g = np.array(g)
rate = 100 / (time.perf_counter() - t)
down = -g[:, 3:].mean(axis=0); down /= np.linalg.norm(down)
print(f"  gyro zero {np.degrees(zero).round(2)} deg/s, gyro noise {jitter.max():.4f} rad/s (sim uses up to 0.02)")
print(f"  'down' = {down.round(3)} (should be about 0, 0, -1); reads per second possible: {rate:.0f}")
print(f"  tilt noise {np.degrees(np.std(g[:, 3:5], axis=0) ).round(2)} deg-ish per reading (sim: up to 0.3 after the filter)")
ok = down[2] < -0.95
input("\nNow tip the NOSE DOWN about 20 deg and hold it. Press Enter.")
gy, acc = imu.raw(); d1 = -acc / np.linalg.norm(acc)
nose = d1[0] > 0.2
input("Back upright. Now tip it onto its LEFT side about 20 deg and hold it. Press Enter.")
gy, acc = imu.raw(); d2 = -acc / np.linalg.norm(acc)
left = d2[1] > 0.2
print(f"  nose down: down = {d1.round(2)} -> {'OK' if nose else 'WRONG (x should be +)'}")
print(f"  left down: down = {d2.round(2)} -> {'OK' if left else 'WRONG (y should be +)'}")
if ok and nose and left:
    print("\nIMU axes are correct. Next: python3 run_brain.py --air")
else:
    print("\nThe axes do not match the robot. Tell Claude these numbers; config.json 'imu_axes' must change.")
