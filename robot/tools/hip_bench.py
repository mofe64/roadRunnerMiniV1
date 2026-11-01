"""Bench test of the hip servos: how fast and how stiff they follow a step. The result goes into the
simulator's servo model (the "proper way": measure the actuator, then model it).

  python3 tools/hip_bench.py              steps of 8 deg
  python3 tools/hip_bench.py 20           steps of 20 deg (the brain's full hip range), to measure the top speed
Hold the robot in the AIR by the body, legs free. Each hip in turn does small steps (+8 deg, back,
-8 deg, back) around the stand pose. The other hips hold the stand pose. Every position reading is
saved with its time in logs/hip_bench_<time>.csv. Send that file to Claude.
Safety: if a hip is more than (step + 12) deg from its target, all servos go limp at once.
"""
import os
import sys
import time
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
import sts
from brain import Brain
from robot import Robot, HIPS

STEP_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
if not 1.0 <= STEP_DEG <= 20.0:
    raise SystemExit("Step must be 1 to 20 deg.")
STEP = np.radians(STEP_DEG)
LIMIT = np.radians(STEP_DEG + 12)        # stop if a hip is further than this from its target
brain = Brain(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "brains", "brain_drive_026.npz"))   # leg geometry only
robot = Robot(use_imu=False)
robot.prepare()
robot.hold_here()
j = robot.read_joints()
if j is None:
    raise SystemExit("A servo did not answer.")
stand = brain.stand_hips(0.12)
input("Hold the robot in the air by the body, legs free. Legs go to the stand pose in 2 s. Press Enter.")
start = j[0]
for k in range(100):
    robot.write_hips(start + (stand - start) * (k + 1) / 100); time.sleep(0.02)
time.sleep(0.5)

os.makedirs("logs", exist_ok=True)
path = time.strftime(f"logs/hip_bench_%Y%m%d_%H%M%S_{STEP_DEG:.0f}deg.csv")
f = open(path, "w")
f.write("t,hip,target,pos,speed,load\n")
t0 = time.perf_counter()
try:
    for i, n in enumerate(HIPS):
        sid = robot.id[n]
        for delta in (STEP, 0.0, -STEP, 0.0):
            tgt = stand.copy(); tgt[i] += delta
            robot.write_hips(tgt)
            t_end = time.perf_counter() + 0.6
            while time.perf_counter() < t_end:
                r = robot.legs.read(sid, sts.PRESENT_POS, 6)
                if r is None:
                    continue
                pos = robot.to_rad(n, r[0] | (r[1] << 8))
                spd = robot.sign[n] * sts.signed(r[2] | (r[3] << 8)) * (2 * np.pi / 4096)
                load = sts.signed(r[4] | (r[5] << 8), 10)
                f.write(f"{time.perf_counter() - t0:.5f},{i},{tgt[i]:.5f},{pos:.5f},{spd:.4f},{load}\n")
                if abs(pos - tgt[i]) > LIMIT:
                    raise RuntimeError(f"{n} is {np.degrees(pos - tgt[i]):+.0f} deg from its target")
        print(f"  {n}: done")
except RuntimeError as e:
    print("STOP:", e)
finally:
    robot.stop(hips_off=True)
    f.close()
print(f"Limp. Saved {path}")
