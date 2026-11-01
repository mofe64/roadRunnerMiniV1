"""SAFE head pitch move: turn the head to a pitch angle and HOLD it there, for the camera tests.

  python3 tools/head_pitch.py          head 20 deg nose down (the angle for obstacle avoidance)
  python3 tools/head_pitch.py 10       another angle in deg, + = nose down (allowed -25 .. +45)

Only the head PITCH servo gets torque. Head yaw, hips and wheels stay limp (yaw raw range crosses the
encoder seam: it is not moved). Safety (as servo_test.py): torque limit 15 % (register 0x30), slow speed,
steps of 1 deg every 0.05 s, and torque off at once if the servo moves the wrong way. The target count must
stay inside 100..3995 (no seam crossing). Press Enter (or Ctrl+C) to end: torque off, the head goes limp.
The 7.4 V servo bus needs power (battery on). run_brain.py must NOT run at the same time.
"""
import sys
import time
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
import sts
from robot import load_config, open_buses, RAD

TORQUE_LIMIT_ADDR = 0x30
SEAM_LO, SEAM_HI = 100, 3995
deg = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
if not -25.0 <= deg <= 45.0:
    raise SystemExit("Use an angle from -25 to +45 deg (+ = nose down).")
cfg = load_config()
if "head_pitch" not in cfg["sign"] or "head_pitch" not in cfg["zero"]:
    raise SystemExit("head_pitch is not calibrated: run calibrate.py head first.")
legs, _ = open_buses(cfg)
if legs is None:
    raise SystemExit("Did not find the legs/head servo bus. Check the USB cable and the battery.")
sid = cfg["ids"]["head_pitch"]
zero, sign = cfg["zero"]["head_pitch"], cfg["sign"]["head_pitch"]


def pos():
    p = legs.read2(sid, sts.PRESENT_POS)
    if p is None:
        off(); raise SystemExit("No answer from the head pitch servo. Torque off.")
    return p


def off():
    for _ in range(3):
        legs.write(sid, sts.TORQUE_ENABLE, [0])
    legs.write2(sid, TORQUE_LIMIT_ADDR, 1000)


def unwrap(c, ref):
    """Count c as the nearest value to ref (the count wraps at 4096)."""
    return ref + (c - ref + 2048) % 4096 - 2048


legs.write(sid, sts.TORQUE_ENABLE, [0])
mode = legs.read(sid, sts.MODE, 1)
if mode is None or mode[0] != sts.MODE_POSITION:
    raise SystemExit("The head pitch servo is not in position mode. Run run_brain.py once (it sets the mode), or tell Claude.")
target = int(round(zero + sign * np.radians(deg) / RAD))
p0 = pos()
path = np.linspace(unwrap(p0, target), target, 50)
if target < SEAM_LO or target > SEAM_HI or path.min() < SEAM_LO or path.max() > SEAM_HI:
    raise SystemExit(f"STOP: the move {p0} -> {target} counts goes outside {SEAM_LO}..{SEAM_HI} (seam). Tell Claude.")
now_deg = sign * (unwrap(p0, zero) - zero) * RAD * 180 / np.pi
print(f"head pitch (ID {sid}): now {now_deg:+.1f} deg (count {p0}), going to {deg:+.1f} deg (count {target})")

legs.write2(sid, TORQUE_LIMIT_ADDR, 150)          # 15 % force
legs.write(sid, sts.ACC, [20])
legs.write2(sid, sts.GOAL_SPEED, 300)             # slow
legs.write2(sid, sts.GOAL_POS, p0)
legs.write(sid, sts.TORQUE_ENABLE, [1])
ok = True
try:
    step = int(round(np.radians(1.0) / RAD))      # 1 deg in counts
    cur = unwrap(p0, target)
    while cur != target and ok:
        goal = cur + int(np.clip(target - cur, -step, step))
        start = pos()
        legs.write2(sid, sts.GOAL_POS, goal % 4096)
        time.sleep(0.05)
        p = unwrap(pos(), goal)
        if abs(p - goal) > abs(unwrap(start, goal) - goal) + 45:    # moved away from the goal (wrong way)
            ok = False
            print(f"STOP: goal {goal}, servo at {p}: it moved the WRONG way. Torque off.")
        cur = goal
    if ok:
        time.sleep(0.5)
        p = unwrap(pos(), target)
        print(f"HOLDING at {sign * (p - zero) * RAD * 180 / np.pi:+.1f} deg (load {legs.read2(sid, sts.PRESENT_LOAD)}). "
              "Run the camera test in the other terminal. Press Enter here to end (head goes limp).")
        try:
            input()
        except KeyboardInterrupt:
            pass
finally:
    off()
print("Torque off: the head is limp. Hold it if it falls.")
