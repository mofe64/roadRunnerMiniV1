"""Make the position gain (P, register 0x15) the SAME on all hip and head servos. Nothing moves.

  python3 set_gains.py          set P = 32 (the STS3215 factory value) on all 6 servos
Found 2026-10-31 (servo_dump.py): the servos came with P = 16, 32 or 48. The simulator has one servo
model for all hips, so the real hips must all be the same. Different P made the left and right legs
follow at different speeds (rear legs up to 23 deg apart) and the robot fell sideways.
"""
import time
import sts
from robot import load_config, open_buses, HIPS, HEAD

P_ADDR, P = 0x15, 32
cfg = load_config()
legs, _ = open_buses(cfg)
for n in HIPS + HEAD:
    sid = cfg["ids"][n]
    old = legs.read(sid, P_ADDR, 1)[0]
    if old != P:
        legs.write(sid, sts.TORQUE_ENABLE, [0])
        legs.write(sid, sts.LOCK, [0]); legs.write(sid, P_ADDR, [P]); legs.write(sid, sts.LOCK, [1])
        time.sleep(0.05)
    print(f"  {n}: P {old} -> {legs.read(sid, P_ADDR, 1)[0]}")
