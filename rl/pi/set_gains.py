"""Set the position gain (P, register 0x15) of the hip and head servos. Nothing moves.

  python3 set_gains.py                 set P = 32 (the STS3215 factory value) on all 6 servos
  python3 set_gains.py hip_r_R 64      set P = 64 on one servo only
Found 2026-10-31 (servo_dump.py): the servos came with P = 16, 32 or 48. The simulator has one servo
model for all hips, so the real hips must all follow the same way. Different P made the left and right legs
follow at different speeds (rear legs up to 23 deg apart) and the robot fell sideways.
Found 2026-10-31 (hip_bench.py, two runs): with P = 32 on all, the rear-right hip pushes with about 40 %
of the force of the other three hips and moves at half their speed. All settings are the same, so it is
the servo itself. Use hip_bench.py after each change to check that the four hips follow the same way.
"""
import sys
import time
import sts
from robot import load_config, open_buses, HIPS, HEAD

P_ADDR = 0x15
cfg = load_config()
legs, _ = open_buses(cfg)
if len(sys.argv) == 3:
    names, P = [sys.argv[1]], int(sys.argv[2])
    if names[0] not in HIPS + HEAD or not 8 <= P <= 96:
        raise SystemExit("Use: set_gains.py <servo name> <P from 8 to 96>, for example: set_gains.py hip_r_R 64")
else:
    names, P = HIPS + HEAD, 32
for n in names:
    sid = cfg["ids"][n]
    old = legs.read(sid, P_ADDR, 1)[0]
    if old != P:
        legs.write(sid, sts.TORQUE_ENABLE, [0])
        legs.write(sid, sts.LOCK, [0]); legs.write(sid, P_ADDR, [P]); legs.write(sid, sts.LOCK, [1])
        time.sleep(0.05)
    print(f"  {n}: P {old} -> {legs.read(sid, P_ADDR, 1)[0]}")
