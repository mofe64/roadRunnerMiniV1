"""Turn a servo's horn half a turn (180 deg) against its link, so that the joint's working range is far
from the servo's 0/4095 point. Use it when calibrate.py prints "Run fix_horns.py <name>".

  python3 fix_horns.py hip_r_L hip_r_R

For each servo:
 1. You take out the 4 screws between the short link and the horn. Leave the horn on the servo.
    The link stays on the knee and hangs free.
 2. The servo turns the bare horn half a turn, slowly, with low force (nothing is attached to it).
 3. You put the link back on the horn with the same 4 screws (and washers), link pointing STRAIGHT DOWN.
    The 4 holes line up again after half a turn.
Then run: python3 calibrate.py legs
"""
import sys
import time
import sts
from robot import load_config, open_buses

TORQUE_LIMIT_ADDR = 0x30
cfg = load_config()
legs, _ = open_buses(cfg)
names = sys.argv[1:]
if not names:
    raise SystemExit(__doc__)


def pos(sid):
    p = legs.read2(sid, sts.PRESENT_POS)
    if p is None:
        legs.write(sid, sts.TORQUE_ENABLE, [0]); raise SystemExit("No answer from the servo. Torque off.")
    return p


for n in names:
    sid = cfg["ids"][n]
    legs.write(sid, sts.TORQUE_ENABLE, [0])
    legs.write(sid, sts.LOCK, [0]); legs.write2(sid, sts.OFFSET, 0); legs.write(sid, sts.LOCK, [1])
    time.sleep(0.05)
    input(f"\n{n}: take out the 4 screws between its short link and the horn. The link must be FREE of the horn.\n"
          f"Keep fingers away from the horn. Press Enter, and the bare horn turns half a turn.")
    p0 = pos(sid)
    goal = p0 + 2048 if p0 < 2048 else p0 - 2048          # this path never crosses 0/4095
    legs.write2(sid, TORQUE_LIMIT_ADDR, 300)            # 30 % force: enough for a bare horn
    legs.write(sid, sts.ACC, [20])
    legs.write2(sid, sts.GOAL_SPEED, 600)
    legs.write2(sid, sts.GOAL_POS, p0)
    legs.write(sid, sts.TORQUE_ENABLE, [1])
    legs.write2(sid, sts.GOAL_POS, goal)
    ok, t_end, best = False, time.time() + 10, abs(goal - p0)
    while time.time() < t_end:
        p = pos(sid)
        err = abs(goal - p)
        if err < 15:
            ok = True; break
        if err > best + 60:                              # going the wrong way
            break
        best = min(best, err)
        time.sleep(0.02)
    legs.write(sid, sts.TORQUE_ENABLE, [0])
    legs.write2(sid, TORQUE_LIMIT_ADDR, 1000)
    if not ok:
        raise SystemExit(f"{n}: the horn did not get there (at {pos(sid)}, goal {goal}). Torque off. Tell Claude.")
    print(f"  {n}: horn turned from {p0} to {pos(sid)}. Torque off.")
    input("Put the link back on the horn with the 4 screws, link pointing STRAIGHT DOWN. Press Enter when done.")
print("\nDone. Now run: python3 calibrate.py legs")
