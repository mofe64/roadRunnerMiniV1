"""SAFE single-servo move test. The servo force is capped LOW and the test stops at once if the servo
moves the wrong way. Use it to check one hip or head servo.

  python3 servo_test.py hip_r_L            small moves (+/- 4 deg) around where the link is now
  python3 servo_test.py hip_r_L --offset0  first set this servo's offset register back to 0 (factory)

Safety: torque limit 15 % (register 0x30), speed limit, steps of 45 counts (4 deg). At 50 checks per
second: if the servo is more than 4 deg PAST or AWAY from where it should go, torque goes off at once.
"""
import sys
import time
import sts
from robot import load_config, save_config, open_buses

TORQUE_LIMIT_ADDR = 0x30
name = sys.argv[1]
cfg = load_config()
legs, _ = open_buses(cfg)
sid = cfg["ids"][name]


def pos():
    p = legs.read2(sid, sts.PRESENT_POS)
    if p is None:
        off(); raise SystemExit("No answer from the servo. Torque off.")
    return p


def off():
    for _ in range(3):
        legs.write(sid, sts.TORQUE_ENABLE, [0])


legs.write(sid, sts.TORQUE_ENABLE, [0])
if "--offset0" in sys.argv:
    old = sts.signed(legs.read2(sid, sts.OFFSET), 11)
    legs.write(sid, sts.LOCK, [0]); legs.write2(sid, sts.OFFSET, 0); legs.write(sid, sts.LOCK, [1])
    time.sleep(0.05)
    print(f"{name}: offset {old} -> {sts.signed(legs.read2(sid, sts.OFFSET), 11)} (factory). config.json zero is now WRONG: redo calibrate.py legs later.")
p0 = pos()
print(f"{name} (ID {sid}): start count {p0}, offset {sts.signed(legs.read2(sid, sts.OFFSET), 11)}")
legs.write2(sid, TORQUE_LIMIT_ADDR, 150)          # 15 % force
legs.write(sid, sts.ACC, [20])
legs.write2(sid, sts.GOAL_SPEED, 300)             # slow
legs.write2(sid, sts.GOAL_POS, p0)
legs.write(sid, sts.TORQUE_ENABLE, [1])
ok = True
try:
    for goal in (p0 + 45, p0, p0 - 45, p0):
        legs.write2(sid, sts.GOAL_POS, goal)
        start = pos(); t_end = time.time() + 1.5
        while time.time() < t_end:
            p = pos()
            err_now = (p - goal + 2048) % 4096 - 2048
            err_start = (start - goal + 2048) % 4096 - 2048
            if abs(err_now) > abs(err_start) + 45:      # going the wrong way or overshooting
                off(); ok = False
                print(f"  STOP: goal {goal}, servo at {p} (moved the WRONG way). Torque off.")
                break
            time.sleep(0.02)
        if not ok:
            break
        print(f"  goal {goal}: at {pos()}  load {legs.read2(sid, sts.PRESENT_LOAD)}")
finally:
    off()
    legs.write2(sid, TORQUE_LIMIT_ADDR, 1000)
print("RESULT:", "moves correctly" if ok else "moves the WRONG way", "- torque off, limp.")
