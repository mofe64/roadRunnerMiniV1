"""Give each servo its own ID. New servos all have ID 1, so connect ONE servo at a time to the adapter.

  python3 tools/setup_ids.py            then follow the questions
IDs: head_pitch 1, head_yaw 2, hip_f_L 3, hip_f_R 4, hip_r_L 5, hip_r_R 6 (legs bus, 7.4 V)
     wheel_L 11, wheel_R 12 (wheels bus, battery voltage)
L and R are the robot's own left and right (look from behind the robot, nose away from you).
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
import sts
from robot import DEFAULT_IDS, load_config, save_config

cfg = load_config()
ps = sts.ports()
if not ps:
    raise SystemExit("No bus adapter found. Plug its USB cable into the Pi, power it, set its switch to USB.")
print("Adapters:", ", ".join(ps))
for name, new in DEFAULT_IDS.items():
    if input(f"\nConnect ONLY the {name} servo to an adapter (with power). Press Enter, or type s to skip: ").strip() == "s":
        continue
    found = []
    for p in sts.ports():
        b = sts.Bus(p)
        found += [(b, i) for i in b.scan(range(0, 21))]
    if len(found) != 1:
        print(f"  Found {len(found)} servos ({[i for _, i in found]}). Connect exactly one, then run again for this servo.")
        continue
    b, old = found[0]
    ok = b.set_id(old, new) if old != new else True
    print(f"  {name}: ID {old} -> {new}", "OK" if ok else "FAILED")
    cfg["ids"][name] = new
save_config(cfg)
print("\nSaved config.json. Next: wire all servos, then run calibrate.py.")
