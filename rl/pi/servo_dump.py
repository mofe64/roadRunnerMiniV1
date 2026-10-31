"""READ ONLY. Print the settings memory (addresses 0..69) of the hip and head servos and show where they differ.
Nothing is written. Nothing moves.

  python3 servo_dump.py
"""
from robot import load_config, open_buses, HIPS, HEAD

cfg = load_config()
legs, _ = open_buses(cfg)
names = HIPS + HEAD
mem = {}
for n in names:
    sid = cfg["ids"][n]
    data = []
    for a in range(0, 70, 10):
        r = legs.read(sid, a, 10)
        data += list(r) if r is not None else [None] * 10
    mem[n] = data
print("addr  " + " ".join(f"{n:>10s}" for n in names))
for a in range(70):
    row = [mem[n][a] for n in names]
    mark = "  <-- differs" if len(set(row)) > 1 else ""
    print(f"0x{a:02X}  " + " ".join(f"{str(v):>10s}" for v in row) + mark)
