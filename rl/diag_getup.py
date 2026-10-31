"""Replay a get-up brain at a fixed level; split the results by start type and record why games fail."""
import sys, os, numpy as np
sys.path.insert(0, '.')
from stable_baselines3 import PPO
from getup_env import GetupEnv
pol = PPO.load(sys.argv[1], device="cpu")
lev = float(sys.argv[2]); n = int(sys.argv[3])
e = GetupEnv(seed=11); e.difficulty = lev
rows = []
for g in range(n):
    o = e.reset(); real = abs(e.start_pitch) > 20; mt = 99
    while True:
        a, _ = pol.predict(o, deterministic=True); o, r, te, tr, info = e.step(a); mt = min(mt, info["tilt_deg"])
        if te or tr: break
    why = "up" if info["got_up"] else ("flipped" if info["tilt_deg"] > 119 else "timeout")
    rows.append((real, why, mt, e.steps / 50, info["tilt_deg"], e.height() * 1000, e.wheel_strength, e.act_delay))
for real in (False, True):
    r = [x for x in rows if x[0] == real]
    if not r: continue
    from collections import Counter
    print("real falls" if real else "easy starts", len(r), dict(Counter(x[1] for x in r)),
          "| failed games: min tilt", [round(x[2]) for x in r if x[1] != "up"][:12], "end tilt/height", [(round(x[4]), round(x[5])) for x in r if x[1] != "up"][:8])
    print("   time to get up (s): median", np.median([x[3] for x in r if x[1] == "up"]) if any(x[1]=="up" for x in r) else None)
