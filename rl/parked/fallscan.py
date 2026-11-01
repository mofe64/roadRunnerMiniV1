"""Real falls (lean + fall speed, Pi reacts at TRIG deg): where does the robot rest? One model per call (MODEL_XML)."""
import sys, os, numpy as np, mujoco
sys.path.insert(0, "../rl"); sys.path.insert(0, "../sim")
import os, sys
HERE_ = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE_, os.path.join(HERE_, ".."), os.path.join(HERE_, "..", "..", "robot"), os.path.join(HERE_, "..", "..", "sim")):
    sys.path.insert(0, _p)   # parked: rl/, robot/ and sim/ modules
from getup_env import GetupEnv
from drive_env import DriveEnv
from build_model_v03 import leg_angles
e = GetupEnv(seed=1); e.difficulty = 0.0; m, d = e.m, e.d
CATCH = float(os.environ.get("CATCH", "0"))
def run(p, w, trig):
    DriveEnv.reset(e, pitch=p); d.qvel[4] = w; mujoco.mj_forward(m, d)
    tuck = np.array(leg_angles(0.065))[[0, 1, 0, 1]]; trig_k = None; target = e.stand_normal.copy()
    for k in range(250):
        g = e.true_gravity(); tilt = np.degrees(np.arccos(-g[2])); pitch = np.degrees(np.arctan2(g[0], -g[2]))
        if trig_k is None and tilt > trig: trig_k = k
        goal = e.stand_normal if trig_k is None else tuck
        target = np.clip(goal, target - 0.094, target + 0.094)
        d.ctrl[e.hip_act] = target
        # CATCH: after the trigger the wheels drive toward the fall (full power) for 0.3 s, then off
        d.ctrl[e.wheel_act] = (CATCH * np.sign(pitch) * 1.85 if trig_k is not None and k - trig_k < 15 else 0.0)
        for _ in range(e.substeps): mujoco.mj_step(m, d)
    return pitch
rests = [run(s * p, s * w, t) for s in (1, -1) for p in (0.1, 0.2) for w in (0, 1.5, 3) for t in (20, 35)]
ok = sum(abs(r) < 45 for r in rests)
print(f"{os.path.basename(os.environ.get('MODEL_XML',''))} catch {CATCH}: rests under 45 deg in {ok}/{len(rests)}; rests {[round(r) for r in rests]}")
if os.environ.get("SAVE_TAG"):
    for face, s in (("front", 1), ("back", -1)):
        r = run(s * 0.2, s * 1.5, 35)
        np.save(f"rest_{face}_65_{os.environ['SAVE_TAG']}.npy", np.concatenate([d.qpos, d.qvel])); print(face, "rest", round(r))
