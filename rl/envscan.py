"""Run in rl/:  RUN=drive_022 .venv/bin/python envscan.py
Level-1 envelope of the v022 plant: drive_022 brain with hips frozen at a=0 (review round 4, 3b)."""
import sys, os, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from drive_env import DriveEnv, POLICY_HZ
from multiprocessing import Pool
PATH = os.path.join(HERE, "runs", os.environ.get("RUN", "drive_022"), "latest.zip")
N = 20
def game(env, model, lean_deg=0.0, shove=0.0):
    hs, poses = env.pose_table; H = 0.12
    obs = env.reset(pitch=np.radians(lean_deg))
    env.command = np.array([0, 0, H]); env.next_command = 10**9; env.max_steps = 10**9
    env.stand_hips = np.array([np.interp(H, hs, poses[:, i]) for i in range(4)]); obs = env.observe()
    T = 7 * POLICY_HZ if shove else 5 * POLICY_HZ
    for k in range(T):
        a = model.predict(obs, deterministic=True)[0]; a[:-2] = 0
        if shove and k == 2 * POLICY_HZ:
            env.d.xfrc_applied[env.torso, :3] = np.array([shove * env.rng.choice([-1, 1]), 0, 0]) * POLICY_HZ
            obs, r, fell, up, info = env.step(a)   # step clears xfrc after the physics steps
        else:
            obs, r, fell, up, info = env.step(a)
        if fell: return 0
    return 1
def job(spec):
    kind, val, seed = spec
    from stable_baselines3 import PPO
    model = PPO.load(PATH, device="cpu"); env = DriveEnv(seed=seed, shoves=False)
    ok = 0
    for g in range(N):
        s = 1 if g % 2 == 0 else -1
        ok += game(env, model, lean_deg=s * val) if kind == "lean" else game(env, model, shove=val)
    return kind, val, ok
if __name__ == "__main__":
    specs = [("lean", v, 10 + i) for i, v in enumerate([3, 5, 7, 9])] + [("shove", v, 50 + i) for i, v in enumerate([0.1, 0.15, 0.2, 0.25, 0.3])]
    with Pool(4) as p:
        for kind, val, ok in p.imap(job, specs): print(f"{kind} {val}: {ok}/{N} recovered", flush=True)
