import sys, numpy as np
from stable_baselines3 import PPO
from drive_env import DriveEnv, POLICY_HZ
model = PPO.load("runs/drive_017/latest.zip", device="cpu")
v, w = float(sys.argv[1]), float(sys.argv[2]); realism = sys.argv[3] == "1"; N = int(sys.argv[4])
env = DriveEnv(seed=11, shoves=False, realism=realism)
hs, poses = env.pose_table
rows = []
for g in range(N):
    obs = env.reset(pitch=env.rng.uniform(-0.035, 0.035))
    env.command = np.array([v, w, .12]); env.next_command = 10**9
    env.stand_hips = np.array([np.interp(.12, hs, poses[:, i]) for i in range(4)]); obs = env.observe()
    sat = 0; acts = []
    while True:
        a = model.predict(obs, deterministic=True)[0]
        obs, r, fell, tu, _ = env.step(a); acts.append(a)
        sat += np.any(np.abs(a[4:6]) > 0.98)
        if fell or tu: break
    gr = env.true_gravity(); pitch = np.degrees(np.arctan2(gr[0], -gr[2]))
    m = env.m
    rows.append(dict(fell=int(fell), t=env.steps/POLICY_HZ, delay=env.act_delay, imu=env.imu_delay,
        ws=env.wheel_strength, fric=m.dof_frictionloss[env.wheel_v].mean()/env.nominal["frictionloss"][env.wheel_v].mean(),
        grip=m.geom_friction[env.tyres,0].mean(), comx=(m.body_ipos[env.torso]-env.nominal["ipos"][env.torso])[0]*1000,
        mass=m.body_mass[env.torso]-env.nominal["mass"][env.torso], sat=sat/env.steps, pitch=pitch))
import pandas as pd
df = pd.DataFrame(rows); pd.set_option("display.width", 200)
print(f"v {v} w {w} realism {realism}: fell {df.fell.sum()}/{N}")
print(df.groupby("fell").mean().round(3).to_string())
df.to_csv(sys.argv[5])
