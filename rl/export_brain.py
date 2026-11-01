"""Export a trained driving brain for the Pi: plain numbers in one .npz file, so the Pi needs only numpy.

Run on the Mac, in rl/:
  RUN=drive_026 .venv/bin/python export_brain.py   (in rl/)
It writes ../robot/brains/brain_<RUN>.npz and checks that the numpy brain gives the same actions as the trained one.
Commit it; the Pi gets it with git pull.
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
import drive_env as de
import balance_env as be
from brain import Brain


def export(run):
    from stable_baselines3 import PPO
    model = PPO.load(os.path.join(HERE, "runs", run, "latest.zip"), device="cpu")
    p = model.policy
    layers = [l for l in p.mlp_extractor.policy_net if hasattr(l, "weight")] + [p.action_net]
    env = de.DriveEnv(seed=0, shoves=False, realism=False)
    hs, poses = env.pose_table
    out = dict(
        # the brain: Linear, tanh, Linear, tanh, Linear (the mean action; the Pi uses no random noise)
        **{f"w{i}": l.weight.detach().numpy().astype(np.float32) for i, l in enumerate(layers)},
        **{f"b{i}": l.bias.detach().numpy().astype(np.float32) for i, l in enumerate(layers)},
        n_layers=len(layers),
        # everything the Pi needs to build the same observation as the simulator
        n_obs=env.n_obs, n_act=env.n_act, history=be.HISTORY, mirror=int(be.MIRROR), policy_hz=be.POLICY_HZ,
        wheel_r=be.WHEEL_R, hip_range=be.HIP_RANGE, hip_max_speed=min(be.HIP_MAX_SPEED, be.HIP_SLEW),
        gyro_lp=be.GYRO_LP_PI, imu_tau=be.IMU_TAU_PI, wheel_posdiff=int(be.WHEEL_POSDIFF),
        speed_max=de.SPEED_MAX, reverse_max=de.REVERSE_MAX, turn_max=de.TURN_MAX,
        height_normal=de.HEIGHT_NORMAL, height_range=np.array(de.HEIGHT_RANGE),
        lag_max=de.LAG_MAX, heading_lag_max=de.HEADING_LAG_MAX, leak=de.LEAK, hold_max=de.HOLD_MAX,
        pose_h=hs, pose_hips=poses, run=run)
    path = os.path.join(HERE, "..", "robot", "brains", f"brain_{run}.npz")
    np.savez(path, **out)
    # check: the numpy brain gives the same action as the trained brain, on real game observations
    brain = Brain(path)
    env = de.DriveEnv(seed=1, shoves=False)
    worst = 0.0
    for g in range(3):
        obs = env.reset()
        for k in range(200):
            a_sb3 = model.predict(obs, deterministic=True)[0]
            a_np = brain.act(obs)
            worst = max(worst, float(np.abs(a_sb3 - a_np).max()))
            obs, r, fell, up, _ = env.step(a_sb3)
            if fell or up:
                break
    print(f"wrote {path}; largest difference to the trained brain: {worst:.2e} (must be below 1e-4)")
    if worst > 1e-4:
        sys.exit("CHECK FAILED: do not use this file")


if __name__ == "__main__":
    export(os.environ.get("RUN", "drive_018"))
