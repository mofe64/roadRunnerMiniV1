"""Dynamic get-up scan with a BRAKE phase (review 2026-10-30 16:20). From the real-fall rest on the bump:
phase 1: leg pose P1 + wheel power w1 for t1 s (launch); phase 2: legs to the 120 mm stand pose + wheel w2
for t2 s (brake / reverse); phase 3: legs at stand, wheels 0 (back-EMF brake) until 2.5 s.
Success = at some step inside drive_018's envelope (tilt < 8 deg, |pitch rate| < 1 rad/s, |speed| < 0.15 m/s,
hips within 0.35 rad of stand), then drive_018 (Pi controller code) takes over and must hold 10 s.
MODEL_XML picks the model; REST tag picks the rest file; REAL=1 uses a randomised robot (one per move)."""
import os, sys, itertools, numpy as np, mujoco
from multiprocessing import Pool
sys.path.insert(0, "../rl"); sys.path.insert(0, "../rl/pi"); sys.path.insert(0, "../sim")
from drive_env import DriveEnv
from balance_env import POLICY_HZ, WHEEL_TORQUE, HIP_MAX_SPEED, WHEEL_R, HIP_RANGE
from brain import Brain
from controller import Controller
REAL = os.environ.get("REAL", "0") == "1"; TAG = os.environ.get("TAG", "fs130_0")
env = DriveEnv(seed=0, shoves=False, realism=REAL); m, d = env.m, env.d
hs, poses = env.pose_table
brain = Brain("../rl/pi/brain_drive_018.npz")
def pose(h, s): return np.clip(np.array([np.interp(h, hs, poses[:, i]) for i in range(4)]) + s, -2.35, 2.35)
STAND = pose(0.12, 0.0)
P1 = [(h, s) for h in (0.065, 0.10, 0.14) for s in (-1.2, -0.6, 0.0, 0.6, 1.2)]
MOVES = list(itertools.product(range(len(P1)), (-1.0, -0.5, 0.5, 1.0), (0.2, 0.4, 0.6), (-1.0, -0.5, 0.0, 0.5, 1.0), (0.1, 0.2, 0.4)))
def pitch_rate():
    return d.sensordata[env.gyro + 1]
def run(job):
    face, (ip, w1, t1, w2, t2), seed = job
    env.rng = np.random.default_rng(seed); env.reset()          # randomised robot if REAL
    st = np.load(f"rest_{face}_65_{TAG}.npy"); d.qpos[:] = st[:m.nq]; d.qvel[:] = 0; mujoco.mj_forward(m, d)
    target = d.qpos[env.hip_q].copy(); k1, k2 = int(t1 * POLICY_HZ), int((t1 + t2) * POLICY_HZ)
    for k in range(int(2.5 * POLICY_HZ)):
        if k < k1: goal, w = pose(*P1[ip]), w1
        elif k < k2: goal, w = STAND, w2
        else: goal, w = STAND, 0.0
        target = np.clip(goal, target - HIP_MAX_SPEED / POLICY_HZ, target + HIP_MAX_SPEED / POLICY_HZ)
        d.ctrl[env.hip_act] = target - env.hip_offset
        d.ctrl[env.wheel_act] = env.wheel_strength * WHEEL_TORQUE * w
        for _ in range(env.substeps): mujoco.mj_step(m, d)
        g = env.true_gravity(); tilt = np.arccos(np.clip(-g[2], -1, 1))
        if tilt > np.radians(120): return (face, ip, w1, t1, w2, t2, "flip", k)
        v = WHEEL_R * np.mean(d.qvel[env.wheel_v])
        if (k >= k1 and tilt < np.radians(8) and abs(pitch_rate()) < 1.0 and abs(v) < 0.15
                and np.all(np.abs(d.qpos[env.hip_q] - STAND + env.hip_offset) < HIP_RANGE)):
            ctl = Controller(brain); ctl.set_command(0, 0, 0.12); ctl.reset(d.qpos[env.hip_q])
            for j in range(10 * POLICY_HZ):
                tg, pw, a = ctl.step(env.true_gravity(), d.sensordata[env.gyro:env.gyro + 3].copy(), d.qpos[env.hip_q].copy(),
                                     d.qvel[env.hip_v].copy(), d.qvel[env.wheel_v].copy())
                d.ctrl[env.hip_act] = np.asarray(tg) - env.hip_offset
                d.ctrl[env.wheel_act] = env.wheel_strength * WHEEL_TORQUE * np.asarray(pw)
                for _ in range(env.substeps): mujoco.mj_step(m, d)
                g = env.true_gravity()
                if np.arccos(np.clip(-g[2], -1, 1)) > np.radians(45):
                    return (face, ip, w1, t1, w2, t2, "envelope_then_fell", k)
            return (face, ip, w1, t1, w2, t2, "held10s", k)
    return (face, ip, w1, t1, w2, t2, "never", k)
if __name__ == "__main__":
    jobs = [(f, mv, i) for f in ("front", "back") for i, mv in enumerate(MOVES)]
    with Pool(4) as pool: res = pool.map(run, jobs, chunksize=40)
    from collections import Counter
    for f in ("front", "back"):
        r = [x for x in res if x[0] == f]
        c = Counter(x[6] for x in r)
        print(f"{TAG} real={REAL} {f}: {len(r)} moves -> {dict(c)}")
        for x in [x for x in r if x[6] == "held10s"][:5]:
            print("    held:", "P1", P1[x[1]], "w1", x[2], "t1", x[3], "w2", x[4], "t2", x[5], "in envelope at %.2f s" % (x[7] / POLICY_HZ))
    np.save(f"scanc_{TAG}_real{int(REAL)}.npy", np.array(res, dtype=object), allow_pickle=True)
