"""Test the Pi program against the simulator, before it touches the robot.

1. Same observation: the Pi controller and the simulator build the same 75 numbers at every step.
2. Closed loop: the Pi controller (not the training code) drives the simulated robot through the
   report card commands, with full realism, and must stay up.
3. Frozen hips (review 2026-11-01): hips held at the stand pose, the brain drives the wheels only, 10 s,
   like the real test T2. Must stay up and stay calm (gyro 9-13 Hz share < 30 %, wheel sign flips < 5/s).
Run in rl/:  RUN=drive_020 .venv/bin/python pi/sim_test.py   (needs pi/brain_<RUN>.npz)
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from drive_env import DriveEnv
from balance_env import WHEEL_TORQUE
from brain import Brain
from controller import Controller

RUN = os.environ.get("RUN", "drive_020")
brain = Brain(os.path.join(HERE, f"brain_{RUN}.npz"))


def sensors(env):
    """What the Pi reads. Wheel speed = position difference over one decision, as robot.read_joints does."""
    d = env.d
    q = d.qpos[env.wheel_q].copy()
    prev = getattr(env, "pi_wheel_prev", None)
    wheel_v = np.zeros(2) if prev is None or env.steps == 0 else (q - prev) * 50.0
    env.pi_wheel_prev = q
    return (env.true_gravity(), d.sensordata[env.gyro:env.gyro + 3].copy(), d.qpos[env.hip_q].copy(),
            d.qvel[env.hip_v].copy(), wheel_v)


def buzz(gyro_y, wheel_a):
    """Share of the gyro power at 9-13 Hz (after 1 s) and wheel action sign flips per second."""
    g = np.asarray(gyro_y[50:]) - np.mean(gyro_y[50:])
    if len(g) < 100:
        return float("nan"), float("nan")
    P = np.abs(np.fft.rfft(g * np.hanning(len(g)))) ** 2; f = np.fft.rfftfreq(len(g), 0.02)
    share = P[(f >= 9) & (f <= 13)].sum() / max(P[f > 0.5].sum(), 1e-12)
    a = np.asarray(wheel_a)
    from drive_env import count_flips
    return float(share), float(count_flips(a) / (len(a) / 50.0))


def frozen(games=10):
    env = DriveEnv(seed=21, shoves=False, realism=True)
    up, shares, flips = 0, [], []
    for g in range(games):
        env.reset(pitch=env.rng.uniform(-0.035, 0.035))
        env.command = np.array([0.0, 0.0, 0.12]); env.next_command = 10 ** 9
        env.stand_hips = brain.stand_hips(0.12)
        ctl = Controller(brain); ctl.set_command(0.0, 0.0, 0.12); ctl.reset(env.d.qpos[env.hip_q])
        G, A = [], []
        while True:
            gr, gy, hq, hv, wv = sensors(env)
            G.append(gy[1])
            gr = gr + env.gravity_bias + env.noise("gravity", 3); gy = gy + env.noise("gyro", 3)
            hq = hq + env.hip_offset + env.noise("hip_pos", 4); hv = hv + env.noise("hip_vel", 4)
            wv = wv + env.noise("wheel_vel", 2)
            target, pwm, a = ctl.step(gr, gy, hq, hv, wv)
            A.append(float(np.mean(pwm)))
            act = np.concatenate([np.zeros(2 if brain.c["mirror"] else 4), pwm])   # hips stay at the stand pose
            _, _, fell, timeup, _ = env.step(np.clip(act, -1, 1))
            if fell or timeup:
                up += not fell; break
        sh, fl = buzz(G, A); shares.append(sh); flips.append(fl)
    sh, fl = np.nanmean(shares), np.nanmean(flips)
    ok = up == games and sh < 0.3 and fl < 5
    print(f"3. frozen hips, standing 120 mm: stayed up {up}/{games}, gyro 9-13 Hz share {100 * sh:.0f} % (< 30), "
          f"wheel flips {fl:.1f}/s (< 5)")
    return ok


def parity(games=5):
    env, worst = DriveEnv(seed=3, shoves=False, realism=False), 0.0
    for g in range(games):
        env.reset(pitch=0.0)
        v, w, h = [(0.1, 0.0, 0.12), (-0.1, 0.5, 0.10), (0.0, 1.0, 0.135), (0.15, 0.0, 0.08), (0.0, 0.0, 0.12)][g]
        env.command = np.array([v, w, h]); env.next_command = 10 ** 9
        env.stand_hips = brain.stand_hips(h)
        obs = env.observe()
        ctl = Controller(brain); ctl.set_command(v, w, h); ctl.reset(env.d.qpos[env.hip_q])
        for k in range(300):
            o2 = ctl.observe(*sensors(env))
            worst = max(worst, float(np.abs(o2 - obs).max()))
            a = brain.act(o2); ctl.last_action = a; ctl.steps += 1
            obs, r, fell, up, _ = env.step(a)
            if fell or up:
                break
    print(f"1. observation parity: largest difference {worst:.2e} (must be below 1e-5)")
    return worst < 1e-5


def closed_loop(games=10):
    """The Pi controller drives; the simulator is the robot (full realism: delays, noise, random robot)."""
    env = DriveEnv(seed=11, shoves=False, realism=True)
    lines = [(0.0, 0.0, 0.12), (0.1, 0.0, 0.12), (-0.1, 0.0, 0.12), (0.0, 1.0, 0.12), (0.0, 0.0, 0.08)]
    ok = True
    for v, w, h in lines:
        up = 0
        for g in range(games):
            env.reset(pitch=env.rng.uniform(-0.035, 0.035))
            env.command = np.array([v, w, h]); env.next_command = 10 ** 9
            env.stand_hips = brain.stand_hips(h)      # reset picked a random height: use the stand pose of THIS command
            ctl = Controller(brain); ctl.set_command(v, w, h); ctl.reset(env.d.qpos[env.hip_q])
            while True:
                gr, gy, hq, hv, wv = sensors(env)
                gr = gr + env.gravity_bias + env.noise("gravity", 3); gy = gy + env.noise("gyro", 3)
                hq = hq + env.hip_offset + env.noise("hip_pos", 4); hv = hv + env.noise("hip_vel", 4)
                wv = wv + env.noise("wheel_vel", 2)
                target, pwm, a = ctl.step(gr, gy, hq, hv, wv)
                # the "servos": convert the Pi's targets back to the simulator's action, then step
                hip_a = (target - env.stand_hips) / 0.35
                act = np.concatenate([hip_a[[0, 1]], pwm]) if brain.c["mirror"] else np.concatenate([hip_a, pwm])
                _, _, fell, timeup, _ = env.step(np.clip(act, -1, 1))
                if fell or timeup:
                    up += not fell; break
        print(f"2. closed loop, command {v:+.2f} m/s, turn {w:+.1f}, height {h*1000:.0f} mm: stayed up {up}/{games}")
        ok &= up >= games - 1
    return ok


if __name__ == "__main__":
    a = parity(); b = closed_loop(); c = frozen()
    print("PI PROGRAM SIM TEST:", "PASS" if a and b and c else "FAIL")
