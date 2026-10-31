"""PARKED 2026-10-30 (review NO-GO: x130 bumps give a 0.02 % open-loop get-up, x160 0.2 %; saved in getup_parked/).
Reopen only when the real robot stands and drives with drive_018, Mofe accepts a 345 mm robot, and one bounded run is agreed.

The get-up game: the robot has fallen onto a bump and must stand up again (sim first, 2026-10-30).

A separate brain from the driving brain. On the robot, the Pi switches between them:
drive -> tilt > 25 deg -> wheels off, legs tucked (65 mm), wait until still -> GET-UP brain
-> upright, slow and at 115-125 mm for 1 s -> drive brain again.

Each game starts with a real fall in the simulation: the robot stands with a lean and is already falling
(0-3 rad/s). At 25 deg the Pi script stops the wheels and tucks the legs, and the robot falls onto its
front or back bump. (A fall from almost still is too easy: the first bump test used it, and the 105 mm
bumps then failed in every real fall.) The brain
gets the robot from that rest. Front and back falls only (a side fall is not trained).

Usage: TASK=getup RUN=getup_001 .venv/bin/python train.py      train
       RUN=getup_001 .venv/bin/python getup_env.py --test      report card (get up, then drive_018 holds it 10 s)
"""
import os
import sys
import numpy as np
import mujoco

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
from build_model_v03 import leg_angles
from balance_env import POLICY_HZ, HIP_RANGE, HIP_MAX_SPEED, WHEEL_TORQUE, MIRROR
from drive_env import DriveEnv, HEIGHT_NORMAL

GETUP_SECONDS = 6.0           # one try lasts at most 6 s (the Pi stops and tries again)
TUCK = 0.065                  # m: legs tucked while it falls and at the start of the get-up
LEG_MIN, LEG_MAX = 0.065, 0.140   # m: the hips can never command a leg outside this range (review 2026-10-30)
FALL_LEAN = (np.radians(5), np.radians(15))    # lean when the fall starts
FALL_RATE = (0.0, 3.0)        # rad/s: how fast it already falls
TRIGGER = np.radians(25)      # the Pi sees the fall at this tilt (the driving brain cannot save it past ~15 deg)
FALL_MAX_S = 4.0              # s: the fall and the wait for "still" take at most this long
STILL_GYRO = 0.2              # rad/s: "still" = every gyro axis below this ...
STILL_S = 0.5                 # ... for 0.5 s
SWING = 1.2                   # rad: the most the whole leg can swing forward or back for the get-up
HIP_LIMIT = 2.35              # rad: hip servo angle limit used in the get-up scans
FLIP = np.radians(120)        # tilted past this = it went over the top: try over
DONE_TILT = np.radians(5)     # handover: upright within 5 deg,
DONE_GYRO = 0.5               # rad/s: slow,
DONE_HEIGHT = 0.005           # m: 115-125 mm,
DONE_HOLD = int(1.0 * POLICY_HZ)   # for 1 s
CARD_GAMES = int(os.environ.get("CARD_GAMES", "50"))
# Reverse curriculum (learning check getup_003: easy starts 83/83 up, real falls 0/57, level stuck at 24 %):
# some games start part of the way along a recorded get-up. These 120 recordings are open-loop moves from
# getup/scanb.py that reach upright from the real-fall rest (getup/demos.py). At level 0 the games start near
# the end of a recording (almost up); as the level rises they start closer to the rest on the bump.
DEMOS = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "getup_demos.npy"), allow_pickle=True)
DRIVE_BRAIN = os.environ.get("DRIVE_BRAIN", os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", "drive_018", "latest.zip"))

assert MIRROR, "the get-up game uses mirrored hips (front, rear) like the driving brain"


class GetupEnv(DriveEnv):
    def __init__(self, seed=None, shoves=False, realism=True):
        super().__init__(seed, shoves=False, realism=realism)   # no random shoves while it gets up
        self.max_steps = int(GETUP_SECONDS * POLICY_HZ)
        # Hip actions -> hip angles. The two mirrored hip actions are LEG LENGTH (-1 = 65 mm, +1 = 140 mm) and
        # LEG SWING (the whole leg turns about the hips, +/-1.2 rad, wheel forward or back). The length can never
        # leave 65-140 mm (review 2026-10-30). A first version limited each hip angle to its 65-140 mm stand value:
        # that box also blocked the swing, and only 36 of 1928 get-up moves found by getup/scanb.py fit in it
        # (learning checks getup_c1/c2 and getup_001: 0 % got up). The legs are outside the body, so a swing
        # cannot hit it; the wheels do touch the body and the bumps in the sim.
        hs = np.linspace(LEG_MIN, LEG_MAX, 76)
        self.leg_table = (hs, np.array([leg_angles(h) for h in hs]))
        self.tuck_hips = np.array(leg_angles(TUCK))[[0, 1, 0, 1]]
        self.mode = "getup"
        self.tyre_set = set(self.tyres)

    def body_on_floor(self):
        """True when anything but the tyres touches the floor: body, bump, head or a leg link (a knee prop)."""
        d, m = self.d, self.m
        for i in range(d.ncon):
            c = d.contact[i]
            if self.floor in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == self.floor else c.geom1
                if other not in self.tyre_set:
                    return True
        return False

    def hips_from_action(self, a):
        """a: 4 hip actions in hip order (L front, L rear, R front, R rear). Mirrored: [length, swing, length, swing]."""
        hs, ang = self.leg_table
        h = LEG_MIN + (np.clip(a[0], -1, 1) + 1) / 2 * (LEG_MAX - LEG_MIN)
        f, r = np.interp(h, hs, ang[:, 0]) + SWING * a[1], np.interp(h, hs, ang[:, 1]) + SWING * a[1]
        return np.clip(np.array([f, r, f, r]), -HIP_LIMIT, HIP_LIMIT)

    def apply(self, action):
        if self.mode == "drive":           # after the handover the driving brain gives the commands
            return super().apply(action)
        self.action_history = np.vstack([np.clip(action, -1.0, 1.0), self.action_history[:-1]])
        late, self.old_substeps = divmod(self.act_delay, self.substeps)
        self.old_ctrl = self.d.ctrl.copy()
        a = self.action_history[late]
        target = self.hips_from_action(a[:4]) - self.hip_offset
        if self.realism:
            step = HIP_MAX_SPEED / POLICY_HZ
            target = np.clip(target, self.hip_target - step, self.hip_target + step)
        self.hip_target = target
        for i, n in enumerate(self.hip_act):
            self.d.ctrl[n] = target[i]
        for i, n in enumerate(self.wheel_act):
            self.d.ctrl[n] = self.wheel_strength * WHEEL_TORQUE * a[4 + i]
        self.d.ctrl[self.act_id["head_yaw"]] = 0.0
        self.d.ctrl[self.act_id["head_pitch"]] = 0.0

    def fall(self):
        """The scripted fall, as the Pi does it: wheels off, legs to 65 mm, wait until still."""
        m, d = self.m, self.d
        target = d.qpos[self.hip_q].copy()
        goal = self.tuck_hips - self.hip_offset
        still = 0
        for k in range(int(FALL_MAX_S * POLICY_HZ)):
            target = np.clip(goal, target - HIP_MAX_SPEED / POLICY_HZ, target + HIP_MAX_SPEED / POLICY_HZ)
            d.ctrl[self.hip_act] = target
            d.ctrl[self.wheel_act] = 0.0
            for _ in range(self.substeps):
                mujoco.mj_step(m, d)
            still = still + 1 if np.all(np.abs(d.sensordata[self.gyro:self.gyro + 3]) < STILL_GYRO) else 0
            if k > 25 and still >= STILL_S * POLICY_HZ:
                break
        self.hip_target = target

    def reset(self, pitch=None):
        self.mode = "getup"
        super().reset(pitch=0.0)               # stand (randomised robot), height reference
        m, d, rng = self.m, self.d, self.rng
        if pitch is None:
            u, L = rng.random(), self.difficulty
            if u < 0.3 * (1 - L):
                return self.easy_start()
            if u < 1 - L:
                return self.demo_start()
        if pitch is None:
            pitch = rng.uniform(*FALL_LEAN) * rng.choice([-1, 1])
        sign = np.sign(pitch)
        # lean the standing robot about its axle line and give it a fall speed in the same direction
        q = np.zeros(4)
        mujoco.mju_euler2Quat(q, np.array([rng.normal(0, np.radians(1)), pitch, 0.0]), "xyz")
        yaw = rng.uniform(-np.pi, np.pi)
        qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        mujoco.mju_mulQuat(d.qpos[3:7], qz, q)
        mujoco.mj_forward(m, d)
        lowest = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.tyres)
        d.qpos[2] -= lowest - 0.0005
        mujoco.mj_forward(m, d)
        R = d.xmat[self.torso].reshape(3, 3)
        d.qvel[3:6] = sign * rng.uniform(*FALL_RATE) * np.array([0.0, 1.0, 0.0])   # body-frame pitch rate
        mujoco.mj_forward(m, d)
        # until the Pi sees the fall: the hips hold the stand pose, the wheels are off
        for _ in range(3 * POLICY_HZ):
            if np.arccos(np.clip(-self.true_gravity()[2], -1, 1)) > TRIGGER:
                break
            d.ctrl[self.hip_act] = self.stand_normal - self.hip_offset
            d.ctrl[self.wheel_act] = 0.0
            for _ in range(self.substeps):
                mujoco.mj_step(m, d)
        self.fall()
        self.start_pitch = float(np.degrees(np.arctan2(self.true_gravity()[0], -self.true_gravity()[2])))
        # the game clock and the memories start now
        self.steps = 0
        self.held = 0
        self.prev_tilt = float(np.arccos(np.clip(-self.true_gravity()[2], -1, 1)))
        self.wheel_start = np.mean(d.qpos[self.wheel_q])
        self.last_action = np.zeros(self.n_act)
        self.action_history[:] = 0.0
        self.action_history[:, :4] = [-1.0, 0.0, -1.0, 0.0]   # "legs tucked, no swing" is what the servos have now
        self.imu_history[:] = np.concatenate([self.true_gravity(), d.sensordata[self.gyro:self.gyro + 3]])
        self.command = np.array([0.0, 0.0, HEIGHT_NORMAL])
        self.stand_hips = self.stand_normal
        self.next_command = 10 ** 9
        return self.observe()

    def easy_start(self):
        """Curriculum (learning check getup_002 never found the catch at the top): at level 0 every game starts
        on the wheels at 120 mm with a small lean (up to 5 deg), so the brain first learns to hold itself up.
        The share of real falls grows with the level (level 100 % = only real falls) and so does the lean."""
        m, d, rng = self.m, self.d, self.rng
        lean = np.radians(5 + 15 * self.difficulty) * rng.uniform(-1, 1)
        d.qpos[3:7] = [np.cos(lean / 2), 0.0, np.sin(lean / 2), 0.0]
        mujoco.mj_forward(m, d)
        lowest = min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.tyres)
        d.qpos[2] -= lowest - 0.0005
        d.qvel[:6] = rng.normal(0.0, 0.05, 6)
        mujoco.mj_forward(m, d)
        self.hip_target = d.qpos[self.hip_q].copy()
        self.start_pitch = float(np.degrees(lean))
        self.steps = self.held = 0
        self.prev_tilt = abs(lean)
        self.wheel_start = np.mean(d.qpos[self.wheel_q])
        self.last_action = np.zeros(self.n_act)
        length = (HEIGHT_NORMAL - LEG_MIN) / (LEG_MAX - LEG_MIN) * 2 - 1
        self.last_action[0] = length
        self.action_history[:] = 0.0
        self.action_history[:, :4] = [length, 0.0, length, 0.0]
        self.imu_history[:] = np.concatenate([self.true_gravity(), d.sensordata[self.gyro:self.gyro + 3]])
        self.command = np.array([0.0, 0.0, HEIGHT_NORMAL])
        self.stand_hips = self.stand_normal
        self.next_command = 10 ** 9
        return self.observe()

    def demo_start(self):
        m, d, rng = self.m, self.d, self.rng
        tr = DEMOS[rng.integers(len(DEMOS))]
        k = int(rng.uniform(0.9 * (1 - self.difficulty), 1.0) * (len(tr) - 1))
        row = tr[k]
        d.qpos[:] = row[:m.nq]; d.qvel[:] = row[m.nq:m.nq + m.nv]
        target, w = row[m.nq + m.nv:m.nq + m.nv + 4], row[-2:]
        yaw = rng.uniform(-np.pi, np.pi)          # face a random way
        qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]); q = d.qpos[3:7].copy()
        mujoco.mju_mulQuat(d.qpos[3:7], qz, q)
        c, s_ = np.cos(yaw), np.sin(yaw); x, y = d.qvel[0], d.qvel[1]
        d.qvel[0], d.qvel[1] = c * x - s_ * y, s_ * x + c * y
        mujoco.mj_forward(m, d)
        lowest = min(min(d.geom_xpos[g][2] - m.geom_size[g][0] for g in self.tyres), 0.0)
        d.qpos[2] -= lowest
        mujoco.mj_forward(m, d)
        self.hip_target = target.copy()
        d.ctrl[self.hip_act] = target
        # the recorded hip pose as length + swing actions (mirrored legs: swing = mean of front and rear)
        swing = np.clip((target[0] + target[1]) / 2 / SWING, -1, 1)
        hs, ang = self.leg_table
        h = np.interp(target[1] - swing * SWING, ang[:, 1][::-1], hs[::-1]) if ang[0, 1] > ang[-1, 1] else np.interp(target[1] - swing * SWING, ang[:, 1], hs)
        length = (h - LEG_MIN) / (LEG_MAX - LEG_MIN) * 2 - 1
        self.start_pitch = float(np.degrees(np.arctan2(self.true_gravity()[0], -self.true_gravity()[2])))
        self.steps = self.held = 0
        self.prev_tilt = float(np.arccos(np.clip(-self.true_gravity()[2], -1, 1)))
        self.wheel_start = np.mean(d.qpos[self.wheel_q])
        self.last_action = np.array([length, swing, w[0], w[1]])
        self.action_history[:] = np.concatenate([[length, swing, length, swing], w])
        self.imu_history[:] = np.concatenate([self.true_gravity(), d.sensordata[self.gyro:self.gyro + 3]])
        self.command = np.array([0.0, 0.0, HEIGHT_NORMAL])
        self.stand_hips = self.stand_normal
        self.next_command = 10 ** 9
        self.demo = True
        return self.observe()

    def done_now(self, tilt):
        # review 2026-10-30 16:30: the hips must also be near the stand pose. height() is vertical, so a long leg
        # swung 30 deg reads 120 mm; the driving brain never saw that pose (handover 1-5/40).
        legs_home = np.all(np.abs(self.d.qpos[self.hip_q] - (self.stand_normal - self.hip_offset)) < HIP_RANGE)
        return (legs_home and tilt < DONE_TILT and np.all(np.abs(self.d.sensordata[self.gyro:self.gyro + 3]) < DONE_GYRO)
                and abs(self.height() - HEIGHT_NORMAL) < DONE_HEIGHT)

    def score(self, action):
        g = self.true_gravity()
        tilt = np.arccos(np.clip(-g[2], -1, 1))
        # Learning check getup_c1 (2M steps): with "1 - tilt/90 deg" here, lying still on the bump (35 deg) earned
        # 60 % of the points, and the brain stopped trying (0 % got up). Now the rest pose earns almost nothing,
        # and points come for each degree it gets closer to upright (and are lost when it falls back).
        # Learning check getup_002 (5M steps): the brain swung the legs and propped the body on a bump at 17 deg,
        # still, for the whole game (0.37 points a step); then it was found resting on the wheels AND a knee.
        # So the upright points now count only when the tyres alone touch the floor.
        on_wheels = 0.0 if self.body_on_floor() else 1.0
        up = on_wheels * np.exp(-(tilt / 0.3) ** 2)             # 0.7 at 10 deg, 1 upright; 0 while on the bump
        progress = 20.0 * (self.prev_tilt - tilt)              # about 12 points for the whole way up
        self.prev_tilt = tilt
        upright = on_wheels * np.exp(-(tilt / 0.1) ** 2)       # extra points close to upright
        tall = np.exp(-(self.height() - HEIGHT_NORMAL) ** 2 / 0.0004) if tilt < 0.2 else 0.0   # normal height once up
        ok = 2.0 if self.done_now(tilt) else 0.0               # the handover condition
        smooth = -0.05 * np.sum((action - self.last_action) ** 2)
        effort = -0.02 * np.sum(action[2:4] ** 2)              # wheels: no full power for nothing
        return up + progress + upright + tall + ok + smooth + effort, tilt

    def step(self, action):
        if self.mode == "drive":
            return DriveEnv.step(self, action)
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self.apply(self.full_action(action))
        new_ctrl = self.d.ctrl.copy()
        for i in range(self.substeps):
            self.d.ctrl[:] = self.old_ctrl if i < self.old_substeps else new_ctrl
            mujoco.mj_step(self.m, self.d)
        reward, tilt = self.score(action)
        self.last_action = action
        self.steps += 1
        self.held = self.held + 1 if self.done_now(tilt) else 0
        got_up = self.held >= DONE_HOLD
        flipped = tilt > FLIP
        if got_up:
            reward += 20.0 + 5.0 * (self.max_steps - self.steps)   # the rest of the game at full points: faster is never worse
        if flipped:
            reward -= 10.0
        truncated = self.steps >= self.max_steps and not got_up
        return self.observe(), float(reward), bool(got_up or flipped), truncated, {"got_up": got_up, "tilt_deg": float(np.degrees(tilt))}

    def handover(self):
        """The Pi switches to the driving brain: fresh memories, lag signals at zero."""
        self.mode = "drive"
        self.steps = 0
        self.last_action = np.zeros(self.n_act)
        self.lag = self.heading_lag = 0.0
        self.max_steps = 10 ** 9
        self.next_command = 10 ** 9
        return self.observe()


def card_games(job):
    path, face, n, seed = job
    from stable_baselines3 import PPO
    getup, drive = PPO.load(path, device="cpu"), PPO.load(DRIVE_BRAIN, device="cpu")
    env = GetupEnv(seed=seed)
    sign = 1 if face == "front" else -1
    first = with_retry = held = 0
    times = []
    for _ in range(n):
        env.max_steps = int(GETUP_SECONDS * POLICY_HZ)
        obs = env.reset(pitch=sign * env.rng.uniform(*FALL_LEAN))
        up = False
        for attempt in range(3):              # the Pi tries at most 3 times
            if attempt:                       # a retry: tuck and wait, as the Pi does
                env.max_steps = int(GETUP_SECONDS * POLICY_HZ)
                env.fall(); env.steps = 0; env.held = 0; env.prev_tilt = float(np.arccos(np.clip(-env.true_gravity()[2], -1, 1))); env.last_action = np.zeros(env.n_act)
                env.action_history[:] = 0.0; env.action_history[:, :4] = [-1.0, 0.0, -1.0, 0.0]
                obs = env.observe()
            while True:
                a, _ = getup.predict(obs, deterministic=True)
                obs, _, term, trunc, info = env.step(a)
                if info["got_up"]:
                    up = True; break
                if term or trunc:
                    break
            if up:
                first += attempt == 0; with_retry += 1
                times.append(env.steps / POLICY_HZ + attempt * GETUP_SECONDS)
                break
        if not up:
            continue
        obs = env.handover()
        ok = True
        for _ in range(10 * POLICY_HZ):       # the driving brain must now hold it for 10 s
            a, _ = drive.predict(obs, deterministic=True)
            obs, _, term, _, _ = env.step(a)
            if term:
                ok = False; break
        held += ok
    return face, first, with_retry, held, times


def test(path):
    from multiprocessing import Pool
    print("GET-UP REPORT CARD for", path, f"(drive brain {DRIVE_BRAIN}), full realism, {CARD_GAMES} falls each way")
    jobs = [(path, face, CARD_GAMES // 5, 1000 * i + (0 if face == "front" else 500)) for face in ("front", "back") for i in range(5)]
    with Pool(min(10, os.cpu_count() or 4)) as pool:
        res = pool.map(card_games, jobs)
    for face in ("front", "back"):
        r = [x for x in res if x[0] == face]
        first, retry, held = (sum(x[i] for x in r) for i in (1, 2, 3))
        t = [v for x in r for v in x[4]]
        verdict = "PASS" if held >= 45 * CARD_GAMES // 50 else "FAIL"
        print(f"fell on the {face:5s}: up on 1st try {first:2d}/{CARD_GAMES}, within 3 tries {retry:2d}/{CARD_GAMES}, "
              f"then driving brain held 10 s {held:2d}/{CARD_GAMES}  {verdict}"
              + (f"  (time to get up: median {np.median(t):.1f} s)" if t else ""))


if __name__ == "__main__" and "--test" in sys.argv:
    run = os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", os.environ.get("RUN", "getup_001"), "latest.zip")
    test(run)
