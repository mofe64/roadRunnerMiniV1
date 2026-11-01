"""Road Runner Mini: the driving game. Same robot and realism as balance_env.py, plus commands.

Every few seconds the robot gets a new command: a forward speed (m/s), a turn rate (rad/s) and a
leg height (m, crouch or stand tall), like a joystick. It scores points for matching the command
while staying upright.

Train:        TASK=drive TOTAL_STEPS=15000000 .venv/bin/python train.py
Watch:        TASK=drive .venv/bin/mjpython train.py --watch
Report card:  TASK=drive .venv/bin/python drive_env.py --test
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
from build_model_v03 import leg_angles      # hip angles for a given leg height (five-bar geometry)
from balance_env import BalanceEnv, POLICY_HZ, EPISODE_SECONDS, HISTORY, WHEEL_R, MIRROR

# Commands (the "joystick"). A balancing robot needs spare wheel speed to catch itself,
# so the top command is well below the wheel top speed (about 0.36 m/s).
SPEED_MAX = float(os.environ.get("SPEED_MAX", "0.15"))   # m/s at level 100 % (70 mm wheels: 0.2 is at the motor's limit, 2026-10-30)
SPEED_MAX_EASY = 0.07     # m/s at level 0 %
REVERSE_MAX = float(os.environ.get("REVERSE_MAX", "0.10"))   # m/s backwards at level 100 % (reverse at 0.15 fell in every run, 2026-10-30)
TURN_MAX = 1.5            # rad/s (about 86 deg/s)
NEW_COMMAND_EVERY = (2.0, 4.0)   # s: a new command after a random time in this range
STOP_CHANCE = 0.2         # some commands are "stand still"
STOP_SECONDS = (4.0, 10.0)   # s: "stand still" lasts longer than the other commands, so a slow creep costs points (real drive_023 crept 1 cm/s, 2026-11-01)
HEIGHT_RANGE = (0.080, 0.135)   # m: hip-to-axle height commands (legs can do 65-140 mm; keep a margin)
HEIGHT_NORMAL = 0.120
LAG_MAX = 0.25            # m: the "behind schedule" signal is capped at 25 cm
HEADING_LAG_MAX = 0.5     # rad: the "off my heading" signal is capped at about 30 deg
# Both signals slowly forget old errors (half gone after about 1.4 s), so they cannot get stuck at the cap
# ("integrator wind-up"): a test of drive_016 showed them pinned at the cap for whole games.
LEAK = 0.99
# "Hold here" odometer (independent review 2026-10-31): while the speed command is 0, how far the robot has
# rolled since that command started (m, + = ahead). It does NOT leak, so a slow creep stays visible
# (the lag above forgets it in ~2 s; drive_020 crept 0.9 m on the real robot and fell). 0 while driving.
HOLD_MAX = 1.0            # m: clipped, as the odometer in balance_env
HOLD_W = float(os.environ.get("HOLD_W", "1.0"))     # score: -HOLD_W x (m off)^2 while holding
HOLD_LIN = float(os.environ.get("HOLD_LIN", "3.0"))  # score: -HOLD_LIN x |m off| while holding (10 cm costs 0.3; the square alone cost 0.01 at 10 cm)
HOLD_FLAT = float(os.environ.get("HOLD_FLAT", "0.02"))  # m: no linear hold cost within 2 cm; a dead-band wheel cannot hold closer, it hunts (real drive_026 rocked +/-5 cm, review round 6)
FLIP_BAND = 0.1                                      # wheel action must cross zero by more than this to count as a flip
HIP_W = float(os.environ.get("HIP_W", "1.0"))       # score: -HIP_W x mean(hip action^2): stand with centred hips (0.1 was too small: review 2026-11-01 round 4)
UPRIGHT_W = float(os.environ.get("UPRIGHT_W", "10.0"))   # score: -UPRIGHT_W x tilt^2: stand straight (2 let it lean 7 deg back, round 4)
SMOOTH_W = float(os.environ.get("SMOOTH_W", "0.2"))  # score: -SMOOTH_W x sum(action change^2); was 0.05 (drive_021 buzzed on the robot)
WOBBLE_PITCH_W = float(os.environ.get("WOBBLE_PITCH_W", "1.0"))   # score: -W x min(pitch rate^2, WOBBLE_CLIP) (review round 8d; was 0.05)
WOBBLE_CLIP = float(os.environ.get("WOBBLE_CLIP", "1.0"))         # (rad/s)^2
# Experiments: DRIVE_PARTS=speed trains speed commands only (no turns, no height changes)
CARD_GAMES = int(os.environ.get("CARD_GAMES", "50"))   # games per report card line (6 was too few: one fall = 17 %)
DRIVE_PARTS = os.environ.get("DRIVE_PARTS", "speed,turn,height").split(",")
# Driving uses smaller start leans and shoves than the balance game (12 deg, 0.45 N s). Those are at the
# robot's physical limit, so the brain kept all its spare wheel speed for catching them and drove at half speed.
DRIVE_START_LEAN = np.radians(6)   # level-1 start lean: 80 % of what the v022 plant recovers 9/10 (envscan 2026-11-01: 7 deg 20/20, 9 deg 14/20)
DRIVE_SHOVE_MAX = 0.20   # N s, level-1 strongest shove (envscan: 0.2 19/20, 0.3 16/20); not below SHOVE_MAX_EASY 0.2


class DriveEnv(BalanceEnv):
    def __init__(self, seed=None, shoves=True, realism=True):
        super().__init__(seed, shoves, realism)
        self.start_lean, self.shove_max = DRIVE_START_LEAN, DRIVE_SHOVE_MAX
        self.n_frame = 3 + 3 + 4 + 4 + 2 + 3 + 2 + 1 + self.n_act   # 3 commands + 2 lags + the hold odometer
        self.n_obs = self.n_frame * HISTORY
        self.command = np.array([0.0, 0.0, HEIGHT_NORMAL])
        self.axles = [self.m.body("axle_L").id, self.m.body("axle_R").id]
        # hip angles for each leg height, so a height command moves the "home" pose of the hips
        hs = np.linspace(0.065, 0.140, 76)
        self.pose_table = (hs, np.array([[f, r, f, r] for f, r in (leg_angles(h) for h in hs)]))
        self.stand_normal = self.stand_hips.copy()

    def new_command(self):
        was_stop = getattr(self, "command", None) is not None and self.command[0] == 0.0
        top = SPEED_MAX_EASY + (SPEED_MAX - SPEED_MAX_EASY) * self.difficulty
        back = min(top, REVERSE_MAX)   # backwards is capped lower than forwards
        # the height range also grows with the level: 115-125 mm at level 0, 80-135 mm at level 100 %
        lo = HEIGHT_NORMAL - (HEIGHT_NORMAL - HEIGHT_RANGE[0]) * max(self.difficulty, 0.1)
        hi = HEIGHT_NORMAL + (HEIGHT_RANGE[1] - HEIGHT_NORMAL) * max(self.difficulty, 0.1)
        height = self.rng.uniform(lo, hi) if self.rng.random() < 0.7 else HEIGHT_NORMAL
        if self.rng.random() < STOP_CHANCE:
            self.command = np.array([0.0, 0.0, height])
        else:
            self.command = np.array([self.rng.uniform(-back, top), self.rng.uniform(-TURN_MAX, TURN_MAX), height])
        if "turn" not in DRIVE_PARTS:
            self.command[1] = 0.0
        if "height" not in DRIVE_PARTS:
            self.command[2] = HEIGHT_NORMAL
        hs, poses = self.pose_table
        self.stand_hips = np.array([np.interp(self.command[2], hs, poses[:, i]) for i in range(4)])
        every = STOP_SECONDS if self.command[0] == 0.0 and self.command[1] == 0.0 else NEW_COMMAND_EVERY
        self.next_command = self.steps + int(self.rng.uniform(*every) * POLICY_HZ)
        self.lag = self.heading_lag = 0.0         # a new command starts a new schedule
        if not (was_stop and self.command[0] == 0.0):
            self.hold = 0.0                       # "stand still" again after "stand still": keep counting (as on the Pi)

    def body_speed(self):
        """True forward speed (m/s) and turn rate (rad/s) of the body, for the score only."""
        R = self.d.xmat[self.torso].reshape(3, 3)
        heading = np.array([R[0, 0], R[1, 0]])
        heading /= np.linalg.norm(heading) + 1e-9
        forward = float(heading @ self.d.qvel[0:2])
        turn = float((R @ self.d.qvel[3:6])[2])          # free joint angular velocity is in the body frame
        return forward, turn

    def leg_lengths(self):
        """Left and right leg length: axle height in the body's own frame (m)."""
        R = self.d.xmat[self.torso].reshape(3, 3)
        return [(R.T @ (self.d.xpos[a] - self.d.xpos[self.torso]))[2] for a in self.axles]

    def height(self):
        """True hip-to-axle height (m), for the score only."""
        return float(self.d.xpos[self.torso][2] - np.mean(self.d.xpos[self.axles, 2])) - self.height_offset

    def observe(self):
        d = self.d
        imu = np.concatenate([self.down_now, d.sensordata[self.gyro:self.gyro + 3]])   # IMU down through the Pi filter
        self.imu_history = np.vstack([imu, self.imu_history[:-1]])
        imu = self.imu_history[self.imu_delay]
        gravity = imu[:3] + self.gravity_bias + self.noise("gravity", 3)
        gyro = self.sense_gyro(imu[3:] + self.noise("gyro", 3))
        hip_pos = d.qpos[self.hip_q] + self.hip_offset - self.stand_hips + self.noise("hip_pos", 4)
        hip_vel = d.qvel[self.hip_v] + self.noise("hip_vel", 4)
        wheel_vel = self.sense_wheels() + self.noise("wheel_vel", 2)
        # "Behind schedule": how far the robot is behind (or ahead of) where the speed command would put it,
        # from the wheel speeds the servos report. Without it the brain only sees the speed NOW, so a robot
        # with more friction or a weaker motor than average settles at a lower speed (like a P controller
        # with no I). With it, the brain can see the error add up and push harder (like the I in PI).
        # The same idea for turning: how far the heading is off from where the turn command would put it,
        # from the IMU's turn rate (gyro z). It fixes a slow drift to one side.
        if self.steps == 0:
            self.lag = self.heading_lag = self.hold = 0.0
        else:
            if self.command[0] == 0.0:
                self.hold = float(np.clip(self.hold + WHEEL_R * np.mean(wheel_vel) / POLICY_HZ, -HOLD_MAX, HOLD_MAX))
            self.lag = float(np.clip(LEAK * self.lag + (self.command[0] - WHEEL_R * np.mean(wheel_vel)) / POLICY_HZ, -LAG_MAX, LAG_MAX))
            self.heading_lag = float(np.clip(LEAK * self.heading_lag + (self.command[1] - gyro[2]) / POLICY_HZ,
                                             -HEADING_LAG_MAX, HEADING_LAG_MAX))
        cmd = (self.command - [0, 0, HEIGHT_NORMAL]) / np.array([SPEED_MAX, TURN_MAX, 0.05])   # the joystick, about -1..1
        frame = np.concatenate([gravity, gyro, hip_pos, hip_vel * 0.1, wheel_vel * 0.1, cmd, [self.lag / LAG_MAX, self.heading_lag / HEADING_LAG_MAX, self.hold / HOLD_MAX], self.last_action])
        if HISTORY == 1:
            return frame.astype(np.float32)
        if self.steps == 0:
            self.frames = np.tile(frame, (HISTORY, 1))
        else:
            self.frames = np.vstack([frame, self.frames[:-1]])
        return self.frames.ravel().astype(np.float32)

    def score(self, action):
        g = self.true_gravity()
        tilt = np.arccos(np.clip(-g[2], -1, 1))
        forward, turn = self.body_speed()
        alive = 1.0
        # points for matching the speed and the turn: a bell curve (big points when close) plus a straight
        # line (a steady push towards the command even when far away, so "half speed" is not good enough). Kept small, so a game is always worth more alive than fallen
        # speed: bigger points and a steeper straight-line part than turn, because a test showed the brain
        # drove at half speed with the wheels far from their limit (it was not a motor limit, it was the score)
        go = 2.0 * np.exp(-(forward - self.command[0]) ** 2 / 0.01) - 3.0 * abs(forward - self.command[0])
        steer = 1.0 * np.exp(-(turn - self.command[1]) ** 2 / 0.03) - 0.2 * abs(turn - self.command[1])
        tall = 0.5 * np.exp(-(self.height() - self.command[2]) ** 2 / 0.0001)   # points for the right height (+/-1 cm)
        upright = -UPRIGHT_W * tilt ** 2
        # pitch and roll wobble, from the TRUE gyro (review round 8d, 2026-11-01): the pitch part now charges for the
        # 3.5 Hz rocking (0.2 rad/s rms costs 0.04 per step); clipped at 1 (rad/s)^2 so that a fast recovery
        # from a shove never costs more than a fall. drive_026 and older: WOBBLE_PITCH_W=0.05, no clip.
        wx, wy = self.d.sensordata[self.gyro:self.gyro + 2]
        wobble = -WOBBLE_PITCH_W * min(wy ** 2, WOBBLE_CLIP) - 0.05 * wx ** 2
        smooth = -SMOOTH_W * np.sum((action - self.last_action) ** 2)
        hips = -HIP_W * np.mean(np.asarray(action)[:-2] ** 2)          # hip actions (the last 2 are the wheels)
        hold = -HOLD_W * self.hold ** 2 - HOLD_LIN * max(abs(self.hold) - HOLD_FLAT, 0.0) if self.command[0] == 0.0 else 0.0
        # keep both legs the same length (Mofe saw it stand with one leg higher): 1 cm difference costs 0.3
        # (measured in the body's own frame: on flat ground both wheels touch, so uneven legs show as a body roll)
        legs = self.leg_lengths()
        level = -0.3 * ((legs[0] - legs[1]) / 0.01) ** 2 if not MIRROR else 0.0   # mirrored legs need no penalty
        return alive + go + steer + tall + upright + wobble + smooth + level + hips + hold, tilt

    def reset(self, pitch=None):
        self.steps = 0
        self.stand_hips = self.stand_normal       # every game starts standing at 120 mm (the keyframe)
        super().reset(pitch)
        self.height_offset = float(self.d.xpos[self.torso][2] - np.mean(self.d.xpos[self.axles, 2])) - HEIGHT_NORMAL
        self.new_command()
        return self.observe()

    def step(self, action):
        if self.steps >= self.next_command:
            self.new_command()
        return super().step(action)


# Report card lines: (speed m/s, turn rad/s, height m, seconds). The 60 s lines test "stays up all the time";
# the last lines are the hard corners (several commands at their limits together).
CARD = [(0.0, 0.0, 0.12, 60), (0.1, 0.0, 0.12, 10), (0.15, 0.0, 0.12, 10), (-0.10, 0.0, 0.12, 10),
        (0.0, 1.0, 0.12, 10), (0.1, 0.8, 0.12, 10), (0.0, 0.0, 0.08, 60), (0.0, 0.0, 0.135, 60), (0.1, 0.0, 0.09, 10),
        (0.15, 1.0, 0.12, 10), (-0.10, 1.0, 0.12, 10), (0.15, 0.0, 0.08, 10), (0.0, 1.5, 0.135, 10),
        (0.0, 0.0, 0.12, 60, "hard"), (0.0, 0.0, 0.12, 20, "frozen"),
        (0.0, 0.0, 0.12, 60, "stress"), (0.1, 0.0, 0.12, 20, "stress"), (0.15, 0.0, 0.12, 20, "stress")]
# "hard" line (review 2026-11-01): the robot most like the real one that buzzed: the softest hip flex
# (10 Nm/rad, damping ratio 0.05), the lightest wheel rotor (x0.15 of 0.004), body inertia x0.5.
# "frozen" line: hips held at the stand pose, wheels only (like the real test T2). The ratio d(wy)/d(wheel)
# means something only with the hips still (review round 3), so it is printed on this line only.


# "stress" lines (review round 8, 2026-11-01): the robot that rocked at 3.5 Hz: full battery (12.6 V), the
# longest command delay (40 ms) and a small wheel dead band (0.02). At stress, drive_026 gives 0.19-0.24 rad/s
# (real robot 0.19-0.31) and drive_023 0.08-0.10 (real robot calm, 0.03).
# Pass: mean 2.5-5 Hz gyro band < 0.10 rad/s, every game < 0.20, no 9-13 Hz buzz (buzzing()).
# Ordinary driving lines: mean 2.5-5 Hz band < 0.05.
STRESS_BAND_MEAN, STRESS_BAND_GAME = 0.10, 0.20
DRIVE_BAND_MEAN = 0.05


def stress_plant(env):
    env.wheel_strength = 1.135
    env.act_delay = int(round(0.040 / env.m.opt.timestep))
    env.wheel_deadband = 0.02


def rock_band(G, lo=2.5, hi=5.0):
    """Gyro y band rms (rad/s) between lo and hi Hz: the 3.5 Hz rocking. Same function for the card and the real CSV."""
    g = np.asarray(G, float) - np.mean(G)
    F = np.fft.rfft(g); f = np.fft.rfftfreq(len(g), 1.0 / POLICY_HZ)
    F[(f < lo) | (f > hi)] = 0.0
    return float(np.std(np.fft.irfft(F, len(g))))


def hard_plant(env):
    m = env.m
    k = float(os.environ.get("HARD_K", "10"))   # Nm/rad: the softest hip flex of the range (review round 8c: 8 with k 8-16)
    m.jnt_stiffness[env.hip_bl] = k
    m.dof_damping[env.hip_bl_dof] = 2 * 0.05 * np.sqrt(k * 0.025 / 4)
    m.dof_armature[env.wheel_v] = 0.004 * 0.15
    m.body_inertia[env.torso] = env.nominal["inertia"][env.torso] * 0.5


def count_flips(A, band=FLIP_BAND):
    """Wheel action sign flips: only a swing from above +band to below -band (or back) counts, so small
    dither around zero is not a flip (review 2026-11-01 round 4). Same function for the card and the real CSV."""
    s = np.sign(np.asarray(A, float)) * (np.abs(np.asarray(A, float)) > band)
    s = s[s != 0]
    return int((np.diff(s) != 0).sum())


def buzzing(g_rms, share):
    """A buzz is real only when the 9-13 Hz part is big: band rms (sqrt(share) x gyro rms) > 0.10 rad/s AND
    share > 30 %. A tiny signal is mostly noise, so the share alone is not enough (review round 5)."""
    return bool(np.sqrt(share) * g_rms > 0.10 and share > 0.3)


def rocking(A, V):
    """Wheel use while standing (review round 6): mean |wheel action|, wheel speed rms (m/s), and growth =
    mean |a| over the last minute / the first minute (a third of the game if shorter) (1.0 = steady; the real drive_024 grew until it fell).
    A: mean wheel action per step, V: wheel speed per step in m/s. Same function for the card and the real CSV."""
    A, V = np.abs(np.asarray(A, float)), np.asarray(V, float)
    n = min(60 * POLICY_HZ, len(A) // 3)
    growth = float(np.mean(A[-n:]) / max(np.mean(A[:n]), 1e-6)) if n > 0 else float("nan")
    return float(np.mean(A)), float(np.std(V)), growth


def buzz_numbers(G, A, DY, DW):
    """Gyro rms, gyro 9-13 Hz share, wheel sign flips per second, d(gyro)/d(wheel speed): the same four
    numbers are computed from the real robot's CSV, so sim and robot are compared the same way."""
    g = np.asarray(G) - np.mean(G)
    P = np.abs(np.fft.rfft(g * np.hanning(len(g)))) ** 2; f = np.fft.rfftfreq(len(g), 1.0 / POLICY_HZ)
    share = P[(f >= 9) & (f <= 13)].sum() / max(P[f > 0.5].sum(), 1e-12)
    flips = count_flips(A) / (len(A) / POLICY_HZ)
    ratio = np.polyfit(DW, DY, 1)[0] if len(DW) > 10 else float("nan")
    return float(np.std(G)), float(share), float(flips), float(ratio)


def card_line(job):
    """Play CARD_GAMES games of one report card line; return the printed line."""
    path, line, seed = job
    v, w, h, seconds = line[:4]
    hard = len(line) > 4 and "hard" in line[4]
    stress = len(line) > 4 and "stress" in line[4]
    freeze = len(line) > 4 and "frozen" in line[4]
    from stable_baselines3 import PPO
    model = PPO.load(path, device="cpu")
    env = DriveEnv(seed=seed, shoves=False)
    env.max_steps = seconds * POLICY_HZ
    hs, poses = env.pose_table
    up, speeds, turns, heights, uneven = 0, [], [], [], []
    creep, at_limit, wheel_peak, hip_mag = [], [], [], []   # standing quality (2026-10-31: the real robot crept forward and fell)
    buzz = []                                               # gyro rms, 9-13 Hz share, flips, ratio per game (2026-11-01)
    rock = []                                               # wheel |a|, wheel speed rms, growth per game (review round 6)
    calm = []                                               # body pitch rms (deg), wheel travel rms (rad), last 5 s (review round 8d)
    for _ in range(CARD_GAMES):
        obs = env.reset(pitch=env.rng.uniform(-0.035, 0.035))
        if hard:
            hard_plant(env)
        if stress:
            stress_plant(env)
        env.command = np.array([v, w, h]); env.next_command = 10 ** 9
        env.stand_hips = np.array([np.interp(h, hs, poses[:, i]) for i in range(4)])
        obs = env.observe()
        x0, far, peak, lim, n = float(np.mean(env.d.qpos[env.wheel_q])), 0.0, 0.0, 0, 0
        G, A, W, PT = [], [], [], []
        while True:
            action = model.predict(obs, deterministic=True)[0]
            if freeze:
                action = np.concatenate([np.zeros(len(action) - 2), action[-2:]])   # hips stay at the stand pose
            obs, r, fell, timeup, info = env.step(action)
            far = max(far, abs(float(np.mean(env.d.qpos[env.wheel_q])) - x0) * WHEEL_R)   # m from the start
            peak = max(peak, float(np.max(np.abs(env.d.qvel[env.wheel_v]))))
            lim += int(np.any(np.abs(action[:-2]) > 0.9)); n += 1; hip_mag.append(float(np.mean(np.abs(action[:-2]))))
            G.append(float(env.d.sensordata[env.gyro + 1])); A.append(float(np.mean(action[-2:])))
            PT.append(float(np.degrees(np.arcsin(np.clip(env.true_gravity()[0], -1, 1)))))
            W.append(float(np.mean(env.d.qpos[env.wheel_q])) * POLICY_HZ)
            if env.steps > 4 * POLICY_HZ:
                f, t = env.body_speed(); speeds.append(f); turns.append(t); heights.append(env.height())
                legs = env.leg_lengths(); uneven.append(abs(legs[0] - legs[1]))
            if fell or timeup:
                up += not fell
                creep.append(far); wheel_peak.append(peak); at_limit.append(lim / max(n, 1))
                if len(G) > 100:
                    Wv = np.diff(W)                       # wheel speed by position difference, as on the Pi
                    buzz.append(buzz_numbers(G[51:], A[51:], np.diff(G[51:]), np.diff(Wv[50:])) + (rock_band(G[51:]),))
                    rock.append(rocking(A[50:], Wv[50:] * WHEEL_R))
                    n5 = min(5 * POLICY_HZ, len(PT))
                    calm.append((float(np.std(PT[-n5:])), float(np.std(np.array(W[-n5:]) / POLICY_HZ))))
                break
    quality = (f", mean |hip| {np.mean(hip_mag):.2f}, hips at limit {100 * np.mean(at_limit):.0f} %, wheel peak {np.max(wheel_peak):.1f} rad/s"
               + (f", creep max {np.max(creep):.2f} m" if v == 0.0 and w == 0.0 else ""))
    if buzz:
        b = np.array(buzz)
        g_rms, share, flips, ratio = np.nanmean(b[:, 0]), np.nanmean(b[:, 1]), np.nanmean(b[:, 2]), np.nanmedian(b[:, 3])
        band = np.nanmean(b[:, 4])
        quality += (f", gyro rms {g_rms:.2f}, 9-13 Hz {100 * share:.0f} %, 2.5-5 Hz band {band:.3f} (max {np.nanmax(b[:, 4]):.3f}), flips {flips:.1f}/s"
                    + (f", d(wy)/d(wheel) {ratio:+.2f} (real -0.21..-0.25)" if freeze else ""))
        if calm:
            c = np.array(calm)
            quality += f", pitch rms {np.mean(c[:, 0]):.2f} deg, wheel travel rms {np.mean(c[:, 1]):.2f} rad (last 5 s)"
            if v == 0.0 and w == 0.0 and not freeze:   # slow hunting check (review round 8d)
                quality += "  CALM MARK " + ("PASS" if np.mean(c[:, 0]) < 1.0 and np.mean(c[:, 1]) < 0.3 else "FAIL")
        if stress:   # review round 8: the 3.5 Hz rocking and the 11 Hz buzz, every game
            ok_s = (up == CARD_GAMES and band < STRESS_BAND_MEAN and np.nanmax(b[:, 4]) < STRESS_BAND_GAME
                    and not any(buzzing(x[0], x[1]) for x in b))
            quality += "  STRESS MARKS " + ("PASS" if ok_s else "FAIL")
        elif v != 0.0 and not hard and not freeze:   # ordinary driving lines: the 3.5 Hz rocking
            quality += "  3.5 Hz MARK " + ("PASS" if band < DRIVE_BAND_MEAN and not buzzing(g_rms, share) else "FAIL")
        elif v == 0.0 and w == 0.0 and not freeze:   # standing pass marks (review 2026-11-01, rounds 3-6)
            k = np.array(rock)
            quality += f", wheel |a| {np.mean(k[:, 0]):.2f}, wheel speed rms {np.mean(k[:, 1]):.3f} m/s, growth {np.max(k[:, 2]):.1f}x"
            ok = (up == CARD_GAMES and g_rms < 0.3 and not buzzing(g_rms, share) and flips < 5 and np.mean(hip_mag) < 0.5
                  and np.max(creep) < 0.5 and np.mean(k[:, 0]) < 0.2 and np.mean(k[:, 1]) < 0.05 and np.max(k[:, 2]) <= 1.2)
            quality += "  MARKS " + ("PASS" if ok else "FAIL")
    return (("  [HARD PLANT]" if hard else "") + ("  [STRESS]" if stress else "") + ("  [FROZEN HIPS]" if freeze else "") + f"  command {v:+.2f} m/s, turn {w:+.1f} rad/s, height {h * 1000:.0f} mm, {seconds:2d} s: "
            f"stayed up {up}/{CARD_GAMES}, drove {np.mean(speeds):+.2f} m/s, turned {np.mean(turns):+.2f} rad/s, "
            f"height {np.mean(heights) * 1000:.0f} mm, legs differ {np.mean(uneven) * 1000:.1f} mm" + quality)


def test(path):
    """Report card for driving: how well it follows commands, at full realism, no random shoves.
    Every game starts nearly upright (+/-2 deg): this card tests driving. The balance card tests big leans.
    The lines run in parallel, one per CPU core. Pass mark: every line stays up CARD_GAMES/CARD_GAMES."""
    from multiprocessing import Pool
    print("DRIVING REPORT CARD for", path, flush=True)
    import balance_env as be
    f = DriveEnv(seed=0, shoves=False, realism=False).gyro_lp
    if f is not None and hasattr(f, "response"):   # README rule 13: the observation filter is part of the plant
        print("gyro filter (Pi values): " + ", ".join(f"{hz} Hz {g:.2f}@{p:.0f} deg" for hz, (g, p) in
                                                      [(hz, f.response(hz, POLICY_HZ)) for hz in (1, 2, 3.5, 5, 10.5, 15)]), flush=True)
    with Pool(min(len(CARD), os.cpu_count() or 4)) as pool:
        for line in pool.imap(card_line, [(path, c, 7 + k) for k, c in enumerate(CARD)]):
            print(line, flush=True)


if __name__ == "__main__":
    if "--test" in sys.argv:
        run = os.environ.get("RUN", "drive_v1")
        test(os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", run, "latest.zip"))
    else:
        env = DriveEnv(seed=0, shoves=False)
        t = []
        for g in range(10):
            obs = env.reset()
            while True:
                obs, r, fell, up, info = env.step(np.zeros(env.n_act))
                if fell or up:
                    t.append(env.steps / POLICY_HZ); break
        print(f"observation: {env.n_obs} numbers; do nothing lasts {np.mean(t):.2f} s on average")
