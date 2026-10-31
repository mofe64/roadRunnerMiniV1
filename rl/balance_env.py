"""Road Runner Mini: the balance "game" (an RL environment).

The robot plays a short game, again and again:
  1. reset()  puts the robot on its wheels, slightly tilted, at a random small angle.
  2. step(action) applies the policy's action for 1/50 s and returns:
       observation  what the robot senses (only things the real robot can measure)
       reward       a score for that step (high when upright and calm)
       terminated   True if it fell over (the game ends)
       truncated    True when the time limit is reached (the game ends, no fall)
  3. Training plays millions of these steps and makes the policy score more.

v3 adds realism (sim-to-real), the same recipe as Open Duck Mini: a measured STS3215 servo
model and gear backlash (in ../sim), command and IMU delays, sensor noise, servo speed limits,
and a slightly different robot every game (domain randomisation). BalanceEnv(realism=False) turns it off.

Run this file to test the game (no window, 20 games):
  .venv/bin/python balance_env.py              "do nothing" moves
  .venv/bin/python balance_env.py --random     random moves
Watch it in the MuJoCo window (5 games, 4x slow motion; on a Mac use mjpython):
  .venv/bin/mjpython balance_env.py --watch
  .venv/bin/mjpython balance_env.py --watch --random
"""
import os
import numpy as np
import mujoco

MODEL = os.environ.get("MODEL_XML") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "roadrunner_mini.xml")
SIM_DT = float(os.environ.get("SIM_DT", "0.002"))   # s: physics step. 2 ms gives the same report card as 1 ms and trains 1.75x faster (2026-10-30)

POLICY_HZ = 50            # the policy decides 50 times a second (like the real Pi loop)
EPISODE_SECONDS = 10      # one game lasts at most 10 s
SLOW_MOTION = 4          # the window plays 4x slower than real time
FALL_TILT = np.radians(45)  # tilted more than this = fallen
HIP_RANGE = 0.35          # rad: how far the policy may move each hip from the stand pose
WHEEL_TORQUE = 1.85       # Nm: full wheel command (ST3215-HS at 11.1 V)
WHEEL_FREE_SPEED = 10.3   # rad/s: wheel speed at full power with no load (11.1 V; bench test 5.76 at 6.2 V, from position)
WHEEL_R = float(os.environ.get("WHEEL_R", "0.035"))   # m: wheel radius (set it to match MODEL_XML)
# v2 training: make the game harder so the brain learns to cope
START_LEAN = np.radians(12)  # start leaning up to 12 degrees at level 100 % (the model's physical limit, see README)
START_LEAN_EASY = np.radians(5)  # ... and up to 5 degrees at level 0 %
SHOVE_EVERY = 2.0         # s: on average, one random shove every 2 s
SHOVE_MAX = 0.45          # N s: strongest random shove at level 100 % (the model's physical limit, see README)
SHOVE_MAX_EASY = 0.2      # N s: strongest random shove at level 0 %
# v3 realism (sim-to-real), following Open Duck Mini:
MAX_DELAY = int(os.environ.get("MAX_DELAY", "1"))   # IMU readings arrive 0-1 steps late (0-20 ms; not measured yet); fixed for one game
# Command delay: from the decision until the servo starts to move. The bench test (wheel_bench.py, 2026-10-30)
# measured 10-25 ms dead time in PWM mode, so each game gets a delay of 10-30 ms, in physics steps.
# Before this it was 0-40 ms in whole 20 ms steps (as Open Duck), and 40 ms caused most falls (diag_017.py).
ACT_DELAY_MS = tuple(float(x) for x in os.environ.get("ACT_DELAY_MS", "10,30").split(","))
# Extra hip delay in decisions. 0: the fitted servo delay (24 ms, sim/build_model_v03.py) is covered by
# ACT_DELAY_MS. Kept as a knob for tests (with the old 0x55 = 254 acceleration limit it would have been 1).
HIP_DELAY = int(os.environ.get("HIP_DELAY", "0"))   # decisions
HIP_MAX_SPEED = 4.7       # rad/s: STS3215 top speed at 7.4 V (0.222 s per 60 deg)
# Sensor errors (uniform +/-), like a real robot (independent review 2026-10-30). The old values came from
# Open Duck and were much larger: gravity 0.05, gyro 0.2, hip_pos 0.03, hip_vel 2.5.
NOISE = dict(gravity=0.005,     # IMU "down" jitter: 0.3 deg per reading
             gyro=0.02,         # rad/s
             hip_pos=0.003,     # rad: about 2 encoder counts (the servo zero error is hip_offset, per game)
             hip_vel=0.3,       # rad/s
             wheel_vel=0.5)     # rad/s
GRAVITY_BIAS = 0.0175           # IMU mounted slightly crooked: up to 1 deg, the same for the whole game
NOISE = {k: v * float(os.environ.get("NOISE_SCALE", "1.0")) for k, v in NOISE.items()}

# Experiments: ABLATE=hips,wheelgear,backlash,hipspeed turns parts of the v3 servo model OFF, and
# ABLATE=delay,noise,random turns parts of the realism OFF, to find which one makes balancing hard.
# Leave it empty for normal training.
ABLATE = [x for x in os.environ.get("ABLATE", "").split(",") if x]
# Short memory: the brain sees the last HISTORY observations, not only the newest one. With memory it
# can average out sensor noise and see how things are changing, which helps with delays.
HISTORY = int(os.environ.get("HISTORY", "3"))

# Mirrored legs: the brain gives ONE front hip and ONE rear hip command, and both legs get the same,
# so the legs stay the same length (independent review 2026-10-30). MIRROR=0 gives each hip its own.
MIRROR = os.environ.get("MIRROR", "1") == "1"
# Battery: the wheel servos run on the 3S battery, 9.4 V (almost empty) to 12.6 V (full) = 0.85-1.135 of 11.1 V.
# Voltage scales the motor push, and with it the stall torque and the no-load speed together.
BATTERY = (0.85, 1.135)

HIPS = ["hip_f_L", "hip_r_L", "hip_f_R", "hip_r_R"]
WHEELS = ["wheel_L", "wheel_R"]


class BalanceEnv:
    def __init__(self, seed=None, shoves=True, realism=True):
        self.m = mujoco.MjModel.from_xml_path(MODEL)
        self.m.opt.timestep = SIM_DT
        self.d = mujoco.MjData(self.m)
        self.rng = np.random.default_rng(seed)
        self.shoves = shoves                    # random shoves on/off (the exam turns them off and shoves on purpose)
        self.realism = realism                  # delays, noise, speed limits and randomised robot (v3)
        # The level: 0 = the perfect sim robot, small leans and soft shoves; 1 = full realism, leans to
        # 20 degrees, shoves to 1.2 N s. Training raises it step by step (a "curriculum"): at full level
        # from the start, the brain never found balance at all.
        self.difficulty = 1.0
        self.start_lean, self.shove_max = START_LEAN, SHOVE_MAX   # the driving game uses smaller ones
        self.substeps = round(1.0 / (POLICY_HZ * self.m.opt.timestep))   # 500 Hz physics / 50 Hz policy = 10
        self.max_steps = EPISODE_SECONDS * POLICY_HZ

        self.act_id = {n: self.m.actuator(n).id for n in HIPS + WHEELS + ["head_yaw", "head_pitch"]}
        self.hip_q = [self.m.jnt_qposadr[self.m.joint(n).id] for n in HIPS]
        self.hip_v = [self.m.jnt_dofadr[self.m.joint(n).id] for n in HIPS]
        self.wheel_v = [self.m.jnt_dofadr[self.m.joint(n).id] for n in WHEELS]
        self.wheel_q = [self.m.jnt_qposadr[self.m.joint(n).id] for n in WHEELS]
        self.torso = self.m.body("torso").id
        self.gyro = self.m.sensor("gyro").adr[0]

        self.key = self.m.key("stand").id
        self.stand_hips = self.m.key_qpos[self.key][self.hip_q].copy()

        self.n_act = 2 + 2 if MIRROR else 4 + 2   # hips (front, rear; or all 4) + 2 wheels
        self.n_frame = 3 + 3 + 4 + 4 + 2 + 1 + self.n_act   # numbers per observation
        self.n_obs = self.n_frame * HISTORY

        self.ablate(ABLATE)
        # the "nominal" robot, so each game can randomise a copy of it and then reset
        m = self.m
        self.floor = m.geom("floor").id
        # tyre geoms: MuJoCo uses the larger friction of the two touching geoms, so grip is randomised on the tyre
        self.tyres = [g for g in range(m.ngeom) if m.geom_bodyid[g] in [m.body(n).id for n in WHEELS]]
        self.hip_act = [self.act_id[n] for n in HIPS]
        self.wheel_act = [self.act_id[n] for n in WHEELS]
        self.hip_dof = list(self.hip_v)
        self.wheel_bl = [m.joint(n + "_backlash").id for n in WHEELS]   # gear slop joints behind the wheels
        self.nominal = dict(mass=m.body_mass.copy(), ipos=m.body_ipos.copy(), friction=m.geom_friction.copy(),
                            gain=m.actuator_gainprm.copy(), bias=m.actuator_biasprm.copy(),
                            frictionloss=m.dof_frictionloss.copy(), armature=m.dof_armature.copy(),
                            jnt_range=m.jnt_range.copy(), dynprm=m.actuator_dynprm.copy())
        self.filtered = [a for a in range(m.nu) if m.actuator_dyntype[a] != mujoco.mjtDyn.mjDYN_NONE]

    def ablate(self, parts):
        """Turn parts of the v3 servo model off (for experiments only)."""
        m = self.m
        hip_j = [m.joint(n).id for n in HIPS]
        hip_d = [m.jnt_dofadr[j] for j in hip_j]
        hip_a = [m.actuator(n).id for n in HIPS]
        wheel_d = [m.jnt_dofadr[m.joint(n).id] for n in WHEELS]
        if "hips" in parts:        # the v2 hip servo: stiffer, no friction, no rotor inertia
            m.actuator_gainprm[hip_a, 0] = 30; m.actuator_biasprm[hip_a, 1] = -30; m.actuator_biasprm[hip_a, 2] = -0.6
            m.dof_damping[hip_d] = 0; m.dof_frictionloss[hip_d] = 0; m.dof_armature[hip_d] = 0
        if "wheelgear" in parts:   # no wheel gearbox friction, no motor rotor inertia
            m.dof_armature[wheel_d] = 0; m.dof_frictionloss[wheel_d] = 0
        if "backlash" in parts:    # no gear slop
            for n in HIPS + WHEELS:
                m.jnt_range[m.joint(n + "_backlash").id] = [-1e-6, 1e-6]
        if parts:
            print("turned OFF for this experiment:", ", ".join(parts))

    # ---------- the three parts of the game ----------

    def true_gravity(self):
        R = self.d.xmat[self.torso].reshape(3, 3)
        return R.T @ np.array([0.0, 0.0, -1.0])           # which way is down, seen from the body

    def noise(self, name, n):
        if not self.realism or "noise" in ABLATE:
            return 0.0
        return self.rng.uniform(-1, 1, n) * NOISE[name] * self.difficulty

    def observe(self):
        """What the robot senses. Every value here has a real sensor on the robot.
        With realism on, readings have small errors and the IMU can be up to 60 ms late."""
        d = self.d
        imu = np.concatenate([self.true_gravity(), d.sensordata[self.gyro:self.gyro + 3]])
        self.imu_history = np.vstack([imu, self.imu_history[:-1]])        # newest first
        imu = self.imu_history[self.imu_delay]
        gravity = imu[:3] + self.gravity_bias + self.noise("gravity", 3)   # IMU: which way is down
        gyro = imu[3:] + self.noise("gyro", 3)                         # IMU: how fast the body is turning
        hip_pos = d.qpos[self.hip_q] + self.hip_offset - self.stand_hips + self.noise("hip_pos", 4)   # servo position feedback
        hip_vel = d.qvel[self.hip_v] + self.noise("hip_vel", 4)       # servo speed feedback
        wheel_vel = d.qvel[self.wheel_v] + self.noise("wheel_vel", 2) # wheel servo speed feedback
        odo = [np.clip(self.distance(), -1, 1)]           # how far it has rolled from the start (wheel turns, like a car odometer)
        frame = np.concatenate([gravity, gyro, hip_pos, hip_vel * 0.1, wheel_vel * 0.1, odo, self.last_action])
        if HISTORY == 1:
            return frame.astype(np.float32)
        if self.steps == 0:                                 # new game: fill the memory with the first frame
            self.frames = np.tile(frame, (HISTORY, 1))
        else:
            self.frames = np.vstack([frame, self.frames[:-1]])   # newest first
        return self.frames.ravel().astype(np.float32)

    def distance(self):
        """Metres rolled since the start, from the wheel angles (the real servos report their position)."""
        return WHEEL_R * (np.mean(self.d.qpos[self.wheel_q]) - self.wheel_start)

    def apply(self, action):
        """What the robot does. Actions are numbers from -1 to +1.
        With realism on, the command can arrive up to 60 ms late and the hip servos have a top speed."""
        self.action_history = np.vstack([np.clip(action, -1.0, 1.0), self.action_history[:-1]])
        # the delay in physics steps: whole decisions late, plus some physics steps of the one before
        late, self.old_substeps = divmod(self.act_delay, self.substeps)
        self.old_ctrl = self.d.ctrl.copy()              # the command now at the servos (one decision older)
        a = self.action_history[late]
        a_hip = self.action_history[late + (HIP_DELAY if self.realism and "delay" not in ABLATE else 0)]
        target = self.stand_hips + HIP_RANGE * a_hip[:4] - self.hip_offset      # the servo's own zero is slightly off
        if self.realism and "hipspeed" not in ABLATE:   # the servo cannot move faster than its top speed
            step = HIP_MAX_SPEED / POLICY_HZ
            target = np.clip(target, self.hip_target - step, self.hip_target + step)
        self.hip_target = target
        for i, n in enumerate(HIPS):
            self.d.ctrl[self.act_id[n]] = target[i]                          # hip target angle
        for i, n in enumerate(WHEELS):
            # Wheel servo in PWM mode, as measured on the bench (wheel_bench.py, 2026-10-30): the action IS
            # the PWM power, -1..+1. Nothing is added by the Pi. The push fades as the wheel speeds up
            # (back-EMF, in the model), and the brain sees the wheel speed, so it learns that itself.
            self.d.ctrl[self.act_id[n]] = self.wheel_strength * WHEEL_TORQUE * a[4 + i]
        self.d.ctrl[self.act_id["head_yaw"]] = 0.0     # head looks straight ahead for now
        self.d.ctrl[self.act_id["head_pitch"]] = 0.0

    def score(self, action):
        """The reward: points for each step. The policy learns to collect more points."""
        g = self.true_gravity()
        tilt = np.arccos(np.clip(-g[2], -1, 1))         # 0 when perfectly upright (true tilt, not the noisy reading)
        alive = 1.0                                       # +1 for every step it has not fallen
        upright = -2.0 * tilt ** 2                        # lose points for leaning
        still = -0.02 * np.sum(self.d.qvel[self.wheel_v] ** 2) ** 0.5   # lose a little for rolling
        home = -1.0 * min(abs(self.distance()), 1.0)      # lose points for every metre away from the start
        spin = -0.05 * np.sum(self.d.sensordata[self.gyro:self.gyro + 3] ** 2)  # lose points for wobbling
        smooth = -0.05 * np.sum((action - self.last_action) ** 2)          # lose points for jerky actions
        return alive + upright + still + home + spin + smooth, tilt

    # ---------- the game loop ----------

    def randomise(self):
        """Every game gets a slightly different robot, because the real one differs from the sim
        in ways we cannot measure. The brain must work for all of them (domain randomisation)."""
        m, nom, rng, s = self.m, self.nominal, self.rng, self.difficulty
        if "random" in ABLATE:
            s = 0.0             # every game the same robot
        r = lambda lo, hi, n=None: rng.uniform(lo, hi, n) * s     # a random spread, scaled by the difficulty
        f = lambda lo, hi, n=None: np.exp(r(np.log(lo), np.log(hi), n))   # a random factor around 1, scaled too
        m.body_mass[:] = nom["mass"] * (1 + r(-0.1, 0.1, m.nbody))               # every part +/-10 %
        m.body_mass[self.torso] += r(-0.1, 0.1)                                    # body +/-100 g (cables, glue, screws)
        m.body_ipos[:] = nom["ipos"]
        m.body_ipos[self.torso] += r(-0.01, 0.01, 3)                              # centre of weight +/-10 mm
        m.geom_friction[:] = nom["friction"]
        m.geom_friction[self.tyres, 0] = 0.75 + r(-0.25, 0.25)                    # tyre-floor grip 0.5-1.0 (floor is 1.0 -> set to 0.5 in the model)
        k = nom["gain"][self.hip_act, 0] * (1 + r(-0.1, 0.1, 4))                  # servo stiffness +/-10 %
        m.actuator_gainprm[self.hip_act, 0] = k
        m.actuator_biasprm[self.hip_act, 1] = -k
        m.dof_frictionloss[self.hip_dof] = nom["frictionloss"][self.hip_dof] * (1 + r(-0.1, 0.1, 4))
        m.actuator_dynprm[self.hip_act, 0] = nom["dynprm"][self.hip_act, 0] * (1 + r(-0.1, 0.1, 4))   # servo filter +/-10 % (measured)
        self.hip_offset = r(-0.03, 0.03, 4)                                        # servo zero offsets +/-1.7 deg
        self.wheel_strength = 1.0 + r(BATTERY[0] - 1.0, BATTERY[1] - 1.0)          # battery charge 0.85-1.135
        self.gravity_bias = 0.0 if "noise" in ABLATE else r(-GRAVITY_BIAS, GRAVITY_BIAS, 3)   # IMU mounted slightly crooked
        # Wheel servo (ST3215-HS) gearbox: not measured, so the ranges are WIDE. A brain that copes
        # with all of them should cope with the real one too, without a bench test.
        m.dof_frictionloss[self.wheel_v] = nom["frictionloss"][self.wheel_v] * f(0.3, 3.0, 2)
        m.dof_armature[self.wheel_v] = nom["armature"][self.wheel_v] * f(0.7, 1.4, 2)    # measured (spin-up test), so a narrower range
        m.jnt_range[self.wheel_bl] = nom["jnt_range"][self.wheel_bl] * f(0.5, 2.0)   # slop +/-0.25..1 deg
        # wheel motor back-EMF slope: the bench test gave 5.76 rad/s at 6.2 V (from position), +/-5 %
        m.actuator_biasprm[self.wheel_act, 2] = nom["bias"][self.wheel_act, 2] * f(0.95, 1.05, 2)
        # how late commands and IMU readings arrive this game (the real delay is about the same every step)
        top = 0 if "delay" in ABLATE else int(round(MAX_DELAY * self.difficulty))
        self.imu_delay = rng.integers(0, top + 1)
        ms = 0.0 if "delay" in ABLATE else rng.uniform(*ACT_DELAY_MS) * self.difficulty
        self.act_delay = int(round(ms / 1000 / m.opt.timestep))   # in physics steps

    def reset(self, pitch=None):
        m, d = self.m, self.d
        if self.realism:
            self.randomise()
        else:
            self.hip_offset, self.wheel_strength, self.gravity_bias = np.zeros(4), 1.0, np.zeros(3)
            self.act_delay = self.imu_delay = 0
        mujoco.mj_resetDataKeyframe(m, d, self.key)
        # start a little different every game, so the policy learns to handle many situations
        if pitch is None:
            lean = START_LEAN_EASY + (self.start_lean - START_LEAN_EASY) * self.difficulty
            pitch = self.rng.uniform(-lean, lean)
        d.qpos[3:7] = [np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0]
        d.qvel[:6] = self.rng.normal(0.0, 0.05, 6)          # a small random nudge
        # servo command filters (sim/build_model_v03.py STS_TIMECONST) start at the joint's present angle, so nothing jumps
        for a in self.filtered:
            d.act[m.actuator_actadr[a]] = d.qpos[m.jnt_qposadr[m.actuator_trnid[a, 0]]]
            d.ctrl[a] = d.act[m.actuator_actadr[a]]
        mujoco.mj_forward(m, d)
        self.wheel_start = np.mean(d.qpos[self.wheel_q])
        self.steps = 0
        self.last_action = np.zeros(self.n_act)
        self.action_history = np.zeros((int(ACT_DELAY_MS[1] / 1000 * POLICY_HZ) + 2 + HIP_DELAY, 6))
        self.imu_history = np.tile(np.concatenate([self.true_gravity(), d.sensordata[self.gyro:self.gyro + 3]]), (MAX_DELAY + 1, 1))
        self.hip_target = d.qpos[self.hip_q].copy()
        return self.observe()

    def full_action(self, action):
        """The 6 servo commands from the brain's action (mirrored: front, rear, wheel L, wheel R)."""
        if MIRROR:
            return np.concatenate([action[[0, 1, 0, 1]], action[2:4]])
        return action

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self.apply(self.full_action(action))
        shove = self.shoves and self.rng.random() < 1.0 / (SHOVE_EVERY * POLICY_HZ)
        if shove:   # a random shove, mostly forward/back, a little sideways; lasts 1/50 s
            strongest = SHOVE_MAX_EASY + (self.shove_max - SHOVE_MAX_EASY) * self.difficulty
            impulse = self.rng.uniform(0, strongest) * np.array([self.rng.choice([-1, 1]), self.rng.normal(0, 0.3), 0])
            self.d.xfrc_applied[self.torso, :3] = impulse * POLICY_HZ
        new_ctrl = self.d.ctrl.copy()
        for i in range(self.substeps):                    # run the physics steps for one decision (1/50 s)
            # the servos still get the older command for the first few steps (the command delay)
            self.d.ctrl[:] = self.old_ctrl if i < self.old_substeps else new_ctrl
            mujoco.mj_step(self.m, self.d)
        self.d.xfrc_applied[self.torso, :] = 0
        reward, tilt = self.score(action)
        self.last_action = np.clip(action, -1, 1)
        self.steps += 1
        terminated = bool(tilt > FALL_TILT)                # it fell: game over
        truncated = self.steps >= self.max_steps           # time is up
        if terminated:
            reward -= 10.0                                 # a fall costs extra points
        return self.observe(), float(reward), terminated, truncated, {"tilt_deg": float(np.degrees(tilt))}


def play(env, policy, games=1, viewer=None, slow=SLOW_MOTION):
    """Play some games with a policy (a function: observation -> action). Return (seconds, score) per game."""
    import time
    results = []
    for _ in range(games):
        obs = env.reset()
        total, done = 0.0, False
        while not done:
            t0 = time.time()
            obs, r, terminated, truncated, info = env.step(policy(obs))
            total += r
            done = terminated or truncated
            if viewer is not None:
                if not viewer.is_running():
                    return results
                viewer.sync()
                time.sleep(max(0.0, slow / POLICY_HZ - (time.time() - t0)))   # play slowed down so you can see it
        if viewer is not None:
            time.sleep(1.0)   # short pause so you can see the fall
        results.append((env.steps / POLICY_HZ, total))
    return results


if __name__ == "__main__":
    import sys
    watch = "--watch" in sys.argv      # open the MuJoCo window (on a Mac, run with .venv/bin/mjpython)
    rand = "--random" in sys.argv      # random moves instead of "do nothing"
    games = 5 if watch else 20
    env = BalanceEnv(seed=0, shoves=False)
    print(f"observation: {env.n_obs} numbers, action: {env.n_act} numbers, {env.substeps} physics steps per action")
    if rand:
        name, policy = "random moves", lambda obs: env.rng.uniform(-1, 1, env.n_act)
    else:
        name, policy = "do nothing", lambda obs: np.zeros(env.n_act)
    if watch:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(env.m, env.d) as v:
            v.cam.lookat[:] = [0, 0, 0.17]; v.cam.distance = 0.9; v.cam.azimuth = 135; v.cam.elevation = -15
            results = play(env, policy, games, v)
    else:
        results = play(env, policy, games)
    for i, (sec, score) in enumerate(results, 1):
        print(f"{name}, game {i}: lasted {sec:.2f} s, score {score:.1f}")
    if results:
        print(f"average: {np.mean([r[0] for r in results]):.2f} s (the full game is {EPISODE_SECONDS} s)")
