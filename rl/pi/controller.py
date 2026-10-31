"""The brain's side of the robot: sensors in, servo commands out. It builds the SAME observation as the
simulator (drive_env.py, DriveEnv.observe) and turns the brain's action into servo targets the same way
(balance_env.py, BalanceEnv.apply). It knows nothing about the hardware, so the simulator test
(sim_test.py) and the real robot (run_brain.py) both use this file.

Units are the simulator's: hip angles in rad in the simulator's joint frame (0 = upper link straight
down, + = swings back), wheel speeds in rad/s (+ = rolling forward), body frame X forward, Y left, Z up.
"""
import numpy as np


class Controller:
    def __init__(self, brain):
        self.brain, c = brain, brain.c
        self.c = c
        self.hz = float(c["policy_hz"])
        self.history = int(c["history"])
        self.n_act = int(c["n_act"])
        self.mirror = bool(c["mirror"])
        self.command = np.array([0.0, 0.0, float(c["height_normal"])])
        self.hold_max = float(c["hold_max"]) if "hold_max" in c else None   # brains before drive_021 have no hold odometer
        self.hold = 0.0
        self.stand = brain.stand_hips(self.command[2])

    def set_command(self, speed, turn, height):
        """A new joystick command, clipped to what the brain was trained on. A new command starts a new
        "schedule", as in the simulator, so the two lag signals restart at 0."""
        c = self.c
        new = np.array([np.clip(speed, -float(c["reverse_max"]), float(c["speed_max"])),
                        np.clip(turn, -float(c["turn_max"]), float(c["turn_max"])),
                        np.clip(height, *[float(x) for x in c["height_range"]])])
        if not np.allclose(new, self.command):
            self.command = new
            self.stand = self.brain.stand_hips(new[2])
            self.lag = self.heading_lag = self.hold = 0.0

    def reset(self, hip_q):
        self.steps = 0
        self.lag = self.heading_lag = self.hold = 0.0
        self.last_action = np.zeros(self.n_act)
        self.hip_target = np.asarray(hip_q, dtype=float).copy()

    def observe(self, gravity, gyro, hip_q, hip_v, wheel_v):
        c = self.c
        gyro, wheel_v = np.asarray(gyro, float), np.asarray(wheel_v, float)
        if self.steps > 0:
            self.lag = float(np.clip(float(c["leak"]) * self.lag + (self.command[0] - float(c["wheel_r"]) * np.mean(wheel_v)) / self.hz,
                                     -float(c["lag_max"]), float(c["lag_max"])))
            self.heading_lag = float(np.clip(float(c["leak"]) * self.heading_lag + (self.command[1] - gyro[2]) / self.hz,
                                             -float(c["heading_lag_max"]), float(c["heading_lag_max"])))
            if self.hold_max is not None and self.command[0] == 0.0:
                self.hold = float(np.clip(self.hold + float(c["wheel_r"]) * np.mean(wheel_v) / self.hz, -self.hold_max, self.hold_max))
        cmd = (self.command - [0, 0, float(c["height_normal"])]) / np.array([float(c["speed_max"]), float(c["turn_max"]), 0.05])
        frame = np.concatenate([gravity, gyro, np.asarray(hip_q, float) - self.stand, np.asarray(hip_v, float) * 0.1,
                                wheel_v * 0.1, cmd, [self.lag / float(c["lag_max"]), self.heading_lag / float(c["heading_lag_max"])]
                                + ([self.hold / self.hold_max] if self.hold_max is not None else []),
                                self.last_action])
        if self.history == 1:
            return frame.astype(np.float32)
        if self.steps == 0:
            self.frames = np.tile(frame, (self.history, 1))
        else:
            self.frames = np.vstack([frame, self.frames[:-1]])
        return self.frames.ravel().astype(np.float32)

    def step(self, gravity, gyro, hip_q, hip_v, wheel_v):
        """One decision. Returns (hip targets rad [f_L, r_L, f_R, r_R], wheel power -1..1 [L, R], action)."""
        obs = self.observe(gravity, gyro, hip_q, hip_v, wheel_v)
        action = self.brain.act(obs)
        full = np.concatenate([action[[0, 1, 0, 1]], action[2:4]]) if self.mirror else action
        target = self.stand + float(self.c["hip_range"]) * full[:4]
        step = float(self.c["hip_max_speed"]) / self.hz          # the same speed limit as the simulator's servo
        target = np.clip(target, self.hip_target - step, self.hip_target + step)
        self.hip_target = target
        self.last_action = action
        self.steps += 1
        return target, full[4:6], action
