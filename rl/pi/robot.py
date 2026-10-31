"""The real robot: servo buses + IMU, in the simulator's units (see controller.py).

config.json (made by setup_ids.py and calibrate.py) holds the servo IDs, the direction of each servo
and the "zero" of each hip and head servo (the count when the upper link points straight down).
"""
import json
import os
import time
import numpy as np
import sts

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "config.json")
HIPS = ["hip_f_L", "hip_r_L", "hip_f_R", "hip_r_R"]
HEAD = ["head_yaw", "head_pitch"]
WHEELS = ["wheel_L", "wheel_R"]
DEFAULT_IDS = {"head_pitch": 1, "head_yaw": 2, "hip_f_L": 3, "hip_f_R": 4, "hip_r_L": 5, "hip_r_R": 6,
               "wheel_L": 11, "wheel_R": 12}   # IDs chosen by Mofe, 2026-10-30
RAD = 2 * np.pi / sts.COUNTS          # rad per count


def load_config():
    if not os.path.exists(CONFIG):
        return {"ids": dict(DEFAULT_IDS), "sign": {}, "zero": {}, "imu_axes": np.eye(3).tolist()}
    with open(CONFIG) as f:
        return json.load(f)


def save_config(cfg):
    with open(CONFIG, "w") as f:
        json.dump(cfg, f, indent=1)


def open_buses(cfg):
    """Find which USB adapter is which bus: the legs bus answers the front-left hip ID, the wheels bus a wheel ID."""
    legs = wheels = None
    for p in sts.ports():
        b = sts.Bus(p)
        if b.ping(cfg["ids"]["hip_f_L"]) or b.ping(cfg["ids"]["head_yaw"]):
            legs = b
        elif b.ping(cfg["ids"]["wheel_L"]) or b.ping(cfg["ids"]["wheel_R"]):
            wheels = b
    return legs, wheels


class Robot:
    def __init__(self, cfg=None, use_imu=True):
        self.cfg = cfg or load_config()
        missing = [n for n in HIPS + HEAD + WHEELS if n not in self.cfg["sign"]]
        if missing or any(n not in self.cfg["zero"] for n in HIPS + HEAD):
            raise SystemExit(f"Not calibrated: {missing or 'zeros'}. Run calibrate.py first.")
        self.legs, self.wheels = open_buses(self.cfg)
        if self.legs is None or self.wheels is None:
            raise SystemExit("Did not find both servo buses. Check both USB cables and both adapter power supplies.")
        self.id = self.cfg["ids"]
        self.sign = self.cfg["sign"]
        self.zero = self.cfg["zero"]
        if use_imu:
            from imu import IMU
            self.imu = IMU(axes=np.array(self.cfg["imu_axes"]))
        self.last_wheel = None

    # ---------- setup ----------
    def prepare(self):
        """Hips and head in position mode (fastest acceleration and speed), wheels in PWM mode at 0."""
        for n in HIPS + HEAD:
            sid = self.id[n]
            r = self.legs.read(sid, sts.MODE, 1)
            if r is None or r[0] != sts.MODE_POSITION:
                self.legs.set_mode(sid, sts.MODE_POSITION)
            self.legs.write(sid, sts.ACC, [0])
            self.legs.write2(sid, sts.GOAL_SPEED, 0)
        for n in WHEELS:
            sid = self.id[n]
            r = self.wheels.read(sid, sts.MODE, 1)
            if r is None or r[0] != sts.MODE_PWM:
                self.wheels.set_mode(sid, sts.MODE_PWM)
            self.wheels.write2(sid, sts.PWM_GOAL, 0)
            self.wheels.write(sid, sts.TORQUE_ENABLE, [1])

    def hold_here(self):
        """Hips and head: set the goal to where they are now, then torque on, so nothing jumps."""
        for n in HIPS + HEAD:
            sid = self.id[n]
            c = self.legs.read2(sid, sts.PRESENT_POS)
            if c is not None:
                self.legs.write2(sid, sts.GOAL_POS, c)
            self.legs.write(sid, sts.TORQUE_ENABLE, [1])

    # ---------- unit conversion ----------
    def to_rad(self, name, count):
        d = (count - self.zero[name] + sts.COUNTS // 2) % sts.COUNTS - sts.COUNTS // 2   # the count wraps at 4096
        return self.sign[name] * d * RAD

    def to_count(self, name, rad):
        return int(round(self.zero[name] + self.sign[name] * rad / RAD))

    # ---------- sensors ----------
    def read_joints(self):
        """Hip angles (rad), hip speeds (rad/s), wheel speeds (rad/s, + = forward). None if a servo did not answer."""
        r = self.legs.sync_read([self.id[n] for n in HIPS], sts.PRESENT_POS, 4)
        if len(r) < 4:
            return None
        q = np.array([self.to_rad(n, r[self.id[n]][0] | (r[self.id[n]][1] << 8)) for n in HIPS])
        v = np.array([self.sign[n] * sts.signed(r[self.id[n]][2] | (r[self.id[n]][3] << 8)) * RAD for n in HIPS])
        w = self.wheels.sync_read([self.id[n] for n in WHEELS], sts.PRESENT_POS, 2)
        if len(w) < 2:
            return None
        now = time.perf_counter()
        pos = np.array([w[self.id[n]][0] | (w[self.id[n]][1] << 8) for n in WHEELS], dtype=float)
        if self.last_wheel is None:
            wheel_v = np.zeros(2)
        else:
            t0, p0 = self.last_wheel
            dp = (pos - p0 + sts.COUNTS / 2) % sts.COUNTS - sts.COUNTS / 2     # the count wraps at 4096
            wheel_v = np.array([self.sign[n] for n in WHEELS]) * dp * RAD / max(now - t0, 1e-3)
        self.last_wheel = (now, pos)
        return q, v, wheel_v

    def battery(self):
        return self.wheels.volts(self.id["wheel_L"])

    # ---------- commands ----------
    def write_hips(self, rad, head=(0.0, 0.0)):
        vals = {}
        for n, a in list(zip(HIPS, rad)) + list(zip(HEAD, head)):
            c = int(np.clip(self.to_count(n, a), 0, sts.COUNTS - 1))
            vals[self.id[n]] = [c & 0xFF, (c >> 8) & 0xFF]
        self.legs.sync_write(sts.GOAL_POS, vals)

    def write_wheels(self, power):
        """power -1..1 per wheel (+ = forward). 1.0 = PWM 1000 = full power, as on the bench test."""
        vals = {}
        for n, p in zip(WHEELS, power):
            raw = sts.unsigned(int(round(np.clip(p, -1, 1) * 1000)) * self.sign[n], bit=10)
            vals[self.id[n]] = [raw & 0xFF, (raw >> 8) & 0xFF]
        self.wheels.sync_write(sts.PWM_GOAL, vals)

    def stop(self, hips_off=True):
        """Wheels to 0 at once. Hips torque off too (the robot goes limp), unless hips_off=False."""
        for _ in range(3):
            self.write_wheels([0.0, 0.0])
        if hips_off:
            self.legs.torque([self.id[n] for n in HIPS + HEAD], False)
