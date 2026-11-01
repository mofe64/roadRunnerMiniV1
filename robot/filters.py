"""Small filters shared by the simulator (balance_env.py) and the Pi (controller.py), so both do the same.

GyroLowPass: drive_022-027 at 6 Hz, drive_030 and later at 10 Hz (review round 8b). GyroNotch: drive_028/029 only (trials).

GyroLowPass: 2-pole Butterworth low-pass on the 3 gyro axes, at the decision rate (50 Hz).
Why (review 2026-11-01): the real body rocks on the legs at 10-11 Hz (play and flex the encoders cannot
see). The brain reacted to that in the gyro and kept the rocking going (the "buzz"). The filter cuts the
gyro at 10 Hz by about 3-4x and costs about 20 deg of phase at 2 Hz, where balancing happens.
The simulator trains with it, so it is part of the robot the brain learns, not an add-on.
"""
import numpy as np


class GyroLowPass:
    def __init__(self, cutoff_hz, rate_hz):
        k = np.tan(np.pi * cutoff_hz / rate_hz)
        n = 1.0 / (1.0 + np.sqrt(2.0) * k + k * k)
        self.b = np.array([k * k * n, 2.0 * k * k * n, k * k * n])
        self.a = np.array([2.0 * (k * k - 1.0) * n, (1.0 - np.sqrt(2.0) * k + k * k) * n])
        self.z = None

    def reset(self):
        self.z = None

    def response(self, f_hz, rate_hz):
        """Gain and phase (deg) at f_hz, for the report card print-out (README rule 13)."""
        z = np.exp(-1j * 2.0 * np.pi * f_hz / rate_hz)
        h = (self.b[0] + self.b[1] * z + self.b[2] * z * z) / (1.0 + self.a[0] * z + self.a[1] * z * z)
        return float(abs(h)), float(np.degrees(np.angle(h)))

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        if self.z is None:            # the first reading: start at rest on it (no jump)
            self.z = np.stack([x * (1.0 - self.b[0]), x * (self.b[2] - self.a[1])])
        y = self.b[0] * x + self.z[0]
        self.z = np.stack([self.b[1] * x - self.a[0] * y + self.z[1], self.b[2] * x - self.a[1] * y])
        return y


class GyroNotch:
    """Gyro filter of the trial brains drive_028/029 (review round 8, 2026-11-01; replaced by a 10 Hz low-pass in round 8b): a notch at the hip-flex resonance, then a
    gentle 2-pole low-pass, on the 3 gyro axes, at the decision rate (50 Hz).
    Why: the 6 Hz low-pass (GyroLowPass) stopped the 10-11 Hz buzz, but it made the gyro about 50 deg late at
    3.5 Hz, and the real robot then rocked at 3.5 Hz (logs run_20261101_180440 / 181813). The notch removes
    10-11 Hz and costs only about 17 deg at 3.5 Hz (Q 1). The low-pass (15 Hz) keeps high frequencies small.
    An observation filter is part of the robot the brain learns: change it only with a new training run."""
    def __init__(self, notch_hz, q, lp_hz, rate_hz):
        w = 2.0 * np.pi * notch_hz / rate_hz
        al, c = np.sin(w) / (2.0 * q), np.cos(w)
        a0 = 1.0 + al
        self.nb = np.array([1.0, -2.0 * c, 1.0]) / a0
        self.na = np.array([-2.0 * c, 1.0 - al]) / a0
        self.lp = GyroLowPass(lp_hz, rate_hz) if lp_hz > 0 else None
        self.z = None

    def reset(self):
        self.z = None
        if self.lp is not None:
            self.lp.reset()

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        if self.z is None:            # the first reading: start at rest on it (no jump); the notch passes a constant
            self.z = np.stack([x * (1.0 - self.nb[0]), x * (self.nb[2] - self.na[1])])
        y = self.nb[0] * x + self.z[0]
        self.z = np.stack([self.nb[1] * x - self.na[0] * y + self.z[1], self.nb[2] * x - self.na[1] * y])
        return self.lp(y) if self.lp is not None else y

    def response(self, f_hz, rate_hz):
        """Gain and phase (deg) at f_hz, for the review print-out."""
        z = np.exp(-1j * 2.0 * np.pi * f_hz / rate_hz)
        h = (self.nb[0] + self.nb[1] * z + self.nb[2] * z * z) / (1.0 + self.na[0] * z + self.na[1] * z * z)
        if self.lp is not None:
            lp = self.lp
            h *= (lp.b[0] + lp.b[1] * z + lp.b[2] * z * z) / (1.0 + lp.a[0] * z + lp.a[1] * z * z)
        return float(abs(h)), float(np.degrees(np.angle(h)))


def make_gyro_filter(c, rate_hz):
    """The gyro filter that a brain was trained with, from its file values (None = no filter)."""
    notch = float(c["gyro_notch"]) if "gyro_notch" in c else 0.0
    lp = float(c["gyro_lp"]) if "gyro_lp" in c else 0.0
    if notch > 0:
        return GyroNotch(notch, float(c["gyro_notch_q"]), lp, rate_hz)
    return GyroLowPass(lp, rate_hz) if lp > 0 else None


class DownFilter:
    """Which way is down, in the body frame: the complementary filter of robot/imu.py, shared with the sim
    (review round 7, 2026-11-01). The gyro turns the estimate; the accelerometer pulls it back with time
    constant tau. The accelerometer also feels the body's own acceleration (wheel pushes), so a fast
    wheel back-and-forth shows up as a false tilt; the sim must see that too."""
    def __init__(self, tau):
        self.tau, self.down = tau, None

    def reset(self):
        self.down = None

    def update(self, gyro, acc, dt):
        meas = -np.asarray(acc, float) / (np.linalg.norm(acc) + 1e-9)   # at rest the accelerometer reads "up"
        if self.down is None:
            self.down = meas
        else:
            pred = self.down - dt * np.cross(gyro, self.down)   # the body turned, so "down" turns the other way
            a = dt / self.tau
            self.down = (1 - a) * pred + a * meas
            self.down /= np.linalg.norm(self.down)
        return self.down.copy()
