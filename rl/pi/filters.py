"""Small filters shared by the simulator (balance_env.py) and the Pi (controller.py), so both do the same.

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

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        if self.z is None:            # the first reading: start at rest on it (no jump)
            self.z = np.stack([x * (1.0 - self.b[0]), x * (self.b[2] - self.a[1])])
        y = self.b[0] * x + self.z[0]
        self.z = np.stack([self.b[1] * x - self.a[0] * y + self.z[1], self.b[2] * x - self.a[1] * y])
        return y


class DownFilter:
    """Which way is down, in the body frame: the complementary filter of pi/imu.py, shared with the sim
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
