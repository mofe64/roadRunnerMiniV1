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
