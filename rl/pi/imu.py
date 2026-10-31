"""Adafruit LSM6DS3TR-C IMU on the Pi 5 I2C bus 1 (pins 3 SDA, 5 SCL), address 0x6A.

Settings (from the independent review, 2026-10-30): accel and gyro at 208 Hz, accel +/-4 g, gyro +/-500 deg/s.
One 12-byte read per decision. A complementary filter gives the "which way is down" vector: the gyro
turns it, and the accelerometer pulls it back slowly (time constant 0.5 s), so short pushes do not upset it.
Body frame: X forward (the board's X arrow to the nose), Y left, Z up. AXES in config.json can change it.
"""
import time
import numpy as np

ADDR = 0x6A
WHO_AM_I, CTRL1_XL, CTRL2_G, CTRL3_C, OUTX_L_G = 0x0F, 0x10, 0x11, 0x12, 0x22
GYRO_SCALE = np.radians(17.5e-3)       # rad/s per count at +/-500 deg/s
ACC_SCALE = 0.122e-3                   # g per count at +/-4 g


class IMU:
    def __init__(self, axes=np.eye(3), bus=1, addr=ADDR, tau=0.5):
        from smbus2 import SMBus
        self.i2c, self.addr = SMBus(bus), addr
        who = self.i2c.read_byte_data(addr, WHO_AM_I)
        if who != 0x6A:
            raise RuntimeError(f"IMU not found at 0x{addr:02x} (WHO_AM_I = 0x{who:02x}, expected 0x6a). Check the wires.")
        self.i2c.write_byte_data(addr, CTRL3_C, 0x44)      # block data update, auto address increment
        self.i2c.write_byte_data(addr, CTRL1_XL, 0x58)     # accel 208 Hz, +/-4 g
        self.i2c.write_byte_data(addr, CTRL2_G, 0x54)      # gyro 208 Hz, +/-500 deg/s
        time.sleep(0.1)
        self.axes = np.asarray(axes, dtype=float)          # body = axes @ sensor
        self.tau = tau
        self.gyro_zero = np.zeros(3)
        self.down = None

    def raw(self):
        """(gyro rad/s, accel g), in the body frame, gyro zero removed."""
        b = bytes(self.i2c.read_i2c_block_data(self.addr, OUTX_L_G, 12))
        v = np.frombuffer(b, dtype="<i2").astype(float)
        gyro = self.axes @ (v[0:3] * GYRO_SCALE) - self.gyro_zero
        acc = self.axes @ (v[3:6] * ACC_SCALE)
        return gyro, acc

    def calibrate(self, seconds=1.0):
        """The robot must be still. Measures the gyro zero (it drifts a little with temperature)."""
        self.gyro_zero[:] = 0
        g = []
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end:
            g.append(self.raw()[0]); time.sleep(0.005)
        g = np.array(g)
        self.gyro_zero = g.mean(axis=0)
        return self.gyro_zero, g.std(axis=0)

    def update(self, dt):
        """Read once and update the filter. Returns (down unit vector in the body frame, gyro rad/s)."""
        gyro, acc = self.raw()
        meas = -acc / (np.linalg.norm(acc) + 1e-9)        # at rest the accelerometer reads "up"
        if self.down is None:
            self.down = meas
        else:
            pred = self.down - dt * np.cross(gyro, self.down)   # the body turned, so "down" turns the other way
            a = dt / self.tau
            self.down = (1 - a) * pred + a * meas
            self.down /= np.linalg.norm(self.down)
        return self.down.copy(), gyro
