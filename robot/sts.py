"""Feetech STS bus servos (STS3215 legs and head, ST3215-HS wheels) over a Waveshare bus adapter.

The packet format is the Feetech STS protocol, the same as in ../spinup_test.py and ../wheel_bench.py.
One USB adapter = one bus. The robot has two buses that must never be joined:
  bus "legs":   7.4 V, hips and head (STS3215)
  bus "wheels": battery voltage, the 2 wheel servos (ST3215-HS)
"""
import glob
import time
import serial

BAUD = 1_000_000
# memory table
ID, MIN_ANGLE, MAX_ANGLE, OFFSET, MODE = 0x05, 0x09, 0x0B, 0x1F, 0x21
TORQUE_ENABLE, ACC, GOAL_POS, PWM_GOAL, GOAL_SPEED = 0x28, 0x29, 0x2A, 0x2C, 0x2E   # PWM_GOAL is the "goal time" register, used as PWM in PWM mode
LOCK, PRESENT_POS, PRESENT_SPEED, PRESENT_LOAD, PRESENT_VOLT, PRESENT_TEMP = 0x37, 0x38, 0x3A, 0x3C, 0x3E, 0x3F
MODE_POSITION, MODE_SPEED, MODE_PWM = 0, 1, 2
COUNTS = 4096          # counts per turn


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, instr, params=()):
    body = [sid, len(params) + 2, instr, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def signed(v, bit=15):
    """Feetech sign-magnitude numbers: bit 15 (or bit 10 for PWM) is the sign."""
    return -(v & ((1 << bit) - 1)) if v & (1 << bit) else v


def unsigned(v, bit=15):
    return (abs(int(v)) & ((1 << bit) - 1)) | ((1 << bit) if v < 0 else 0)


class Bus:
    def __init__(self, port):
        self.port = port
        self.s = serial.Serial(port, BAUD, timeout=0.01)

    def _reply(self, n):
        r = self.s.read(6 + n)
        if len(r) < 6 + n or r[0] != 0xFF or r[1] != 0xFF or checksum(list(r[2:-1])) != r[-1]:
            return None
        return r[5:5 + n]

    def ping(self, sid):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, 0x01))
        return self._reply(0) is not None

    def scan(self, ids=range(0, 21)):
        return [i for i in ids if self.ping(i)]

    def read(self, sid, addr, n):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, 0x02, (addr, n)))
        return self._reply(n)

    def write(self, sid, addr, data):
        self.s.reset_input_buffer()
        self.s.write(packet(sid, 0x03, (addr, *data)))
        return self._reply(0) is not None

    def write2(self, sid, addr, value):
        return self.write(sid, addr, [value & 0xFF, (value >> 8) & 0xFF])

    def read2(self, sid, addr):
        r = self.read(sid, addr, 2)
        return None if r is None else r[0] | (r[1] << 8)

    def sync_read(self, ids, addr, n):
        """One request, every servo answers in turn. Returns {id: bytes} (missing servos are left out)."""
        self.s.reset_input_buffer()
        self.s.write(packet(0xFE, 0x82, (addr, n, *ids)))
        out = {}
        for _ in ids:
            r = self.s.read(6 + n)
            if len(r) == 6 + n and r[0] == 0xFF and r[1] == 0xFF and checksum(list(r[2:-1])) == r[-1]:
                out[r[2]] = r[5:5 + n]
        return out

    def sync_write(self, addr, values):
        """values = {id: [bytes]} (all the same length). One packet, no answers."""
        n = len(next(iter(values.values())))
        params = [addr, n]
        for sid, data in values.items():
            params += [sid, *data]
        self.s.write(packet(0xFE, 0x83, params))

    def torque(self, ids, on):
        for i in ids:
            self.write(i, TORQUE_ENABLE, [1 if on else 0])

    def set_mode(self, sid, mode):
        """The mode is in EEPROM: unlock, write, lock."""
        self.write(sid, TORQUE_ENABLE, [0])
        self.write(sid, LOCK, [0])
        self.write(sid, MODE, [mode])
        self.write(sid, LOCK, [1])

    def set_id(self, old, new):
        self.write(old, LOCK, [0])
        self.write(old, ID, [new])
        time.sleep(0.05)
        self.write(new, LOCK, [1])
        return self.ping(new)

    def volts(self, sid):
        r = self.read(sid, PRESENT_VOLT, 1)
        return None if r is None else r[0] / 10.0


def ports():
    """Waveshare adapters show up as USB serial ports (CH343 or CH340 chip)."""
    return sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") +
                      glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*") + glob.glob("/dev/cu.usbmodem*")))
