"""Calibrate the servos: the "zero" of each hip and head servo, and which way each servo turns.

You need: the robot fully wired, both adapters powered, and a phone with a level app
(iPhone: Measure app > Level; Android: any "bubble level" app).
Hold the robot upright with the body level (phone flat on the lid reads 0 deg).

Do the three parts one at a time, in any order. Each part saves its result:
  python3 calibrate.py legs     the 4 short links (hips)
  python3 calibrate.py head     head yaw and pitch
  python3 calibrate.py wheels   roll the robot forward on the floor
  python3 calibrate.py center   only move each servo's middle (2048) to its measured zero
  python3 calibrate.py          all three, one after the other
After "legs" or "head" the script writes each servo's offset register (EEPROM), so that the
measured zero reads 2048, the middle of the 0..4095 count range. Then a joint never crosses
the 0/4095 wrap point inside its working range. The zero in config.json becomes 2048.
The servos stay limp (torque off) the whole time. Nothing moves by itself.
"""
import sys
PARTS = sys.argv[1:] or ["legs", "head", "wheels"]
if any(p not in ("legs", "head", "wheels", "center") for p in PARTS):
    raise SystemExit("Use: calibrate.py [legs] [head] [wheels] [center]")
import numpy as np
import sts
from robot import HIPS, HEAD, WHEELS, RAD, load_config, save_config, open_buses

cfg = load_config()
legs, wheels = open_buses(cfg)
if legs is None or wheels is None:
    raise SystemExit("Did not find both servo buses. Run setup_ids.py first, then check the cables.")
ids = cfg["ids"]
legs.torque([ids[n] for n in HIPS + HEAD], False)
for n in WHEELS:
    wheels.set_mode(ids[n], sts.MODE_PWM); wheels.write2(ids[n], sts.PWM_GOAL, 0)


def count(bus, name):
    c = bus.read2(ids[name], sts.PRESENT_POS)
    if c is None:
        raise SystemExit(f"{name} (ID {ids[name]}) did not answer.")
    return c


NICE = {"hip_f_L": "FRONT LEFT", "hip_r_L": "REAR LEFT", "hip_f_R": "FRONT RIGHT", "hip_r_R": "REAR RIGHT"}
print(__doc__)
for n in (HIPS if "legs" in PARTS else []):
    input(f"\n{NICE[n]} short link (the link on the servo horn).\n"
          f"Push the knee by hand until this link hangs STRAIGHT DOWN (like a clock hand at 6). Hold it. Press Enter.")
    z = count(legs, n)
    while True:
        input("Now turn the same link toward the NOSE: its bottom end swings forward, about 20 deg. Press Enter.")
        c1 = count(legs, n)
        d = (c1 - z + 2048) % 4096 - 2048
        print(f"  The servo moved {abs(d) * RAD * 57.3:.0f} deg.")
        if abs(d) >= 60:
            break
        print("  That is too small (need 5 deg or more). Turn it further. If it moved a lot and this still shows ~0, tell Claude.")
    cfg["zero"][n] = z
    cfg["sign"][n] = -1 if d > 0 else 1        # forward = negative angle in the simulator
    print(f"  {n}: zero {z}, direction {cfg['sign'][n]:+d}, moved {abs(d) * RAD * 57.3:.0f} deg")

if "head" in PARTS:
    input("\nHEAD: turn the head by hand so it looks straight ahead, nose level (phone on top of the head reads 0). Press Enter.")
    for n in HEAD:
        cfg["zero"][n] = count(legs, n)
        cfg["sign"][n] = 1
    input("Now turn the head a little to the LEFT. Press Enter.")
    d = (count(legs, "head_yaw") - cfg["zero"]["head_yaw"] + 2048) % 4096 - 2048
    cfg["sign"]["head_yaw"] = 1 if d > 0 else -1           # + yaw = left (simulator: about +Z)
    input("Turn it back to straight, then tip the nose a little DOWN. Press Enter.")
    d = (count(legs, "head_pitch") - cfg["zero"]["head_pitch"] + 2048) % 4096 - 2048
    cfg["sign"]["head_pitch"] = 1 if d > 0 else -1         # + pitch = nose down (simulator)

if "wheels" in PARTS:
    import time
    wheels.torque([ids[n] for n in WHEELS], False)          # free wheels: no brake while you roll it
    while True:
        input("\nWHEELS: put the robot on the floor and hold it upright. Press Enter, then roll it FORWARD (camera direction) about 30 cm.")
        p0 = {n: count(wheels, n) for n in WHEELS}
        total = {n: 0 for n in WHEELS}
        t_end = time.time() + 6
        print("Roll it now... (6 s)")
        while time.time() < t_end:
            for n in WHEELS:
                c = count(wheels, n)
                total[n] += (c - p0[n] + 2048) % 4096 - 2048; p0[n] = c
            time.sleep(0.01)
        for n in WHEELS:
            print(f"  {n}: turned {abs(total[n]) * RAD * 57.3:.0f} deg = {abs(total[n]) * RAD * 0.035 * 100:.0f} cm of floor")
        if all(abs(total[n]) * RAD * 0.035 > 0.10 for n in WHEELS):
            break
        print("  Less than 10 cm on a wheel. Nothing saved for the wheels. Roll it again (or tell Claude if the wheels did turn).")
    for n in WHEELS:
        cfg["sign"][n] = 1 if total[n] > 0 else -1
        print(f"  {n}: direction {cfg['sign'][n]:+d}")


def center(names):
    """Move the servo's middle count (2048) to the measured zero with the offset register.
    The servo reports present = raw - offset (sign bit 11). Wanted: raw_zero - new = 2048."""
    import time
    for n in names:
        if n not in cfg["zero"]:
            continue
        sid = ids[n]
        r = legs.read2(sid, sts.OFFSET)
        if r is None:
            raise SystemExit(f"{n} (ID {sid}) did not answer.")
        old = sts.signed(r, 11)
        new = (cfg["zero"][n] + old - 2048 + 2048) % 4096 - 2048
        new = max(-2047, min(2047, new))
        def put(v):
            legs.write(sid, sts.TORQUE_ENABLE, [0])
            legs.write(sid, sts.LOCK, [0])
            legs.write2(sid, sts.OFFSET, sts.unsigned(v, 11))
            legs.write(sid, sts.LOCK, [1])
            time.sleep(0.05)
            back = sts.signed(legs.read2(sid, sts.OFFSET), 11)
            if back != v:
                raise SystemExit(f"{n}: offset write failed (wrote {v}, read {back}).")
        before = count(legs, n)
        put(new)
        now = count(legs, n)
        want = (before + old - new) % 4096                      # present = raw - offset
        if abs((now - want + 2048) % 4096 - 2048) > 8:
            # This servo uses present = raw + offset: write the other sign.
            new2 = max(-2047, min(2047, (2048 - (cfg["zero"][n] - old) + 2048) % 4096 - 2048))
            put(new2)
            now = count(legs, n)
            want = (before - old + new2) % 4096
            if abs((now - want + 2048) % 4096 - 2048) > 8:
                put(old)
                raise SystemExit(f"{n}: the offset did not act as expected. Old offset put back. Do not touch the legs while this runs; tell Claude.")
            new = new2
        cfg["zero"][n] = 2048
        print(f"  {n}: offset {old} -> {new}. Zero is now 2048. Now at {(now - 2048) * RAD * 57.3:+.0f} deg from zero.")


if "legs" in PARTS or "center" in PARTS:
    print("\nCentering the hips (writes the servo offset):")
    center(HIPS)
if "head" in PARTS or "center" in PARTS:
    print("\nCentering the head (writes the servo offset):")
    center(HEAD)
save_config(cfg)
print("\nSaved config.json (" + ", ".join(PARTS) + "). When legs, head and wheels are all done: python3 check_imu.py")
