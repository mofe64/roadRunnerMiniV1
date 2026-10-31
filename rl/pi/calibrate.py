"""Calibrate the servos: the "zero" of each hip and head servo, and which way each servo turns.

You need: the robot fully wired, both adapters powered, and a phone with a level app
(iPhone: Measure app > Level; Android: any "bubble level" app).
Hold the robot upright with the body level (phone flat on the lid reads 0 deg).

Do the three parts one at a time, in any order. Each part saves its result:
  python3 calibrate.py legs     the 4 short links (hips)
  python3 calibrate.py head     head yaw and pitch
  python3 calibrate.py wheels   roll the robot forward on the floor
  python3 calibrate.py          all three, one after the other
The servos stay limp (torque off) the whole time. Nothing moves by itself.
"""
import sys
PARTS = sys.argv[1:] or ["legs", "head", "wheels"]
if any(p not in ("legs", "head", "wheels") for p in PARTS):
    raise SystemExit("Use: calibrate.py [legs] [head] [wheels]")
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
    input("Now push the same knee a little toward the NOSE (about a finger width). Press Enter.")
    c1 = count(legs, n)
    d = (c1 - z + 2048) % 4096 - 2048
    if abs(d) < 30:
        print("  It did not move enough. Run calibrate.py again."); raise SystemExit(1)
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
    input("\nWHEELS: put the robot on the floor, hold it upright, and get ready to roll it FORWARD about 30 cm. Press Enter, then roll it.")
    p0 = {n: count(wheels, n) for n in WHEELS}
    total = {n: 0 for n in WHEELS}
    import time
    t_end = time.time() + 4
    print("Rolling... (4 s)")
    while time.time() < t_end:
        for n in WHEELS:
            c = count(wheels, n)
            total[n] += (c - p0[n] + 2048) % 4096 - 2048; p0[n] = c
        time.sleep(0.01)
    for n in WHEELS:
        cfg["sign"][n] = 1 if total[n] > 0 else -1
        print(f"  {n}: turned {abs(total[n]) * RAD * 57.3:.0f} deg = {abs(total[n]) * RAD * 0.035 * 100:.0f} cm of floor, direction {cfg['sign'][n]:+d}")
save_config(cfg)
print("\nSaved config.json (" + ", ".join(PARTS) + "). When legs, head and wheels are all done: python3 check_imu.py")
