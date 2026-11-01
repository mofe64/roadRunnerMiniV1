"""Calibrate the servos: the "zero" of each hip and head servo, and which way each servo turns.

You need: the robot fully wired, both adapters powered, and a phone with a level app
(iPhone: Measure app > Level; Android: any "bubble level" app).
Hold the robot upright with the body level (phone flat on the lid reads 0 deg).

Do the three parts one at a time, in any order. Each part saves its result:
  python3 tools/calibrate.py legs     the 4 short links (hips)
  python3 tools/calibrate.py head     head yaw and pitch
  python3 tools/calibrate.py wheels   roll the robot forward on the floor
  python3 tools/calibrate.py          all three, one after the other
Each servo counts 0..4095 per turn. A joint must NOT cross the 0/4095 point inside its working range:
near that point the servo can run the long way round (2026-10-31: the rear-left hip did this with an
offset register set). So this script sets each offset register back to 0 (factory) and checks that
each zero is far enough from 0/4095. If not, it tells you to run fix_horns.py.
The servos stay limp (torque off) the whole time. Nothing moves by itself.
"""
import sys
PARTS = sys.argv[1:] or ["legs", "head", "wheels"]
if any(p not in ("legs", "head", "wheels") for p in PARTS):
    raise SystemExit("Use: calibrate.py [legs] [head] [wheels]")
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # robot/: the shared modules
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


def offsets_to_zero(names):
    import time
    for n in names:
        sid = ids[n]
        legs.write(sid, sts.TORQUE_ENABLE, [0])
        if sts.signed(legs.read2(sid, sts.OFFSET), 11) != 0:
            legs.write(sid, sts.LOCK, [0]); legs.write2(sid, sts.OFFSET, 0); legs.write(sid, sts.LOCK, [1])
            time.sleep(0.05)
            print(f"  {n}: offset register set back to 0")


# Working range of each joint in the simulator's angles (deg) with a margin. Every count in it must stay
# inside 100..3995, away from the servo's 0/4095 point.
RANGE = {"hip_f_L": (-110, 10), "hip_f_R": (-110, 10), "hip_r_L": (-10, 110), "hip_r_R": (-10, 110),
         "head_yaw": (-100, 100), "head_pitch": (-40, 60)}
BAD = []


def seam_check(n):
    z, sg = cfg["zero"][n], cfg["sign"][n]
    ends = [z + sg * np.radians(a) / RAD for a in RANGE[n]]
    if min(ends) < 100 or max(ends) > 3995:
        BAD.append(n)
        print(f"  WARNING {n}: zero {z}: its working range reaches counts {min(ends):.0f}..{max(ends):.0f}, "
              f"over the 0/4095 point. Run fix_horns.py {n}.")


if "legs" in PARTS:
    offsets_to_zero(HIPS)
if "head" in PARTS:
    offsets_to_zero(HEAD)

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
    seam_check(n)

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
    for n in HEAD:
        seam_check(n)

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


save_config(cfg)
print("\nSaved config.json (" + ", ".join(PARTS) + ").")
if BAD:
    print("NOT READY: " + ", ".join(BAD) + " too close to the 0/4095 point. Run fix_horns.py, then this part again.")
