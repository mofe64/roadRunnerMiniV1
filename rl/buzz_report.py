"""Buzz numbers from a real run log, computed the same way as the sim report card (drive_env.buzz_numbers):
gyro rms, gyro 9-13 Hz share, 2.5-5 Hz band, wheel action flips per second (zero crossings by more than 0.1).
Run in rl/:  .venv/bin/python buzz_report.py ../robot/logs/run_<time>.csv [more files] [--brain ../robot/brains/brain_<RUN>.npz]
The first 1 s (hand release) is skipped. Pass marks: gyro rms < 0.3, no buzz (9-13 Hz band rms > 0.10 rad/s with share > 30 %), flips < 5/s,
wheel |a| < 0.2, wheel speed rms < 0.05 m/s, growth (last minute / first minute of |a|) <= 1.2 (review round 6).
3.5 Hz marks (review round 8, use a FULL battery: the rocking grows with the voltage): 2.5-5 Hz band < 0.10 rad/s,
no 9-13 Hz buzz, creep < 0.5 m (standing parts). With --brain, the log is replayed through the brain to print
how many steps the hip targets hit the 1 rad/s speed limit (drive_026: 77 %; aim well under 30 %)."""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
from drive_env import count_flips, buzzing, rocking, rock_band, POLICY_HZ, WHEEL_R

REAL_BAND_MAX = 0.10    # rad/s: the real 3.5 Hz mode is 2-3x the sim's (sim stress mark 0.05)


def slew_clipped(d, brain_path):
    """Share of steps where the Controller's hip target hit the speed limit (replay of the log through the brain)."""
    from brain import Brain
    from controller import Controller
    ctl = Controller(Brain(brain_path)); step = float(ctl.c["hip_max_speed"]) / ctl.hz
    T = []
    for i in range(len(d)):
        r = d[i]
        ctl.set_command(r["cmd_v"], r["cmd_w"], r["cmd_h"])
        q = np.array([r["q1"], r["q2"], r["q3"], r["q4"]])
        if i == 0:
            ctl.reset(q)
        t, _, _ = ctl.step(np.array([r["gx"], r["gy"], r["gz"]]), np.array([r["bwx"], r["bwy"], r["bwz"]]), q,
                           np.array([r["v1"], r["v2"], r["v3"], r["v4"]]), np.array([r["wl"], r["wr"]]))
        T.append(t.copy())
    dT = np.abs(np.diff(np.array(T), axis=0)).max(axis=1)
    return float(np.mean(dT > 0.999 * step))


args = [a for a in sys.argv[1:]]
brain_path = None
if "--brain" in args:
    k = args.index("--brain"); brain_path = args[k + 1]; del args[k:k + 2]
for path in args:
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"]; keep = t > t[0] + 1.0
    g, a = d["wy"][keep], 0.5 * (d["a3"][keep] + d["a4"][keep])     # gyro y (body pitch rate), mean wheel action
    x = g - g.mean()
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2; f = np.fft.rfftfreq(len(x), 1.0 / POLICY_HZ)
    share = P[(f >= 9) & (f <= 13)].sum() / max(P[f > 0.5].sum(), 1e-12)
    flips = count_flips(a) / (len(a) / POLICY_HZ)
    ua, vrms, growth = rocking(a, 0.5 * (d["wl"][keep] + d["wr"][keep]) * WHEEL_R)
    ok = (np.std(g) < 0.3 and not buzzing(np.std(g), share) and flips < 5 and ua < 0.2 and vrms < 0.05 and growth <= 1.2)
    print(f"{os.path.basename(path)}: {t[-1] - t[0]:.1f} s, gyro rms {np.std(g):.2f}, 9-13 Hz {100 * share:.0f} %, "
          f"band rms {np.sqrt(share) * np.std(g):.2f}, flips {flips:.1f}/s, wheel |a| {ua:.2f}, wheel speed rms {vrms:.3f} m/s, growth {growth:.1f}x  MARKS {'PASS' if ok else 'FAIL'}")
    # 3.5 Hz marks (review round 8)
    band = rock_band(g)
    still = keep & (np.abs(d["cmd_v"]) < 1e-6) & (np.abs(d["cmd_w"]) < 1e-6)
    creep = float(np.max(np.abs(np.cumsum(0.5 * (d["wl"] + d["wr"])[still]) * WHEEL_R / POLICY_HZ))) if still.any() else 0.0
    ok35 = band < REAL_BAND_MAX and not buzzing(np.std(g), share) and creep < 0.5
    volts = np.median(d["volts"]) if "volts" in d.dtype.names else float("nan")
    line = (f"   3.5 Hz: 2.5-5 Hz band {band:.3f} rad/s (< {REAL_BAND_MAX}), standing creep {creep:.2f} m (< 0.5), "
            f"battery {volts:.1f} V{' (NOT FULL: test again at 12.3 V or more)' if volts < 12.2 else ''}")
    if brain_path:
        line += f", hip targets at the speed limit {100 * slew_clipped(d, brain_path):.0f} % of steps"
    print(line + "  3.5 Hz MARKS " + ("PASS" if ok35 else "FAIL"))
