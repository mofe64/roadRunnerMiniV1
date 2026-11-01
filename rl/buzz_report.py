"""Buzz numbers from a real run log, computed the same way as the sim report card (drive_env.buzz_numbers):
gyro rms, gyro 9-13 Hz share, wheel action flips per second (zero crossings by more than 0.1).
Run in rl/:  .venv/bin/python buzz_report.py ../robot/logs/run_<time>.csv [more files]
The first 1 s (hand release) is skipped. Pass marks: gyro rms < 0.3, no buzz (9-13 Hz band rms > 0.10 rad/s with share > 30 %), flips < 5/s,
wheel |a| < 0.2, wheel speed rms < 0.05 m/s, growth (last minute / first minute of |a|) <= 1.2 (review round 6)."""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from drive_env import count_flips, buzzing, rocking, POLICY_HZ, WHEEL_R

for path in sys.argv[1:]:
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
