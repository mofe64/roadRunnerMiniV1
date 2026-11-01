"""Wheel servo bench test: measures the ST3215-HS in BOTH ways we could drive it, so we can choose one.

  Part A, PWM mode (the brain sends a POWER): steps of 25, 50, 75, 100 % power, forward and back.
          Gives the speed for each power (the motor curve) and how fast it speeds up.
  Part B, speed mode (the brain sends a SPEED, like Upkie): steps of commanded speed.
          Gives the top speed, how close it gets to each target, how fast and how late it reacts.

  1. Wheel ON the servo, servo held still, wheel in the air (it spins both ways, up to full speed).
  2. Servo -> Waveshare bus adapter -> USB to the Mac. Adapter power: the 12 V adapter or the 3S battery.
  3. .venv/bin/python wheel_bench.py               (asks before anything moves, takes about 30 s)

It changes the servo mode for the test and puts the old mode back at the end.
Results: printed, and saved to wheel_bench_<id>.csv (mode, step, target, time, speed, angle).
"""
import time

import numpy as np

from spinup_test import Bus, find_port, unwrap_counts, TORQUE_ENABLE, LOCK, MODE, PRESENT_POS, PRESENT_VOLT

ACC, PWM_GOAL, GOAL_SPEED = 0x29, 0x2C, 0x2E          # Feetech STS memory table
STEP = 2 * np.pi / 4096                              # rad per position count (speeds are counts per second)
PWM_TARGETS = [250, 500, 750, 1000, 0, -500, 0]      # power, 1000 = 100 %
SPEED_TARGETS = [1000, 2000, 3000, 4000, 0, -2000, 0]  # counts/s (4000 = about 6.1 rad/s; above top speed it saturates)
HOLD = 1.5                                           # s at each target


def signed(v):
    """Feetech sign: bit 15 = negative."""
    return -(v & 0x7FFF) if v & 0x8000 else v


def speed_bytes(v):
    raw = (abs(int(v)) & 0x7FFF) | (0x8000 if v < 0 else 0)
    return [raw & 0xFF, raw >> 8]


def pwm_bytes(v):
    raw = min(abs(int(v)), 1000) | (0x400 if v < 0 else 0)   # bit 10 = reverse
    return [raw & 0xFF, raw >> 8]


def set_mode(bus, sid, mode):
    bus.write(sid, TORQUE_ENABLE, [0])
    bus.write(sid, LOCK, [0])
    bus.write(sid, MODE, [mode])
    bus.write(sid, LOCK, [1])
    bus.write(sid, TORQUE_ENABLE, [1])


def log(bus, sid, seconds):
    """Read speed and position as fast as the bus allows."""
    t, w, p, t0 = [], [], [], time.perf_counter()
    while (now := time.perf_counter() - t0) < seconds:
        r = bus.read(sid, PRESENT_POS, 4)              # position (2 bytes) then speed (2 bytes)
        if r is not None:
            t.append(now)
            p.append(r[0] | (r[1] << 8))
            w.append(signed(r[2] | (r[3] << 8)))
    return np.array(t), np.array(w, dtype=float) * STEP, np.array(p, dtype=float)


def main():
    port = find_port()
    bus = Bus(port)
    ids = [i for i in range(1, 21) if bus.ping(i)]
    if not ids:
        raise SystemExit("No servo answered on IDs 1-20. Check power on the adapter and the cable.")
    import sys
    sid = int(sys.argv[1]) if len(sys.argv) > 1 else ids[0]
    volt = bus.read(sid, PRESENT_VOLT, 1)
    volt = volt[0] / 10 if volt else float("nan")
    old_mode = bus.read(sid, MODE, 1)
    old_mode = old_mode[0] if old_mode else 0
    print(f"port {port} | servos {ids} -> testing ID {sid} | supply {volt:.1f} V | current mode {old_mode}")
    input("Wheel in the air and servo held still? Press Enter to start (Ctrl+C to cancel)... ")

    parts = [("PWM", 2, PWM_GOAL, pwm_bytes, PWM_TARGETS), ("SPEED", 1, GOAL_SPEED, speed_bytes, SPEED_TARGETS)]
    rows, results = [], {}
    try:
        for name, mode, reg, enc, targets in parts:
            print(f"\nPart {name} mode")
            set_mode(bus, sid, mode)
            if mode == 1:
                bus.write(sid, ACC, [0])                   # 0 = no speed ramp: we want the servo's raw response
            bus.write(sid, reg, enc(0))
            time.sleep(0.8)
            prev, steps = 0, []
            for k, target in enumerate(targets):
                bus.write(sid, reg, enc(target))
                t, w, p = log(bus, sid, HOLD)
                ang = unwrap_counts(p) * STEP
                wp = np.gradient(ang, t) if len(t) > 3 else np.zeros_like(t)   # speed from the position (a check)
                rows += [(mode, k, target, ti, wi, ai) for ti, wi, ai in zip(t, w, ang)]
                steps.append((prev, target, t, w, wp))
                prev = target
                print(f"  step {k + 1}/{len(targets)}: {len(t)} readings ({1000 * HOLD / max(len(t), 1):.1f} ms each)")
            bus.write(sid, reg, enc(0))
            time.sleep(0.5)
            results[name] = steps
    finally:
        bus.write(sid, PWM_GOAL, pwm_bytes(0))
        bus.write(sid, GOAL_SPEED, speed_bytes(0))
        time.sleep(0.3)
        bus.write(sid, TORQUE_ENABLE, [0])
        bus.write(sid, LOCK, [0])
        bus.write(sid, MODE, [old_mode])
        bus.write(sid, LOCK, [1])
        print("servo stopped, mode restored to", old_mode)

    np.savetxt(f"wheel_bench_{sid}.csv", np.array(rows), delimiter=",",
               header="mode,step,target,time_s,speed_rad_s,angle_rad", comments="")
    print("\nRESULT (paste this to Claude)")
    print(f"  supply {volt:.1f} V")
    for name, steps in results.items():
        print(f"  {name} mode:")
        w_prev = 0.0
        for prev, target, t, w, wp in steps:
            unit = f"{target / 10:+.0f} % power" if name == "PWM" else f"{target * STEP:+.2f} rad/s"
            late = t > HOLD - 0.4
            final = np.median(w[late]) if np.any(late) else np.nan
            final_p = np.median(wp[late]) if np.any(late) else np.nan
            line = f"    -> {unit}: settled at {final:+.2f} rad/s (position says {final_p:+.2f})"
            if abs(final - w_prev) > 0.2:
                frac = (w - w_prev) / (final - w_prev)
                line += (f" | starts after {t[np.argmax(frac >= 0.1)] * 1000:.0f} ms"
                         f" | 63 % at {t[np.argmax(frac >= 0.63)] * 1000:.0f} ms | overshoot {(np.max(frac) - 1) * 100:.0f} %")
            print(line)
            w_prev = final


if __name__ == "__main__":
    main()
