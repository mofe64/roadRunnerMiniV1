"""Fit the simulator's hip servo model to the real step tests (2026-10-31).

  python fit_hip_servo.py
Real tests (robot/tools/hip_bench.py, front-left hip, register 0x55 = 0 = no acceleration limit): legs in the air,
steps of 8 deg (data/hip_bench_20261031_231154_8deg.csv) and 20 deg (data/hip_bench_20261031_231209_20deg.csv)
around the stand pose, position read every 1 ms. Here the same steps run in the simulator (body held still
in the air) and the servo values change until both simulated steps look like the average real steps.
Result: kp 98.8, damping 0.49, force 1.94 Nm, armature 0.0037, filter 0.024 s, delay 24 ms; error 0.022.
(data/hip_bench_20261031_221856.csv is the older test with the factory limit 0x55 = 254: 1.7x slower.)
"""
import os
import sys
import numpy as np
import mujoco
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
HIPS = ["hip_f_L", "hip_r_L", "hip_f_R", "hip_r_R"]
DT = 0.002
GRID = np.arange(0, 0.4, DT)


def real_step(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    t, h, tg, p = d["t"], d["hip"], d["target"], d["pos"]
    ch = np.where((np.diff(tg) != 0) | (np.diff(h) != 0))[0] + 1
    curves = []
    for s, e in zip(np.r_[0, ch], np.r_[ch, len(t)]):
        if s == 0 or h[s] != h[s - 1]:
            continue                       # the first step of each hip starts from another hip's data
        p0 = p[s - 1]; size = tg[s] - p0
        if abs(size) < 0.1:
            continue
        curves.append(np.interp(GRID, t[s:e] - t[s], (p[s:e] - p0) / size))
    curves = np.array(curves)
    ref = curves.mean(0)
    return ref, float(np.sqrt(np.mean((curves - ref) ** 2))), len(curves)


def model(kp, damping, timeconst, force):
    xml = open(os.path.join(HERE, "roadrunner_mini.xml")).read()
    xml = xml.replace(xml[xml.index('<default class="sts3215">'):xml.index("</default>", xml.index('<default class="sts3215">'))],
                      f'<default class="sts3215">\n      <joint damping="{damping:.4f}" frictionloss="0.052" armature="0.028"/>\n'
                      f'      <position kp="{kp:.3f}" kv="0" forcerange="-{force:.3f} {force:.3f}" timeconst="{timeconst:.4f}"/>\n    ')
    path = os.path.join(HERE, "_fit_tmp.xml")
    open(path, "w").write(xml)
    m = mujoco.MjModel.from_xml_path(path); os.remove(path)
    m.opt.timestep = DT
    return m


def sim_step(m, delay, size=np.radians(8)):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
    d.qpos[2] += 0.2; hold = d.qpos[:7].copy()
    act = [m.actuator(n).id for n in HIPS]; q = [m.jnt_qposadr[m.joint(n).id] for n in HIPS]
    for a, j in zip(act, q):
        d.ctrl[a] = d.qpos[j]; d.act[m.actuator_actadr[a]] = d.qpos[j]
    for _ in range(300):
        d.qpos[:7] = hold; d.qvel[:6] = 0; mujoco.mj_step(m, d)
    p0, out = d.qpos[q[0]], []
    for k in range(len(GRID)):
        if k * DT >= delay:
            d.ctrl[act[0]] = p0 + size
        d.qpos[:7] = hold; d.qvel[:6] = 0; mujoco.mj_step(m, d); out.append(d.qpos[q[0]])
    return (np.array(out) - p0) / size


def real_hip(path, hip=0):
    """Average step of one hip (normalised: 0 at the start, 1 at the target)."""
    d = np.genfromtxt(path, delimiter=",", names=True)
    t, h, tg, p = d["t"], d["hip"], d["target"], d["pos"]
    ch = np.where((np.diff(tg) != 0) | (np.diff(h) != 0))[0] + 1
    curves = []
    for s, e in zip(np.r_[0, ch], np.r_[ch, len(t)]):
        if s == 0 or h[s] != hip or h[s - 1] != hip:
            continue
        p0 = p[s - 1]; size = tg[s] - p0
        if abs(size) < 0.1:
            continue
        curves.append(np.interp(GRID, t[s:e] - t[s], (p[s:e] - p0) / size))
    return np.mean(curves, 0)


def model_arm(kp, damping, force, armature, timeconst):
    m = model(kp, damping, max(timeconst, 0.001), force)
    for n in HIPS:
        m.dof_armature[m.jnt_dofadr[m.joint(n).id]] = armature
    return m


if __name__ == "__main__":
    r8 = real_hip(os.path.join(HERE, "data", "hip_bench_20261031_231154_8deg.csv"))
    r20 = real_hip(os.path.join(HERE, "data", "hip_bench_20261031_231209_20deg.csv"))

    def err(x):
        kp, damping, force, armature, delay, tc = x
        if kp < 2 or damping < 0 or not 0.3 < force < 2.0 or not 0.002 < armature < 0.1 or not 0 <= delay < 0.05 or not 0 <= tc < 0.05:
            return 9.0
        m = model_arm(kp, damping, force, armature, tc)
        e8 = sim_step(m, delay, np.radians(8)) - r8
        e20 = sim_step(m, delay, np.radians(20)) - r20
        return float(np.sqrt((np.mean(e8 ** 2) + np.mean(e20 ** 2)) / 2))
    starts = ([40, 0.4, 1.9, 0.03, 0.02, 0.01], [80, 0.35, 1.8, 0.05, 0.015, 0.02], [25, 0.45, 1.9, 0.02, 0.025, 0.005])
    best = min((minimize(err, x0, method="Nelder-Mead", options=dict(maxiter=400, xatol=1e-4, fatol=1e-5)) for x0 in starts),
               key=lambda r: r.fun)
    kp, damping, force, armature, delay, tc = best.x
    print(f"error {best.fun:.4f}: kp {kp:.1f}  damping {damping:.3f}  force {force:.2f} Nm  armature {armature:.4f}  "
          f"delay {delay * 1000:.0f} ms  filter {tc:.4f} s")
