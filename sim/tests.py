import math, os, sys, json
import numpy as np
import mujoco
sys.path.insert(0, os.path.dirname(__file__))
from build_model import leg_angles, knee_angle, WHEEL_R, HS_TORQUE

m = mujoco.MjModel.from_xml_path(os.environ.get("MODEL_XML", os.path.join(os.path.dirname(__file__), "eggbot_v0.xml")))
d = mujoco.MjData(m)
DT = m.opt.timestep
HIPS = ["hip_f_L", "hip_r_L", "hip_f_R", "hip_r_R"]
act = {m.actuator(i).name: i for i in range(m.nu)}
jnt = {m.joint(i).name: i for i in range(m.njnt)}


def leg_q(h):
    th_f, th_r = leg_angles(h)
    kb = knee_angle(h)
    return th_f, (math.pi - kb), th_r, -(math.pi - kb)


def set_pose(h, pitch=0.0, z=None):
    mujoco.mj_resetData(m, d)
    tf, kf, tr, kr = leg_q(h)
    q = d.qpos
    q[:] = 0
    q[2] = WHEEL_R + h if z is None else z
    # quaternion for pitch about Y
    q[3] = math.cos(pitch / 2); q[5] = math.sin(pitch / 2)
    for s in ("L", "R"):
        q[m.jnt_qposadr[jnt[f"hip_f_{s}"]]] = tf
        q[m.jnt_qposadr[jnt[f"knee_f_{s}"]]] = kf
        q[m.jnt_qposadr[jnt[f"hip_r_{s}"]]] = tr
        q[m.jnt_qposadr[jnt[f"knee_r_{s}"]]] = kr
    mujoco.mj_forward(m, d)


def hip_targets(h):
    tf, kf, tr, kr = leg_q(h)
    d.ctrl[act["hip_f_L"]] = tf; d.ctrl[act["hip_r_L"]] = tr
    d.ctrl[act["hip_f_R"]] = tf; d.ctrl[act["hip_r_R"]] = tr


def pitch_of(quat):
    w, x, y, z = quat
    # rotation about Y (pitch), small roll assumed
    return math.atan2(2 * (w * y - z * x), 1 - 2 * (y * y + x * x))


class Balancer:
    """Cascaded PD: wheel torque from pitch, pitch rate, forward speed and position."""
    def __init__(self, kp=6.0, kd=0.35, kv=0.6, kx=0.6):
        self.kp, self.kd, self.kv, self.kx = kp, kd, kv, kx
        self.x_ref = 0.0

    def __call__(self, v_ref=0.0):
        p = pitch_of(d.qpos[3:7])
        prate = d.sensordata[m.sensor("gyro").adr[0] + 1]
        x = d.qpos[0]; v = d.qvel[0]
        self.x_ref += v_ref * DT
        tau = self.kp * p + self.kd * prate + self.kv * (v - v_ref) + self.kx * (x - self.x_ref)
        tau = float(np.clip(tau, -HS_TORQUE, HS_TORQUE))
        d.ctrl[act["wheel_L"]] = tau; d.ctrl[act["wheel_R"]] = tau
        return p, tau


def hip_torques():
    return np.array([d.actuator_force[act[n]] for n in HIPS])


def static_torques():
    out = {}
    for h in (0.07, 0.09, 0.12, 0.15):
        set_pose(h)
        hip_targets(h)
        # settle with balance controller running for 2 s, then read mean actuator forces
        bal = Balancer()
        tq = []
        for i in range(3000):
            bal(); mujoco.mj_step(m, d)
            if i > 2000: tq.append(np.abs(hip_torques()))
        tq = np.array(tq)
        out[f"{h*1000:.0f}"] = {"mean_abs_hip_Nm": tq.mean(axis=0).round(3).tolist(), "peak": float(tq.max().round(3)), "pitch_deg": round(math.degrees(pitch_of(d.qpos[3:7])), 2)}
    return out


def push_test(h=0.12, impulse=1.0):
    set_pose(h); hip_targets(h); bal = Balancer()
    log = []; fallen = False
    for i in range(8000):
        t = i * DT
        if 3.0 <= t < 3.02:
            d.xfrc_applied[1, 0] = impulse / 0.02  # N for 20 ms => impulse N s
        else:
            d.xfrc_applied[1, 0] = 0
        p, tau = bal(); mujoco.mj_step(m, d)
        if i % 10 == 0: log.append((t, p, tau, d.qpos[0], np.abs(hip_torques()).max()))
        if abs(p) > 1.0: fallen = True; break
    L = np.array(log)
    return {"fallen": fallen, "max_pitch_deg_after_push": float(np.degrees(np.abs(L[L[:, 0] > 3, 1]).max())), "max_wheel_tau": float(np.abs(L[:, 2]).max()), "max_hip_tau": float(L[:, 4].max()), "drift_m": float(L[-1, 3])}, L


def crouch_sweep():
    set_pose(0.12); hip_targets(0.12); bal = Balancer()
    log = []
    for i in range(12000):
        t = i * DT
        h = 0.11 + 0.04 * math.sin(2 * math.pi * t / 6.0 - math.pi / 2) if t > 1 else 0.12
        h = max(0.07, min(0.15, h)); hip_targets(h)
        p, tau = bal(); mujoco.mj_step(m, d)
        if i % 10 == 0: log.append((t, h, p, np.abs(hip_torques()).max(), abs(tau)))
        if abs(p) > 1.0: break
    L = np.array(log)
    return {"min_h": float(L[:, 1].min()), "max_pitch_deg": float(np.degrees(np.abs(L[:, 2]).max())), "max_hip_tau": float(L[:, 3].max()), "max_wheel_tau": float(L[:, 4].max())}, L


def drive_test(v=0.25):
    set_pose(0.12); hip_targets(0.12); bal = Balancer()
    log = []
    for i in range(8000):
        t = i * DT
        vr = v if 1 < t < 5 else 0.0
        p, tau = bal(vr); mujoco.mj_step(m, d)
        if i % 10 == 0: log.append((t, p, d.qvel[0], d.qvel[m.jnt_dofadr[jnt['wheel_L']]]))
        if abs(p) > 1.0: break
    L = np.array(log)
    return {"max_speed_mps": float(L[:, 2].max()), "max_wheel_radps": float(np.abs(L[:, 3]).max()), "max_pitch_deg": float(np.degrees(np.abs(L[:, 1]).max()))}, L


def standup_test(face="front", tau_cmd=None):
    """Start lying on the front (or back) with legs folded; drive wheels until upright, then balance."""
    h = 0.07
    pitch = math.radians(75) if face == "front" else math.radians(-75)
    set_pose(h, pitch=pitch, z=WHEEL_R + 0.02)
    hip_targets(h)
    # let it settle onto the floor
    for i in range(1500):
        d.ctrl[act["wheel_L"]] = d.ctrl[act["wheel_R"]] = 0
        mujoco.mj_step(m, d)
    p0 = pitch_of(d.qpos[3:7])
    bal = Balancer()
    tau_cmd = HS_TORQUE if tau_cmd is None else tau_cmd
    phase = "drive"; log = []; t_up = None
    for i in range(8000):
        t = i * DT
        p = pitch_of(d.qpos[3:7])
        if phase == "drive":
            tau = -tau_cmd if face == "front" else tau_cmd
            d.ctrl[act["wheel_L"]] = d.ctrl[act["wheel_R"]] = tau
            if abs(p) < math.radians(25):
                phase = "balance"; t_up = t; bal.x_ref = d.qpos[0]
        else:
            # rise to standing height slowly while balancing
            hh = min(0.12, h + 0.03 * (t - t_up)); hip_targets(hh)
            p, tau = bal()
        mujoco.mj_step(m, d)
        if i % 10 == 0: log.append((t, p, d.ctrl[act["wheel_L"]], np.abs(hip_torques()).max()))
    L = np.array(log)
    final_p = math.degrees(pitch_of(d.qpos[3:7]))
    return {"face": face, "start_pitch_deg": round(math.degrees(p0), 1), "t_upright_s": t_up, "final_pitch_deg": round(final_p, 1),
            "success": t_up is not None and abs(final_p) < 10, "max_hip_tau": float(L[:, 3].max())}, L


if __name__ == "__main__":
    res = {}
    res["static"] = static_torques(); print("static", json.dumps(res["static"]))
    r, Lp = push_test(); res["push_1Ns"] = r; print("push", r)
    r, _ = push_test(impulse=2.0); res["push_2Ns"] = r; print("push2", r)
    r, Lc = crouch_sweep(); res["crouch"] = r; print("crouch", r)
    r, Ld = drive_test(); res["drive"] = r; print("drive", r)
    for face in ("front", "back"):
        r, Ls = standup_test(face); res[f"standup_{face}"] = r; print("standup", r)
        np.save(os.path.join(os.path.dirname(__file__), f"standup_{face}.npy"), Ls)
    json.dump(res, open(os.path.join(os.path.dirname(__file__), "results.json"), "w"), indent=1)
    np.save(os.path.join(os.path.dirname(__file__), "push.npy"), Lp); np.save(os.path.join(os.path.dirname(__file__), "crouch.npy"), Lc)
