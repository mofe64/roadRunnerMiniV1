import os, math, sys
os.environ.setdefault('MODEL_XML', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'eggbot_v03.xml'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tests as T, mujoco, numpy as np
def settle(face, p0, h):
    pitch = math.radians(p0) if face == 'front' else math.radians(-p0)
    T.set_pose(h, pitch=pitch, z=T.WHEEL_R + 0.02); T.hip_targets(h)
    for i in range(1500):
        T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = 0; mujoco.mj_step(T.m, T.d)
    return math.degrees(T.pitch_of(T.d.qpos[3:7]))
def hip_swing(h, delta):
    tf, kf, tr, kr = T.leg_q(h)
    for s in ('L', 'R'):
        T.d.ctrl[T.act['hip_f_'+s]] = tf + delta; T.d.ctrl[T.act['hip_r_'+s]] = tr + delta
def run(face, p0, tau, h, dmax, rate=1.5, hswing=0.07, hand=40):
    sgn = 1 if face == 'front' else -1
    ps = settle(face, p0, h)
    bal = T.Balancer(kd=0.8); phase = 'swing'; t_up = None; minp = abs(ps); t0 = 0
    for i in range(8000):
        t = i * T.DT; p = T.pitch_of(T.d.qpos[3:7]); minp = min(minp, abs(math.degrees(p)))
        if phase == 'swing':
            delta = max(-abs(dmax), min(abs(dmax), np.sign(dmax)*rate*t)); hip_swing(hswing, delta)
            T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = sgn*tau
            if abs(p) < math.radians(hand): phase = 'balance'; t_up = t; bal.x_ref = T.d.qpos[0]
        else:
            hh = min(0.12, 0.07 + 0.03 * (t - t_up)); T.hip_targets(hh); p, tt = bal()
        mujoco.mj_step(T.m, T.d)
        if abs(p) > math.radians(150): break
    fp = math.degrees(T.pitch_of(T.d.qpos[3:7]))
    return ps, t_up, fp, minp, (t_up is not None and abs(fp) < 10)
print('--- from face-flat: swing the legs (delta rad) while driving wheels (tau sign relative to the 52-deg get-up sign) ---')
for face in ('front', 'back'):
    for dmax in (1.5, -1.5, 2.2, -2.2):
        row = []
        for tau in (0.0, 0.9, -0.9):
            ps, t_up, fp, minp, ok = run(face, 90, tau, 0.07, dmax)
            row.append('t%+.1f:%s(min%.0f,fin%.0f)' % (tau, 'OK' if ok else '--', minp, fp))
        print('%-5s rest %.0f swing %+.1f | ' % (face, ps, dmax) + '  '.join(row))
