import os, math, sys
os.environ.setdefault('MODEL_XML', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'roadrunner_mini.xml'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tests as T, mujoco, numpy as np
def settle(face, p0, h):
    pitch = math.radians(p0) if face == 'front' else math.radians(-p0)
    T.set_pose(h, pitch=pitch, z=T.WHEEL_R + 0.02); T.hip_targets(h)
    for i in range(1500):
        T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = 0; mujoco.mj_step(T.m, T.d)
    return math.degrees(T.pitch_of(T.d.qpos[3:7]))
def run(face, p0, tau, h, hand=25, brake=0.0, kd=0.35, hdrive=None, head=0.0):
    sgn = 1 if face == 'front' else -1
    ps = settle(face, p0, h)
    bal = T.Balancer(kd=kd); phase = 'drive'; t_up = None; t_b = None; maxw = 0
    for i in range(6000):
        t = i * T.DT; p = T.pitch_of(T.d.qpos[3:7])
        if phase == 'drive':
            T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = sgn*tau
            T.d.ctrl[T.act['head_pitch']] = head
            if hdrive is not None: T.hip_targets(hdrive)
            if abs(p) < math.radians(hand): phase = 'brake'; t_b = t
        elif phase == 'brake':
            T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = -sgn*tau
            if t - t_b > brake: phase = 'balance'; t_up = t; bal.x_ref = T.d.qpos[0]
        else:
            T.d.ctrl[T.act['head_pitch']] = 0
            hh = min(0.12, max(h, 0.07) + 0.03 * (t - t_up)); T.hip_targets(hh); p, tt = bal()
        mujoco.mj_step(T.m, T.d)
        if abs(p) > math.radians(150): break
    fp = math.degrees(T.pitch_of(T.d.qpos[3:7]))
    ok = t_up is not None and abs(fp) < 10
    return ps, t_up, fp, ok
print('--- handover from the 52 deg rest (tuck 0.07) ---')
for face in ('front', 'back'):
    for tau in (0.7, 0.9, 1.2):
        row = []
        for hand, brake, kd in ((25, 0, 0.35), (40, 0, 0.35), (45, 0.05, 0.35), (45, 0.1, 0.35), (40, 0, 0.8), (50, 0.08, 0.8)):
            ps, t_up, fp, ok = run(face, 55, tau, 0.07, hand, brake, kd)
            row.append('h%d/b%.2f/kd%.2f:%s' % (hand, brake, kd, 'OK' if ok else ('fin%.0f' % fp)))
        print('%-5s tau %.1f | ' % (face, tau) + '  '.join(row))
print('--- from face-flat (98 deg): extend legs while driving ---')
for face in ('front', 'back'):
    for hdrive in (0.10, 0.12, 0.15):
        row = []
        for tau in (0.6, 0.9, 1.2):
            for sg in (1, -1):
                ps, t_up, fp, ok = run(face, 90, sg*tau, 0.07, 40, 0.05, 0.8, hdrive=hdrive)
                row.append('t%+.1f:%s' % (sg*tau, 'OK' if ok else ('fin%.0f' % fp)))
        print('%-5s legs->%.2f | ' % (face, hdrive) + '  '.join(row))
