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
def standup(face, p0, tau, h, head=0.0, hdrive=None):
    ps = settle(face, p0, h)
    bal = T.Balancer(); phase = 'drive'; t_up = None; minabs = abs(ps)
    for i in range(6000):
        t = i * T.DT; p = T.pitch_of(T.d.qpos[3:7])
        if phase == 'drive':
            T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = tau
            T.d.ctrl[T.act['head_pitch']] = head
            if hdrive is not None: T.hip_targets(hdrive)
            minabs = min(minabs, abs(math.degrees(p)))
            if abs(p) < math.radians(25): phase = 'balance'; t_up = t; bal.x_ref = T.d.qpos[0]
        else:
            T.d.ctrl[T.act['head_pitch']] = 0
            hh = min(0.12, h + 0.03 * (t - t_up)); T.hip_targets(hh); p, tau2 = bal()
        mujoco.mj_step(T.m, T.d)
    fp = math.degrees(T.pitch_of(T.d.qpos[3:7]))
    return ps, t_up, fp, minabs
# A: both signs from the 52 deg basin and from the 98 deg face-flat pose, tuck 0.07
for face, p0 in (('front', 55), ('front', 90), ('back', 55), ('back', 90)):
    for sgn in (1, -1):
        row = []
        for tau in (0.3, 0.5, 0.7, 0.9, 1.2):
            ps, t_up, fp, mn = standup(face, p0, sgn*tau, 0.07)
            row.append('t%.1f:%s/min%.0f/fin%.0f' % (tau, ('UP%.1fs' % t_up) if t_up else '--', mn, fp))
        print('%-5s rest %5.1f sign %+d | ' % (face, ps, sgn) + '  '.join(row))
# C: tuck height variants, sign that looked best is unknown so try both at tau 0.9
for h in (0.065, 0.08, 0.10, 0.12):
    for face, p0 in (('front', 55), ('back', 55)):
        row = []
        for sgn in (1, -1):
            ps, t_up, fp, mn = standup(face, p0, sgn*0.9, h)
            row.append('sign %+d rest %5.1f -> %s min%.0f fin%.0f' % (sgn, ps, ('UP%.1fs' % t_up) if t_up else '--', mn, fp))
        print('tuck %.3f %-5s | ' % (h, face) + ' | '.join(row))
