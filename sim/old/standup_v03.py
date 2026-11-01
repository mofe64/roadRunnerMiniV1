import os, math, sys
os.environ.setdefault('MODEL_XML', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'roadrunner_mini.xml'))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # sim/
import tests as T, mujoco, numpy as np
print('total mass %.3f' % T.m.body_subtreemass[1])
def settle(face, p0, h=0.07):
    pitch = math.radians(p0) if face == 'front' else math.radians(-p0)
    T.set_pose(h, pitch=pitch, z=T.WHEEL_R + 0.02); T.hip_targets(h)
    for i in range(1500):
        T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = 0; mujoco.mj_step(T.m, T.d)
    return math.degrees(T.pitch_of(T.d.qpos[3:7]))
def standup(face, p0, tau_cmd, h=0.07, head=0.0):
    ps = settle(face, p0, h)
    bal = T.Balancer(); phase = 'drive'; t_up = None; maxhip = 0
    for i in range(8000):
        t = i * T.DT; p = T.pitch_of(T.d.qpos[3:7])
        if phase == 'drive':
            tau = -tau_cmd if face == 'front' else tau_cmd
            T.d.ctrl[T.act['wheel_L']] = T.d.ctrl[T.act['wheel_R']] = tau
            T.d.ctrl[T.act['head_pitch']] = head
            if abs(p) < math.radians(25): phase = 'balance'; t_up = t; bal.x_ref = T.d.qpos[0]
        else:
            T.d.ctrl[T.act['head_pitch']] = 0
            hh = min(0.12, h + 0.03 * (t - t_up)); T.hip_targets(hh); p, tau = bal()
        mujoco.mj_step(T.m, T.d); maxhip = max(maxhip, np.abs(T.hip_torques()).max())
    fp = math.degrees(T.pitch_of(T.d.qpos[3:7]))
    return ps, t_up, fp, maxhip
print('rest poses:')
for face in ('front', 'back'):
    print(' ', face, ' '.join('%d->%.0f' % (p0, settle(face, p0)) for p0 in (35, 45, 55, 65, 75, 90)))
for face, p0 in (('front', 55), ('back', 55)):
    for head in (0.0, -0.8, 0.8):
        for tau in (0.4, 0.6, 0.8, 1.0, 1.2):
            ps, t_up, fp, mh = standup(face, p0, tau, head=head)
            ok = t_up is not None and abs(fp) < 10
            print('%-5s settled %5.1f head %+.1f tau %.1f -> up %-5s final %6.1f hip %.2f %s' % (face, ps, head, tau, ('%.2f' % t_up) if t_up else '-', fp, mh, 'OK' if ok else 'FAIL'))
