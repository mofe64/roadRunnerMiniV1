import sys, numpy as np
sys.path.insert(0, "../rl")
from getup_env import GetupEnv
e = GetupEnv(seed=7, realism=False); e.difficulty=1.0; m, d = e.m, e.d
def dxz():
    R = d.xmat[e.torso].reshape(3, 3); hx = np.array([R[0, 0], R[1, 0]]); hx /= np.linalg.norm(hx)
    com = d.subtree_com[e.torso]; ax = np.mean(d.xpos[e.axles], axis=0)
    return hx @ (com[:2] - ax[:2])
for face, p in (("front", 0.15), ("back", -0.15)):
    print(face)
    for ln in (-1, -0.5, 0, 0.5, 1):
        row = []
        for sw in np.linspace(-1, 1, 9):
            e.reset(pitch=p)
            for k in range(75):   # slow ramp of both over 1.5 s
                f = min(1, (k + 1) / 50)
                e.step(np.array([-1 + (ln + 1) * f, sw * f, 0, 0]))
            row.append(f"{dxz()*1000:+4.0f}/{e.start_pitch*0+np.degrees(np.arccos(-e.true_gravity()[2])):3.0f}{'p' if e.body_on_floor() else 'W'}")
        print(f"  length {ln:+.1f}: " + " ".join(row))
print("columns: swing -1 .. +1; cell = COM ahead of axle (mm, along heading) / tilt deg, p = propped, W = wheels only")
