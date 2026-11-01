"""Find the nearest obstacle in front of the robot from an OAK-D Lite depth image.

This file is step 2 of the obstacle plan (see DEPTH.md). It has no camera code and no servo code,
so you can test it on the Mac or in the cloud: `python3 obstacle.py` runs the self-test.

Frames:
  camera: x right, y down, z forward (OAK depth image, metres).
  level:  F forward along the floor, L to the left, H height above the floor (metres).
The camera tilt "tilt" is the angle of the lens axis below the horizontal (+ = looks down).
tilt = head pitch (+ = nose down, as in the sim) + body pitch (+ = nose down, from the IMU).

Geometry (sim model roadrunner_mini.xml, CAD S67, NOT MEASURED on the robot; depth_cam.py --check
measures the camera height and tilt from the floor, use that to correct these numbers):
  torso origin is 35 mm (axle) + leg height above the floor (155 mm at the 120 mm stand),
  head pitch pivot at (+4, 0, +151) mm from the torso origin,
  lens centre about (+47.5, 0, +14.5) mm from the pivot at head pitch 0,
  front bump (fall protection, the nose of the robot) at x +130 mm, radius 12 mm.
"""
import numpy as np

AXLE_H = 0.035                    # m, wheel axle above the floor (wheel radius 35 mm)
PIVOT = np.array([0.004, 0.151])  # m, head pitch pivot (forward, up) from the torso origin
LENS = np.array([0.0475, 0.0145]) # m, lens centre from the pivot at head pitch 0 (forward, up)
NOSE_X = 0.130 + 0.012            # m, front of the front bump from the torso origin

# What counts as an obstacle (level frame, metres)
FLOOR_MARGIN = 0.03   # higher than 3 cm above the floor = obstacle (lower = floor, cable, small bump)
MARGIN_PER_M = 0.03   # + 3 cm per metre of distance (stereo noise and IMU pitch error grow with distance)
TOP_MARGIN = 0.10     # up to 10 cm above the camera (taller things also hit the head)
HALF_WIDTH = 0.15     # corridor half width: robot about 0.18 m wide + 6 cm each side
MAX_RANGE = 3.0       # m, ignore points farther than this (stereo noise grows with distance^2)
MIN_POINTS = 15       # an obstacle needs this many points (after the 4x decimation): rejects speckle
STEP = 4              # use every 4th pixel in x and y (640x400 -> 160x100)


def camera_pose(leg_height, head_pitch=0.0, body_pitch=0.0):
    """Lens height above the floor (m), lens forward offset from the torso origin (m), tilt (rad)."""
    tilt = head_pitch + body_pitch
    c, s = np.cos(tilt), np.sin(tilt)
    # head pitch + = nose down: the lens offset turns down and back
    lens_f = PIVOT[0] + LENS[0] * c + LENS[1] * s
    lens_u = PIVOT[1] - LENS[0] * s + LENS[1] * c
    return AXLE_H + leg_height + lens_u, lens_f, tilt


class Obstacles:
    def __init__(self, fx, fy, cx, cy, width, height):
        """fx, fy, cx, cy: depth camera intrinsics (pixels) for the full image width x height."""
        v, u = np.mgrid[0:height:STEP, 0:width:STEP]
        self.rx = ((u - cx) / fx).astype(np.float32)   # ray x/z for each used pixel
        self.ry = ((v - cy) / fy).astype(np.float32)   # ray y/z

    def points(self, depth_mm, cam_h, tilt):
        """Depth image (uint16 mm) -> F, L, H in the level frame (only valid pixels)."""
        z = depth_mm[::STEP, ::STEP].astype(np.float32) * 0.001
        ok = (z > 0.05) & (z < MAX_RANGE)
        self.ok = ok                                    # grid of the used pixels (for the picture in depth_cam)
        z, x, y = z[ok], self.rx[ok] * z[ok], self.ry[ok] * z[ok]
        c, s = np.cos(tilt), np.sin(tilt)
        f = z * c - y * s           # forward along the floor from the lens
        d = z * s + y * c           # down from the lens
        return f, -x, cam_h - d

    def find(self, depth_mm, leg_height, head_pitch=0.0, body_pitch=0.0, cam=None):
        """Return a dict:
          ahead: free distance (m) from the robot's nose to the nearest obstacle in the corridor
                 in front of the wheels (MAX_RANGE if none),
          left, right: the same for the sectors 15-45 deg to each side (for turning),
          points: number of valid depth points (low = camera blind: too close, too dark, covered).
        cam: optional (cam_h, lens_f, tilt) from a floor fit, in place of the model pose."""
        cam_h, lens_f, tilt = cam if cam is not None else camera_pose(leg_height, head_pitch, body_pitch)
        f, l, h = self.points(depth_mm, cam_h, tilt)
        f = f + lens_f                                   # forward from the torso origin
        # f > NOSE_X - 1 cm: nearer than the nose is the robot's own body (lid, bump), not an obstacle
        solid = (h > FLOOR_MARGIN + MARGIN_PER_M * f) & (h < cam_h + TOP_MARGIN) & (f > NOSE_X - 0.01)
        out = {"points": int(f.size)}
        ang = np.arctan2(l, f)
        self.solid = np.zeros(self.ok.shape, bool)      # grid: obstacle points in the corridor ahead (red)
        self.solid[self.ok] = solid & (np.abs(l) < HALF_WIDTH)
        self.side = np.zeros(self.ok.shape, bool)       # grid: obstacle points outside the corridor (yellow)
        self.side[self.ok] = solid & ~(np.abs(l) < HALF_WIDTH)
        for name, sel in (("ahead", np.abs(l) < HALF_WIDTH),
                          ("left", (ang > np.radians(15)) & (ang < np.radians(45))),
                          ("right", (ang < -np.radians(15)) & (ang > -np.radians(45)))):
            r = np.hypot(f, l)[solid & sel] if name != "ahead" else f[solid & sel]
            if r.size < MIN_POINTS:
                out[name] = MAX_RANGE
            else:
                out[name] = max(0.0, float(np.partition(r, MIN_POINTS - 1)[MIN_POINTS - 1]) - NOSE_X)
        return out


DEPTH_PORT = 5800     # depth_cam.py --watch -> run_brain.py: {"ahead", "left", "right", "t", ...}
POSE_PORT = 5801      # run_brain.py -> depth_cam.py: {"pitch", "leg", "head", "t"} (rad, m, rad)


class DepthGuard:
    """run_brain.py side of step 3: limits the FORWARD speed command from the depth camera result.
    Free distance ahead (from the nose): >= slow: no limit; slow..stop: limit falls in steps of `step` m/s;
    <= stop: 0. No result for `stale` s (camera program not running, crashed, USB lost): 0.
    Reverse and turn are never limited. The limit goes DOWN at once and UP only after it stayed higher
    for `up_s` s (the depth noise must not make the command jump: each new command restarts the brain's
    lag signals). Speeds snap to `step` (0.05 m/s = one w key press) for the same reason."""
    def __init__(self, speed_max, slow=0.60, stop=0.30, stale=0.3, step=0.05, up_s=0.5, port=DEPTH_PORT):
        import socket
        self.speed_max, self.slow, self.stop, self.stale, self.step, self.up_s = speed_max, slow, stop, stale, step, up_s
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.pose = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ahead, self.t_msg, self.limit, self.t_higher = None, -1e9, 0.0, None

    def raw_limit(self, ahead):
        if ahead is None or ahead <= self.stop:
            return 0.0
        if ahead >= self.slow:
            return self.speed_max
        v = self.speed_max * (ahead - self.stop) / (self.slow - self.stop)
        return np.floor(v / self.step + 1e-9) * self.step

    def update(self, now):
        """Read all waiting results, return the forward speed limit (m/s). now = time.monotonic()."""
        import json
        while True:
            try:
                msg = json.loads(self.sock.recv(4096))
            except BlockingIOError:
                break
            except (ValueError, OSError):
                continue
            self.ahead, self.t_msg = float(msg["ahead"]), now
        want = self.raw_limit(self.ahead if now - self.t_msg < self.stale else None)
        if want < self.limit:
            self.limit, self.t_higher = want, None
        elif want > self.limit:
            self.t_higher = now if self.t_higher is None else self.t_higher
            if now - self.t_higher >= self.up_s:
                self.limit, self.t_higher = want, None
        else:
            self.t_higher = None
        return self.limit

    def send_pose(self, down, leg, head):
        """Tell depth_cam.py the camera pose: body pitch (+ = nose down) from the IMU "down" vector."""
        import json
        pitch = float(np.arctan2(down[0], -down[2]))
        try:
            self.pose.sendto(json.dumps({"pitch": pitch, "leg": float(leg), "head": float(head)}).encode(),
                             ("127.0.0.1", POSE_PORT))
        except OSError:
            pass


def fit_floor(depth_mm, fx, fy, cx, cy, tries=500, seed=0):
    """Fit the floor plane in the lower half of the image (robot still, open floor in front).
    Only planes that look like a floor count: below the lens by 5-60 cm, tilt within +/-40 deg.
    (Without this rule a wall or a box face can win: 2026-11-01 test gave lens -1535 mm, tilt -90 deg.)
    Returns (cam_h m, tilt rad, inlier share) or None. Use it to check camera_pose()."""
    hgt, wid = depth_mm.shape
    v, u = np.mgrid[hgt // 2:hgt:STEP, 0:wid:STEP]
    z = depth_mm[hgt // 2::STEP, ::STEP].astype(np.float64) * 0.001
    ok = (z > 0.1) & (z < 2.5)
    if ok.sum() < 200:
        return None
    p = np.stack([(u[ok] - cx) / fx * z[ok], (v[ok] - cy) / fy * z[ok], z[ok]], 1)
    rng = np.random.default_rng(seed)

    def floor_like(n, d):
        tilt = np.arctan2(n[2], n[1])
        return 0.05 < d < 0.60 and abs(tilt) < np.radians(40)

    best, best_n, best_score = None, 0, -np.inf
    for _ in range(tries):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n = n / np.linalg.norm(n)
        if n[1] < 0:
            n = -n                                     # normal points down (camera y is down)
        if not floor_like(n, a @ n):
            continue
        r = (p - a) @ n                                # + = beyond the plane (under the floor)
        inl = np.abs(r) < 0.01
        score = inl.sum() - 2 * (r > 0.03).sum()       # the camera cannot see points under the floor
        if score > best_score:
            best, best_n, best_score = inl, inl.sum(), score
    if best is None or best_n < 0.25 * len(p):
        return None                                    # too little floor in view
    zs = p[best][:, 2]
    if np.percentile(zs, 95) - np.percentile(zs, 5) < 0.4:
        return None                                    # the floor patch is too short to give a good tilt
    q = p[best]
    centre = q.mean(0)
    n = np.linalg.svd(q - centre)[2][2]               # least-squares plane through the inliers
    if n[1] < 0:
        n = -n
    cam_h = float(centre @ n)                          # lens to floor distance
    tilt = float(np.arctan2(n[2], n[1]))               # floor normal leans toward +z when looking down
    if not floor_like(n, cam_h):
        return None
    return cam_h, tilt, best_n / len(p)


def _render(cam_h, tilt, fx, fy, cx, cy, w, h, boxes=()):
    """Self-test helper: depth image (mm) of a flat floor plus boxes (f0, f1, l0, l1, top) in metres."""
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    rx, ry = (u - cx) / fx, (v - cy) / fy
    c, s = np.cos(tilt), np.sin(tilt)
    df, dd = c - ry * s, s + ry * c          # level-frame forward and down per unit z
    z = np.full((h, w), np.inf)
    zf = np.where(dd > 1e-6, cam_h / np.maximum(dd, 1e-9), np.inf)   # floor
    z = np.minimum(z, zf)
    for f0, f1, l0, l1, top in boxes:        # front face of each box (a wall at F = f0)
        zb = f0 / np.where(df > 1e-6, df, np.nan)
        hb = cam_h - zb * dd
        lb = -rx * zb
        hit = (zb > 0) & (hb > 0) & (hb < top) & (lb > l0) & (lb < l1)
        z = np.where(hit & (zb < z), zb, z)
    z[~np.isfinite(z) | (z > 10)] = 0
    return (z * 1000).astype(np.uint16)


if __name__ == "__main__":
    # OAK-D Lite stereo at 640x400: HFOV about 73 deg -> fx about 433 px (NOT read from the camera here)
    W, H = 640, 400
    fx = fy = 433.0; cx, cy = W / 2, H / 2
    obs = Obstacles(fx, fy, cx, cy, W, H)
    leg = 0.120
    fails = 0
    for head in (0.0, np.radians(20)):
        cam_h, lens_f, tilt = camera_pose(leg, head)
        print(f"head pitch {np.degrees(head):.0f} deg: lens {cam_h * 1000:.0f} mm above the floor, "
              f"floor visible from {cam_h / np.tan(tilt + np.arctan(cy / fy)) * 100:.0f} cm in front of the lens")
        for dist, top, name in ((0.30, 0.10, "10 cm box"), (0.60, 0.10, "10 cm box"), (0.30, 0.50, "wall"),
                                (1.00, 0.50, "wall"), (0.50, 0.02, "2 cm cable")):
            f0 = NOSE_X + dist - lens_f                # box face measured from the lens
            img = _render(cam_h, tilt, fx, fy, cx, cy, W, H, [(f0, f0 + 0.2, -0.1, 0.1, top)])
            r = obs.find(img, leg, head)
            want = MAX_RANGE if top < FLOOR_MARGIN else dist
            seen = r["ahead"]
            ok = abs(seen - want) < 0.03 or (want == dist and seen == MAX_RANGE and head == 0 and top < 0.2)
            note = "" if abs(seen - want) < 0.03 else ("  (NOT SEEN: below the view)" if ok else "  FAIL")
            fails += not ok
            print(f"  {name} at {dist * 100:.0f} cm from the nose -> ahead {seen * 100:.0f} cm{note}")
        fl = fit_floor(_render(cam_h, tilt, fx, fy, cx, cy, W, H), fx, fy, cx, cy)
        print(f"  floor fit: lens {fl[0] * 1000:.0f} mm, tilt {np.degrees(fl[1]):.1f} deg (true {cam_h * 1000:.0f}, "
              f"{np.degrees(tilt):.1f})")
        fails += abs(fl[0] - cam_h) > 0.002 or abs(fl[1] - tilt) > 0.005
    # robot off (lens 141 mm, tilt 9.4 deg, as on 2026-11-01) with a box in front: the fit must find the
    # floor (box at 0.8 m) or refuse (box at 0.35 m: too little floor), never return a wall
    cam_h, tilt = 0.141, np.radians(9.4)
    for f0, must in ((0.8, True), (0.35, False)):
        img = _render(cam_h, tilt, fx, fy, cx, cy, W, H, [(f0, f0 + 0.3, -0.35, 0.35, 0.40)])
        fl = fit_floor(img, fx, fy, cx, cy)
        print(f"box {f0 * 100:.0f} cm in front of the lens, floor fit: "
              f"{'refused' if fl is None else f'{fl[0] * 1000:.0f} mm, {np.degrees(fl[1]):.1f} deg'} (true 141, 9.4)")
        if fl is None:
            fails += must
        else:
            fails += abs(fl[0] - cam_h) > 0.005 or abs(fl[1] - tilt) > np.radians(1.5)
    # DepthGuard: starts at 0, limit steps down at once, up only after 0.5 s, 0 when stale
    g = DepthGuard(0.15, port=0)
    seq = [(0.0, 1.0, 0.0), (0.6, 0.50, 0.10), (0.7, 0.40, 0.05), (0.8, 0.29, 0.0), (0.9, 1.0, 0.0),
           (1.2, 1.0, 0.0), (1.45, 1.0, 0.15), (2.0, None, 0.0)]
    for t, ahead, want in seq:
        if ahead is not None:
            g.ahead, g.t_msg = ahead, t
        got = g.update(t)
        fails += abs(got - want) > 1e-9
        print(f"guard t {t:.2f} s ahead {ahead} -> limit {got:.2f} (want {want:.2f})")
    print("SELF-TEST", "PASS" if not fails else f"FAIL ({fails})")
