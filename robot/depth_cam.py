"""OAK-D Lite depth camera on the Pi 5: step 1 and step 2 of the obstacle plan (see DEPTH.md).

  python3 depth_cam.py --check        step 1: USB speed, frame rate, floor fit, one depth picture
  python3 depth_cam.py --watch        step 2: print the free distance ahead, left and right, 15 times a second
  python3 depth_cam.py --watch --head-pitch 20   the same, head turned 20 deg nose down
  python3 depth_cam.py --watch --floor           robot off (not standing): measure the camera height and tilt
                                                  from the floor at the start and use them
  python3 depth_cam.py --watch --floor --save    the same, and every 2 s save the camera picture with the
                                                  obstacle points in red (logs/watch_cam.png, watch_depth.png/.npy)
  python3 depth_cam.py --watch --robot           with run_brain.py --depth (second terminal): run_brain sends the
                                                  body pitch, leg height and head pitch 50 times a second
  python3 depth_cam.py --watch --robot --record  the same, and record for the 3D colour map: colour pictures
                                                  (JPEG) + depth (mm) 3 times a second and the wheel/gyro
                                                  numbers 50 times a second, into logs/rec_<time>/

This program does not move a servo. The robot can be off (only the Pi and the camera need power).
Put the robot on the floor with open floor in front of it (at least 1.5 m).
--watch also sends each result to 127.0.0.1 port 5800 (UDP, JSON) for the run_brain.py --depth safety layer.
Needs: depthai 2.33.0.0 in .venv (see DEPTH.md). Depth: 640 x 400, 15 frames a second, extended disparity
(the near limit is about 20 cm from the lens; without it, about 35 cm).
"""
import argparse, json, os, socket, sys, time, zlib, struct, warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)   # depthai 2.33: HIGH_DENSITY name
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
from obstacle import Obstacles, camera_pose, fit_floor, DEPTH_PORT, POSE_PORT

W, H, FPS = 640, 400, 15
RGB_W, RGB_H = 640, 360       # colour camera (CAM_A, 1080p scaled 1/3) for --record
REC_HZ = 3.0                  # --record: pictures per second (depth 512 kB each before compression)


def make_pipeline(dai, camera=False, colour=False):
    p = dai.Pipeline()
    left, right = p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
    for cam, sock in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        cam.setBoardSocket(sock)
        cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam.setFps(FPS)
    st = p.create(dai.node.StereoDepth)
    st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    st.setDepthAlign(dai.CameraBoardSocket.CAM_C)     # depth in the right camera's view (its intrinsics below)
    st.setLeftRightCheck(True)                        # removes wrong depth at object edges
    st.setExtendedDisparity(True)                     # sees nearer (about 20 cm instead of 35 cm)
    st.setSubpixel(False)
    st.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
    left.out.link(st.left); right.out.link(st.right)
    xo = p.create(dai.node.XLinkOut); xo.setStreamName("depth")
    st.depth.link(xo.input)
    if camera:                                        # the right camera picture, same view as the depth
        xr = p.create(dai.node.XLinkOut); xr.setStreamName("rect")
        st.rectifiedRight.link(xr.input)
    if colour:                                        # --record: colour camera, JPEG made on the camera
        rgb = p.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setIspScale(1, 3)                         # 1920x1080 -> 640x360
        rgb.setVideoSize(RGB_W, RGB_H)
        rgb.setFps(FPS)
        enc = p.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.MJPEG)
        enc.setQuality(85)
        rgb.video.link(enc.input)
        xj = p.create(dai.node.XLinkOut); xj.setStreamName("jpg")
        enc.bitstream.link(xj.input)
    return p


class Recorder:
    """--record: saves into logs/rec_<time>/ for the 3D map (built later on the Mac):
      calib.json      camera intrinsics (depth = right camera CAM_C 640x400, colour = CAM_A 640x360) and the
                      CAM_C -> CAM_A extrinsics (depthai units: cm)
      rgb/NNNNN.jpg   colour picture, depth/NNNNN.npz (uint16 mm, compressed), REC_HZ times a second
      frames.csv      idx, wall time of the depth frame, depth and colour camera time stamps, pose from run_brain
      odom.csv        every run_brain pose message (50 a second): wall time, pitch, wheel speeds, gyro, command
    Files are written in a second thread: the obstacle loop does not wait for the SD card."""
    def __init__(self, dev, dai):
        import threading, queue
        self.dir = os.path.join(HERE, "logs", "rec_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(os.path.join(self.dir, "rgb")); os.makedirs(os.path.join(self.dir, "depth"))
        cal = dev.readCalibration()
        C, A = dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_A
        with open(os.path.join(self.dir, "calib.json"), "w") as f:
            json.dump({"depth_K": cal.getCameraIntrinsics(C, W, H), "depth_size": [W, H],
                       "rgb_K": cal.getCameraIntrinsics(A, RGB_W, RGB_H), "rgb_size": [RGB_W, RGB_H],
                       "depth_to_rgb_cm": cal.getCameraExtrinsics(C, A), "rec_hz": REC_HZ}, f, indent=1)
        self.frames = open(os.path.join(self.dir, "frames.csv"), "w")
        self.frames.write("idx,wall_t,depth_ts,rgb_ts,pitch,leg,head\n")
        self.odom = open(os.path.join(self.dir, "odom.csv"), "w")
        self.odom_keys = ["pitch", "leg", "head", "wl", "wr", "gx", "gy", "gz", "v", "w", "r"]
        self.odom.write("wall_t," + ",".join(self.odom_keys) + "\n")
        self.q, self.n, self.t_last, self.jpg = queue.Queue(maxsize=30), 0, 0.0, None
        self.dropped = 0
        self.th = threading.Thread(target=self._writer, daemon=True); self.th.start()

    def pose(self, msg, wall_t):
        self.odom.write(f"{wall_t:.4f}," + ",".join(f"{float(msg.get(k, float('nan'))):.5g}" for k in self.odom_keys) + "\n")

    def frame(self, depth_mm, depth_ts, wall_t, jpg_pkt, pose):
        """Call for each depth frame; jpg_pkt = newest colour packet (or None). Saves REC_HZ times a second."""
        if jpg_pkt is not None:
            self.jpg = jpg_pkt
        if self.jpg is None or wall_t - self.t_last < 1.0 / REC_HZ:
            return
        self.t_last = wall_t
        p = pose or {}
        self.frames.write(f"{self.n},{wall_t:.4f},{depth_ts:.4f},{self.jpg.getTimestamp().total_seconds():.4f},"
                          f"{p.get('pitch', float('nan')):.5g},{p.get('leg', float('nan')):.5g},"
                          f"{p.get('head', float('nan')):.5g}\n")
        try:
            self.q.put_nowait((self.n, depth_mm.copy(), bytes(self.jpg.getData())))
        except Exception:
            self.dropped += 1
        self.n += 1

    def _writer(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            n, d, j = item
            with open(os.path.join(self.dir, "rgb", f"{n:05d}.jpg"), "wb") as f:
                f.write(j)
            np.savez_compressed(os.path.join(self.dir, "depth", f"{n:05d}.npz"), depth=d)

    def close(self):
        self.q.put(None); self.th.join(timeout=10)
        self.frames.close(); self.odom.close()
        print(f"Recorded {self.n} pictures (dropped {self.dropped}) in {self.dir}")


def write_png(path, img):
    """img: (h, w) uint8 grey or (h, w, 3) uint8 colour. No extra packages."""
    g = np.ascontiguousarray(img)
    colour = 2 if g.ndim == 3 else 0
    raw = b"".join(b"\x00" + g[r].tobytes() for r in range(g.shape[0]))
    def chunk(t, data):
        return struct.pack(">I", len(data)) + t + data + struct.pack(">I", zlib.crc32(t + data) & 0xffffffff)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", g.shape[1], g.shape[0], 8, colour, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def overlay(grey, obs):
    """Camera picture in grey, obstacle points in the corridor RED, other obstacle points YELLOW."""
    from obstacle import STEP
    rgb = np.repeat(grey[:, :, None], 3, axis=2).copy()
    h, w = grey.shape
    for mask, colour in ((obs.side, (255, 210, 0)), (obs.solid, (255, 0, 0))):
        big = np.repeat(np.repeat(mask, STEP, 0), STEP, 1)[:h, :w]
        rgb[big] = colour
    return rgb


def save_png(path, depth_mm, far=3000):
    """Grey depth picture: near = white, far or no depth = black."""
    d = depth_mm.astype(np.float32)
    write_png(path, np.where(d > 0, 255 * (1 - np.clip(d / far, 0, 1)), 0).astype(np.uint8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--leg", type=float, default=120, help="leg height in mm (65-140, stand 120)")
    ap.add_argument("--head-pitch", type=float, default=0.0, help="head pitch in deg, + = nose down")
    ap.add_argument("--floor", action="store_true", help="--watch: use the camera height and tilt from a floor fit "
                    "at the start (open floor in front), not the standing-robot model")
    ap.add_argument("--robot", action="store_true", help="--watch: take the camera pose from run_brain.py --depth; "
                    "no pose for 0.3 s = send nothing (run_brain then stops forward driving)")
    ap.add_argument("--save", action="store_true", help="--watch: every 2 s save logs/watch_cam.png (camera picture, "
                    "obstacle points red), logs/watch_depth.png and logs/watch_depth.npy")
    ap.add_argument("--record", action="store_true", help="--watch --robot: record colour + depth + odometry "
                    "for the 3D map into logs/rec_<time>/")
    ap.add_argument("--seconds", type=float, default=10.0, help="--check: how long to count frames")
    args = ap.parse_args()
    if not (args.check or args.watch):
        ap.error("use --check or --watch")
    if args.record and not (args.watch and args.robot):
        ap.error("--record needs --watch --robot (the odometry comes from run_brain.py --depth)")
    try:
        import depthai as dai
    except ImportError:
        raise SystemExit("depthai is not in this Python. Do DEPTH.md step 1 (install in .venv), "
                         "then run with .venv/bin/python.")
    if not dai.Device.getAllAvailableDevices():
        raise SystemExit("No OAK found. Check the USB-C cable (blue USB 3 port on the Pi) and the udev rule (DEPTH.md).")

    with dai.Device(make_pipeline(dai, camera=args.save and args.watch, colour=args.record)) as dev:
        usb = dev.getUsbSpeed().name
        print(f"OAK connected: {dev.getDeviceName()}  USB {usb}")
        if usb not in ("SUPER", "SUPER_PLUS"):
            print("WARNING: not USB 3. Use the blue USB port and a USB 3 cable. Depth at USB 2 is slow.")
        K = np.array(dev.readCalibration().getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, W, H))
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        print(f"right camera: fx {fx:.1f} fy {fy:.1f} cx {cx:.1f} cy {cy:.1f} px, "
              f"baseline {dev.readCalibration().getBaselineDistance() :.2f} cm")
        q = dev.getOutputQueue("depth", maxSize=1, blocking=False)
        obs = Obstacles(fx, fy, cx, cy, W, H)
        leg = args.leg / 1000.0
        head = np.radians(args.head_pitch)

        if args.check:
            for _ in range(10):                       # let the auto exposure settle
                q.get()
            n, lag, t0, last = 0, [], time.perf_counter(), None
            while time.perf_counter() - t0 < args.seconds:
                fr = q.get()
                lag.append((dai.Clock.now() - fr.getTimestamp()).total_seconds() * 1000)
                last = fr.getFrame(); n += 1
            fps = n / (time.perf_counter() - t0)
            valid = (last > 0).mean() * 100
            print(f"frames: {fps:.1f} per second (want about {FPS}), age when received {np.median(lag):.0f} ms, "
                  f"valid depth {valid:.0f} % of the picture")
            model_h, _, model_t = camera_pose(leg, head)
            fl = fit_floor(last, fx, fy, cx, cy)
            if fl is None:
                print("floor fit: not enough floor in view (point the head at open floor, or use --head-pitch)")
            else:
                print(f"floor fit: lens {fl[0] * 1000:.0f} mm above the floor, tilt {np.degrees(fl[1]):+.1f} deg "
                      f"({fl[2] * 100:.0f} % of the lower half is floor)")
                print(f"model:     lens {model_h * 1000:.0f} mm, tilt {np.degrees(model_t):+.1f} deg "
                      f"(leg {args.leg:.0f} mm, head pitch {args.head_pitch:+.0f} deg, body level)")
            os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            png = os.path.join(HERE, "logs", f"depth_{stamp}.png")
            save_png(png, last)
            np.save(png.replace(".png", ".npy"), last)
            r = obs.find(last, leg, head)
            print(f"now: ahead {r['ahead'] * 100:.0f} cm, left {r['left'] * 100:.0f} cm, right {r['right'] * 100:.0f} cm")
            print(f"Saved {png} and the .npy (raw depth, mm). Send both to Claude.")
            ok = fps > FPS * 0.8 and usb in ("SUPER", "SUPER_PLUS") and valid > 50
            print("STEP 1", "PASS" if ok else "FAIL")
            return

        cam = None
        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        if args.floor:
            for _ in range(10):                       # let the auto exposure settle
                q.get()
            d0 = q.get().getFrame()
            fl = fit_floor(d0, fx, fy, cx, cy)
            if fl is None:
                if args.save:                          # keep what the camera saw, so the cause can be found
                    save_png(os.path.join(HERE, "logs", "watch_depth.png"), d0)
                    np.save(os.path.join(HERE, "logs", "watch_depth.npy"), d0)
                    rf = dev.getOutputQueue("rect", maxSize=1, blocking=True).get()
                    write_png(os.path.join(HERE, "logs", "watch_cam.png"),
                              np.asarray(rf.getFrame(), dtype=np.uint8).reshape(rf.getHeight(), rf.getWidth()))
                    print("Saved logs/watch_cam.png, watch_depth.png and watch_depth.npy. Send them to Claude.")
                raise SystemExit("Floor fit failed: put open floor in front of the camera and start again.")
            cam = (fl[0], camera_pose(leg, head)[1], fl[1])
            print(f"Using the floor fit: lens {fl[0] * 1000:.0f} mm above the floor, tilt {np.degrees(fl[1]):+.1f} deg")
        qr = dev.getOutputQueue("rect", maxSize=1, blocking=False) if args.save else None
        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        t_save = 0.0
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pose_sock, pose, t_pose = None, None, -1e9
        if args.robot:
            pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            pose_sock.bind(("127.0.0.1", POSE_PORT))
            pose_sock.setblocking(False)
            print("Robot mode: start run_brain.py --depth in the other terminal.")
        rec = Recorder(dev, dai) if args.record else None
        qj = dev.getOutputQueue("jpg", maxSize=2, blocking=False) if rec is not None else None
        if rec is not None:
            print(f"Recording into {rec.dir}")
        print("Watching. Move your hand or a box in front of the robot. Ctrl+C = stop.")
        try:
            while True:
                fr = q.get()
                if pose_sock is not None:
                    while True:
                        try:
                            pose, t_pose = json.loads(pose_sock.recv(4096)), time.monotonic()
                            if rec is not None:
                                rec.pose(pose, time.time())
                        except BlockingIOError:
                            break
                        except (ValueError, OSError):
                            continue
                    if pose is None or time.monotonic() - t_pose > 0.3:
                        print("\rwaiting for run_brain.py --depth (no robot pose)            ", end="", flush=True)
                        continue                      # send nothing: run_brain allows no forward driving
                    r = obs.find(fr.getFrame(), pose["leg"], pose["head"], pose["pitch"])
                else:
                    r = obs.find(fr.getFrame(), leg, head, cam=cam)
                r["t"] = time.time()
                r["age_ms"] = (dai.Clock.now() - fr.getTimestamp()).total_seconds() * 1000
                if rec is not None:
                    rec.frame(fr.getFrame(), fr.getTimestamp().total_seconds(), r["t"] - r["age_ms"] / 1000,
                              qj.tryGet(), pose)
                sock.sendto(json.dumps(r).encode(), ("127.0.0.1", DEPTH_PORT))
                if qr is not None and time.monotonic() - t_save > 2.0:
                    rf = qr.tryGet()
                    if rf is not None:
                        t_save = time.monotonic()
                        d = fr.getFrame()
                        grey = np.asarray(rf.getFrame(), dtype=np.uint8).reshape(rf.getHeight(), rf.getWidth())
                        write_png(os.path.join(HERE, "logs", "watch_cam.png"), overlay(grey, obs))
                        save_png(os.path.join(HERE, "logs", "watch_depth.png"), d)
                        np.save(os.path.join(HERE, "logs", "watch_depth.npy"), d)
                bar = "#" * int(min(r["ahead"], 1.5) * 20)
                zone = "STOP" if r["ahead"] < 0.20 else "SLOW" if r["ahead"] < 0.40 else "go  "
                print(f"\r{zone} ahead {r['ahead'] * 100:4.0f} cm  left {r['left'] * 100:4.0f}  "
                      f"right {r['right'] * 100:4.0f}  {bar:<30}", end="", flush=True)
        except KeyboardInterrupt:
            print()
        finally:
            if rec is not None:
                rec.close()


if __name__ == "__main__":
    main()
