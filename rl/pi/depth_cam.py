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


def make_pipeline(dai, camera=False):
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
    return p


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
    ap.add_argument("--seconds", type=float, default=10.0, help="--check: how long to count frames")
    args = ap.parse_args()
    if not (args.check or args.watch):
        ap.error("use --check or --watch")
    try:
        import depthai as dai
    except ImportError:
        raise SystemExit("depthai is not in this Python. Do DEPTH.md step 1 (install in .venv), "
                         "then run with .venv/bin/python.")
    if not dai.Device.getAllAvailableDevices():
        raise SystemExit("No OAK found. Check the USB-C cable (blue USB 3 port on the Pi) and the udev rule (DEPTH.md).")

    with dai.Device(make_pipeline(dai, camera=args.save and args.watch)) as dev:
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
        if args.floor:
            for _ in range(10):                       # let the auto exposure settle
                q.get()
            fl = fit_floor(q.get().getFrame(), fx, fy, cx, cy)
            if fl is None:
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
        print("Watching. Move your hand or a box in front of the robot. Ctrl+C = stop.")
        try:
            while True:
                fr = q.get()
                if pose_sock is not None:
                    while True:
                        try:
                            pose, t_pose = json.loads(pose_sock.recv(4096)), time.monotonic()
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
                zone = "STOP" if r["ahead"] < 0.30 else "SLOW" if r["ahead"] < 0.60 else "go  "
                print(f"\r{zone} ahead {r['ahead'] * 100:4.0f} cm  left {r['left'] * 100:4.0f}  "
                      f"right {r['right'] * 100:4.0f}  {bar:<30}", end="", flush=True)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
