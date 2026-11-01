# Obstacle avoidance with the OAK-D Lite

This document is written in ASD-STE100 Simplified Technical English.

The depth camera finds the nearest obstacle in front of the robot. A safety layer then makes the speed command smaller. The brain (drive_026) does not change. The brain still keeps the balance. The camera only changes the speed and turn commands that you give with the keys.

## The plan (5 steps)
1. **Camera on the Pi.** The Pi gets depth pictures from the OAK-D Lite, 15 each second, 640 x 400. Program: `depth_cam.py --check`.
2. **Nearest obstacle.** The Pi finds the nearest object in a corridor 30 cm wide in front of the wheels. It ignores the floor and objects lower than 3 cm. Programs: `obstacle.py` (the calculation), `depth_cam.py --watch` (live).
3. **Safety layer.** `run_brain.py --depth`: if the free distance ahead is less than 60 cm, the forward speed becomes smaller (0.10, then 0.05 m/s). At 30 cm, the forward speed becomes 0. Reverse and turn stay available. If the camera stops sending for 0.3 s, the forward speed becomes 0. The head turns 20 deg nose down in the stand move. Written 2026-11-01, see "Step 5" below.
4. **Turn to the open side.** When the robot stops for an obstacle, it turns to the side with more free distance (left or right sector, 15 to 45 deg), then it drives again. Not written yet.
5. **Later (optional).** A brain that sees depth in the simulator. Do this only after steps 1 to 4 work.

Do the steps in this sequence. Do not go to the next step if a step fails.

## Why the head must look down
The lens is about 32 cm above the floor at the 120 mm stand height. The camera sees 25 deg above and 25 deg below the lens axis. With the head level (pitch 0), the camera does not see the floor nearer than about 69 cm from the lens. Thus a 10 cm box 30 cm in front of the robot is not visible (self-test in `obstacle.py`). With the head 20 deg nose down, the camera sees the floor from about 31 cm in front of the lens (23 cm in front of the bump). Then it sees the same box.

Head pitch 20 deg nose down moves the robot's centre of mass forward by less than 1 mm. Step 3 tests this with the robot standing before it drives.

The front bump is about 8 cm in front of the lens. All distances in the programs are from the front of the bump (the nose of the robot).

## Step 1: camera on the Pi (one time setup, then a test)
The robot can be off. Only the Pi and the camera need power. No servo moves in steps 1 and 2.

1. Connect the OAK-D Lite USB-C cable to a **blue** (USB 3) port on the Pi 5.
2. Let the Pi give more USB current. The OAK needs more than the 600 mA limit that the Pi 5 uses with a power supply that is not the official one. The 5.2 V UBEC (8 A) can supply this. Add this line to `/boot/firmware/config.txt`, then restart the Pi:
   `usb_max_current_enable=1`
3. Let a normal user open the camera (one time):
   ```
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   Then disconnect the camera and connect it again.
4. Install the camera library in the Pi program environment only (not globally):
   ```
   cd ~/roadRunnerMiniV1/rl/pi
   source .venv/bin/activate
   pip install depthai==2.33.0.0
   ```
5. Put the robot on the floor. Make sure that the floor in front of it is open for 1.5 m. Turn the head down by hand a little (the head servo is limp) or leave it level.
6. Run: `python depth_cam.py --check`
7. Make sure that the result shows: `USB SUPER`, about 15 frames a second, `STEP 1 PASS`.
8. Send the output and the two files `logs/depth_<time>.png` and `logs/depth_<time>.npy` to Claude. The floor fit line gives the real lens height and tilt. Claude compares them with the model numbers.

If you see `No OAK found`: check the cable and do item 3 again. If you see `not USB 3`: use the other blue port or a different USB-C cable.

## Step 2: nearest obstacle (live)
1. Robot on the floor as in step 1.
2. Run: `python depth_cam.py --watch` (or `--watch --head-pitch 20` if the head looks 20 deg down).
3. Put a box or your foot in front of the robot. Move it nearer and farther. The line shows `go`, `SLOW` (less than 60 cm) or `STOP` (less than 30 cm).
4. Do these checks and tell Claude the results:
   - Open floor only: the line must show `go` all the time.
   - A box 10 cm high at 50 cm: about 50 cm.
   - A box at the left side: the `left` number becomes small, not `ahead`.
   - A cable on the floor: `go` (lower than 3 cm).

## Step 5: the robot stops for an obstacle
Two programs run at the same time on the Pi. Open two SSH terminals (`ssh mofe@roadrunner-mini.local`), and in each do `cd ~/roadRunnerMiniV1/rl/pi && source .venv/bin/activate`.

1. Terminal 1: `python depth_cam.py --watch --robot`. It shows `waiting for run_brain.py --depth` until step 2 starts.
2. Terminal 2: `python run_brain.py --depth` (add `--pad` for the gamepad). Do the usual start. In the stand move, the head turns 20 deg nose down.
3. Test A (stand only, 30 s): do not drive. The robot must stand as well as before. Terminal 1 must show `go` with nothing in front.
4. Test B (drive to a wall): put the robot 1.5 m from a wall or a big box, nose to the wall. Press `w` one time (0.05 m/s). The robot must stop about 30 cm from the wall. Terminal 2 shows `(camera: .. cm)` when the camera limits the speed.
5. Press `s` to drive back. Press `q` to stop.

If the robot does not stop: press `q` (or B on the pad) at once. Send the two terminal outputs and the newest `logs/run_<time>.csv` to Claude (columns `ahead` and `limit` are at the end).

## Files
- `obstacle.py`: the calculation (no camera, no servos). `python obstacle.py` runs the self-test on the Mac or the Pi.
- `depth_cam.py`: the camera program (steps 1, 2 and 5).
- `run_brain.py --depth`: the safety layer (`DepthGuard` in `obstacle.py`). run_brain sends the body pitch, leg height and head pitch to depth_cam on port 5801. depth_cam sends the free distance to run_brain on port 5800.
- The numbers that are not measured yet: lens position (from the sim model, CAD S67), the 30 cm wide corridor, the 3 cm floor limit. Step 1 measures the lens height and tilt.
