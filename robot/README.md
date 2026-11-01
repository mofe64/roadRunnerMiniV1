# Robot program (runs on the Pi)

This document is written in ASD-STE100 Simplified Technical English.

These files run the trained driving brain on the Raspberry Pi 5, 50 decisions each second. They read the IMU and the servos, make the same 75 numbers as the simulator, run the brain and send the commands to the servos. They stop the robot if it tips more than 35 degrees, if the battery is low, or if a servo does not answer.

## Files
- `run_brain.py`: the run program. Options: `--pad` (gamepad), `--depth` (obstacle stop with the camera, see `DEPTH.md`), `--air`, `--held`.
- `brain.py`, `controller.py`, `filters.py`: the brain and the observation. The simulator (`../rl/`) uses the same files.
- `robot.py`: the robot in the simulator's units. `sts.py`: the Feetech bus servo protocol. `imu.py`: the LSM6DS3TR-C IMU.
- `gamepad.py`: the Xbox pad. `depth_cam.py`, `obstacle.py`: the OAK-D Lite depth camera and the obstacle and explore rules (`DEPTH.md`).
- `brains/`: the trained brains. `brain_drive_026.npz` is the driving brain (the default). `brain_drive_023.npz` is the calm backup.
- `tools/`: setup and test programs (`setup_ids.py`, `calibrate.py`, `check_imu.py`, `servo_test.py`, `head_pitch.py` and more). Run them from this folder, for example `.venv/bin/python tools/calibrate.py`.
- `config.json` (made by the setup tools, not in git): the servo IDs, directions and zeros. `logs/` (not in git): the run logs.
- On the Mac, in `../rl/`: `export_brain.py` makes `brains/brain_<RUN>.npz` from a trained brain. `sim_test.py` tests this program against the simulator.

## Wiring summary
- IMU (Adafruit LSM6DS3TR-C) to the Pi 5 header: VIN to pin 1 (3.3 V), GND to pin 6, SDA to pin 3, SCL to pin 5. Lead shorter than 20 cm. Board flat on the upper deck, X arrow to the nose. The Pico stays unwired.
- Bus adapter 1 (legs and head, 7.4 V UBEC) and bus adapter 2 (wheels, battery voltage after the fuse): each adapter has a USB cable to the Pi. Set each adapter switch to USB. Do not connect the two servo buses together.

## Setup on the Pi (one time)
1. Turn on I2C: add this line to `/boot/firmware/config.txt`, then restart the Pi:
   `dtparam=i2c_arm=on,i2c_arm_baudrate=400000`
2. Get the code on the Pi: `git clone https://github.com/mofe64/roadRunnerMiniV1.git ~/roadRunnerMiniV1`. Later, get changes with `git pull`.
3. On the Pi, make a Python environment for this folder only (do not install packages globally):
   ```
   cd ~/roadRunnerMiniV1/robot
   python3 -m venv .venv
   .venv/bin/pip install numpy pyserial smbus2 pygame depthai==2.33.0.0
   ```
4. Check the IMU wires: `i2cdetect -y 1` must show `6a`.

## Steps (in this sequence)
Do each step. Do not go to the next step if a step fails.
1. **Servo IDs:** `.venv/bin/python tools/setup_ids.py`. Connect one servo at a time. L and R are the robot's own left and right.
   IDs: head pitch 1, head yaw 2, hip front left 3, hip front right 4, hip rear left 5, hip rear right 6, wheel left 11, wheel right 12.
2. **Calibration:** wire all servos. Put the robot upright on a box, wheels free. `.venv/bin/python tools/calibrate.py`. You need a phone level app. The servos stay limp.
3. **IMU check:** `.venv/bin/python tools/check_imu.py`. All three checks must show OK.
4. **Wheels in the air:** robot on the box. `.venv/bin/python run_brain.py --air` (add `--bench-power` if the wheels bus is on the 6 V adapter). The legs go to the stand pose. Then tip the robot forward a little by hand: the wheels must turn forward (the robot tries to drive under its body). Tip it back: the wheels must turn backward. If a wheel turns the wrong way, stop (q) and tell Claude.
5. **Held on the floor:** `.venv/bin/python run_brain.py --held`. Hold the robot upright on its wheels. Press Enter. Keep your hands near it, but do not touch it. Press q to stop.
6. **Normal run:** `.venv/bin/python run_brain.py`. Keys: w/s speed, a/d turn, r/f height, space = stand still, q = stop.

When the robot stops, the legs go limp. Be ready to catch it.

## After each run
Each run writes `logs/run_<time>.csv`. Send the newest log to Claude. It shows the real loop time, the IMU and servo times, and the sensor readings. These replace the simulator's guesses (IMU delay 0 to 20 ms, command delay 10 to 30 ms).

## Battery
The program reads the battery voltage from the wheel servos. It warns at 10.8 V and stops at 10.5 V (3S LiPo). Disconnect the battery when the robot is off.

## Drive with a gamepad (Xbox One pad over Bluetooth, the same as Open Duck)

1. **Install pygame (one time):** on the Pi, in `robot/`: `.venv/bin/pip install pygame` (setup step 3 installs it already). Install it in the venv only.
2. **Pair the pad (one time):** turn the pad on. Push and hold the small pair button on the top of the pad until the logo flashes fast. On the Pi: `bluetoothctl`, then `scan on`. When the pad shows (for example "Xbox Wireless Controller"), type `pair <address>`, `trust <address>`, `connect <address>`, then `exit`. The logo stops flashing. Next time, turn on the pad and it connects by itself.
3. **Test the pad (no servos move):** `.venv/bin/python gamepad.py`. Push the left stick up: speed goes up to +0.15. Push the right stick left: turn goes up (+ = left). Press A, B and the D-pad: their names show. If a stick or a button does the wrong thing, stop (Ctrl-C) and tell Claude what the screen shows.
4. **Drive:** `.venv/bin/python run_brain.py --pad`. The script stops before any servo moves if it does not find the pad.
   - Left stick up/down: speed (forward to 0.15 m/s, back to 0.10 m/s). Right stick left/right: turn (to 1.0 rad/s).
   - D-pad up/down: taller/lower, 1 cm for each press. A: stand still. B: STOP (the legs go limp, be ready to catch the robot).
   - The speed and the turn change in steps (0.05 m/s, 0.25 rad/s), at most 5 steps a second. Release the stick and the robot slows down in steps to a stand.
   - If the pad disconnects, the robot stands still in place and keeps its balance. Press q in the terminal to stop it, or connect the pad again. After a connect, put the sticks at the centre before the robot moves again.
   - q and space in the terminal work as before. The w/s/a/d keys do nothing while the pad drives.
   - The log has one more column at the end, `pad`: 1 = pad connected, 0 = pad lost, -1 = no pad.

## Move from rl/pi to robot (one time, 2026-11-01)
The Pi program was in `rl/pi/`. It is now in `robot/`. On the Pi, do these steps one time after `git pull`:
```
cd ~/roadRunnerMiniV1
mv rl/pi/config.json robot/
mv rl/pi/logs robot/
cd robot
python3 -m venv .venv
.venv/bin/pip install numpy pyserial smbus2 pygame depthai==2.33.0.0
```
Do not move the old `rl/pi/.venv`: a Python environment does not work after a move. When the new one works, delete the old folder: `rm -rf ~/roadRunnerMiniV1/rl/pi`.
