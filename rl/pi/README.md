# Pi program (run the brain on the robot)

This document is written in ASD-STE100 Simplified Technical English.

These files run the trained driving brain on the Raspberry Pi 5, 50 decisions each second. They read the IMU and the servos, make the same 75 numbers as the simulator, run the brain and send the commands to the servos. They stop the robot if it tips more than 35 degrees, if the battery is low, or if a servo does not answer.

## Files
- `export_brain.py` (Mac): makes `brain_<RUN>.npz` from a trained brain and checks it. `brain_drive_020.npz` (the current driving brain: CAD S67 body with bumps, weighed mass 1754 g + bumps) is made and checked already. `run_brain.py` uses it.
- `brain.py`, `controller.py`: the brain and the observation. The simulator test and the robot use the same files.
- `sim_test.py` (Mac or cloud): tests the Pi program against the simulator. Result 2026-10-30: the observation is the same to 0.0, and the program stays up in 50 of 50 simulator games.
- `sts.py`: the Feetech bus servo protocol. `imu.py`: the LSM6DS3TR-C IMU and the "which way is down" filter.
- `robot.py`: the robot in the simulator's units. `config.json` (made by the tools below) keeps the servo IDs, directions and zeros.
- `setup_ids.py`, `calibrate.py`, `check_imu.py`: the setup tools. `run_brain.py`: the run program.

## Wiring summary
- IMU (Adafruit LSM6DS3TR-C) to the Pi 5 header: VIN to pin 1 (3.3 V), GND to pin 6, SDA to pin 3, SCL to pin 5. Lead shorter than 20 cm. Board flat on the upper deck, X arrow to the nose. The Pico stays unwired.
- Bus adapter 1 (legs and head, 7.4 V UBEC) and bus adapter 2 (wheels, battery voltage after the fuse): each adapter has a USB cable to the Pi. Set each adapter switch to USB. Do not connect the two servo buses together.

## Setup on the Pi (one time)
1. Turn on I2C: add this line to `/boot/firmware/config.txt`, then restart the Pi:
   `dtparam=i2c_arm=on,i2c_arm_baudrate=400000`
2. Copy this folder to the Pi. On the Mac, in `rl/`: `scp -r pi <user>@<pi-name>.local:~/roadrunner_pi`
3. On the Pi, make a Python environment for this folder only (do not install packages globally):
   ```
   cd ~/roadrunner_pi
   python3 -m venv .venv
   .venv/bin/pip install numpy pyserial smbus2
   ```
4. Check the IMU wires: `i2cdetect -y 1` must show `6a`.

## Steps (in this sequence)
Do each step. Do not go to the next step if a step fails.
1. **Servo IDs:** `.venv/bin/python setup_ids.py`. Connect one servo at a time. L and R are the robot's own left and right.
   IDs: head pitch 1, head yaw 2, hip front left 3, hip front right 4, hip rear left 5, hip rear right 6, wheel left 11, wheel right 12.
2. **Calibration:** wire all servos. Put the robot upright on a box, wheels free. `.venv/bin/python calibrate.py`. You need a phone level app. The servos stay limp.
3. **IMU check:** `.venv/bin/python check_imu.py`. All three checks must show OK.
4. **Wheels in the air:** robot on the box. `.venv/bin/python run_brain.py --air` (add `--bench-power` if the wheels bus is on the 6 V adapter). The legs go to the stand pose. Then tip the robot forward a little by hand: the wheels must turn forward (the robot tries to drive under its body). Tip it back: the wheels must turn backward. If a wheel turns the wrong way, stop (q) and tell Claude.
5. **Held on the floor:** `.venv/bin/python run_brain.py --held`. Hold the robot upright on its wheels. Press Enter. Keep your hands near it, but do not touch it. Press q to stop.
6. **Normal run:** `.venv/bin/python run_brain.py`. Keys: w/s speed, a/d turn, r/f height, space = stand still, q = stop.

When the robot stops, the legs go limp. Be ready to catch it.

## After each run
Each run writes `logs/run_<time>.csv`. Send the newest log to Claude. It shows the real loop time, the IMU and servo times, and the sensor readings. These replace the simulator's guesses (IMU delay 0 to 20 ms, command delay 10 to 30 ms).

## Battery
The program reads the battery voltage from the wheel servos. It warns at 10.8 V and stops at 10.5 V (3S LiPo). Disconnect the battery when the robot is off.
