# Road Runner Mini v1

This document is written in ASD-STE100 Simplified Technical English.

Road Runner Mini is a small wheeled biped robot. It is a hobby project and it is open source.

The robot has two legs. Each leg has a wheel at the bottom. A neural network (the "brain") keeps the balance. Reinforcement learning (RL) in a simulator trains the brain.

The robot can do these tasks:
- It keeps its balance on its wheels.
- It drives and turns with commands from a keyboard or a gamepad.
- It changes its height.
- It stops for obstacles and steers around them with its depth camera.

These tasks are not done yet:
- The robot gets up after it falls.
- The robot follows a person with its camera.

The design is similar to Upkie, Diablo, TITA and Ascento.

## Main parts
- Computer: Raspberry Pi 5.
- Camera: Luxonis OAK-D Lite (depth camera and RGB camera).
- IMU: Adafruit LSM6DS3TR-C.
- Leg and head servos: Feetech STS3215, 7.4 V version, quantity 6.
- Wheel servos: Waveshare ST3215-HS, quantity 2.
- Servo interface: Waveshare Bus Servo Adapter (A), quantity 2. Bus 1 is for the leg and head servos. Bus 2 is for the wheel servos.
- Battery: 3S LiPo, 5200 mAh. The wheel servos use battery voltage.
- UBEC 1 (8 A, set to 5.2 V): Pi power.
- UBEC 2 (8 A, set to 7.4 V): power for the leg and head servos.

The full list is in `docs/BOM.md`.

## Folders
| Folder | Contents |
|---|---|
| `cad/biped_v02/` | Fusion archives (`.f3d`) of the full robot. Each archive is one design state. The latest state is S72. |
| `cad/biped_v02/stl/` | One STL file for each printed part (18 parts). Units are mm. |
| `cad/biped_v02/plates/` | Print plates (11 Bambu Studio 3MF files). Each plate holds a group of parts in the correct print orientation, with the print settings. |
| `cad/biped_v02/coupon/` | Test coupons. Use them to make sure that your printer gives the correct fits. |
| `cad/vendor/measurements.yaml` | Dimensions of the purchased parts. |
| `docs/` | Bill of materials (`BOM.md`), design description (`DESIGN.md`), part notes and fastener locations (`PARTS.md`), print instructions (`PRINT.md`) and the assembly guide (`assembly/ASSEMBLY.html`). |
| `sim/` | MuJoCo model of the robot and test scripts. Refer to `sim/README.md`. |
| `rl/` | RL training on the Mac. Refer to `rl/README.md`. |
| `robot/` | The program that runs on the Raspberry Pi 5, the trained brains and the setup tools. Refer to `robot/README.md` and `robot/DEPTH.md`. |
| `renders/` | Images of the design. |

## Build sequence
1. Buy the parts in `docs/BOM.md`.
2. Print the parts. Refer to `docs/PRINT.md`.
3. Assemble the robot. Refer to `docs/assembly/ASSEMBLY.html`.
4. Set up the Pi and the servos. Refer to `robot/README.md`.

## Vendor CAD
The vendor CAD files are not in this repository. Get them from the vendors:
- Raspberry Pi 5: https://datasheets.raspberrypi.com/
- OAK-D Lite: https://github.com/luxonis/depthai-hardware
- Feetech STS3215 and Waveshare ST3215-HS: https://www.waveshare.com/wiki/ST3215_Servo
- Waveshare Bus Servo Adapter (A): https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)
- Adafruit LSM6DS3TR-C: https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu
- Pololu 70 x 8 mm wheel: https://www.pololu.com/product/1425

## Licence
MIT. Refer to `LICENSE`.
