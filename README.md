# Road Runner Mini v1

Road Runner Mini is a small wheeled biped robot. It is a hobby project and it is open source.

The robot has two legs. Each leg has a wheel at the bottom. The robot can do these tasks:
- It keeps its balance on its wheels.
- It moves around obstacles.
- It gets up after it falls.
- It follows a person with its camera and records video.

The design is similar to Upkie, Diablo, TITA and Ascento.

## Main parts
- Computer: Raspberry Pi 5.
- Camera: Luxonis OAK-D Lite (depth camera, RGB camera and IMU).
- Leg and head servos: Feetech STS3215 (7.4 V), quantity 6.
- Wheel servos: Waveshare ST3215-HS, quantity 2.
- Battery: 3S LiPo, 5200 mAh.

## Folders
| Folder | Contents |
|---|---|
| `cad/biped_v02/` | Fusion archive (`.f3d`) and STEP file (in a `.zip`) of the full robot, design state S47. |
| `cad/biped_v02/stl/` | One STL file for each printed part (16 parts). Units are mm. |
| `cad/biped_v02/plates/` | Print plates. Each plate holds a group of parts in the correct print orientation. |
| `cad/biped_v02/coupon/` | Test coupons. Use them to make sure that your printer gives the correct fits. |
| `cad/vendor/measurements.yaml` | Dimensions that we measured on the real parts. |
| `docs/` | Design notes (`DESIGN.md`), parts list (`PARTS.md`) and print instructions (`PRINT.md`). |
| `sim/` | MuJoCo model of the robot and test scripts. |
| `rl/` | Reinforcement learning (RL) work area. |
| `renders/` | Images of the design. |

## STEP file
The STEP file is 112 MB. This is too large for GitHub, thus it is in `cad/biped_v02/biped_v02_S47.step.zip`. Extract the zip file before you open the STEP file.

## Vendor CAD
The vendor CAD files are not in this repository. Get them from the vendors:
- Raspberry Pi 5: https://datasheets.raspberrypi.com/
- OAK-D Lite: https://github.com/luxonis/depthai-hardware
- Feetech STS3215 and Waveshare ST3215-HS: https://www.waveshare.com/wiki/ST3215_Servo
- Pololu 70 x 8 mm wheel: https://www.pololu.com/product/1425

## Licence
MIT. Refer to `LICENSE`.
