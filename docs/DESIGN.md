# Road Runner wheeled biped: design notes (v0.2, 2026-10-27)

This document is written in ASD-STE100 Simplified Technical English.

The robot is a wheeled biped. It is a hobby project and it is open source. It must keep its balance, move around obstacles, get up after a fall and follow a subject with its camera. The design is similar to Upkie, Diablo, TITA and Ascento.

## Layout
- Axes: X points forward, Y points left and Z points up. The origin is at the middle of the four hip axes (the "hip frame").
- Stand height: at the normal stand height of 120 mm, the wheels touch the ground at z = -155 mm.
- Legs: each side has a symmetrical five-bar leg. Both hip servos of each leg are in the body. The upper link is 60 mm. The lower link is 100 mm. The hip axes are 50 mm apart.
- Leg range: the hip-to-axle height is 65 mm to 150 mm. The normal stand height is 120 mm. The tucked height is 65 mm to 70 mm.
- Body: the body has a printed chassis and an upper deck.
  - The chassis holds the servo spine, the battery cradle, the lower deck for the Pi 5 and the Pico, and four columns.
  - The upper deck holds the UBECs, the bus servo adapter and the IMU.
  - A shell with rounded corners covers the body. The shell is 150 x 100 x 115 mm. It has a tray and a lid. The corner radius is 16 mm and the walls are 2 mm thick.
- Neck: the neck is a pan-tilt unit.
  - The yaw STS3215 servo stands on the lid. M2 countersunk screws attach it from the inner side of the lid.
  - A printed U-bracket on the yaw servo horn holds the pitch STS3215 servo.
  - The head turns on the pitch servo horn and on an idler.
- Head: the head is a printed box, 95.5 x 100 x 65 mm. The bottom of the box is open. The OAK-D Lite sits on ledges in the head. Two M4 screws attach the OAK-D Lite through its rear holes, which are 75 mm apart. The visor opening is at the front.
- Wheels: two Waveshare ST3215-HS bus servos in motor mode turn Pololu 70 x 8 mm wheels.
- Power: a 3S 5200 mAh LiPo battery supplies the power through a 10 A blade fuse. One UBEC supplies 7.4 V to the servo bus. One UBEC supplies 5.2 V to the Pi 5.

## Verified data
Source: Fusion design "Biped v0.1", saved state v0.2 S47 (version 60). Exports: `cad/biped_v02/biped_v02_S47.f3d` and `cad/biped_v02/biped_v02_S47.step.zip` (extract the zip to get the STEP file).

- Mass: 1493 g in total.
  - Body: 1000 g.
  - Head: 203 g. The ribs are fused to the roof and have side webs (S43).
  - Neck: 66 g.
  - Each leg: 40 g.
  - Each wheel with its servo: 72 g.
- Centre of mass: 27 mm above the hip axes and 1 mm forward of them.
- Head pivot: the pitch pivot is 139 mm above the hips.
- Head movement (set these limits in the software):
  - Nose up: 30 degrees at all yaw angles.
  - Nose down: 50 degrees between -45 and +45 degrees of yaw. 40 degrees at +/-90 degrees of yaw.
- Clearances:
  - The legs do not touch the shell in the full 65 mm to 150 mm range. The smallest gap is 3.8 mm.
  - No parts intersect.
  - The model has a keep-out block for the plug and cable of each connector.
- PLA fits (from the printed test coupons, 2026-10-27):

| Feature | Size (mm) |
|---|---|
| M2 clearance hole | 2.5 |
| M3 clearance hole | 3.5 |
| M3 heat-set insert hole | 4.0 |
| M2 self-tap pilot hole (Pi and Pico) | 1.8 |
| Servo horn pocket | 20.2 |
| Lid and tray tab clearance | 0.2 |
| Tray servo pocket | 46.2 x 25.4 |

Do not use M2.5 screws. The design has no M2.5 holes.

## Simulation (MuJoCo, `sim/eggbot_v03.xml`)
- The robot keeps its balance at all heights.
- The robot stays upright after a push of 1.5 N s.
- The robot crouches from 150 mm to 70 mm.
- The maximum speed is 0.36 m/s.
- Get-up from the sit position (the robot rests on its chin and wheels, at 52 degrees): the wheels lift the robot. This needs a wheel torque of 0.9 Nm to 1.2 Nm. Turn the wheels in the direction of the chin. The controller must catch the robot at the end of the lift.
- Get-up from flat face-down: the simple scripts cannot do this. The RL policy must learn it.

## Open items
- The battery door cover and the TPU chin and tail skids are not in the model.
- The gap between each hip servo plug and the tray corner is approximately 11.5 mm. This gap is sufficient if the socket is at the centre of the servo end face.
- The OAK-D Lite needs a right-angle USB-C cable for the full nose-down range.
- The side of the Pi 5 power plug is an assumption. Examine it when you install the Pi.
- For print orientation and supports, refer to `PRINT.md`. The coupon tests are complete and the fits are in the model.

## S47 (2026-10-27 21:30): check on a real servo
Photos of a real STS3215 show that the cable sockets are on the rear face, at the middle of the length. The plugs are flat when they are fully in.

These pockets are now longer in the direction of the shaft end:
- Chassis: x = 36 to 55.2 mm, z = -10.8 to +10.8 mm, 3.1 mm deep.
- Bracket rear arm window: z = 152 to 169 mm.
- Lid relief: x = -30 to 0 mm.

Use the M2 x 6 pointed screws from the servo bags for all servo rear faces. New masses: chassis 127 g, bracket 7 g, lid 56 g.
