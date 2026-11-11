# Part notes and fastener locations

This document is written in ASD-STE100 Simplified Technical English.

The quantities and the links are in `BOM.md`. This document tells you where each part goes and which fastener holds it.

## Electronics

| Part | Location | Notes |
|---|---|---|
| Raspberry Pi 5 with active cooler | Lower deck (chassis upper), above the battery | 4 x M2 x 8. |
| Luxonis OAK-D Lite | In the head, on 2 ledges | 2 x M4 x 10 with washers, into the rear holes of the camera. |
| Adafruit LSM6DS3TR-C IMU | On the IMU plate, at the front of the lower deck | 2 x M2 x 6. The X arrow points to the nose. |
| IMU plate (printed) | On the 4 posts at the front of the lower deck | 4 x M2 x 8. |
| Bus servo adapter 1 (leg and head servos) | Upper deck, rear left | 4 x M2 x 6. |
| Bus servo adapter 2 (wheel servos) | Upper deck, rear right | 4 x M2 x 6. |
| UBEC 1 (5.2 V, Pi power) | Upper deck, front right | Foam tape below, 2 zip ties through the deck slots. |
| UBEC 2 (7.4 V, leg and head servo power) | Upper deck, front left | Foam tape below, 2 zip ties through the deck slots. |
| Battery | Battery cradle in the chassis upper | Slide it in from the rear. |
| Fuse holder | Outside of the tray left wall, on the centre post | 2 zip ties through the 4 holes in the wall. |
| XT60 splice | Right side bay of the chassis | Zip ties through the slots in the cradle wall. |
| Eye LEDs | Head | |

## Bus servo adapters
- The edge with the USB-C socket, the power screw terminal and the DC jack points to the front of the robot.
- The servo sockets are at the rear edge. The servo plugs go in from above.
- Connect the power wires to the screw terminal. Do not use the DC jack.
- Set the switch on each adapter to USB.
- The deck has an open cable channel, 35 mm wide, in front of the 2 adapters. Use it for the USB plugs and the power wires.

## Power
- The battery supplies bus servo adapter 2, UBEC 1 and UBEC 2 through the 10 A fuse.
- Bus servo adapter 2 (wheel servos) gets battery voltage (3S, 12.6 V when the battery is full).
- UBEC 2 supplies bus servo adapter 1 (hip and head servos). Set it to 7.4 V.
- UBEC 1 supplies the Pi 5 through the 5 V and GND pins of the Pi header. Set it to 5.2 V.
- WARNING: do not connect battery voltage to bus servo adapter 1. Battery voltage damages the 7.4 V servos.
- Do not connect the 2 servo buses together.
- The robot has no power switch. Disconnect the battery when you do not use the robot.

## Servos
- The 4 hip servos, the head yaw servo and the head pitch servo are Feetech STS3215, 7.4 V version.
- The 2 wheel servos are Waveshare ST3215-HS. They use battery voltage.
- Wheel servo horn: use one of the 2 aluminium disc horns from the ST3215-HS box. The wheel hub attaches to this horn.
- Use Feetech or Waveshare bus servo cables (Molex 5264, 3-pin). RC servo extension cables do not fit the sockets.
- The horn holes take M3 screws. The servo case holes take M2 screws.

## Fasteners by location

| Location | Fastener |
|---|---|
| Hip servo rear faces (16) | M2 x 8 button head + M2 washer, from inside the spine |
| Lower front plate to wheel servo (4) | M2 x 8 button head |
| Pi 5 (4) | M2 x 8 button head, into 1.8 mm pilot holes |
| IMU plate to the lower deck posts (4) | M2 x 8 button head, into 1.8 mm pilot holes |
| Head yaw servo to the lid (4) | M2 x 6 pointed screw from the servo bag |
| Head pitch servo in the bracket (8) | M2 x 6 pointed screw from the servo bag |
| Bus servo adapters 1 and 2 on the upper deck (8) | M2 x 6, into 2.0 mm pilot holes |
| IMU on the IMU plate (2) | M2 x 6, into 2.0 mm pilot holes |
| Upper links to hip horns (16) | M3 x 8 button head + 1 mm washer |
| Lower rear links to wheel servo idler discs (8) | M3 x 8 button head + 1 mm washer |
| Wheel hubs to wheel servo horns (8) | M3 x 8 button head, no washer |
| Pitch bracket to yaw servo horn (4) | M3 x 8 button head, no washer |
| Head to pitch horn and idler disc (8) | M3 x 8 button head, no washer |
| Chassis lower to chassis upper (4) | M3 x 8 button head, into heat-set inserts |
| Tray to chassis upper (4) | M3 x 8 button head, into heat-set inserts |
| Upper deck to the deck columns (4) | M3 x 8 button head, into heat-set inserts |
| Lid to the upper deck end pockets (4) | M3 x 8 button head, into heat-set inserts |
| Knees (4) | M3 x 16 button head, 1 mm washer between the links, M3 nyloc nut, threadlocker on the screw tip |
| Wheel to hub (4) | M3 x 12 button head, into heat-set inserts in the hub |
| OAK-D Lite (2) | M4 x 10 button head + M4 washer |

Heat-set inserts (M3 x 5 x 4), 20 in total:
- Chassis upper, 12: 4 corner posts, 4 deck columns, 2 rear bosses, 2 front pads.
- Upper deck, 4: one in each end pocket. An M3 nut does not fit there.
- Wheel hubs, 4: 2 in each hub.

Do not use M2.5 screws.
