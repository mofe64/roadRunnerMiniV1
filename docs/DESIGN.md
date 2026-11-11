# Road Runner Mini: design description (S72)

This document is written in ASD-STE100 Simplified Technical English.

This document describes the mechanical design of the robot at design state S72. The name of the Fusion design is "Biped v0.1". The archive of this design state is `cad/biped_v02/biped_v02_S72.f3d`. The two names are different, but they are the same design.

## Coordinates
- X points forward, Y points left and Z points up.
- The origin is the middle point of the 4 hip axes (the "hip frame"). All dimensions in this document are in mm in the hip frame, unless the text says "world".
- World z = hip z + 155. This is correct at the normal stand height (120 mm), with the wheels on the ground.

## Overview
- Legs: one five-bar linkage on each side. The 2 hip servos of each leg are in the body. The upper links are 60 mm long and the lower links are 100 mm long. The hip axes are 50 mm apart.
- Leg height (hip axis to wheel axle): 65 mm to 140 mm. The normal stand height is 120 mm. Above 140 mm the lower links touch, so the software limit is 140 mm.
- Wheels: Pololu 70 x 8 mm wheels on Waveshare ST3215-HS servos in motor mode. The track is 138 mm.
- Body: a printed chassis in 2 parts, an upper deck and an IMU plate, in a shell (tray and lid). The shell is 150 x 100 x 127 mm with 16 mm corner radius and 2 mm walls.
- Neck: pan-tilt. The yaw servo is on the lid. A printed U-bracket on the yaw horn holds the pitch servo. The head is on the pitch horn and the idler disc.
- Head: a printed box, 95.5 x 100 x 65 mm, open at the bottom. The OAK-D Lite is on 2 ledges behind an opening in the front face.
- Mass: approximately 1750 g with the battery (measured on the real robot).

## Body parts

| Part | Function |
|---|---|
| Chassis lower | Floor and spine. The spine holds the 4 hip servos. |
| Chassis upper | Battery cradle, lower deck (Pi 5 and IMU plate), 4 deck columns, 4 corner posts. |
| Upper deck | 2 bus servo adapters and 2 UBECs. |
| IMU plate | IMU. |
| Tray | Lower shell. The chassis goes down into the tray. |
| Lid | Upper shell. The yaw servo is on the lid. |

### Chassis lower
- The floor is 2 mm thick. The top face of the floor is at z 14.4. The chassis splits into its 2 parts at this face.
- The spine has 2 walls, 5 mm thick, and 2 end walls. It is open at the bottom. The space between the walls is 22.4 mm.
- Each hip servo attaches to the outer face of a spine wall with 4 M2 x 8 screws and washers. The screws go in from inside the spine. The servo back plate touches 4 pads on the wall.
- Each servo has a cable slot through the wall (7 x 18 mm). The hip cables go through the slots into the spine and out at the bottom.
- The floor has 2 openings above the spine (x 26 to 59.9 and x -59.9 to -26, 22.4 mm wide). They give access to the upper hip screws. The middle 52 mm of the floor is solid because it connects the 2 spine walls.
- Tool for the hip screws: 1.5 mm L-key or 1.5 mm ball-end hex driver. A straight screwdriver does not fit.

### Chassis upper
- 4 corner posts have an M3 insert in their bottom face. 4 M3 x 8 screws go up through the floor of the chassis lower into these inserts.
- The battery cradle is open at the rear. The battery (135.6 x 43.5 x 22.1 mm) goes in from the rear.
- The lower deck is above the battery. It has 4 pilot holes (1.8 mm, 7 mm deep) for the Pi 5 and 4 posts at the front (x 48.8 and 60.2, y 27.5 and -19.5) for the IMU plate.
- 4 deck columns (9 x 9 mm, at x +/-67, y +/-35) have an M3 insert in their top face for the upper deck.
- 2 rear bosses and 2 front pads (y +/-29, z 25) have an M3 insert for the tray screws.

### Upper deck
The deck plate is 2 mm thick (z 61 to 63, world 216 to 218).

| Item | Position | Mount |
|---|---|---|
| Bus servo adapter 1 (leg and head servos) | Rear left: x -60.0 to -26.2, y 1.5 to 43.5 | 4 posts, 4 x M2 x 6 |
| Bus servo adapter 2 (wheel servos) | Rear right: x -60.0 to -26.2, y -43.5 to -1.5 | 4 posts, 4 x M2 x 6 |
| Cable channel | x -26.2 to 9, full deck width | No parts |
| UBEC 1 (Pi, 5.2 V) | Front right: x 13 to 64, y -39 to -3 | Foam tape, 2 zip ties |
| UBEC 2 (leg and head servos, 7.4 V) | Front left: x 13 to 64, y 3 to 39 | Foam tape, 2 zip ties |

- Adapter posts: 5 mm diameter, 2.5 mm high, with 2.0 mm pilot holes. Positions: x -57.46 and -29.46, |y| 4 and 41. The hole pattern of the board is 37 x 28 mm.
- Adapter orientation: the edge with the USB-C socket, the power screw terminal and the DC jack is at x -26.2 and points forward into the cable channel. The servo sockets are at the rear edge and the plugs go in from above.
- Cable channel: a straight USB-C plug and its cable bend need 35 mm. The channel gives this space.
- UBEC size: 51 x 36 x 8 mm (seller data). Each UBEC is on 1 mm foam tape.
- Tie slots: 6 slots, 5 x 2.5 mm, through the deck. Positions: x 30 to 35 and x 55 to 60; y -43 to -40.5, y -1.5 to 1.5 and y 40.5 to 43. Each zip tie (2.5 mm wide, 150 mm long) goes down through one slot, below the deck and up through the next slot. The centre slots take 2 ties. Below the deck, the ties are in front of the Pi cooler and 5.3 mm above the IMU.
- Vents: 20 holes, 4 mm diameter, above the Pi cooler.
- End pockets: 4 M3 inserts (x +/-66 to 72.8, y +/-23) for the lid side screws.

### IMU plate
- The IMU plate is 3 mm thick. It is on the 4 posts at the front of the lower deck (4 x M2 x 8).
- Size: x 45.5 to 63.5, y -22.5 to 30.0, with a tongue x 63.4 to 69.6, y -10 to 10.
- The IMU (25.4 x 17.8 mm) is on 2 posts at (46.54, 6.25) and (66.86, 6.25), with 2 x M2 x 6. A pad holds the free edge of the board.
- IMU position: x 44 to 69.4, y -8.9 to 8.9, board bottom at z 50 (world 205). The X arrow points to the nose.
- The STEMMA QT cable goes into the rear socket of the IMU.

### Tray
- The tray is an open box. Each side wall has 2 slots that are open to the top edge. The body goes straight down into the tray with the hip servos and their horns on.
- Smallest gaps when the body goes down: 1.0 mm from each horn disc to the slot edge, 0.5 mm from each servo case to the slot.
- 4 M3 x 8 screws attach the tray to the chassis upper: 2 through the rear wall and 2 through the front wall.
- The left wall has 4 holes (4.5 x 2.5 mm) in the centre post for the 2 zip ties of the fuse holder. The fuse holder (65 x 25 x 18 mm) is on the outside of the wall.

### Lid
- The lid is 34.5 mm high. The inner ceiling is at world z 247.5 (245 near the edges).
- 4 M3 x 8 screws go through the lid end walls into the inserts in the upper deck end pockets.
- The yaw servo is on the lid top with the horn up. 4 M2 x 6 servo screws go in from inside the lid. The servo plugs and the cable go down through a window in the lid.

## Legs and wheels
- Upper links: 4 x M3 x 8 with a 1 mm washer into each hip horn. Each screw goes 3 mm into the horn.
- Lower rear link: 2 plates with a 14 mm post between them. It attaches to the idler disc on the back of the wheel servo with 4 x M3 x 8 and 1 mm washers. Without the washers, the screws go through the idler disc.
- Lower front plate: a closed ring around the wheel servo horn. 2 x M2 x 8 attach it to the servo case. It has 2 pairs of 2.6 mm holes for cable ties. Tie the wheel servo cable to this plate.
- Knee: M3 x 16 through 6 mm of plastic on each link, a 1 mm washer between the links and a nyloc nut. The joint must turn freely.
- Wheel hub: 4 x M3 x 8 into the servo horn. 2 M3 inserts in the hub hold the wheel with 2 x M3 x 12.
- Clearance over the full leg range, with the axle below the hip centre: the legs are 3.8 mm or more from the shell. The lower links are 2.3 mm apart at 140 mm.
- At 65 mm leg height, the leg can touch the tray floor if the axle is 20 mm or more away from the centre position.

## Neck and head
- Pitch bracket: the base is 4.8 mm thick and attaches to the yaw horn with 4 x M3 x 8. The 2 arms are 2 mm thick and 34 mm wide. 4 M2 x 6 servo screws go through each arm into the pitch servo.
- Head: 4 x M3 x 8 through each side rib, 4 into the pitch horn and 4 into the idler disc. The ribs have grooves for the bracket screw heads, so the head goes straight down over the bracket.
- OAK-D Lite: 2 x M4 x 10 with washers into the rear holes of the camera (75 mm apart). 2 holes, 8 mm diameter, in the head rear wall give access to these screws.
- The pitch axis is 151 mm above the hip axes.
- Head limits (software): 30 degrees nose up, 50 degrees nose down (40 degrees at +/-90 degrees yaw), yaw +/-90 degrees.

## Power and wiring
- Battery (3S LiPo, 5200 mAh) to the 10 A blade fuse, then to the XT60 splice.
- From the splice: bus servo adapter 2 (wheel servos), UBEC 1 and UBEC 2.
- The 2 wheel servos (ST3215-HS) use battery voltage.
- UBEC 2 (7.4 V) supplies bus servo adapter 1. The 4 hip servos and the 2 head servos are the 7.4 V version of the STS3215. Battery voltage damages them.
- Connect the power of each adapter to its screw terminal.
- UBEC 1 (5.2 V) supplies the Pi 5 through the 5 V and GND header pins.
- Each bus servo adapter has a USB cable to the Pi.
- The IMU connects to the Pi I2C pins with a STEMMA QT cable.
- The OAK-D Lite connects to a USB 3 port of the Pi.
- The robot has no power switch.
- Do not connect the 2 servo buses together. They have different voltages.

## Plug and cable space
The model has a keep-out body for each plug and its cable bend. The overlap check includes these bodies.

| Keep-out | Size or limit |
|---|---|
| USB-C plug on each bus servo adapter | 35 x 12 x 7 mm |
| Power wires on each bus servo adapter | In the cable channel |
| Servo plugs on each bus servo adapter | Up to world z 237.5 (lid ceiling 247.5) |
| UBEC wires | At the ends of each UBEC |
| UBEC zip ties | Around each UBEC and below the deck |
| STEMMA QT plug on the IMU | At the rear socket |
| Hip servo plugs | 4, one at each hip servo |

Rule for a new circuit board mount: add the plug and the cable bend to the model for each connector before you accept the layout. An overlap check does not find a plug that has no space.

## Fits for printed PLA
| Feature | Size |
|---|---|
| M2 pilot hole (screw cuts its thread) | 1.8 mm (2.0 mm in the circuit board posts) |
| M2 clearance hole | 2.5 mm |
| M3 clearance hole | 3.5 mm |
| Hole for an M3 x 5 x 4 heat-set insert | 4.0 mm |
| Servo horn pocket | 20.2 mm |
| Tab clearance (lid and tray) | 0.2 mm |
| Smallest clearance between printed parts | 0.5 mm |

The servo horn holes take M3 screws. The servo case holes take M2 screws. The design does not use M2.5 screws.

## Design checks
Do these checks after each change to a printed body:
- The body is one solid (1 lump, 1 shell) and its material is PLA.
- The guard script `s57_normalise_guard.py` shows `GUARD OK`. Run it 2 times.
- The overlap check of the changed bodies against all other bodies, with the keep-out bodies, shows no overlap.
- The holes align with the holes of the purchased parts within 0.1 mm.
- The volume of the STL mesh and of the 3MF mesh agrees with the Fusion body.

The scripts are in `cad/biped_v02/scripts/` on the design computer. They are not in the repository.

## Simulation model
The MuJoCo model is in `sim/`. Refer to `sim/README.md`.

## Open items
- The battery door cover is not designed.
- The skids (chin and tail) are not designed.
- The UBEC size is seller data. It is not measured.
- The cable routes from the upper deck down to the Pi are not in the model.
- The rear side holes of the lid print as open slots.
