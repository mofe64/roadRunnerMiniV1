# Print instructions: Biped v0.2

This document is written in ASD-STE100 Simplified Technical English.

The part STLs are in `cad/biped_v02/stl`. The test coupons are in `cad/biped_v02/coupon`. The print plates are in `cad/biped_v02/plates`. All STL units are mm.

Default settings for all parts (unless a step gives different settings):
- Material: PLA.
- Layer height: 0.2 mm.
- Walls: 4.
- Infill: 30 %.

## 1. Test coupons (complete, 2026-10-27)
We printed `coupon_all.stl` and `coupon_horn_gauge.stl`. The results are in the model (S39 and S40):

| Test | Result | Change in the model |
|---|---|---|
| Clearance holes 2.5, 2.9 and 3.5 mm, countersink | Correct. The printer makes holes at the correct size or a little larger. | No change. |
| M2 self-tapping screw | The screw cuts a thread in 1.8 mm (tight, use a screwdriver) and 1.9 mm holes. It does not go into 1.4 mm or 1.5 mm holes. | All M2 pilot holes are 1.8 mm (Pi and Pico bosses). |
| M2.5 screws | Not necessary. | The Pi uses M2 x 8 screws. |
| M3 x 5 x 4 heat-set insert | The 4.0 mm hole is correct. The insert needs heat to go in. | No change. |
| Servo horn pocket | 20.2 mm is correct. The horn stays in when the part is upside down. A light tap removes it. | All horn pockets changed from 20.5 to 20.2 mm. |
| 8 mm tab in 8.4, 8.6 and 8.8 mm slots | Too loose. | Lid and tray tab clearance changed from 0.4 to 0.2 mm. Tray servo pockets changed from 47 x 26 to 46.2 x 25.4 mm. |
| STS3215 notch | The servo goes in and moves freely from side to side. Pass. | No change. |

## 2. Parts to print
There are 16 parts on 8 plate files. Refer to `renders/print_plates.png` for a picture.

Each plate STL lies flat in its print orientation and is at the centre of the bed. The single-part STLs in `cad/biped_v02/stl` are the same parts. Use them if you want to print one part at a time.

Do these steps in Bambu Studio:
1. Import the plate file.
2. When Bambu Studio asks if the file is one object with multiple parts, select Yes.
3. Select the 0.20 Standard profile.
4. Set the wall loops to 4.
5. Make sure that the number of objects is the same as the number of parts in the table. If there are more objects, the export is broken. Stop and ask before you print.

| Plate | Parts | Bed area (mm) | Supports |
|---|---|---|---|
| plate_1_head_bracket | Head and pitch bracket (2 objects). The bracket connects the yaw servo horn to the head pitch servo. | 129 x 100 | Tree supports, on the build plate only (visor window, ledges, rib bosses). |
| plate_2_upper_links_hubs | 4 upper leg links and 2 wheel hubs | 125 x 165 | None. |
| plate_3_lower_front | 2 lower front plates | 181 x 96 | None. |
| plate_4_lower_rear | 2 lower rear doglegs. One arm is 39 mm above the bed. | 175 x 94 | Tree supports under the raised arm. |
| plate_5_chassis | Chassis. Bambu Studio can show a small automatic repair at the lid seat. This is satisfactory. | 142 x 92 | Yes. The lower deck extends 20 mm past the battery cradle on each side. |
| plate_6_upper_deck | Upper deck | 146 x 92 | None. |
| plate_7_tray | Tray (prints as an open box) | 150 x 100 | Only in the servo pockets and the side windows. |
| plate_8_lid | Lid, with the top face down | 150 x 100 | None. |

Plates 1, 5 and 8 are from S47 (2026-10-27 21:30). These plates have the longer pockets for the servo sockets.

Special settings:
- Leg links (plates 2, 3 and 4): use 100 % infill or 6 walls.
- Bracket and head: use 4 walls and 30 % infill.
- Plates 3 and 4 need a bed that is a minimum of 185 mm wide. If your bed is smaller, print the four links one at a time from `cad/biped_v02/stl`.

Approximate PLA mass: 400 g for the body parts and 100 g for the legs.

## 3. Print sequence
1. Test coupons (complete).
2. Plate 1. Use it to test the fit of the servo, the horn and the OAK-D Lite.
3. Plates 2, 3 and 4 (legs).
4. Plates 5 and 6 (chassis and upper deck).
5. Plates 7 and 8 (tray and lid).

## 4. Servo rear faces (checked on a real STS3215, 2026-10-27)
The two cable sockets are on the rear face of the servo. The rear face is the flat face opposite the horn. The sockets are between the four corner screw holes. The cable channel goes to the motor end. The plugs are flat when they are fully in.

Each surface that a servo rear face attaches to has a pocket over the socket area:
- The chassis spine walls (hip servos).
- The rear arm of the pitch bracket.
- The lid, below the yaw servo.

Fasteners:
- Servo rear faces: use the M2 x 6 pointed self-tapping screws from the servo bags.
- Servo horns: use M2 x 8 button-head screws with 1 mm washers.
