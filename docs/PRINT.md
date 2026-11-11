# Print instructions

This document is written in ASD-STE100 Simplified Technical English.

The robot has 18 printed parts on 11 print plates. The plates are in `cad/biped_v02/plates/`. The single parts are in `cad/biped_v02/stl/` (units: mm).

## Printer and material
- Printer: Bambu Lab A1 with a 0.4 mm nozzle.
- Material: Bambu PLA Basic. The full set uses approximately 540 g.
- Process: 0.20 mm Standard.

## Print the plates
Each plate is a Bambu Studio project file (3MF). The print settings are saved in the file.

1. Open the plate file in Bambu Studio with File > Open Project.
2. Make sure that the number of objects agrees with the "Parts" column of the table. If it does not agree, the file is damaged. Do not print it.
3. Do not change the settings.
4. Slice the plate and print it.

| Plate | Parts | Orientation | Walls | Infill | Supports | Brim |
|---|---|---|---|---|---|---|
| plate_1_head | Head | Upright | 4 | 30 % gyroid | Tree, on build plate only | Auto |
| plate_1b_bracket_flat | Pitch bracket | On its long side | 5 | 40 % gyroid | Tree, on build plate only | Outer, 5 mm |
| plate_2_upper_links_hubs | 4 upper links, 2 wheel hubs | Links: outer face down. Hubs: flange face down | 4 | 100 % | None | Auto |
| plate_3_lower_front | 2 lower front plates | Inner face down | 4 | 100 % | None | Auto |
| plate_4_lower_rear | 2 lower rear links | As in the plate file. One arm is 39 mm above the bed | 4 | 100 % | Tree, everywhere | Auto |
| plate_5a_chassis_lower | Chassis lower | Floor down | 4 | 30 % gyroid | None | Outer, 5 mm |
| plate_5b_chassis_upper | Chassis upper | On its walls and posts | 4 | 30 % gyroid | Tree, on build plate only | Outer and inner, 5 mm |
| plate_6_upper_deck | Upper deck | Bottom face down | 4 | 30 % gyroid | None | Auto |
| plate_7_tray | Tray | Floor down (open box) | 4 | 30 % gyroid | Tree, on build plate only | Auto |
| plate_8_lid | Lid | Top face down | 4 | 30 % gyroid | Tree, on build plate only | Auto |
| plate_9_imu_plate | IMU plate | Flat side down, posts up | 4 | 100 % | None | Auto |

The mass of each plate is in `BOM.md`.

## Print sequence
1. Print the test coupons (see "Test coupons"). Do this one time for a new printer or a new material.
2. Print plate 1 and plate 1b. Make sure that a servo, a horn and the OAK-D Lite fit.
3. Print plates 2, 3 and 4 (legs).
4. Print plates 5a, 5b, 6 and 9 (chassis, upper deck, IMU plate).
5. Print plates 7 and 8 (tray and lid).

## Notes for the parts
- Pitch bracket: print it only on its long side (plate 1b). Then the layers go along the 2 mm arms. If you print the bracket upright, the arms break.
- Lower rear links: one arm is 39 mm above the bed. The supports below it are necessary.
- Chassis: the chassis has two parts because a one-part chassis does not have a large flat face for the bed.
- Chassis upper: after the print, remove the brim from the holes in the corner posts.
- Tray: the 4 small holes for the fuse holder zip ties print without supports.

## Test coupons
The coupons are in `cad/biped_v02/coupon/`. Print `coupon_all.stl` and `coupon_horn_gauge.stl` with the same printer, material and process as the parts. Then do these checks:

| Check | Correct result |
|---|---|
| M2 screw in the 1.8 mm hole | The screw cuts a thread. You need a screwdriver to turn it. |
| M2 screw in the 2.5 mm hole | The screw goes through freely. |
| M3 screw in the 3.5 mm hole | The screw goes through freely. |
| M3 x 5 x 4 heat-set insert in the 4.0 mm hole | The insert goes in only with heat. |
| Servo horn in the 20.2 mm pocket | The horn stays in when you turn the coupon upside down. It comes out when you tap the coupon. |
| 8 mm tab in the slots | The tab goes into each slot without force. |
| STS3215 servo in the notch | The servo goes in. It moves freely from side to side. |

If a check fails, adjust the printer before you print the parts.

The CAD model uses these sizes: M2 pilot hole 1.8 mm (2.0 mm for the circuit board posts), M2 clearance hole 2.5 mm, M3 clearance hole 3.5 mm, insert hole 4.0 mm, horn pocket 20.2 mm, tab clearance 0.2 mm.

## Heat-set inserts
Push in the 20 M3 x 5 x 4 heat-set inserts with a soldering iron before you assemble the robot. Each insert must be flush with the face and straight. The positions are in step 1 of the assembly guide.

## Assembly
The assembly guide is `assembly/ASSEMBLY.html`. It gives the sequence, the fastener for each joint and a picture for each step. The fastener list is in `BOM.md` and in `PARTS.md`.

## Make the plate files again after a CAD change
The plate scripts are in `cad/biped_v02/scripts/` on the design computer. They are not in the repository.

1. Export the changed bodies from Fusion as STL files into `cad/biped_v02/stl/`.
2. Run the export script for the design state (for example `s72_export.py`). It uses `make3mf.py` to write the 3MF files of the changed plates.
3. Open each new plate in Bambu Studio and make sure that the settings in the table above are active.

The settings come from `Metadata/project_settings.config`, saved from Bambu Studio 02.08.02.61. The key `different_settings_to_system` must list each setting that is different from the system preset. If it does not, Bambu Studio ignores the setting.
