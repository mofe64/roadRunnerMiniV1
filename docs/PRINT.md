# Print prep — Biped v0.2 (STLs in cad/biped_v02/stl, coupon in cad/biped_v02/coupon)

All parts PLA, 0.2 mm layers, 4 walls, 30 % infill unless noted. STLs are in mm.

## 1. Test coupons — DONE (2026-10-27), fits applied in CAD (S39/S40)
Mofe printed `coupon_all.stl` and `coupon_horn_gauge.stl`. Results and what changed in the model:
- Printer prints openings true size or a touch over. Clearance holes 2.5 / 2.9 / 3.5 and the countersink: fine, unchanged.
- M2 self-tap: bites in 1.8 (tight, driver needed) and 1.9; not in 1.4 / 1.5. All M2 pilots are now 1.8 mm (Pi and Pico bosses). M2.5 screws dropped: Pi uses M2 x 8.
- M3 x 5 x 4 heat-set insert: 4.0 boss hole correct (needs heat to seat, as it should).
- Servo horn: 20.2 pocket is the fit (stays in upside down, drops out with a tap). All horn pockets 20.5 -> 20.2.
- 8 mm tab loose in 8.4 / 8.6 / 8.8: lid and tray tab fits tightened 0.4 -> 0.2 mm; tray servo pockets 47 x 26 -> 46.2 x 25.4.
- STS3215 notch: slides in, free side to side: pass.
Screws (from the Lukaisen button-head kit): M2 x 8 at the hips, Pico and Pi; M2 x 6 pointed screws from the servo bags for the yaw servo back and the pitch bracket arms (2026-10-30: the arm is 2 mm thick, so M2 x 8 goes 6 mm into the servo); M3 x 8 for the bracket to the yaw horn; M3 x 8 into the inserts; S53: M3 x 16 at the knee pivots (screw head on the inner side, a washer between the links, one nut on the outer side; put a small drop of threadlocker on the screw tip only, not on the plastic; do not tighten the nut hard, the joint must turn freely); S52: 2 x M2 x 8 per lower front plate into the wheel servo, 4 x M3 x 8 per hub into the horn, 2 x M3 x 12 per wheel into 2 heat-set inserts in the hub (2026-10-30: the wheel is 7.75 mm thick at the holes, so M3 x 8 goes only 0.25 mm into the insert); M4 x 10 + washer for the OAK; M3 x 8 at every horn; on the upper links and the lower rear links put a 1 mm washer under each screw head (the screw then goes 3 mm into the horn; on the lower rear link, without the washer the screw goes too far through the idler disc) (the STS3215 horn holes take M3; Feetech and the SO-101 guide use M3 x 6). M2 is only for the servo case holes.

## 2. What to print: 16 parts on 9 plate files (cad/biped_v02/plates/*.3mf)
Each plate is a Bambu Studio project file (3MF). Open it with File > Open Project. The print settings are saved in the file: printer Bambu Lab A1 0.4 nozzle, Bambu PLA Basic, 0.20 mm Standard, plus the plate settings in the table. Do not change them. The object count must match the parts column; extra objects mean a broken export, stop and ask.
| Plate | Parts | Bed area | Walls | Infill | Supports | Brim |
|---|---|---|---|---|---|---|
| plate_1_head | head (S55: grooves in the ribs for the bracket screw heads; reprint, the older head does not fit) | 96 x 100 | 4 | 30 % gyroid | tree, on build plate only | auto |
| plate_1b_bracket_flat | pitch bracket, lying on its long side (S49: arms 34 mm wide, tops widened) | 57 x 40 | 5 | 40 % gyroid | tree, on build plate only (only under the arm tops) | outer, 5 mm |
| plate_2_upper_links_hubs | 4 upper leg links (S61b: horn holes 3.5 for M3, no counterbore; S53: 2 mm knee boss on the inner face, outer face down) + 2 wheel hubs (S52: horn and wheel holes, flange face down) | 199 x 138 | 4 | 100 % | none | auto |
| plate_3_lower_front | 2 lower front plates (S52: closed ring + 2 screw bosses at the wheel servo; S53: 2 mm knee boss on the outer face, 2 pairs of cable-tie holes, inner face down) | 194 x 102 | 4 | 100 % | none | auto |
| plate_4_lower_rear | 2 lower rear doglegs (S52: 14 mm post with collars; S61b: horn holes 3.5 for M3, no counterbore; S53: outer plate 6 mm thick); one arm sits 39 mm up | 182 x 96 | 4 | 100 % | tree, everywhere (the raised arm sits over the lower plate) | auto |
| plate_5a_chassis_lower | chassis, lower part (S63: floor + hip servo spine, split from the old one-piece chassis after the plate 5 print came off the bed; the floor lies flat on the bed; 4 x 3.5 mm screw holes in the floor corners. S64: 1 mm ribs inside the spine behind the thin strips between the idler circles and the socket windows, and flat tops on the idler circles, after the strip broke in the first print) | 143 x 92 | 4 | 30 % gyroid | none | outer, 5 mm |
| plate_5b_chassis_upper | chassis, upper part (S63: battery walls + deck floor + columns, stands on its walls; 4 corner posts with M3 insert holes in their bottoms) | 144 x 92 | 4 | 30 % gyroid | tree, on build plate only (under the deck floor, 22 mm tall) | outer and inner, 5 mm |
| plate_6_upper_deck | upper deck | 146 x 92 | 4 | 30 % gyroid | none | auto |
| plate_7_tray | tray (prints as an open box; S62b: battery door holes at y +/-29, Pico USB window 2 mm higher; S65: the 4 servo windows are open to the top edge, so the body goes straight down into the tray with the hip horns on. S66: 4 zip-tie holes in the left centre post for the fuse holder; 75.0 cm3, 93 g) | 150 x 100 | 4 | 30 % gyroid | tree, on build plate only | auto |
| plate_8_lid | lid, top face down (S67: walls 12 mm taller, so the plugs and cables on the deck have room; 34.5 mm tall, 56.0 cm3, 69 g) | 150 x 100 | 4 | 30 % gyroid | tree, on build plate only (S63: under the rounded top edge and the servo recess floors) | auto |
The single-part STLs in cad/biped_v02/stl are the same parts if you need one at a time. To rebuild the 3MFs after a CAD change: cad/biped_v02/scripts/make3mf.py + build_plates.py (S63: plates 5a and 5b replace plate 5) (the settings base is Metadata/project_settings.config saved from Bambu Studio 02.08.02.61; the key different_settings_to_system must list the changed keys or Bambu Studio ignores them).
Total about 400 g of PLA for the body set, 100 g for the legs.

## Order
1. Coupon (done). 2. Plate 1 (test the servo, horn and OAK fit). 3. Plates 2-4 (legs). 4. Plates 5a, 5b and 6 (chassis lower, chassis upper, deck). 5. Plates 7-8 (tray + lid).

## Servo backs (checked on a real STS3215, 2026-10-27 21:15)
The two cable sockets are on the BACK face (the flat face opposite the horn), between the four corner screw holes, with the cable channel running to the motor end. Plugs press flat (Mofe). Every surface a servo back screws to has a pocket over the socket area: chassis spine walls (hips), pitch-bracket rear arm, lid under the yaw servo. Screws for servo backs: the M2 x 6 pointed (self-tapping) screws that came with the servos. Horns: M3 x 8 button head. The case holes take M2; the horn holes take M3.

## Pitch bracket reprint (2026-10-28)
The bracket printed upright on plate 1 snapped an arm: upright, the layers run across the 2 mm arms and the strips beside the 20 mm holes were only 2.7 mm wide. S48/S49 widened the arms to 34 mm below the socket window and widened the arm tops. Print plate_1b_bracket_flat.3mf: the bracket lies on its long side (the rear edge of the plate and both arm edges on the bed), so the layers run along the arms and the only supports are the small ones under the arm tops.

## Assembly guide
The full assembly order with pictures is in docs/assembly/ASSEMBLY.html (2026-10-30). Fit the hip servos before the tray. Since S65 the horns can stay on: the body goes straight down into the tray through the open slots.

## Screw access (checked 2026-10-28)
Assembly order that keeps every screw reachable:
1. Wheel unit: wheel servo + lower rear link (4 x M3 x 8 + 1 mm washer into the idler disc on the servo back, from the inside face; the washer is necessary) + hub + wheel, before the link pivots onto the upper links.
2. Hip servos into the lower chassis spine from below (the spine is open underneath and at both ends since S51). Use M2 x 8 button heads + 1 mm washer and the 1.5 mm L-key, not the Phillips bag screws: a straight screwdriver does not fit inside the spine. The servo back plate must sit flat on the four pads of the spine wall (S62). The upper screw heads go into the pockets in the spine roof.
2b. S63: push 4 M3 heat-set inserts into the holes in the bottoms of the 4 corner posts of the upper chassis. Set the upper chassis on the floor of the lower chassis (the door bosses go to the rear). Put 4 x M3 x 8 up through the floor holes from below into the inserts. The screw heads sit under the floor, outside the hip servos.
3. Pi: 4 x M2 x 8 into the 1.8 mm pilots (S62: 7 mm deep). Pico: 4 x M2 x 8 into the four 6 mm bosses on the deck (S62: 1.8 mm pilots, 7 mm deep; the screws do not reach the battery).
4. Upper deck (M3 x 8 into the inserts in the four 9 x 9 mm columns), then the lid. Before assembly, push one M3 heat-set insert into each of the 4 deck tab pockets (plate 6). An M3 nut does not fit. Put 4 x M3 x 8 through the lid side holes into these inserts (the screw tip stops approximately 1 mm before the pocket bottom).
5. Yaw servo to the lid, lid off the body. The horn faces up. The back face of the servo sits on the lid. Put 4 x M2 x 6 servo-bag screws through the lid from below. The plugs and the cable go down through the window in the lid.
6. Bracket onto the yaw horn: 4 x M3 x 8 button head, from the top, inside the U. (Mofe used M3 x 8 here on 2026-10-28.)
7. Pitch servo into the bracket: 4 screws through each side of the U, from the outside. Do this before the head goes on.
8. Horn onto the pitch servo spline (centre screw). Idler disc onto the idler pin.
9. Lower the head straight down over the servo. The screw heads on the bracket sides slide through the grooves in the head ribs.
10. Head to the horn and to the idler disc: 4 x M3 x 8 button head on each side, 8 in total. Do not use a washer. Each screw goes 3 mm into the horn. Each screw goes through the head rib into the disc. Reach up from under the head (it has no bottom). Put the short end of the L-key in the screw. Turn in small strokes. A small piece of Blu Tack on the key holds the screw. Do not put the screws in before the head is on: the tips stop the head.
11. OAK M4 screws through the two 8 mm holes in the rear wall. The 10 mm holes in the side walls are not used.
