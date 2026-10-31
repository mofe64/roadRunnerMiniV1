# Road Runner wheeled biped — design notes (v0.2, 2026-10-27)

Hobby / open-source wheeled biped: self-balance, obstacle avoidance, fall recovery, subject-tracking video. Looks inspired by Upkie, Diablo, TITA, Ascento.

## Layout
- Frame: X forward, Y left, Z up, origin at the midpoint of the four hip axes ("hip frame"). Wheels on the ground at hip z = -155 mm at the 120 mm stand height.
- Legs: symmetric five-bar per side, both hip servos in the body (upper link 60 mm, lower 100 mm, hip axes 50 mm apart). Hip-to-axle height 65..140 mm (software limit since S52: above 140 mm the lower links touch), normal stand 120 mm, tuck 65-70 mm.
- Body: a printed chassis in two parts since S63 (lower: floor + servo spine; upper: battery cradle, lower deck for Pi 5 + Pico, four columns; joined by 4 x M3 x 8 into inserts at the corners) + upper deck (UBECs, bus-servo adapter, IMU), inside a 150 x 100 x 127 mm rounded shell (S67: lid walls +12 mm) (tray + lid, R16, 2 mm walls).
- Neck: standard pan-tilt. Yaw STS3215 stands on the lid (M2 countersunk from inside the lid); printed U-bracket on its horn carries the pitch STS3215; the head rides on the pitch horn and idler.
- Head: printed box 95.5 x 100 x 65 mm, open bottom, OAK-D Lite on ledges held by two M4 screws into its back holes (75 mm apart), visor opening at the front.
- Wheels: Waveshare ST3215-HS bus servos in motor mode, Pololu 70 x 8 mm wheels.
- Power: 3S 5200 mAh LiPo, 10 A blade fuse, UBEC 7.4 V for the leg and head servo bus (bus adapter 1), UBEC 5.2 V for the Pi 5. The 2 wheel servos (ST3215-HS) are on their own bus (bus adapter 2). Adapter 2 gets battery voltage after the fuse. Do not connect the two servo buses together. The battery voltage can damage the 7.4 V STS3215 servos.

## Verified numbers (Fusion "Biped v0.1" design, saved state v0.2 S51, archive biped_v02_S40.f3d)
- Mass 1476 g: body 983, head 203 (ribs fused to the roof + side webs, S43), neck 66, each leg 40, each wheel+servo 72. COM 27 mm above the hip axes, 1 mm ahead.
- Head: pitch pivot 139 mm above the hips. Movement: 30 deg nose-up at any yaw; 50 deg nose-down within +/-45 deg yaw, 40 deg at +/-90 (software limits).
- Legs clear the shell over the whole 65..140 mm range (worst 3.8 mm). No part intersections; plug/cable keep-out blocks modelled for every connector.
- PLA fits (set from the printed coupons, 2026-10-27): M2 clearance 2.5, M3 clearance 3.5, M3 heat-set insert 4.0, M2 self-tap pilot 1.8 (Pi and Pico), horn pocket 20.2, lid/tray tab fit 0.2, tray servo pockets 46.2 x 25.4. No M2.5 screws anywhere.

## Sim (MuJoCo, sim/eggbot_v03.xml)
- Balances at all heights; survives a 1.5 N s push; crouches 70..150 mm; 0.36 m/s.
- Wheel-driven get-up from the chin-and-wheels sit (52 deg) works at 0.9-1.2 Nm wheel torque (drive the wheels TOWARD the chin). Catching the rise is the controller's job.
- WARNING (review 2026-10-28): the sim wheels have no top speed. In the get-up runs they turn at more than 12,000 rpm. With a real speed limit (ST3215-HS: 106 rpm, about 65 rpm at 7.4 V) the wheel get-up fails, and the robot survives only a 0.3 N s push at 7.4 V (0.5 N s at 106 rpm). Add real actuator limits to the sim before RL training.
- From fully face-down the simple scripts do not get up; this is left to the RL policy.

## Open items
- Battery door cover and TPU chin/tail skids not modelled.
- Hip servo plugs have ~11.5 mm to the tray corner (fine if the socket is centred on the servo end face).
- OAK-D Lite needs a right-angle USB-C for the full nose-down range.
- Pi 5 power-plug side assumed; check when the Pi goes in.
- Print orientation / supports: see PRINT.md (coupons done, fits applied).

## S47 (2026-10-27 21:30): real servo check
Mofe's photos: STS3215 sockets are on the back face at mid-length, plug presses flat. Socket pockets stretched toward the shaft end: chassis x 36..55.2 z +/-10.8 (3.1 deep), bracket rear-arm window z 152..169, lid relief x -30..0. Included M2 x 6 pointed screws used for all servo backs. Chassis 127 g, bracket 7 g, lid 56 g.

## S48-S50 (2026-10-28): after the first prints
- S48: pitch bracket arms and plate widened to x -14..20 below the socket window (the upright-printed S47 bracket snapped at the 2.7 mm strips beside the 20 mm hole). Two 8 mm holes in the head rear wall at (y +-37.5, z 158) so the OAK M4 screws can be driven from behind.
- S49: bracket arm tops widened to x -14..16.8 (z 160..174.4) and x 16.8..20 (z 160..166); the +x edge stays at 16.8 above z 166 for the camera at 30 deg nose-up (1.2 mm). Leg link ends: r 11.5 rings at the hip horn centres (upper links) and at the wheel-servo back (lower rear), so the M2 holes have 3.4 mm of plastic beside them instead of 0.9.
- S50: 10 mm holes in both head side walls at (x 4, z 139), in line with the horn and idler centre screws.
- S51: the chassis spine hollow is open at both ends (x +-61) and its inner roof is at z 11.4 (was 9.4), inner walls at y +-13.5. All four back screws of each hip servo can be driven from below with the 1.5 mm L-key. Chassis 110 g (was 127). The spine walls stay 3 mm thick (1.8 mm at the socket reliefs).
- Plates are now 3MF files with the print settings embedded (docs/PRINT.md).

## S52 (2026-10-28): leg fixes after the design review
- Lower front plate: the plate end is now a closed ring around the wheel servo horn (hole r 10.6). Two bosses hold 2.5 mm holes on the two far horn-face holes of the wheel servo (the same pair that the pitch bracket front arm uses). Use 2 M2 x 8 screws on each side. Before S52 the plate had no screws to the servo and the five-bar was not closed.
- Wheel hub: a boss (r 9.5) sits on the horn face and a flange (r 12) starts 0.8 mm outboard of the plate. 4 holes of 2.5 mm on the horn bolt circle with 4.2 mm counterbores (M2 x 8 button heads). 2 holes of 4.0 mm for M3 heat-set inserts on the wheel holes (z +/-6.3 from the axle); M3 x 8 holds the wheel. The ST3215-HS has no horn in the box: buy 2 metal disc horns (assumed 25T, the same as the STS3215 horn).
- Horn screw holes in the upper links and the dogleg: 2.5 mm (tested M2 size, was 2.19).
- Dogleg: the post between the two plates is 14 mm in diameter (was 10 mm), with 3 mm collars (r 10 to r 7) at both ends.
- Leg range: 65 to 140 mm. At 145 mm the gap between the lower links is 0.4 mm. At 150 mm they touch.
- Checked: every printed body is one solid; plate holes are 0.04 mm from the servo holes; hub holes are 0.02 mm from the horn holes; dogleg to tray 6.2 mm at 65 mm; wheel servo to tray 3.9 mm at 65 mm.

## S53 (2026-10-28): knees and cable holes
- Each knee pin now goes through 6 mm of plastic on each link (was 4 mm). The upper links have a 2 mm boss on the inner face. The lower front plates have a 2 mm boss on the outer face. The outer plate of the dogleg is 6 mm thick (was 4 mm).
- Knee screw: M3 x 16, washer between the links, one nut, threadlocker on the screw tip only.
- The lower front plates have 2 pairs of 2.6 mm holes for cable ties. Tie the wheel servo cable to the lower front plate. This plate does not move relative to the wheel servo, so the cable does not bend there.
- Clearance check over the leg range 65..140 mm: the knee screw parts are 5.9 mm or more from the body; the dogleg is 4.3 mm or more from the wheel and hub; the lower links are 2.3 mm apart at 140 mm.
- S54 (2026-10-28): the yaw servo was upside down in the model. The driven horn pointed down into the lid and the bracket sat on the free idler. Now the horn faces up. The bracket base is 4.8 mm thick, with a 0.3 mm horn pocket, 4 x 2.5 mm holes and a 7 mm centre hole. The lid has a window under the servo plugs. S54b: the lower 2 screw holes in each bracket side had been filled in S48. They are cut again.
- S55: the head ribs had no room for the bracket screw heads. Horn side: two 4.2 mm grooves in the rib pad. Idler side: the rib inner face is 1.5 mm further out, the rib is 1.5 mm thicker, and there are grooves in the pad and a groove for the idler pin collar. The web beside each rib has a 1 mm notch so that every horn screw takes a washer. Checked with the screw heads modelled: the head slides on with no clash, 0.6 mm clearance at all pitch angles, and an L-key reaches all 8 horn screws from below.
- S56 (2026-10-28, review): the slide-on fit of the head was too tight. The rib pads sat 0.05 mm from the bracket side plates, and the pad grooves sat 0.1 mm from the bracket screw heads. Now each pad face is 0.4 mm further from the servo disc (the 4 screws pull the pad onto the disc), and the grooves are 5.6 mm wide. Slide-on clearance is now 0.45 mm to the plates and 0.7 mm to the screw heads. The bracket did not change. Head 82 g.
- S60/S61 (2026-10-28): the STS3215 horn holes take M3 (Feetech: M3 x 6; SO-101 guide: M3 x 6 into every horn, M2 x 6 into the case). Before S60 every horn joint had 2.5 mm M2 holes. Now: head ribs, upper links, rear doglegs, wheel hubs and the bracket base have 3.5 mm holes. Use M3 x 8 button heads. Each screw goes about 3 mm into the horn. S61b: links and doglegs have no counterbore (the S61 1 mm counterbores let the screw go 5 mm into the horn); put a 1 mm washer under each M3 x 8 head. Hubs have 6.3 mm counterbores. The idler rib of the head has 1.5 mm bosses.

## S62 (2026-10-28): chassis fixes after the pre-print review
The read-only review of plate 5 found four faults that stop the print. All are fixed in the chassis (S62) and the tray (S62b). The upper deck and the lid did not change.
- Hip servo pads: the spine wall was cut back 1.2 mm over the full servo length (S46), so the servo back plate floated 0.9 mm off the wall and the screws pulled it in at an angle. Now each wall has four pads (x +/-30.5..35 and +/-55.2..60, z +/-12.4) at y +/-16.2 that touch the servo back plate (gap 0.01 mm left, 0.08 mm right). The 3.1 mm socket relief stays over the real sockets (x 35..42).
- Upper hip screw heads: the four upper holes (z +10.2) were 1.2 mm under the spine roof, so an M2 button head and washer could not sit on the wall. Now the roof has eight pockets (r 2.8, y 10.6..13.5) over the upper holes. Checked with the head and washer modelled: no overlap, L-key channel free.
- Spine walls: extended 3 mm down (z -15.4) so the lower holes have 4 mm of wall under them (was 0.95 mm). The 0.5 mm lips at x +/-59.5..60 are removed.
- Deck columns: the four columns (x +/-67, y +/-35) were 4 x 8 mm with a 4 mm insert hole, so the insert had no wall. Now they are 9 x 9 mm (2.5 mm wall all round the M3 insert hole, 8.5 mm deep) with a notch at the outer corner so they stay 0.5 mm from the tray.
- Battery door bosses: the two insert holes (x -68, z 25) had 1 mm of wall to the battery cavity. They moved out to y +/-29 and sit in a block that reaches y +/-34 (2.8 mm wall each side). The tray door holes (3.5 mm) moved to y +/-29 with them.
- Deck floor over the battery: a slot (x -55..55, y +/-12) under the Pi removes the 48 mm bridge that no support could reach. The Pi still has its four pilots.
- Pico: moved 1.5 mm toward the centre and 2 mm up (bbox min now 44.0, -21.5, 199.5 world). It sits on four 6 mm bosses with 1.8 mm pilots, 7 mm deep, so the screws stop 1.5 mm above the battery gap. The tray USB window moved up 2 mm with it. The old pilots are filled.
- Pi pilots: 7 mm deep (were 9 mm and broke into the battery cavity wall).
- Checked before the plate was built: chassis one solid, 89.3 cm3, 111 g; chassis to tray 0.50 mm, chassis to lid 0.88 mm, tray to lid 0.00 (they touch by design); no overlap with any part; outer-skin scan shows only the intended openings; the thin-wall scan shows only the 2 mm rib beside the idler hole, which is unchanged since S45.
- Guard: the Pico transform and want-table entry in s57_normalise_guard.py are updated.
- Print note (after the independent re-check): the two 12 mm deck strips beside the Pi slot hang over the battery space for 111 mm, so plate_5_chassis.3mf now has tree supports everywhere (not only from the build plate). The trees stand on the chassis floor inside the battery space, in the side bays and in the idler holes.

## S63 (2026-10-29): chassis split in two for printing
The plate 5 print came off the bed and broke. Cause: the one-piece chassis stood on the two 3 mm bottom edges of the spine walls (685 mm2, 5 % of its outline), with the 144 x 92 mm floor 28 mm up on tree supports and a second level of supports on the floor for the deck. No orientation of the one-piece chassis puts a large face on the bed, because the floor is in the middle of the part.
- The chassis is now two bodies, split at the top face of the floor (z 14.4): chassis_lower (floor + spine walls, z -15.4..14.4, 39.2 cm3, 49 g) and chassis_upper (battery cradle walls, deck floor, columns, z 14.4..61, 54.4 cm3, 68 g). The hip servo mounting, the battery cradle, the deck floor and the columns are unchanged.
- Joint: four posts on the upper body, x -72..-61.5 (merged with the battery door boss block) and x 61..70, y +/-24..34, z 14.4..37.5. Each post has a 4.0 mm hole, 6.5 mm deep, in its bottom for an M3 x 5 x 4 heat-set insert at (-66.75, +/-29) and (65.5, +/-29). The floor has 3.5 mm holes at the same points. Fasteners: 4 x M3 x 8 button head from below. The screw heads sit under the floor, in the empty corners beside the hip servos. The battery door insert holes (y +/-29, z 25) are re-cut through the posts and stay 2.1 mm from the post insert holes.
- Checked: both bodies one solid; PLA 1.24; guard OK; no overlap between the new bodies and any part (tray, lid, deck, battery, fuse holder, XT60 splice, UBECs, Pi, Pico, cooler, IMU, adapter, hip servos, keep-outs); four modelled M3 x 8 screws with heads overlap nothing; chassis to tray 0.50 mm, to lid 0.88 mm, to deck 0.00 (by design); thin-wall scan shows only the pre-existing spots (spine wall ends, socket reliefs, Pico pilots). STL volumes match Fusion within 0.03 %.
- Print: plate_5a_chassis_lower prints floor down (12,900 mm2 on the bed, 30 mm tall, no supports, 5 mm outer brim). plate_5b_chassis_upper prints standing on its walls and posts (47 mm tall, tree supports from the build plate under the deck floor, 5 mm brim outer and inner). The old plate_5_chassis.3mf must not be printed.

## S64 (2026-10-29): spine wall strips (plate 5a print)
The first plate 5a print lost the strip between one idler circle and its socket window. The strip is 0.9 mm wide (x 35.1..36.0) and 1.8 mm thick (inside the servo relief), and it stands 20 mm tall in the print. The idler disc (19.2 mm) fixes the circle and the sockets fix the window, so the strip cannot be wider. Fix in chassis_lower only:
- Four 1 mm ribs on the inside of the spine behind the strips (x +/-35.1..36.0, y +/-12.5..13.5, z -7.5..7.5). The strip is now 2.8 mm thick. The outside face, the servo pads, the relief and the window do not change. The idler disc (max x 34.66) and the upper screw heads (lowest point z 8.45 at that x) stay clear.
- The four idler circles have a flat top on the side printed last (hip -z): 8 mm wide at z -10.3. A flat bridge prints cleanly; the round crest curled and pulled on the strip. The disc crest (z -9.6) gets 0.7 mm of room; 5.1 mm of wall remains to the free edge.
- Checked: one solid, 39.25 cm3, 48.7 g, PLA; guard OK; no overlap with the tray, lid, deck, battery, upper chassis, hip servos (idler discs included) or keep-outs; the 16 hip screw models give the same result as before S64; both ribs and all four flat tops present in the STL; bed contact and supports unchanged (floor down, no supports). plate_5a_chassis_lower.3mf rebuilt. Plate 5b unchanged. The first 5a print is usable as it is (the strip carries no load).

## S65 (2026-10-30): tray open slots
The body could not go down into the tray. With the horns on, the hip servos go out to |y| 51.41 (left) and 51.48 (right). The inside of the tray wall is at |y| 48, and the 4 servo windows were closed at the top. Fix in the tray only:
- Each of the 4 servo windows is now a slot that is open to the top edge: x 14.4..60.55 and -60.55..-14.4, |y| 45.9..52, from z -12.65 (window bottom, not changed) to the top. The 0.6 mm inner rim over each window (|y| 46.5, z 12.65..13.2) is removed with it. The floor, the window bottoms, the end walls, the 28.8 mm centre wall between the slots, the rear door holes and the top rim at the ends do not change. The Pico USB window (+y face, x 47.5..61.5, z 42..51) is inside the front-left slot area, so it is now part of that slot. Only a 0.95 mm notch of it stays in the slot end (x 60.55..61.5). USB access is the same or better.
- The body (chassis, deck, Pi, Pico, power parts, head and neck) now goes straight down into the tray with the 4 hip horns on.
- Checked: tray one solid (1 lump, 1 shell), 92.10 -> 75.11 cm3, 93.1 g, PLA 1.24, bounding box not changed (x -75..75, y -50..50, z -32.5..60); guard OK (2 separate runs); no overlap with any part or with the legs in the current pose. Drop path: the body set lifted 0..80 mm in 1 mm steps has no overlap with the tray at any step. The same is true with the 4 upper links on and turned straight up (1.45 mm from the slot edge at the outer face); the lid then cannot go on, so fit the links after the body is in the tray, as before. Clearances: horn disc to slot edge 1.03 mm (front) and 0.97 mm (rear); servo case to the window bottom ledge 0.51 mm; case ends to the slot ends 0.52 mm and 0.63 mm. Lid on the tray: no overlap. The lid sits on 6 tray-top patches (about 363 mm2): the 2 centre posts (x -14.4..14.4) and the 4 end corners (|x| 60.55..75). The 4 hip-plug keep-outs touch the tray end walls by 0.45 mm (3.7 mm3 each, at x +/-71.65..72.1); this was the same before S65 and the cut does not go near it. The Pico USB keep-out goes into the USB window, so plug in the USB cable after the body is in the tray.
- Print: plate_7_tray.3mf rebuilt with the same settings (STL volume within 0.07 % of Fusion, height 92.5 mm). No new overhang that build-plate trees cannot reach; the 8 window-top bridges are gone.

## S66 (2026-10-30): fuse holder zip-tie holes in the tray
The real fuse holder (ATC blade, in-line, hinged cap, 65 x 25 x 18 mm, wires out of both ends) does not fit in the left side bay (free space 46 x 23.5 x 22 mm). It goes on the outside of the tray left wall (+y), on the centre post, lying along x: x -32.5..32.5, y 50.5..68.5, z 14.4..39.4. Its wires go in through the rear-left slot. Two zip ties hold it.
- Tray only: 4 holes, 4.5 mm (x) x 2.5 mm (z), through the +y wall (y 48..50) at x -8 and +8, z centres 11.5 and 42.5. Each hole has 4.15 mm of wall to the post edges (x +/-14.4). The -y wall does not change.
- The tie runs inside the wall between each upper and lower hole. Gap from the tray inner face (y 48.00) to the nearest part (chassis_lower and chassis_upper at y 46.00) is 2.00 mm. A tie is 1.2 mm thick.
- Checked: tray one solid (1 lump, 1 shell), 75113.7 -> 75023.7 mm3 (exactly the 4 holes, 90.0 mm3), 93.0 g, PLA 1.24, bounding box not changed; guard OK (2 separate runs); no overlap with any part (the 4 hip-plug keep-outs still touch the end walls by 3.7 mm3, as before S65). Drop path 0..80 mm: no overlap, with or without the upper links pointing up. Holder box: no overlap with any part, at least 2.9 mm above the upper-link sweep over 65..140 mm, 20.6 mm below the lid.
- The S65 outer-face scan points on the bottom fillets were mesh noise: the exact surface there is 2.45 mm inside the face plane (the scan limit was 2.5 mm), and all of them are below z -17, where no cut was made.
- Print: plate_7_tray.3mf rebuilt with the same settings (STL volume -0.067 % against Fusion, height 92.5 mm). The 4 hole tops are 4.5 mm bridges (36 mm2 that build-plate trees cannot reach); they print without support.


## S67 (2026-10-31): lid walls 12 mm taller for the plugs and cables
Mofe found that the lid pressed down on the cables and pins of the IMU and the bus servo adapter. Measured in S66: the lid inner ceiling was 235.5 mm (world z), and 233 to 233.5 mm near the edges and over the centre band. The deck top is 223.0 mm.
- Adapter servo sockets: top 228.0. A plug and the cable bend need to about 237 to 238. Clash.
- Adapter 3-pin and jumper pins: top 231.1. A Dupont lead and its bend need to about 242.6. Clash.
- IMU header (not in the model) with a Dupont lead and its bend: about 242.6. Ceiling over the IMU 233.0 to 235.5. Clash.
- UBEC boxes: top 232.0, 1.5 mm to the lid. No change needed.
The plug and lead sizes are standard sizes (Dupont housing 14 mm, header spacer 2.5 mm, bend about 4 mm). They are not measured on the real parts.

Change (Mofe chose this on 2026-10-31):
- Lid: the walls are 12 mm taller. The lid was cut at z 221.8 (above the side M3 holes, which end at 221.75, and below the edge fillet). The top part moved up 12 mm and the wall section was extruded 12 mm to join the two parts. All features above the cut (top, fillet, yaw servo seat, cable window, servo screw holes) are the same as before, 12 mm higher. All features below the cut (side M3 holes, rear notch, tab fits) did not change.
- The yaw servo screws to the lid top, so the Neck and Head occurrences moved up 12 mm (transform z 15.5 -> 16.7 cm). The guard (s57_normalise_guard.py) has the new GOOD and WANT values.
- New lid ceiling: 247.5 mm (245 to 245.5 near the edges). Room above each plug and lead: at least 2.4 mm.
- Lid: one solid, 44.85 -> 56.04 cm3 (+11.19 cm3), 55.6 -> 69.5 g, PLA 1.24. Robot about 1499 g in Fusion (bodies without the coupon).
- The yaw servo cable and the OAK USB cable must be 12 mm longer than before.
- Sim: the RL model must move the yaw axis, the yaw servo, the bracket and the head 12 mm up and add 14 g to the torso.
- Files: biped_v02_S67.f3d/.step, stl/lid.stl, plates/plate_8_lid.3mf, scripts/s67_lid_plus12.py (with s67_lid_local.step), s67_export.py.
