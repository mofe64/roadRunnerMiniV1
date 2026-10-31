# Parts list (2026-10-27)

This document is written in ASD-STE100 Simplified Technical English.

## Parts that we have
- Raspberry Pi 5 with active cooler.
- Luxonis OAK-D Lite (with IMU).
- Feetech STS3215 servo, 7.4 V, quantity 6: 4 for the hips, 1 for head pitch, 1 for head yaw.
- Pololu 1425 wheel, 70 x 8 mm, quantity 2.
- 3S LiPo battery, 5200 mAh, 277 g, 135.6 x 43.5 x 22.1 mm.
- Nuofany 8 A UBEC, quantity 2.
- Raspberry Pi Pico 2 W (spare, not fitted: the IMU connects to the Pi).
- Adafruit LSM6DS3TR-C IMU.
- Waveshare Bus Servo Adapter (A), quantity 2 (Mofe has both). Adapter 1: legs and head on the 7.4 V UBEC. Adapter 2: wheel servos on battery voltage. Adapter 2 is not in the CAD. Put it inside the body on the tray floor. Wrap it in heat-shrink or Kapton tape and hold it with a zip-tie or foam tape.
- XT60 connectors.
- 2 eye LEDs with resistors.

## Parts on order
- GTIWUNG inline blade fuse holder.
- 10 A ATO fuses.
- 16 AWG silicone wire.

## Parts to buy
- Waveshare ST3215-HS servo, quantity 2, for the wheels (The Pi Hut).
- Wheel servo horn: use one of the 2 aluminium disc horns that come in the ST3215-HS box (Pi Hut listing). Nothing to buy. The wheel hub bolts to this horn.
- Feetech or Waveshare bus servo cables. Do not use the Adafruit RC extension cables. They do not connect to the Molex 5264 sockets.
- Right-angle USB-C cable for the OAK-D Lite.
- TPU or rubber feet for the skids.

## Fasteners
All fasteners come from the button-head screw kit or from the servo bags. You do not have to buy more fasteners.

| Location | Fastener |
|---|---|
| Hip servo rear faces | M2 x 8 button head + M2 washer, driven with an L-key from inside the spine |
| Head pitch servo (bracket arms) and head yaw servo (lid) | M2 x 6 pointed self-tapping screws (from the servo bags) |
| Servo horns (all parts that bolt to a horn) | M3 x 8 button head. The horn holes take M3. The servo case holes take M2. |
| Pitch bracket to the yaw servo horn | M3 x 8 button head (used by Mofe, 2026-10-28) |
| Pi 5 | M2 x 8 button head, into 1.8 mm pilot holes |
| Bus servo adapter 1 and IMU (upper deck) | M2 x 6, into the 2.0 mm pilot holes (used by Mofe, 2026-10-30) |
| Upper deck, lid and battery door | M3 x 8, into M3 x 5 x 4 heat-set inserts (18 inserts: 10, plus 4 in the deck tab pockets for the 4 lid side screws, plus 4 in the chassis corner posts for the chassis joint (S63). Push the 4 pocket inserts in before assembly. An M3 nut does not fit.) |
| Leg link pivots (knees) | M3 x 16, a 1 mm washer between the links, one nut, threadlocker on the screw tip |
| Wheel to hub | M3 x 12, into 2 heat-set inserts in each hub |
| OAK-D Lite rear holes | M4 x 10 with a washer |

Do not use M2.5 screws.

## Parts that we do not use now
- Cytron MDD3A motor driver.
- Pololu level shifter.
- Pololu 25D gearmotors.

UK suppliers: The Pi Hut and amazon.co.uk.
