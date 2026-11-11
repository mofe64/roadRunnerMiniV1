# Road Runner Mini: bill of materials (S72)

This document is written in ASD-STE100 Simplified Technical English.

## Actuators

| Part | Qty | Use | Link |
|---|---|---|---|
| Feetech STS3215 servo, 7.4 V version | 6 | 4 hips, head yaw, head pitch | |
| Waveshare ST3215-HS servo (20 kg.cm) | 2 | Wheels | [The Pi Hut](https://thepihut.com/products/serial-bus-servo-20kg-cm) |

## Electronics

| Part | Qty | Use |
|---|---|---|
| Raspberry Pi 5 + active cooler | 1 | Main computer |
| microSD card | 1 | Pi |
| Luxonis OAK-D Lite | 1 | Depth camera |
| Adafruit LSM6DS3TR-C IMU | 1 | Balance |
| Waveshare Bus Servo Adapter (A) | 2 | Bus 1 hips + head, bus 2 wheels |
| Nuofany 8 A UBEC | 2 | UBEC 1, set to 5.2 V: Pi power. UBEC 2, set to 7.4 V: bus 1 (hip and head servos) |
| 3S LiPo 5200 mAh, 135.6 x 43.5 x 22.1 mm, 277 g | 1 | Battery |
| Eye LED + resistor | 2 | Head |

## Wiring

| Part | Qty | Use |
|---|---|---|
| XT60 connector pair | 3 | Battery lead, splice |
| Inline blade fuse holder (GTIWUNG) | 1 | Battery lead |
| 10 A ATO blade fuse | 1 | Battery lead |
| 16 AWG silicone wire, red + black | 1 m each | Power |
| USB-A to USB-C cable, 20-30 cm | 2 | Pi to bus adapters |
| USB 3 cable, USB-C to USB-A | 1 | Pi to OAK-D Lite |
| Adafruit 4397 STEMMA QT to female Dupont cable | 1 | Pi to IMU |
| Female Dupont lead, 2-pin | 1 | UBEC 1 to the Pi 5 V and GND pins |
| Bus servo cable (Molex 5264 3-pin) | 8 | Servo chain (in the servo boxes) |
| Heat-shrink tube | 1 set | Splices |
| Zip ties, 2.5 mm, 150 mm or longer | 20 | 2 fuse holder, 4 UBECs, cables |
| Double-sided foam tape | 1 roll | UBECs |

## Fasteners

| Part | Qty | Where |
|---|---|---|
| M2 x 8 button head | 28 | 16 hip servo backs, 4 lower front plates, 4 Pi 5, 4 IMU plate |
| M2 washer | 16 | Hip servo screws |
| M2 x 6 pointed screw (servo bag) | 12 | 4 yaw servo to lid, 8 pitch servo in bracket |
| M2 x 6 | 10 | 8 bus adapters 1 and 2 on the upper deck, 2 IMU on the IMU plate |
| M3 x 8 button head | 60 | 16 upper links, 8 lower rear links, 8 hubs, 4 bracket, 8 head, 16 into inserts (4 chassis joint, 4 tray, 4 upper deck, 4 lid) |
| M3 washer, 1 mm | 28 | 16 upper links, 8 lower rear links, 4 knees |
| M3 x 12 button head | 4 | Wheel to hub |
| M3 x 16 button head | 4 | Knee pivots |
| M3 nyloc nut | 4 | Knee pivots |
| M3 x 5 x 4 heat-set insert | 20 | Chassis upper 12 (4 corner posts, 4 deck columns, 2 rear bosses, 2 front pads), 4 upper deck end pockets, 4 hubs |
| M4 x 10 button head | 2 | OAK-D Lite |
| M4 washer | 2 | OAK-D Lite |
| Threadlocker, medium | 1 | Knee screw tips |

## Printed parts (PLA)

| Plate | Parts | Infill | Mass (g) |
|---|---|---|---|
| plate_1_head | head | 30 % | 70 |
| plate_1b_bracket_flat | pitch bracket | 40 % | 12 |
| plate_2_upper_links_hubs | 4 upper links, 2 wheel hubs | 100 % | 32 |
| plate_3_lower_front | 2 lower front plates | 100 % | 28 |
| plate_4_lower_rear | 2 lower rear links | 100 % | 45 |
| plate_5a_chassis_lower | chassis lower | 30 % | 58 |
| plate_5b_chassis_upper | chassis upper | 30 % | 88 |
| plate_6_upper_deck | upper deck | 30 % | 33 |
| plate_7_tray | tray | 30 % | 97 |
| plate_8_lid | lid | 30 % | 69 |
| plate_9_imu_plate | IMU plate | 100 % | 4 |
| Total | 18 parts | | 536 |

## Materials

| Part | Qty |
|---|---|
| Bambu PLA Basic, 1 kg spool | 1 |
| Pololu 1425 wheel, 70 x 8 mm | 2 |
