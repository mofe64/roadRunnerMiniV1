# Road Runner Mini: bill of materials (S70)

## Actuators

| Part | Qty | Use | Link |
|---|---|---|---|
| Feetech STS3215 12 V servo (30 kg.cm) | 6 | 4 hips, head yaw, head pitch | [amazon.co.uk](https://www.amazon.co.uk/Serial-Bus-Servo-ST3215-Programmable/dp/B0B6P9271Q) |
| Waveshare ST3215-HS servo (20 kg.cm) | 2 | Wheels | [The Pi Hut](https://thepihut.com/products/serial-bus-servo-20kg-cm) |

## Electronics

| Part | Qty | Use |
|---|---|---|
| Raspberry Pi 5 + active cooler | 1 | Main computer |
| microSD card | 1 | Pi |
| Luxonis OAK-D Lite | 1 | Depth camera |
| Adafruit LSM6DS3TR-C IMU | 1 | Balance |
| Waveshare Bus Servo Adapter (A) | 2 | Bus 1 hips + head, bus 2 wheels |
| Nuofany 8 A UBEC, set to 5.2 V | 1 | Pi power |
| 3S LiPo 5200 mAh, 135.6 x 43.5 x 22.1 mm | 1 | Battery |
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
| Female Dupont lead, 2-pin | 1 | UBEC to Pi 5 V and GND pins |
| Bus servo cable (Molex 5264 3-pin) | 8 | Servo chain (in the servo boxes) |
| Heat-shrink tube | 1 set | Splices |
| Kapton tape | 1 roll | Bus adapter 2 |
| Zip ties, 2.5 mm | 20 | Fuse holder, cables |
| Double-sided foam tape | 1 roll | UBEC, bus adapter 2 |

## Fasteners

| Part | Qty | Where |
|---|---|---|
| M2 x 8 button head | 24 | 16 hip servo backs, 4 lower front plates, 4 Pi 5 |
| M2 washer | 16 | Hip servo screws |
| M2 x 6 pointed screw (servo bag) | 12 | 4 yaw servo to lid, 8 pitch servo in bracket |
| M2 x 6 | 6 | Bus adapter 1 and IMU on the upper deck |
| M3 x 8 button head | 64 | 16 upper links, 8 lower rear links, 8 hubs, 4 bracket, 8 head, 20 into inserts |
| M3 washer, 1 mm | 28 | 16 upper links, 8 lower rear links, 4 knees |
| M3 x 12 button head | 4 | Wheel to hub |
| M3 x 16 button head | 4 | Knee pivots |
| M3 nyloc nut | 4 | Knee pivots |
| M3 x 5 x 4 heat-set insert | 24 | 10 deck/lid/battery door, 4 deck tab pockets, 4 chassis corner posts, 2 tray front pads, 4 hubs |
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
| plate_5a_chassis_lower | chassis lower (S69) | 30 % | 58 |
| plate_5b_chassis_upper | chassis upper (S70) | 30 % | 88 |
| plate_6_upper_deck | upper deck (S70) | 30 % | 32 |
| plate_7_tray | tray (S70) | 30 % | 97 |
| plate_8_lid | lid (S67) | 30 % | 69 |
| Total | 17 parts | | 530 |

## Materials

| Part | Qty |
|---|---|
| Bambu PLA Basic, 1 kg spool | 1 |
| Pololu 1425 wheel, 70 x 8 mm | 2 |
