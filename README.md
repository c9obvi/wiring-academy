# McLaren F1 — ESP32 ride-on conversion

Huffy 12V McLaren F1 ride-on (model 17154). The stock control board comes out, and an ESP32 becomes the controller. Two BTS7960 drivers handle the motors, and a pot on the steering column gives closed-loop steering that can also go limp for manual driving.

- **[wiring-academy.html](wiring-academy.html)**: interactive top-down wiring tutorial. Open it in Chrome or Safari. It has 51 wires in build order, quizzes, safety checklists, simulators (motor direction, drive + tail light, steering PID), a printable pin map, and the plan review.
- **[Live Wiring Academy](https://wiring-academy.tech619.com/)**
- **[firmware/](firmware/README.md)**: milestone 1 sketch (rear drive + tail light from a phone over the ESP32's own Wi-Fi), with the minimal wiring list, flashing steps and first-drive test.

## Review of the original plan (2026-11-16)

The core architecture is right: an ESP32 brain, a BTS7960 for the rear motors, Wagos to join the two rear motors in parallel, a Mzhou buck for 5V, a MAX7219 on hardware SPI, pull-up inputs for the pedal and switches, and the stock back-drivable steering gearbox with a pot and PID. **Phase 1 (motors → Wagos → driver M+/M−) is correct.**

Changes before building:

| Severity | Change | Why |
|---|---|---|
| High | BTS7960 **VCC to 3.3V**, not 5V | The IBT-2 board's 74HC244 input buffer at 5V wants about 3.5V for a HIGH. The ESP32 gives 3.3V, which is marginal. |
| High | **R_EN+L_EN on a GPIO with a 10k pull-down**, not tied to 3.3V | Gives a software kill switch, and the motors stay off while the ESP32 boots. |
| High | **Fuse → main switch → 5-port Wago +12V / GND buses**; a separate 2A fuse for the buck | The diagram had no switch and shared the B+ screw terminal with the buck feed. |
| High | Fully disconnect the pedal and dash switches from 12V; bypass the Fwd/Rev shifter if it carries motor current | 12V on a GPIO kills the ESP32. |
| Med | Verify motor direction with the wheels off the ground; swap one motor at the Wagos if needed | The motors are mirror-mounted, so wire color doesn't guarantee direction. |
| Med | Steering: explicit RC/Manual mode (optional idle timeout), not "stick centered = limp" | Centering the stick to go straight would make the steering floppy. |
| Med | Second BTS7960 for steering, not an L298N | The L298N drops about 2V and is limited to about 2A. |
| Med | Board: the **long ESP32-S3 (DevKitC-1 style)** runs the car; pins below are remapped for it | The small S3 has too few free pins for 14 signals. The S3 is BLE-only, so use an Xbox Wireless controller (PlayStation won't pair). |
| Med | 18 AWG is fine except the four heavy runs (battery → driver, Wagos → driver): salvage stock harness wire or double up 18 AWG, 20A main fuse | Two motors' current through one 18 AWG run gets hot. |
| Low | MAX7219 VIH is 3.5V; add a 74AHCT125 if the display glitches | Long wire to the rear. |
| Low | Pot on 3.3V, wiper to an ADC1 pin (GPIO1) | The ADC tops out around 3.1V, and ADC2 conflicts with the radio. |
| Low | Screw-terminal ESP32 breakout, hot glue on headers | Vibration. |

## Pin map

The firmware builds for either board and picks its pins automatically. The wiring page's **Board** selector switches every pin number on the page.

| Function | Classic ESP32 (WROOM-32 / DOIT) | ESP32-S3 DevKitC-1 | Notes |
|---|---|---|---|
| Rear RPWM / LPWM | 16 / 17 | 4 / 5 | ~16–20 kHz LEDC PWM |
| Rear R_EN+L_EN | 4 | 6 | 10k pull-down to GND |
| Steering RPWM / LPWM | 25 / 26 | 7 / 15 | |
| Steering R_EN+L_EN | 27 | 16 | 10k pull-down; LOW = limp |
| Steering pot wiper | 34 | 1 | ADC1, 100 nF to GND |
| MAX7219 DIN / CLK / CS | 23 / 18 / 5 | 11 / 12 / 10 | hardware SPI pins |
| Pedal / Fwd-Rev / High-Low / Manual-RC | 32 / 33 / 19 / 21 | 13 / 14 / 17 / 18 | INPUT_PULLUP, switch to GND |
| (later) battery voltage | 35 | 2 | 100k/22k divider, ADC1 |

Classic, avoid: 6–11 (flash) · 0, 2, 12, 15 (strapping) · 1/3 (USB serial) · 34–39 for switches (no pull-ups).
S3, avoid: 0, 3, 45, 46 (strapping) · 19/20 (USB) · 43/44 (serial) · 35–37 (PSRAM on N8R8/N16R8) · the RGB LED pin (38 or 48). Spares: 8, 9, 21, 39–42, 47.

Power: battery+ → 20A fuse → switch → +12V bus → (rear B+, steering B+, 2A fuse → buck IN+). Battery− → GND bus → (rear B−, steering B−, buck IN−). Buck 5V → 3-port Wago → ESP32 5V and MAX7219 VCC. ESP32 3V3 → both driver VCC pins and the pot (via a breadboard rail).

## Stock harness map (from the car)

Stock board: **JR1630RX-RM2-12V** (12V, max 20A load, 2.4 GHz remote). Everything plugs in; nothing is soldered to the car.

| Plug | Wires | What it is | How we know |
|---|---|---|---|
| Small 2-pin, top socket on the board's right edge | blue + brown | **Steering motor** (remote steering) | Unplugged it: everything still worked except remote steering |
| Big white 2-pin | red (via the breaker box) + black | Battery feed to the board; becomes the new system's power source | Traced by hand |
| Big white 2-pin ×2 | orange + white on one motor, red + black on the other | Rear motors | Seen in photos; which is left/right still to label |
| Small multi-pin, bottom-left of the board | yellow, orange, blue, green, white, red, black… | Dash bundle: pedal, Fwd/Rev, High/Low, power button, dial | Seen; pin-by-pin map still to do |

Battery: its leads are sealed to the battery, so the battery plug is the only disconnect and the battery half of it is always live. Battery red → small black box (probably a self-resetting breaker; rating not read yet) → splits to the charge port and the board. Leave the box and the charge-port wire alone.

Still unknown: the breaker's rating, the yellow tag text, the multi-pin wire order, what the round dash dial does, and whether the red power button carries main current.

## Firmware must-haves

EN LOW at boot · soft-start ramp (~1 s) · direction change only at zero speed · remote disconnect → ramp to stop · parent e-stop overrides the pedal · speed cap · never drive RPWM and LPWM together unless braking deliberately · steering soft end-stops from pot calibration.

## Parts status (2026-11-17)

Update 2026-11-19: on hand now: a second BTS7960 and the 14 AWG red/black wire. The fuse kit and heat shrink arrive today. The buck converter has its own protection, so the separate 2A fuse is optional.

Have: WAGO 221 28-pc assortment (221-2401 inline, 221-412, 221-413, 221-415 5-port), HiLetgo BTS7960 (IBT-2), ESP32-S3 (a small board + a long DevKitC-1-style board), 10–60V→5V 3A IP67 buck, MAX7219, Arduino starter kit (10k resistors, 220 Ω, LEDs, pushbuttons, 10k pot, breadboard, jumpers), 18 AWG wire, solder, multimeter.

### Shopping basket (ordered for stage 1)

| # | Item | Why | Need? |
|---|---|---|---|
| 1 | [HiLetgo BTS7960 43A driver](https://www.amazon.com/HiLetgo-BTS7960-Driver-Arduino-Current/dp/B00WSN98DC) | Steering motor driver, identical to the one you have | **Must** |
| 2 | [Newdeli 10× 14 AWG inline fuse holders + 100 fuses (2A–35A)](https://www.amazon.com/Newdeli-Waterproof-Standard-Assorted-Automotive/dp/B0CRYWCKXX) | 20A main fuse, 2A buck fuse, 10A steering fuse, plus spares | **Must** |
| 3 | [BNTECHGO 14 AWG silicone wire, red + black, 10 ft](https://www.amazon.com/BNTECHGO-Flexible-Conductor-Resistant-Extension/dp/B08PBZG7QL) | The heavy runs: battery → fuse → buses → both drivers, and Wagos → M+/M− | **Must** |
| 4 | [ELEGOO 120 dupont jumpers (M-M, M-F, F-F)](https://www.amazon.com/Elegoo-EL-CP-004-Multicolored-Breadboard-arduino/dp/B01EV70C78) | Female ends for the driver headers and ESP32 pins | Only if the kit is male-male only |
| 5 | [30A on/off toggle switch, pre-wired, with boot](https://www.amazon.com/Toggle-Switch-12V-30A-Waterproof/dp/B0G4LNNGMY) | Main power switch; the pre-wired leads join with Wagos, so no crimping | If the car has no power switch that carries the main current |
| 6 | [Heat shrink tubing, 2:1 polyolefin assortment](https://www.amazon.com/560PCS-Heat-Shrink-Tubing-Kit/dp/B09NLJ5FHN) | Insulates every soldered joint (harness wire → dupont jumper) | **Must** if you solder |

Gauge notes: 14 AWG silicone is right for this car. It fits the Wago 221s (max 12 AWG) and the BTS7960 screw terminals, and comfortably carries the 20A fuse's current over these short runs. 12 AWG would be overkill and a tight squeeze in the driver terminals. Keep 18 AWG for the buck feed, the steering motor and the cockpit switches, and dupont jumpers for logic.

### Connecting the car's existing harness

Keep the car's wire runs; don't pull new ones through the car.

1. **Photograph and label every wire first** (tape flags: "PEDAL A", "L MOTOR red", and so on).
2. **Best: don't cut the connectors.** Unplug each harness connector from the stock board, identify its type (measure the pin pitch; it's often JST XH 2.54 mm or VH 3.96 mm), and buy matching pigtails. The harness then plugs into your adapters today and into your custom PCB later, with no cutting at all.
3. **Fallback, if a connector is odd:** cut about 5 cm behind it, so the connector keeps a short tail. To go back to stock, you just Wago the tails back on.
4. **Heavy wires (motors, battery):** Wagos, no solder.
5. **Signal wires (pedal, switches):** solder each one to half of a female dupont jumper, heat-shrink it, then plug onto the ESP32 pin. Don't Wago dupont wire: it's thinner than the Wago 221's 24 AWG minimum and won't clamp reliably.
6. **Strain relief:** zip-tie the harness near the board, so a yank pulls on the tie, not the joint.

Fuse map: **20A** on battery + (main) · **2A** on the buck feed · **10A** on the steering driver's B+ (optional).

Stage 2, later: a shafted 10k pot for the steering column (the kit pot is fine for bench testing) and a 3D-printed PETG bracket. Optional anytime: an Xbox Wireless controller (the S3 only pairs with BLE pads).

## Next

1. Bench rehearsal (tutorial Phase 4, step 2): S3 on the breadboard over USB, LEDs on GPIO 4/5, a pushbutton as the pedal, the kit pot on GPIO1.
2. Photos of the stock board and harness: where the motor, steering, pedal and shifter wires land, and whether there's a stock fuse or breaker and a power switch.

## Deploy

Run `./deploy.sh` with Wrangler authenticated to the Cloudflare account that manages `tech619.com`. The script uploads the tutorial as the site index.
