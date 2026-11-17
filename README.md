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

## Pin map (ESP32-S3 DevKitC-1)

| GPIO | Function | Notes |
|---|---|---|
| 4 / 5 | Rear RPWM / LPWM | ~16–20 kHz LEDC PWM |
| 6 | Rear R_EN+L_EN | 10k pull-down to GND |
| 7 / 15 | Steering RPWM / LPWM | |
| 16 | Steering R_EN+L_EN | 10k pull-down; LOW = limp |
| 1 | Steering pot wiper | ADC1_CH0, 100 nF to GND |
| 11 / 12 / 10 | MAX7219 DIN / CLK / CS | FSPI default pins |
| 13 / 14 / 17 / 18 | Pedal / Fwd-Rev / High-Low / Manual-RC | INPUT_PULLUP, switch to GND |
| 2 | (later) battery voltage | 100k/22k divider, ADC1 |

Avoid: 0, 3, 45, 46 (strapping) · 19/20 (USB) · 43/44 (serial) · 35–37 (PSRAM on N8R8/N16R8) · the RGB LED pin (38 or 48). Spares: 8, 9, 21, 39–42, 47.

Power: battery+ → 20A fuse → switch → +12V bus → (rear B+, steering B+, 2A fuse → buck IN+). Battery− → GND bus → (rear B−, steering B−, buck IN−). Buck 5V → 3-port Wago → ESP32 5V and MAX7219 VCC. ESP32 3V3 → both driver VCC pins and the pot (via a breadboard rail).

## Firmware must-haves

EN LOW at boot · soft-start ramp (~1 s) · direction change only at zero speed · remote disconnect → ramp to stop · parent e-stop overrides the pedal · speed cap · never drive RPWM and LPWM together unless braking deliberately · steering soft end-stops from pot calibration.

## Parts status (2026-11-16)

Have: WAGO 221 28-pc assortment (221-2401 inline, 221-412, 221-413, 221-415 5-port), HiLetgo BTS7960 (IBT-2), ESP32-S3 (a small board + a long DevKitC-1-style board), 10–60V→5V 3A IP67 buck, MAX7219, Arduino starter kit (10k resistors, 220 Ω, LEDs, pushbuttons, 10k pot, breadboard, jumpers), 18 AWG wire, solder.

Buy before first power-up: a multimeter, a 20A inline blade fuse holder + fuse (unless the stock harness has one), a 2A inline fuse, and female-ended dupont jumpers if the kit has only male-male. (The Wago kit already covers the two 5-port buses.)
Buy for steering (Phase 8): a second BTS7960, plus a proper shafted 10k pot for the column (the kit pot is fine for bench testing).
Optional: an Xbox Wireless controller (BLE) for the parent remote.

## Next

1. Bench rehearsal (tutorial Phase 4, step 2): S3 on the breadboard over USB, LEDs on GPIO 4/5, a pushbutton as the pedal, the kit pot on GPIO1.
2. Photos of the stock board and harness: where the motor, steering, pedal and shifter wires land, and whether there's a stock fuse or breaker and a power switch.

## Deploy

Run `./deploy.sh` with Wrangler authenticated to the Cloudflare account that manages `tech619.com`. The script uploads the tutorial as the site index.
