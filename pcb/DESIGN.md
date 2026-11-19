# Ride-on ECU board: design brief (draft v0.1, 2026-11-19)

Goal: one board that converts **any** 12V kids' ride-on, not just this McLaren. The builder snips the car's own connectors off and lands each wire on a standard, easy-to-buy connector on our board. Nothing car-specific is on the board; the car-specific part is a wiring table (like the McLaren's in `../README.md`).

## Scope for v1

- **Logic carrier, not a power board.** Motor current stays on the BTS7960 modules (off-board, 8-pin cable to each). The PCB only ever carries small signals and the ESP32's 5V, which keeps the first board simple and safe.
- **ESP32 plugs in, never soldered:** female headers for a 30-pin ESP32 DevKit (DOIT V1 style). Check both row widths (0.9" and 1.0" boards exist); consider a second footprint for the ESP32-S3 DevKitC-1.
- **Every input connector carries its own GND** (and 3V3 where needed), so there's no "3 grounds into 1 pin" problem anywhere.

## Connectors (proposal, to review)

| Job | Connector | Why |
|---|---|---|
| Switch inputs (pedal/direction, speed, mode, power, spares) | **Pluggable screw terminal blocks, 3.81 mm** (e.g. 15EDG), one signal + GND per input | No crimp tool; takes 26–16 AWG, so the car's own wire or our 18 AWG; plugs off for service; on Amazon and LCSC everywhere |
| 12V in (for the onboard 5V, battery sense) | 2-pin pluggable terminal **5.08 mm** (or XT30) | Same no-crimp idea, heavier wire |
| BTS7960 drivers ×2 | 8-pin **2.54 mm header**, same order as the module (RPWM LPWM R_EN L_EN R_IS L_IS VCC GND) | One straight ribbon/dupont cable per driver; can't mix up pin order |
| Tail light (MAX7219) | 5-pin 2.54 mm header or 3.81 mm terminal | Matches the module's header |
| Steering pot | 3-pin terminal (3V3, wiper, GND) | |
| Future sensors | **Qwiic / STEMMA QT** (JST-SH 4-pin I²C) + a few spare GPIO on terminals | Plug-and-play sensor ecosystem, no soldering |
| USB | ESP32's own port | Already there |

"JST" note: JST XH (2.5 mm) is the common hobby connector, but doing it right needs a crimp tool (or pre-crimped pigtails). Screw terminals keep it buildable by anyone with a screwdriver. We can offer XH as a footprint option later.

## On the board

- Pull-ups plus an RC filter (10k / 100 nF) and ESD protection on every switch input.
- 10k pull-downs on both drivers' enable lines (motors stay off during boot).
- 74AHCT125 level shifter for the MAX7219 (3.3V → 5V logic).
- Battery-voltage divider (100k / 22k) into an ADC1 pin.
- 5V from either the ESP32's USB, or an onboard buck from 12V with reverse-polarity protection and a resettable fuse (decide: onboard buck vs. external USB buck).
- A status LED, test points on 3V3/5V/GND, clear silkscreen labels at every connector.

## Open questions

1. Onboard 12V→5V buck, or keep the external USB converter?
2. ESP32 classic only, or dual footprint with S3?
3. Board size/mounting: under the seat, screw holes, or a 3D-printed case on the P1S?
4. Fab: JLCPCB with assembly (SMD parts placed for us) vs. through-hole we solder by hand.
