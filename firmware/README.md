# Milestone 1: drive it from your phone, with the tail light

What you get: you hold a throttle pad on your phone, the rear wheels ramp up smoothly, letting go ramps to a stop, and the 8×8 tail light shows an arrow, a flicker while slowing, and solid when stopped. Luka steers by hand; the steering motor is just unplugged for now. Uses only parts you own, plus a fuse if the car doesn't already have one.

An Xbox controller comes next (milestone 2, via Bluepad32). The phone works with zero extra installs, so it goes first.

## Fuses and switches, in plain terms

- **Fuse = a deliberate weak link.** If a wire ever gets pinched and shorts, a 12V battery can push 100+ amps. The fuse melts in a fraction of a second instead of your wiring catching fire. You need exactly one, on the battery's **red** wire, as close to the battery as possible.
  - **First, look:** follow the car's red battery wire. Many ride-ons already have an inline fuse holder or a small black "thermal breaker" in it. If yours does, keep it and you're done.
  - If not: buy an **inline blade fuse holder (12 or 14 AWG)** and a **20A blade fuse** (any auto parts store, about $8).
  - The extra 2A fuse for the buck is a nice-to-have. Skip it for this milestone.
- **Switch = a way to turn everything off.** For now, **unplugging the battery connector is your off switch**, and the phone's E-STOP is the instant software stop. Before Luka drives regularly, add a proper switch (the car's own, if it carries the main power, or a 30A rocker).

## Wiring for this milestone (battery unplugged the whole time)

Heavy wires: use stock wire salvaged from the car, or two 18 AWG wires twisted together.

| From | To | Wire |
|---|---|---|
| Left motor red, right motor red | Wago 3-port #1 | motor leads |
| Left motor black, right motor black | Wago 3-port #2 | motor leads |
| Wago #1 free hole | BTS7960 **M+** | heavy |
| Wago #2 free hole | BTS7960 **M−** | heavy |
| Battery + | fuse (stock or new 20A) | heavy |
| Fuse | Wago 5-port "+12V bus" | heavy |
| +12V bus | BTS7960 **B+** | heavy |
| +12V bus | buck red **input** lead | 18 AWG |
| Battery − | Wago 5-port "GND bus" | heavy |
| GND bus | BTS7960 **B−** | heavy |
| GND bus | buck black **input** lead | 18 AWG |
| Buck red **output** (5V) | Wago 3-port → ESP32 **5V** pin and MAX7219 **VCC** | 18 AWG / jumper |
| Buck black **output** | ESP32 **GND** | jumper |
| ESP32 **3V3** | BTS7960 **VCC** | jumper |
| ESP32 **GND** | BTS7960 **GND** and MAX7219 **GND** | jumper |
| ESP32 **GPIO6** | BTS7960 **R_EN** and **L_EN** (tie them together) | jumper |
| 10 kΩ resistor (starter kit) | from R_EN to GND on the driver header | resistor |
| ESP32 **GPIO4** | BTS7960 **RPWM** | jumper |
| ESP32 **GPIO5** | BTS7960 **LPWM** | jumper |
| ESP32 **GPIO11** | MAX7219 **DIN** | jumper |
| ESP32 **GPIO12** | MAX7219 **CLK** | jumper |
| ESP32 **GPIO10** | MAX7219 **CS** | jumper |

The interactive tutorial (`../wiring-academy.html`) walks through each of these with pictures. This milestone is phases 1–4 and 6; skip 7 and 8 for now.

## Flash the ESP32-S3

1. Install **Arduino IDE 2**. In Boards Manager, install **esp32 by Espressif Systems**.
2. Open `mclaren_ecu/mclaren_ecu.ino`. `secrets.h` (your Wi-Fi password) is already in that folder. It's gitignored, so it never goes to GitHub. To change the password, edit it there.
3. Tools → Board → **ESP32S3 Dev Module**. Plug into the board's **COM** USB port (or the **USB** port, with Tools → USB CDC On Boot → **Enabled** to see the serial log).
4. Upload. Open Serial Monitor at 115200 and you'll see `McLaren ECU up`.

Do steps 1–4 with the ESP32 on USB and nothing else connected. The tail light can be connected; it's safe.

## First drive test

1. **Rear wheels off the ground** (car on a box). Nobody in the seat.
2. Multimeter: with the battery plugged in and the buck output disconnected, the buck output reads **4.9–5.2V**. Unplug the battery, then connect the buck output.
3. Plug in the battery. The tail light goes solid (stopped). Nothing spins.
4. Phone: join Wi-Fi **McLaren-F1** (password in `secrets.h`), then open **http://192.168.4.1**.
5. Push the pad up a little. Both wheels should turn forward.
   - **One wheel backward:** unplug the battery, swap that motor's red and black at the Wagos.
   - **Both backward:** set `INVERT_DRIVE = true` in the sketch and re-upload.
6. Let go: the wheels ramp down, and the tail light flickers, then goes solid. Press **E-STOP**: everything stops instantly. Then **Reset**.
7. Turn the phone's Wi-Fi off mid-throttle. The car must stop within half a second. **Don't put Luka in it until this works.**

## Safety built into the firmware

- Motors are disabled before anything else runs at boot, and the 10k pull-down holds them off while the board starts.
- Speed is capped at 30% by default. The phone slider goes up to 80%, and `HARD_MAX_DUTY` is the ceiling the phone can't exceed.
- Soft-start and soft-stop ramps. Forward/reverse changes only happen at zero speed.
- No command from the phone for 0.5 s → ramp to stop. Letting go of the pad, locking the phone, or leaving the page → stop.
- Only one phone can drive at a time. E-STOP works from any phone.

Tuning knobs are at the top of the sketch: `DEFAULT_MAX_DUTY`, `ACCEL_PER_S`, `DECEL_PER_S`, `INVERT_DRIVE`, and the matrix flip flags.
