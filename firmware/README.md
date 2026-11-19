# Stage 1a: pedal, dash switches, phone control and tail light

What you get:
- Luka drives with the **pedal**. The **Fwd/Rev** and **High/Low** switches work like stock (Low = half speed, reverse capped at 60%).
- **Your phone overrides him.** Hold the throttle pad to drive, or press E-STOP to stop everything. By default the pedal only works while your phone page is open, so you're always supervising.
- Soft-start and soft-stop ramps. The tail light shows an arrow while driving, flickers while slowing, and goes solid when stopped.
- The phone page shows every switch live, so you can test the dashboard before the motors are even connected.

Steering, two options until the second BTS7960 arrives (stage 1):
- **Manual:** the steering motor is unplugged and Luka steers by hand.
- **Hybrid (half old, half new):** the stock board stays in the car for steering only (see below), so the stock remote still steers.

Uses only parts you own, plus a fuse if the car doesn't already have one.

## Hybrid option: keep the stock board for steering

The stock board can keep doing just one job, steering, while the ESP32 takes over everything else.

- **Stays on the stock board:** its battery feed (Wago it onto the +12V and GND buses, so it's behind the main fuse) and the steering motor (the small 2-pin plug with **blue and brown** wires, in the top socket on the board's right edge). Its receiver for the stock remote is built in.
- **Moves to the ESP32:** both rear motors (through BTS7960 #1), the pedal, Fwd/Rev and High/Low. Unplug them from the stock board, and cap each now-unused stock-board motor lead on its own Wago, so nothing bare can touch.
- **Important:** the stock remote's go and stop buttons now do nothing, because the rear motors aren't on that board anymore. **Your phone's E-STOP is the parent stop.** Use the stock remote for steering only.
- Check after the swap that the stock remote still steers. Some stock boards won't respond without their pedal connected; if yours is one, use manual steering until stage 1.

An Xbox controller comes next (milestone 2, via Bluepad32). The phone works with zero extra installs, so it goes first.

## Fuses and switches, in plain terms

- **Fuse = a deliberate weak link.** If a wire ever gets pinched and shorts, a 12V battery can push 100+ amps. The fuse melts in a fraction of a second instead of your wiring catching fire. You need exactly one, on the battery's **red** wire, as close to the battery as possible.
  - **First, look:** follow the car's red battery wire. Many ride-ons already have an inline fuse holder or a small black "thermal breaker" in it. If yours does, keep it and you're done.
  - If not: buy an **inline blade fuse holder (12 or 14 AWG)** and a **20A blade fuse** (any auto parts store, about $8).
  - The extra 2A fuse for the buck is a nice-to-have. Skip it for this milestone.
- **Switch = a way to turn everything off.** For now, **unplugging the battery connector is your off switch**, and the phone's E-STOP is the instant software stop. Before Luka drives regularly, add a proper switch (the car's own, if it carries the main power, or a 30A rocker).

## Which board?

The firmware supports two boards and picks the right pins automatically when you build for one. The wiring page has a **Board** selector at the top, which switches every pin number on the page.

| Signal | Classic ESP32 (WROOM-32 / DOIT) | ESP32-S3 DevKitC-1 |
|---|---|---|
| Rear RPWM / LPWM | **16 / 17** (may be printed RX2 / TX2) | 4 / 5 |
| Rear enable (R_EN + L_EN) | **4** | 6 |
| Tail light DIN / CLK / CS | **23 / 18 / 5** | 11 / 12 / 10 |
| Dash plug: orange / blue / yellow / green / white | **32 / 33 / 19 / 21 / 22** | 13 / 14 / 17 / 18 / 8 |
| Steering RPWM / LPWM / enable (stage 1) | **25 / 26 / 27** | 7 / 15 / 16 |
| Steering pot (stage 2) | **34** | 1 |
| 5V in | **VIN** (or 5V) | 5V |

The table below uses S3 numbers; on a classic board, use the left column.

## Wiring for this milestone (battery unplugged the whole time)

Heavy wires: 14 AWG (red to +, black to −). Everything else: 18 AWG or jumpers.

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
| Buck **USB socket** (USB-output converters) | ESP32 **USB port**, with a USB cable | USB cable: powers the ESP32 and shares ground |
| *or* buck red **output** lead (5V) | ESP32 **5V/VIN** | 18 AWG / jumper |
| *or* buck black **output** lead | ESP32 **GND** | jumper |
| ESP32 **VIN** | MAX7219 **VCC** | jumper (VIN outputs 5V when the ESP32 is USB-powered) |
| ESP32 **3V3** | BTS7960 **VCC** | jumper |
| ESP32 **GND** | BTS7960 **GND** and MAX7219 **GND** | jumper |
| ESP32 **GPIO6** | BTS7960 **R_EN** and **L_EN** (tie them together) | jumper |
| 10 kΩ resistor (starter kit) | from R_EN to GND on the driver header | resistor |
| ESP32 **GPIO4** | BTS7960 **RPWM** | jumper |
| ESP32 **GPIO5** | BTS7960 **LPWM** | jumper |
| ESP32 **GPIO11** | MAX7219 **DIN** | jumper |
| ESP32 **GPIO12** | MAX7219 **CLK** | jumper |
| ESP32 **GPIO10** | MAX7219 **CS** | jumper |
| Dash plug **orange** (pedal + Forward) | ESP32 **GPIO13** | splice to a female jumper |
| Dash plug **blue** (pedal + Reverse) | ESP32 **GPIO14** | jumper |
| Dash plug **yellow** (turtle) | ESP32 **GPIO17** | jumper |
| Dash plug **green** (remote mode) | ESP32 **GPIO18** | jumper |
| Dash plug **white** (power button) | ESP32 **GPIO8** | jumper |
| Dash plug **black** and **pink/red** | ESP32 **GND** (two GND pins) | jumpers |

**The dash on this car is one 7-wire plug** that went into the stock board. The pedal's brown wires run into the dash and chain through the Fwd/Rev rocker, so the ESP32 sees "go forward" (orange) and "go backward" (blue) directly. Unplug the dash plug from the stock board and splice each of its 7 wires to a jumper; you never open the dash. With the stock board out, nothing in that plug carries 12V.

The interactive tutorial (`../wiring-academy.html`) walks through each of these with pictures. In the page's **Build stages** tab, this is **Stage 1a** (press Play to watch it go in).

## No laptop at the car: garage mode

Firmware updates don't need a laptop at the car. In garage mode the car joins your home Wi-Fi as well as making its own network, so new firmware goes over the air from the Mac mini.

1. Edit `mclaren_ecu/secrets.h` (on this Mac, gitignored). Fill in your home Wi-Fi name and password and uncomment those two lines. `OTA_PASSWORD` is already set. **Wi-Fi names are case-sensitive**, and the classic ESP32 only sees 2.4 GHz networks. If it can't connect, the serial log says which of those it is.
2. Flash once over USB (below). After that, updates go over Wi-Fi while the car sits in the garage, as long as your home Wi-Fi reaches it.
3. **`http://mclaren.local`** on your home Wi-Fi opens the **telemetry dashboard**: a lamp for each dash control, the drive output, phone and Wi-Fi links, system health (including whether the last restart was a brownout), and a live event log. It's view-only; the only button is E-STOP. On the car's own Wi-Fi the same address is the driving page. Raw data: `/telemetry`.

Safety stays the same: **driving and E-STOP reset only work from the car's own Wi-Fi** (your phone). Home-network devices can read status and press E-STOP, but can't drive. Starting an update forces the motors off first.

USB is only needed for the first flash, or if the car can't reach your home Wi-Fi.

## Telemetry dashboard

The car serves its own dashboard, so it works anywhere, with no internet needed:

- **On the car's Wi-Fi (McLaren-F1), driving or at the park:** the phone page has **Drive | Telemetry** tabs. Telemetry shows every dash control as a lamp, the drive output, links, system health (restart reason, loop rate, memory) and a live event log. **Leaving the Drive tab stops the car**, because the phone stops sending commands. That's the link-loss safety working.
- **On the home Wi-Fi:** http://mclaren.local opens the dashboard directly, view-only.
- **Add to Home Screen** (Safari share menu) to get either page as an app-style icon.

**Adding your own sensor** is one line in `setup()`; it appears in the dashboard's "More sensors" card automatically:

```cpp
addMetric("Battery", "V", [] { return analogReadMilliVolts(35) * 6.55f / 1000; }, 2);   // name, unit, how to read it, decimals
```

Raw data for scripts: `/telemetry` (JSON).

## Flash it

One script does both kinds of flashing (it needs `arduino-cli` and the esp32 core, both installed on the Mac mini):

```
./flash.sh classic usb     # first flash over USB
./flash.sh classic ota     # every update after that, over Wi-Fi (garage mode)
./flash.sh s3 usb          # same for the ESP32-S3
```

`secrets.h` (the car's Wi-Fi password, your home Wi-Fi and the update password) is already in `mclaren_ecu/`. It's gitignored, so it never goes to GitHub.

Prefer the Arduino IDE? Install **esp32 by Espressif Systems** in Boards Manager, open `mclaren_ecu/mclaren_ecu.ino`, and pick **ESP32 Dev Module** (classic) or **ESP32S3 Dev Module** (S3). Serial Monitor at 115200 shows `McLaren ECU up on <board>`.

Do steps 1–4 with the ESP32 on USB and nothing else connected. The tail light can be connected; it's safe.

## Test the dashboard (USB only, no battery)

The wiring page's **Build stages → Stage 0 · bench test** shows exactly this setup and a table of what each control should read.

With the ESP32 on USB and the dash plug wired, join the car's Wi-Fi and open 192.168.4.1. Or stay on your home Wi-Fi and open **http://mclaren.local**: it shows the same readout, **view only** (driving is ignored from the home network). The grey line under the speed shows each switch live:

`power ON | pedal down FORWARD | HIGH | Luka can drive`

Press the pedal in Forward and in Reverse, flip rabbit/turtle, turn the dial and press the power button, and watch it change. The settings already match what the multimeter showed on this car (yellow closes in turtle, green closes in remote). If anything reads backwards, flip `HIGH_WHEN_CLOSED` or `RC_ONLY_WHEN_CLOSED` at the top of the sketch. With no motor power connected, it's completely safe.

**Power button off = car disabled,** phone included, like stock. If you test before wiring the dash plug, the page will say `power OFF`. Set `USE_POWER_BUTTON = false` to test without it.

## First drive test

1. **Rear wheels off the ground** (car on a box). Nobody in the seat.
2. Multimeter: with the battery plugged in and the buck output disconnected, the buck output reads **4.9–5.2V**. Unplug the battery, then connect the buck output.
3. Plug in the battery. The tail light goes solid (stopped). Nothing spins.
4. Phone: join Wi-Fi **McLaren-F1** (password in `secrets.h`), then open **http://192.168.4.1**.
5. Push the pad up a little. Both wheels should turn forward. Then let go and press the pedal: the wheels should ramp up on their own, at half speed in LOW.
   - **One wheel backward:** unplug the battery, swap that motor's red and black at the Wagos.
   - **Both backward:** set `INVERT_DRIVE = true` in the sketch and re-upload.
6. Let go: the wheels ramp down, and the tail light flickers, then goes solid. Press **E-STOP**: everything stops instantly. Then **Reset**.
7. Turn the phone's Wi-Fi off mid-throttle. The car must stop within half a second. **Don't put Luka in it until this works.**

## Safety built into the firmware

- Motors are disabled before anything else runs at boot, and the 10k pull-down holds them off while the board starts.
- A pedal held down at boot, after an E-STOP, or when the power button comes on, is ignored until Luka lets go once.
- Orange and blue on at the same time (a wiring fault) is ignored.
- Your phone always beats the pedal. Set `PEDAL_NEEDS_PHONE = false` only when you want it to drive with no phone at all, like stock.
- Speed is capped at 30% by default. The phone slider goes up to 80%, and `HARD_MAX_DUTY` is the ceiling the phone can't exceed.
- Soft-start and soft-stop ramps. Forward/reverse changes only happen at zero speed.
- No command from the phone for 0.5 s → ramp to stop. Letting go of the pad, locking the phone, or leaving the page → stop.
- Only one phone can drive at a time. E-STOP works from any phone.

Tuning knobs are at the top of the sketch: `DEFAULT_MAX_DUTY`, `ACCEL_PER_S`, `DECEL_PER_S`, `INVERT_DRIVE`, `LOW_GEAR_FACTOR`, `REVERSE_FACTOR`, the switch polarity flags, `USE_POWER_BUTTON`, and the matrix flip flags.
