# Rover wiring — ESP32 + L298N + 4 × 12V motors

## Power chain

```
4S LiPo, 14.8 V nominal / 16.8 V full / 13.2 V empty (3.3 V per cell)
   |
   +-- main leads (XT60) -- [fuse] -- [switch] --+--> left  L298N "12V" terminal
   |                                             +--> right L298N "12V" terminal
   |   black ---------------------------------------> both L298N "GND" terminals
   |
   +-- balance plug -- 2S-6S balance-to-USB-C module (5 V out) --> Raspberry Pi 5
                                                                       |
                                                        USB --> ESP32 (powered by the Pi)

ESP32 GND -------------------------> L298N "GND"   <-- mandatory
```

- **The ESP32 is powered over USB from the Pi.** That cable is also the
  micro-ROS link, so the L298N 5V -> ESP32 VIN wire stays **disconnected** (see
  "USB and the 5V pin" below). Only GND joins the ESP32 to the drivers.
- **The 5V-EN jumper stays ON.** Each L298N's 78M05 then powers only its own
  logic, a few tens of mA. At 16.8 V that is about 0.4 W, which is fine. It
  would not be fine feeding the ESP32 as well: (16.8 - 5) V x 0.25 A is 3 W in a
  regulator with no heatsink.

### Motor voltage
The motors are rated 12 V. A full 4S pack minus the L298N's ~2 V drop puts up to
~14.8 V across them at 100 % duty, about 25 % over. Short bursts are fine; long
runs flat out make them hot. Nav2's speed limit (0.3 m/s against a ~0.55 m/s
top speed) keeps the duty well below full in normal driving.

### Check the balance-to-USB-C module once
"Type-C to balance head" cables exist in two kinds: battery -> 5 V USB output
(what this needs), and USB-C charger -> battery (for charging the pack). They
look identical. Before the first connection to the Pi, measure the USB-C end
with the module on the battery: it must read **5 V**, not 13-17 V.

Then, with the robot driving and the LiDAR spinning, check on the Pi:

```bash
vcgencmd get_throttled
```

`throttled=0x0` means no undervoltage. Anything else means the module cannot
supply the Pi 5 (it wants 5 V / 5 A); swap to a 5 V 5 A buck converter fed from
the main leads through a fuse.

### Low-voltage alarm
Nothing in this system stops the motors draining the pack below a safe voltage;
the L298Ns will keep pulling current. Fit a LiPo cell-voltage buzzer on the
balance plug (a balance splitter lets it share the plug with the USB-C module).

### Meter new leads before connecting
Wire colours on cheap adapters are not a promise. Multimeter on DC volts, red
probe on the red lead: you want **+14 to +16.8 V**. A negative reading means the
colours are swapped. Reverse polarity kills an L298N instantly.

### Put a switch in the red wire
A switch (rated 5 A+) in the positive lead, after the fuse. Pulling a connector
to stop a moving rover is not a control you want to rely on.

### Charging
Charge only with a **LiPo balance charger set to 4S**, through the balance plug.
Never leave it charging unattended, and never charge a puffed pack.

## L298N jumpers

| Jumper | Position | Why |
|---|---|---|
| ENA | **removed** | frees the pin for PWM from GPIO13 |
| ENB | **removed** | frees the pin for PWM from GPIO14 |
| 5V-EN | **left ON** | switches on the 78M05 so the 5V pin becomes an output |

## The three power terminals

The blue screw block has three positions. The middle one is straightforward;
the outer two are not what people expect.

| Terminal | Direction | Connect to |
|---|---|---|
| **12V** | input | battery **+** (through the switch) |
| **GND** | — | battery **−** *and* ESP32 GND |
| **5V** | **depends on the jumper** | ESP32 VIN |

The 5V terminal is the one that confuses everyone, because it reverses:

- **Jumper ON** — the 5V pin is an **output**. The onboard regulator makes 5V
  from the 12V input, and you take power *from* this pin. This is your setup.
- **Jumper OFF** — the 5V pin becomes an **input**. The regulator is bypassed
  and you must *feed* it 5V from elsewhere, or the driver's logic stays dead.

So with the jumper on: never connect anything that supplies power to the 5V
terminal. It is already supplying.

### GND is not optional
The ESP32 and the L298N must share a ground or the IN pins have no reference
voltage to switch against. Symptoms of a missing ground connection are motors
that twitch, run at the wrong speed, or ignore direction changes. This single
wire causes more confusion than everything else on the board combined.

## Full pin map

Go by the **GPIO number printed on the board**, not physical position — 30-pin
and 38-pin DevKits order them differently. `rover_control/pins.h` is the code's
copy of this table; change a wire, change that file.

### Wired now — L298N

| ESP32 | To | Function |
|---|---|---|
| GPIO 13 | ENA | left pair speed (PWM) |
| GPIO 26 | IN1 | left pair direction A |
| GPIO 27 | IN2 | left pair direction B |
| GPIO 14 | ENB | right pair speed (PWM) |
| GPIO 25 | IN3 | right pair direction A |
| GPIO 33 | IN4 | right pair direction B |
| GND | GND | **required** — no shared ground, nothing works |

### Reserved for sensors

| ESP32 | To | Notes |
|---|---|---|
| GPIO 34 | battery divider | ADC1, input-only |
| GPIO 4 / 16 / 17 | HC-SR04 TRIG L/C/R | one each — a shared trigger makes them hear each other |
| GPIO 36 / 39 / 35 | HC-SR04 ECHO L/C/R | **5V — divider required, see below** |
| GPIO 18 / 19 | cliff sensors L/R | TCRT5000, powered at 3.3V |
| GPIO 21 / 22 | MPU6050 SDA / SCL | ESP32 default I2C |
| GPIO 23 | vibration motor | through a transistor, never direct |
| GPIO 32 | buzzer | |
| GPIO 2 | onboard LED | status indicator |

Free: 0 (BOOT button), 5, 12, 15.

### Why these pins and not others

| Pins | Reason |
|---|---|
| 6–11 | flash memory. Using them stops the board booting |
| 34–39 | **input only** — no output, no pullup. Fine for ECHO and ADC, useless for TRIG |
| 0, 2, 5, 12, 15 | strapping pins, read at boot. GPIO12 held HIGH at reset sets flash to 1.8V and the board won't boot |
| 1, 3 | USB serial. Use them and you lose the Serial Monitor |
| 0, 2, 4, 12–15, 25–27 | ADC2 — `analogRead()` on these **fails whenever WiFi is on**. This is why battery sense is on 34, not a nearer pin |

## ⚠️ HC-SR04 ECHO is 5V — it will damage the ESP32

The sensor needs 5V on VCC to work reliably, and its ECHO pin therefore swings
to 5V. ESP32 GPIOs are 3.3V. Every ECHO line needs a divider:

```
ECHO ---[1k]---+---[2k]--- GND
               |
          ESP32 GPIO   (5V * 2/3 = 3.3V)
```

TRIG needs nothing — 3.3V from the ESP32 is enough to trigger it.
This is the single most common way these boards get killed.

## Motors — 4 × brushed DC, 2 wires each

Skid steer: no steering linkage. You turn by running the two sides at different
speeds, so all four motors reduce to two controlled groups — **left** and **right**.

Polarity cannot damage a brushed DC motor. Red/black is only a convention for
which way it turns. If a wheel goes the wrong way, swap its two wires.

### The one thing that catches everyone

The motors on the left and right of the chassis are **mirror images** of each
other. Wire all four identically and the right side will spin backwards when the
left side goes forward — the rover spins on the spot instead of driving.

Fix: wire one side with red and black **swapped**. Below, the right side is the
swapped one.

### Option B — two L298N boards, one board per side (recommended)

Each motor gets its own H-bridge channel, so nothing shares a 2A limit.
Call them **board L** (left side) and **board R** (right side) — mark them
with tape so you never have to trace a wire to work out which is which.

#### Motors

Wire all four **identically**. Do not try to pre-compensate for the left and
right motors being mirror images — that gets fixed in software afterwards,
and guessing at it here just means unscrewing terminals later.

| Board | Motor | Red → | Black → |
|---|---|---|---|
| L | front-left | OUT1 | OUT2 |
| L | rear-left  | OUT3 | OUT4 |
| R | front-right | OUT1 | OUT2 |
| R | rear-right  | OUT3 | OUT4 |

#### Control pins

**Tie both channels of each board to the same ESP32 pin.** One GPIO drives two
L298N inputs without trouble, so a side pair moves as one unit while still
getting two separate H-bridges' worth of current headroom. Six wires total,
the same six you already have.

| ESP32 | Board L | Board R |
|---|---|---|
| D13 | ENA **and** ENB | — |
| D26 | IN1 **and** IN3 | — |
| D27 | IN2 **and** IN4 | — |
| D14 | — | ENA **and** ENB |
| D25 | — | IN1 **and** IN3 |
| D33 | — | IN2 **and** IN4 |

Remember the ENA/ENB pins are the **inner** pin of each 2-pin pair — the one
next to IN1 / IN4. The outer pin is +5V and stays empty.

#### Splitting one GPIO into two pins

A dupont connector only fits one pin, so each of those six wires has to fan
out to two. Three ways, pick whichever you have parts for:

1. **Mini breadboard** — one wire from the ESP32 into a row, two wires out of
   that row to the two board pins. Tidiest, easiest to change.
2. **Twist and tape** — strip three wire ends, twist together, heatshrink or
   tape. No extra parts.
3. **Crimp two wires into one dupont housing** — neatest if you own a crimper.

Do not try to stack two female connectors on one male pin. They will look
connected and one of them will not be.

#### Jumper caps

| Board | ENA cap | ENB cap | 5V-EN |
|---|---|---|---|
| L | off | off | **on** |
| R | off | off | **on** |

ENA and ENB caps come off both boards — you are driving those pins from the
ESP32 now, and a cap ties them to +5V where the GPIO is trying to drive them.

**Both boards keep their own 5V-EN cap.** That jumper enables the onboard
regulator, and that 5V is what powers the L298 chip's own logic — remove it
and the board has no logic supply and will not respond at all.

The thing to avoid is not two regulators existing, it is two regulators tied
together: **never run a wire between the two boards' 5V pins.** Each powers
only its own board.

For untethered running, take the ESP32's VIN from **one** board's 5V pin,
either one. The other board's 5V pin stays empty.

#### Power

| From | To |
|---|---|
| Battery + (via switch + fuse) | 12V on **both** boards |
| Battery − | GND on **both** boards |
| Board L GND | ESP32 GND |
| Board L 5V | ESP32 VIN — untethered running only, never with USB plugged in |

#### Code changes

Two flags in `rover_control/motors.h`:

```cpp
constexpr bool SWAP_SIDES = false;   // was true for the single-board wiring
constexpr bool INVERT_LEFT  = false; // set by the direction test below
constexpr bool INVERT_RIGHT = false;
```

`SWAP_SIDES` becomes **false** because board L is now on the D13/D26/D27 pin
group, which the code already calls "left".

Reset both INVERT flags to `false` and re-run the direction test — the old
`INVERT_RIGHT = true` was calibrated for the single-board wiring and no longer
applies. Expect exactly one side to need inverting.

### Option A — one L298N, two motors in parallel per channel

Workable only if your motors are small. Both motors on a channel share one 2A limit.

| Terminal | Connect |
|---|---|
| OUT1 | front-left red **+** rear-left red |
| OUT2 | front-left black **+** rear-left black |
| OUT3 | front-right **black** + rear-right **black** |
| OUT4 | front-right **red** + rear-right **red** |

⚠️ Measure stall current before running under load. Two 12V gearmotors stalling
together will pass 4–6A through a 2A channel; the L298N thermal-shuts-down if
you are lucky and dies if you are not.

## As currently wired (single L298N)

| Terminal | Connected |
|---|---|
| OUT3 | front-left red + rear-left red |
| OUT4 | front-left black + rear-left black |
| OUT1 | front-right red + rear-right red |
| OUT2 | front-right black + rear-right black |

So the **left** side sits on channel B (ENB/IN3/IN4 = GPIO 14/25/33) and the
**right** side on channel A (ENA/IN1/IN2 = GPIO 13/26/27) — the opposite of the
code's default. That is already handled: `SWAP_SIDES = true`.

Both sides also have red on the lower-numbered terminal. Because the left and
right motors are mirror images on the chassis, one side will therefore run
backwards. Which one depends on how you physically mounted them, so don't
guess — run the test below and set the flag to match.

### Direction test — wheels off the ground

Nothing here needs a screwdriver. All three corrections live at the top of
`motors.h` (or `rover_base.ino` for the bench sketch):

```cpp
constexpr bool SWAP_SIDES   = true;
constexpr bool INVERT_LEFT  = false;
constexpr bool INVERT_RIGHT = false;
```

1. Rover on a box, wheels free. Flash `rover_base`, Serial Monitor at 115200,
   line ending "No line ending".
2. Press `w` and watch what the wheels actually do.

| What happens on `w` | Fix |
|---|---|
| All four drive forward | Nothing. Done. |
| Left pair forward, right pair backward | `INVERT_RIGHT = true` |
| Right pair forward, left pair backward | `INVERT_LEFT = true` |
| All four drive backward | set **both** INVERT flags true |
| One single wheel disagrees with its partner | swap that one motor's two wires |
| `a` and `d` are swapped, but `w`/`s` are right | flip `SWAP_SIDES` |

3. Re-flash and re-test until `w` drives forward, `s` reverses, `a` spins left
   and `d` spins right.
4. Copy the final flag values into `rover_control/motors.h` so both sketches agree.

Only put it on the floor once all four commands are correct.

## ⚠️ USB and the 5V pin are two power sources — never both at once

While you are developing, the ESP32 is powered over **USB**. Once it drives
itself around, it is powered from the **L298N 5V pin**. Those are alternatives,
not a pair.

Having USB plugged in *and* the 5V wire connected ties your laptop's 5V rail to
the L298N's regulator, and the two will fight — one backfeeds the other.

| Stage | ESP32 power | 5V wire to VIN |
|---|---|---|
| Flashing / bench testing | USB | **disconnected** |
| Running with the Pi (micro-ROS, the normal setup) | USB from the Pi | **disconnected** |
| Running standalone (`rover_control`, no Pi) | L298N 5V | connected, USB unplugged — only below 12.6 V input; on 4S use a buck converter instead |

The **GND wire stays connected in both cases.** That one is never removed.

## First bench test — wheels off the ground

1. Rover on a box, wheels free-spinning.
2. Motors + L298N on **battery**. ESP32 on **USB**, 5V wire off. Grounds tied.
   Keeping the ESP32 on USB during testing means motor current spikes can't
   brown it out while you are debugging.
3. Flash `rover_base/rover_base.ino`, open Serial Monitor at 115200,
   set line ending to **"No line ending"**.
4. Press `t` for the automatic test sequence.
5. Confirm forward is actually forward and both sides match before it touches
   the floor.

## Known-good Arduino IDE setup

- Boards Manager → install **esp32 by Espressif Systems**
- Tools → Board → **ESP32 Dev Module**
- Upload Speed 921600, Flash Frequency 80MHz, Partition Scheme "Default 4MB"
- If upload fails with "Failed to connect": hold **BOOT** while it says
  *Connecting...*, release when it starts writing.
- If the port doesn't appear: install the CP210x or CH340 USB driver
  (depends on which chip your board carries).
