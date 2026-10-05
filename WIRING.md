# Rover wiring — ESP32 + L298N + 4 × 12V motors

## Power chain

```
NEAKO 3S1P 18650 pack, 12.6V full / 9.0V empty
   |
   +-- female barrel jack on the pack
         |
   [male barrel plug -> bare red/black adapter cable]
         |
   red  ------ [switch] -----> L298N "12V" terminal
   black -------------------> L298N "GND" terminal
                                     |
                              L298N "5V" pin (output, jumper ON)
                                     |
                                     +-----------> ESP32 VIN pin

ESP32 GND -------------------------> L298N "GND"   <-- mandatory
```

No separate converter is needed. The L298N has a 78M05 regulator on board that
makes 5V from the 12V input, and that 5V pin can run the ESP32.

### The XL6009 is not usable here
XL6009E1 is a **boost** (step-up) converter — it only raises voltage. It cannot
make 5V from 12V. Leave it out of this project entirely.

### If the ESP32 keeps resetting, this is why
The 78M05 is a linear regulator burning (12.6 − 5) × current as heat, and it is
rated ~500mA while ESP32 WiFi transmit peaks near that. Symptoms of it running
out: random reboots, "Brownout detector was triggered" on the serial monitor, or
WiFi that drops when the motors start.

The fix is an **LM2596 buck module** (~₹60) between the battery and the ESP32:
set its output to exactly 5.0V with a multimeter *before* connecting it, feed
it from the 12V rail, take the 5V-EN jumper off the L298N, and run its output
to the ESP32 VIN. Worth buying, but you can get everything working first
without it.

## Connecting the NEAKO pack

The pack ends in a barrel jack, and the second cable is the mating plug with
bare red/black tails. Plug them together and the bare ends screw straight into
the L298N's 12V and GND terminals. Nothing else is required.

### Meter the bare ends first — every time

Barrel connectors are not standardised for polarity, and the wire colours on a
cheap adapter cable are not a promise. Before those tails go anywhere near the
driver:

1. Plug the adapter into the battery.
2. Multimeter on DC volts, red probe on the red tail, black on the black tail.
3. You want **+12.x V**. A *negative* reading means the colours are reversed —
   swap which tail goes to which terminal.

Reverse polarity kills an L298N instantly and it does not fail gracefully.
Thirty seconds with a meter is cheaper than the board.

### Put a switch in the red wire
Cut the red tail and wire a rocker or toggle switch (rated 5A+) into the break.
Unplugging a barrel jack to stop a moving rover is not a control you want to
rely on, and repeated plugging wears the connector out.

### About the 2.2A on the label
That pack is 3S**1P** — a single string of cells, so its current capability is
whatever one 18650 can deliver, and the label prints 2.2A. Four 12V gearmotors
starting together will ask for considerably more than that.

Expect this to work for gentle driving on a light chassis and to complain when
you load it: the BMS cuts out on overcurrent, so the symptom is the whole rover
dying instantly and coming back when you release the throttle. If that happens
it is the battery, not your code, and the answer is a pack with a higher
discharge rating (a 2P or 3P build) rather than anything in software.

### Charging
Charge only with a **12.6V lithium (3S) charger**. A generic 12V DC adapter is
not a charger — it has no cell balancing and no termination, and using one on a
lithium pack is how these catch fire. Never leave it charging unattended.

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
| Running standalone | L298N 5V | connected, USB unplugged |

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
