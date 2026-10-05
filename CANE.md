# Haptic cane — design spec

The user-facing half of the guide dog. The rover decides where to go; the cane
tells the person. Four directions plus stop, communicated by vibration on the
grip. No screen, no sound.

---

## How the person feels direction

**Spatial mapping.** Four small vibration motors set into the grip at 90°
intervals — front, back, left, right. The person feels *where* the grip
vibrates, and that is the instruction. Nothing to memorise.

The palm's two-point discrimination threshold is roughly 10 mm. On a 34 mm
grip the motors sit ~27 mm apart around the circumference, comfortably
distinguishable — provided the shell does not smear the vibration between
them (see "Crosstalk" below).

### Suggested vocabulary

| Instruction | Pattern |
|---|---|
| Go forward / all clear | Silence. Default state |
| Turn left / right | Steady buzz on that side. Stronger = sharper turn |
| Obstacle ahead | Front motor pulsing. Faster pulses as it gets closer, like a parking sensor |
| Stop | All four, strong, pulsing |
| Step back | Back motor, steady |
| Link lost | All four, three short pulses, repeating — the person needs to know the cane has gone blind |

Silence-as-forward is deliberate: a cane that buzzes constantly gets tuned
out within minutes.

---

## Parts

| Item | Spec | Approx ₹ |
|---|---|---|
| Microcontroller | **ESP32-C3 Super Mini** | 150–200 |
| Vibration motors ×4 | **ERM coin motor, 10 mm dia × 2.7 mm, 3 V** | 30–50 each |
| Motor driver | **ULN2003** (7-channel Darlington array, one DIP chip) | 20 |
| Battery | Single **18650** cell or 800–1000 mAh LiPo pouch | 100–150 |
| Charger | **TP4056** — the one you already have. It is a 1S charger; this is a 1S cane | 0 |
| Slide switch | any | 10 |
| Shaft | see below | 200–500 |
| Foam grip tape | bicycle handlebar tape, or foam sheet | 50 |
| PLA | grip + tip, ~60 g | ~50 |

**Total: roughly ₹700–1200.** Yes, it is cheap.

### Why these

**ERM coin motors** are the vibration motors in every phone. Apply voltage,
they buzz. Flat, adhesive-backed, embed easily in a printed pocket. ~80 mA
each at 3 V. LRAs are crisper but need a resonant driver chip (DRV2605L,
~₹300 each) — not worth it here.

**ULN2003** because a GPIO cannot drive a motor. Each coin motor wants
~80 mA; an ESP32 pin safely gives ~12 mA. The ULN2003 sinks up to 500 mA per
channel, has flyback diodes built in, and takes 3.3 V logic directly. One
chip drives all four motors with zero extra parts. PWM through it works, so
intensity is controllable.

**ESP32-C3 Super Mini** because the cane must talk to the rover wirelessly,
and both ends being ESP32 unlocks **ESP-NOW** — Espressif's peer-to-peer
radio. No WiFi network, no pairing, no router, ~200 m range, a few ms
latency, and about 20 lines of code each side. Sending a packet is one
function call. It coexists with the rover ESP32's micro-ROS link because
that runs over USB.

A full-size ESP32 DevKit works too but is three times the size and draws more
idle current.

---

## Power

Single-cell: 3.7 V nominal, 4.2 V full, 3.0 V empty.

```
18650 (+) --[switch]--+--> TP4056 B+     (charging)
                      +--> ESP32-C3 "5V" pin   (onboard LDO makes 3.3 V)
                      +--> ULN2003 COM pin, motors' common (+)

18650 (-) ------------+--> TP4056 B-
                      +--> ESP32-C3 GND
                      +--> ULN2003 GND
```

The motors run straight off the cell through the ULN2003 — at 3.7 V minus the
Darlington's ~1 V drop they see ~2.7 V, right in their range. No regulator
needed for them.

Draw: ESP32-C3 ~30 mA idle, ~100 mA transmitting; each motor ~80 mA when on.
Duty is low (motors are off most of the time), so a 2000 mAh 18650 runs the
cane for a day easily.

The TP4056's OUT+/OUT- pins are the protected output — take the ESP32 and
ULN2003 supply from there rather than straight off the cell, so the board's
over-discharge cutoff protects the cell.

---

## Wiring the motors

```
ESP32-C3 GPIO  -->  ULN2003 IN1..IN4
ULN2003 OUT1..OUT4  -->  motor (-) lead of motors 1..4
ULN2003 COM  -->  battery (+)   (enables the internal flyback diodes)
all motor (+) leads  -->  battery (+)
ULN2003 GND  -->  battery (-)
```

The ULN2003 is a low-side switch: it connects the motor's negative lead to
ground when the input goes high. Motor positive stays permanently on the
battery rail.

Any four GPIOs on the C3 work. Avoid GPIO 8 and 9 (strapping pins) if you
want it to boot reliably with the motors attached.

---

## The 3D printed part — and the part you should not print

### Do not print the shaft

PLA is brittle. A 1.2 m PLA tube will crack the first time it is dropped or
leaned on. Your printer's bed is ~250 mm, so it would be five sections glued
together — every joint a guaranteed failure point. Real white canes are
aluminium, fibreglass or carbon for exactly this reason.

**Buy the shaft.** Either:

- a 12–16 mm aluminium tube from a hardware store (~₹200), or
- a cheap folding white cane (~₹300–500) and discard its handle

Then print only the **grip** and, if you want one, a **tip**.

### The grip

A cylinder ~34 mm diameter, ~130 mm long, with:

- a hollow core for the battery, C3 and ULN2003
- four shallow pockets around the outside at 90°, each 11 mm dia × 3 mm deep,
  for the coin motors, leaving a **1.0–1.2 mm wall** between motor and skin
- a socket at one end to press-fit or glue the shaft
- a removable end cap for the battery and the TP4056's USB port

### Print settings

| | |
|---|---|
| Material | PLA is fine for the grip. PETG if you want it tougher |
| Layer height | 0.2 mm |
| Walls / perimeters | **3** (1.2 mm at a 0.4 nozzle). This is what gives it strength; infill barely matters |
| Infill | **20–25 % gyroid**. Not 100 % — heavier, no stronger in any way that matters, and it damps vibration |
| Top/bottom layers | 4 |
| Orientation | Standing upright. Round cross-section prints true, and grip loads are compression not bending |
| Supports | Only for the motor pockets if they are undercut; design them to open outward and you need none |

"Density" in slicer terms is infill percentage. 20–25 % is right. The wall
count is the number that actually determines how the grip feels in the hand.

### Crosstalk — the design problem that actually matters

A rigid PLA shell carries vibration everywhere. Fire the left motor and the
whole grip buzzes; the person cannot tell left from right. This is the thing
that makes or breaks a haptic grip, and most first attempts get it wrong.

Three fixes, use at least two:

1. **Slots between zones.** Cut four 2 mm slots through the shell wall,
   running the length of the grip, between adjacent motor pockets. Each
   motor now sits on its own quarter-shell, mechanically decoupled from the
   others.
2. **Thin windows.** Keep the wall over each motor at 1.0–1.2 mm. Thin PLA
   transmits vibration to the skin efficiently; thick PLA spreads it.
3. **Foam overgrip.** Wrap the finished grip in thin foam tape. Foam couples
   each motor to the skin locally and damps transmission sideways. It also
   makes it comfortable to hold for an hour.

Test with eyes shut: have someone fire one motor at random and say which.
If you get it right nine times in ten, the isolation works.

---

## Software, in outline

**Cane** (ESP32-C3): boots, initialises ESP-NOW, registers the rover's MAC
address as a peer, and sits in a loop waiting for packets. Each packet is a
tiny struct — four intensity bytes, one for each motor, plus a pattern id.
Motors follow the packet. If no packet arrives for 2 s, play the "link lost"
pattern and drop all motors.

**Rover** (existing ESP32): gains an ESP-NOW sender. Whatever decides the
direction — the Pi via micro-ROS, or the autonomous stub — calls one
function, `haptic(front, back, left, right)`, and the packet goes out.

Both are short. The cane side is under 100 lines.

---

## Order of build

1. Breadboard it first: C3 + ULN2003 + four motors + a bench supply or the
   18650. Send patterns over serial and confirm all four motors work.
2. Get ESP-NOW talking between the C3 and the rover ESP32 on the bench.
3. Design and print the grip. Iterate on the crosstalk fixes with the
   eyes-shut test.
4. Fit the shaft, wrap the grip, done.

Step 3 will take more attempts than the electronics. Print the grip before
you finalise the electronics layout — it is easier to move a pocket than
to bend a battery.
