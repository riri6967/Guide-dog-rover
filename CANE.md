# Haptic handle — design spec

The user-facing half of the guide dog. The rover decides where to go; the handle
tells the person. A rigid rod from the rover ends in a grip held in **one hand**,
like a guide-dog harness. Four vibration motors in the grip say which way to go;
a thumb switch lets the person ask for a turn; letting go stops the rover.

No screen, no sound. Silence means "carry on straight".

This replaces the earlier wireless-cane design (ESP32-C3, battery, ESP-NOW).
The handle is wired to the rover: no second battery to charge, no radio link
to drop while someone is walking.

---

## 1. How it is held

- **One hand**, arm relaxed at the side, hand at about hip height. The rod runs
  down and forward to the rover, which walks just ahead and to one side.
- **Usually the left hand**, as with guide dogs, leaving the right hand free for
  doors, rails or a white cane. The firmware needs a left/right-handed setting.
- **Palm down on a horizontal bar**: palm and finger bases across the top,
  fingers wrapped round the front, thumb on the switch.
- **The rigid rod is itself the strongest cue**: when the rover turns, it pulls
  the hand. The vibration adds advance warning and detail on top.

The rover is small (27 x 26 cm) and the rod is about 1 m, a long lever. **Hinge
the rod at the rover so it pivots up and down but is rigid side to side**:
vertical arm movement then cannot lift the wheels, while turns still pull the
hand. Mount the hinge low and between the wheels, through a **rubber bushing**
(see §6).

---

## 2. The handle

**Dimensioned drawings:** `_cane_later/drawings/handle_views.png` (top, front,
left, right) and `handle_details.png` (motor cup, board pod, rod and hinge).
They are drawn from `_cane_later/handle_drawings.py`; every CAD dimension is a
named number at the top of that file.

Handle frame used everywhere: origin at the centre of the grip bar, **+X to
the person's right, +Y forward (toward the rover), +Z up**. All mm.

### Shape: a U-frame, like a guide-dog harness handle

The grip bar is held between two side legs, which join below in a cross-member
carrying the board pod and the rod socket. The hand is fully free, the front
face of the bar is free for the F motor, and the boards have a home.

| Part | Size | Notes |
|---|---|---|
| Grip bar | **110** long (X) × **36** deep (Y) × **30** tall (Z), rounded rectangle, corner R6 | Hollow, **2.5 wall**. Flats: 24 wide on top and bottom, 18 tall front and back |
| Legs | 15 (X) × 24 (Y), from the top of the bar (Z +15) down to Z −75 | Inner faces at X ±55, outer at ±70. Right leg hollow (8 mm channel) for the motor wires |
| Cross-member | 140 × 24 × 10, Z −65 to −75 | **50 mm hand gap** from the bar's underside: room for the fingers |
| Board pod | 60 (X) × 55 (Y) × 23 (Z), under the cross-member, Z −75 to −98 | Removable lid; see §6 |
| Rod socket | Boss Ø24 × 40 long on the pod's underside, bore **Ø16.2** | Axis tilted **55° from horizontal**, forward. M4 grub screw or cross bolt |
| Overall | 140 wide × 113 tall (top of bar to bottom of pod) | |

**Why these sizes:**
- **Length 110:** adult hand width is ~75–90 mm; the extra lets the hand land
  anywhere near the middle. Much under 100 and the side motors get too close.
- **36 × 30:** full-hand grips are comfortable at roughly 30–40 mm. A
  **rounded rectangle**, not an oval, so it cannot roll in the hand **and** has
  flat faces for the motor cups (a cup ring cannot sit on a tightly curved end).
- **Print:** the bar in two halves split horizontally (top and bottom shells),
  screwed together with M2 screws, so cups and wires go in from inside. The
  legs, cross-member and pod can be one print with the bottom shell.
- **Covering:** 2 mm foam handlebar tape over the bar.

### Motor positions

| Motor | Centre (X, Y, Z) | Face | Under | Why there |
|---|---|---|---|---|
| **L** | (−33, 0, +15) | top flat | palm / finger base, one end | The top is covered by the hand across its full width; the underside has gaps between fingers |
| **R** | (+33, 0, +15) | top flat | the other end | 66 mm apart keeps both under even a 75 mm hand |
| **F** | (0, +18, 0) | front flat | middle fingers | Fingers and palm are separate areas, easy to tell apart |
| **B** | (0, −18, 0) | back flat | heel of the palm | |
| 5-way switch | (0, 0, +15) | top flat | thumb | Pocket 12 × 12 × 5 deep |
| Hold paddle | under the bar, X −30…+30 | bottom | fingertips | 60 × 20 × 3, pivots on the rear edge (Y −12); presses a microswitch inside through a slot |

Spacing (target: at least 30 mm between any two, the rough limit below which
people start to mislocate vibration on the hand):

| Pair | Distance |
|---|---|
| L – R | 66 mm |
| L – F (closest diagonal) | √(33² + 18² + 15²) ≈ 40 mm |
| F – B | 36 mm straight; ≈ 61 mm round the surface (half the rounded rectangle's perimeter: 24 + 18 + π × 6) |

Left/right, the most frequent instruction, gets the most room.

### Rod and hinge

- **Tube:** aluminium Ø16 × 1 mm. The 10-core cable runs inside it.
- **Rover end:** a printed plug in the tube with a cross hole for a **rubber
  grommet** on an **M6 pivot bolt**, in a clevis bracket on the rover's top
  (two 3 mm plates, 13 mm gap, 35 tall, pivot 25 above the rover top). It
  pivots up and down only; add stops so the rod stays between about 35° and 75°.
- **Length** (hinge to grip centre): `L = √((H_grip − 155)² + 450²)`, with
  `H_grip ≈ 0.45 × user height` (mm). Subtract ~140 mm for the handle and the
  plug to get the tube length:

| User height | H_grip | L | Rod angle | Cut tube to |
|---|---|---|---|---|
| 1.55 m | 700 | 707 | 50.5° | ~565 |
| 1.70 m | 765 | 758 | 53.6° | ~620 |
| 1.85 m | 830 | 811 | 56.3° | ~670 |

The socket is fixed at 55°, so the grip is level for ~1.70–1.85 m users and
tilts a few degrees for shorter ones. A telescoping tube is the later fix.

---

## 3. The motor mount — the part that makes or breaks it

A solid PLA bar shakes as one piece at 150 Hz: the vibration's wavelength in
PLA is metres (≈ 11 m for compression waves, ≈ 150 mm even for bending waves
in a 1 mm wall), far longer than the 110 mm grip. Spacing alone cannot keep
the motors apart; **each motor has to be mechanically decoupled from the bar**.

### The TPU cup

```
   skin ─────────────────────────────────
   foam  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░   2 mm tape
   shell ███████   ┌──────────┐   ███████   PLA, 2.5 mm, with a 16 mm hole
                   │ ▓▓motor▓▓│             TPU cup Ø12, 1 mm top, 0.5 mm proud
              ═════╧══      ══╧═════        3 spiral spokes across a 2 mm gap
                   │  wires   │             to a ring glued under the shell
                   ▼          ▼
```

| Part | Dimension |
|---|---|
| Hole in the shell | **Ø16** |
| Cup | outside **Ø12**, cavity **Ø10.2** (motor Ø10 × 2.7, press fit), top **1.0** thick, open underneath for the wires, about 5 deep |
| Cup top | **0.5 proud** of the shell's outer surface |
| Spokes | **3**, spiral, **0.6 thick × 2 wide**, ~6 long, spanning the **2 mm** gap between cup and hole edge |
| Ring, L and R | **Ø21 × 1.0**, ID 16, glued to the inside of the top flat |
| Ring, F and B | **stadium 21 × 17**, ID 16, glued to the inside of the 18 mm front/back flat at its two ends |

- **Take the motor off its module board** (adhesive pad; desolder the two
  leads, solder on longer twisted ones). Only the bare 10 mm motor goes in the
  grip; the boards go in the pod (§6).
- Push the motor into the cup, push the cup out through the hole from inside
  the shell, glue the ring.
- **Why 0.5 mm proud:** the cup is far too soft to carry grip force; gripping
  pushes it flush and the shell takes the load, while the cup stays lightly
  pressed to the skin.
- **No TPU printer:** a pocket 3 mm wider than the motor with 2–3 mm of EVA foam
  or foam tape under it. The simulation rates it the same.

### Why soft: the numbers

A mount isolates only if its own natural frequency is well below the motor's
150 Hz (at most about a third, ~50 Hz).

| Mount | Stiffness | Natural frequency | Verdict |
|---|---|---|---|
| Glued to the bar | — | — | Each motor shakes the whole bar |
| PLA flap ("tongue"), 15 x 15 x 1 mm | ≈ 3,900 N/m | ≈ 300 Hz | Too stiff to isolate on its own |
| TPU spokes / foam | ≈ 150–180 N/m | ≈ 30–50 Hz with the skin | Isolates |

### Simulation

`_cane_later/grip_sim/grip_vibration_sim.py` models the rod (beam elements),
the rigid grip, the four motors on each mount type, the hand and arm, the skin
at each site, and the rover's vibration coming up the rod. It reports how far
the skin is pushed in at each motor site.

"Gap" = how much stronger the active motor is felt than the next strongest
site. Target ≥ 10 dB (about 3x).

| Mount | Gap at 150 Hz | Worst of 500 random variations of the assumed values |
|---|---|---|
| Glued to the bar | 7.8 dB | 7.4 dB — **fails** |
| PLA tongue | 19 dB | 9.2 dB — marginal |
| **TPU spoke cup** | **49.5 dB** | **34.7 dB** |
| **Foam pad** | **49.5 dB** | **34.8 dB** |

Limits: hand, skin and motor-force values are estimates (hence the variation
run); vibration spreading *through the skin itself* is not modelled, so real
gaps will be smaller than 49 dB. The eyes-shut test (§8) is the real check.

```bash
python _cane_later/grip_sim/grip_vibration_sim.py out_dir
```

Needs numpy and matplotlib (`pip install numpy matplotlib`). It prints the
tables and writes four plots into `out_dir`.

---

## 4. The vibration language

Give each direction **more than one difference** — where, rhythm, and movement —
so a cold or tired hand still gets it right.

| Meaning | Pattern |
|---|---|
| Carry on straight | **Silence** (a grip that always buzzes gets tuned out) |
| Turn left | L motor, **short-short** (`▮ ▮`) |
| Turn right | R motor, **long** (`▮▮▮`) |
| Obstacle ahead / slowing | F motor, quick taps, faster as it gets closer |
| Step back | B motor, **long-short** (`▮▮▮ ▮`) |
| Stop | All four, strong pulses |
| Button press accepted | Two short taps on that button's side |
| Button press refused (blocked) | One long buzz on that side |
| Arrived | Two soft pulses on all four, then silence |
| Link lost | Three short pulses on all four, repeating |

**Turn angle = how often the pattern repeats.** The angle is between the rover's
heading and the planned path about 1 m ahead, so the hand is warned *before* the
turn, as a dog leans into a corner early.

| Turn ahead | Repeat rate |
|---|---|
| under ~10° | silent |
| 10–30° | ~1.5 per second |
| 30–60° | ~3 per second |
| 60–90° | ~5 per second |
| over 90° | continuous |

**Start of a turn: a sweep.** Fire R → (F/B) → L about 60–100 ms apart; it feels
like one buzz sliding left across the palm. Then continue with the side's rhythm.

**Timing limit:** coin motors take ~50 ms to spin up and down, so pulses need at
least ~100 ms on and 100 ms off — about 5 per second at most.

**Textures:** give each motor's spot a different raised texture on the grip
surface (lines, dots, bump, ridge), so the hand can confirm which spot is which.

---

## 5. Controls

| Control | Where | Does |
|---|---|---|
| **5-way thumb switch** (up/down/left/right/press) | Top centre, under the thumb, slightly recessed | ← next left · → next right · ↓ turn around · ↑ carry on / resume · press = stop / confirm |
| **Hold lever** (microswitch) | Under the fingers, held closed while gripping | **Released = rover stops**, immediately |
| Speed + / − (optional) | Beside the thumb switch, a different shape | Walking pace |
| Emergency stop (optional) | Separate and raised, far end | Backup stop |

- A press counts only after **~0.3 s held**, so a brushing thumb is not a command.
- The rover **checks the LiDAR before obeying** and always answers with
  "accepted" or "refused" — a blind user must know the press registered.
- **The hold lever is the most important safety part.** The ESP32 stops the
  wheels itself when it opens, without waiting for the Pi, like the 900 ms
  command failsafe.
- Fingers rest only on smooth surface and motors; buttons under the fingers get
  pressed by every squeeze.

---

## 6. Electronics

| Part | Notes | Approx ₹ |
|---|---|---|
| 4 × DC vibration motor module (Robu SKU 611201) | 5 V, 60 mA running / 90 mA start, 9000 rpm = 150 Hz; MOSFET on `IN` | 236 |
| 5 V buck converter (MP1584 or LM2596), ≥ 1 A, + 1 A fuse | Powers the modules' VCC from the 4S pack | 60–100 |
| 5-way navigation switch + 5 resistors | Resistor ladder: five directions on one wire | 40–50 |
| Lever microswitch + 10k resistor | Hold sensor | 20 |
| 10-core cable + GX16 10-pin connector | Pod to rover, inside the rod, unpluggable | 200 |
| TPU (or EVA foam) for 4 cups | | — |
| Aluminium tube 12–16 mm, ~1 m; hinge mount; rubber bushing | | ~250 |
| PLA grip halves, frame and pod; M2 screws; M2.5 standoffs; foam tape | | ~150 |

**About ₹900–1,000.**

### Wiring

The four module boards sit in the **pod under the grip** on M2.5 standoffs at
their two Ø3 holes (15 mm apart), headers removed. Only motor wires run inside
the handle; one cable runs down the rod to the rover.

```
in the handle:  each board's motor pads ── twisted pair up the right leg ──► its motor
in the cable:   rover 5 V buck (fused)  ──► VCC of all 4 boards
                rover GND               ──► GND of all 4 boards
                ESP32 23 / 32 / 21 / 22 ──► IN of boards F / B / L / R
                ESP32 36 ◄── 5-way switch resistor ladder (3V3 and GND also in the cable)
                ESP32 39 ◄── hold microswitch, 10k pull-up to 3V3, LOW = held
```

- **Never power the modules from the ESP32's 3V3**: four motors start at up to
  360 mA together.
- Pins are in `rover_microros/pins.h`. 21/22 are free because the MPU6050 is on
  the Pi.
- **Test one module first**: `IN` to an ESP32 pin set HIGH, `VCC` 5 V, shared
  GND. It should buzz with 3.3 V on `IN`.

### Cable cores (10)

5 V, GND, IN × 4, 3V3, switch signal, hold signal, 1 spare.

---

## 7. Software

```
Pi (Nav2 path, LiDAR)                          ESP32 (rover)
  ├─ turn angle ahead ───────────/haptic──►  pulse patterns on the 4 motors
  ├─ accept / refuse requests ◄──/buttons──  reads the 5-way switch (0.3 s hold)
  └─ stops navigation          ◄──/grip────  reads the hold lever, and stops the
                                             wheels itself, not waiting for the Pi
```

`_cane_later/haptics.cpp` / `.h` already drive the four PWM pins, with patterns
and velocity-based guidance. Still to add: the rhythm vocabulary above, the
switch and hold-lever inputs, the left/right-handed setting, and the Pi-side
node that computes the turn angle from the Nav2 path.

---

## 8. Build order

1. **One module on the bench.** Confirm it switches from a 3.3 V pin.
2. **Test grip**: just the bar, four cups, four motors. **Eyes-shut test**:
   someone fires motors at random, you name them. Aim for 19 of 20.
   - Left/right confused → cups too stiff or touching the shell; widen the gap.
   - A motor hard to feel → cup not proud enough, or the hand doesn't rest there.
3. Add the thumb switch recess, hold lever and textures; print the full grip.
4. Rod, hinge and rubber bushing on the rover.
5. Firmware, then the Pi node.

Step 2 takes the most iterations. Each test print is small; iterate there,
before anything else is built around it.

---

## Not yet measured

- **The rover's own vibration.** The simulation says rover vibration in the
  100–200 Hz band barely reaches the skin (≤ 0.3 µm per m/s², against ~380 µm
  from a motor), and lower frequencies arrive equally at all four sites as
  "road feel". The rigid-hinge rod has a resonance near 180 Hz that a rubber
  bushing removes. The rover's real vibration levels are unknown — measure them
  with the MPU6050 while driving.
