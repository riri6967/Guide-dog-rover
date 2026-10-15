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

## 2. The grip

| | |
|---|---|
| Length | **110 mm**: hand width (75–90 mm for adults) plus room at each end, so the hand need not be placed precisely. Not much shorter than 100 mm, or the side motors end up too close |
| Cross-section | **34 x 28 mm oval**. 30–40 mm suits a full-hand grip; the oval stops it rolling, so front stays front |
| Construction | PLA, printed as **top and bottom halves** screwed together (M2), so the cups and wires can be fitted from inside |
| Covering | 2 mm foam handlebar tape: comfort, grip, and it damps vibration travelling sideways |

### Motor positions

Origin at the centre of the grip; x along the bar, y forward, z up (mm).

| Motor | Position | Under | Why there |
|---|---|---|---|
| **L** | (−33, 0, +14), top | palm / finger base, one end | The top is covered by the hand across its full width; the underside has gaps between fingers |
| **R** | (+33, 0, +14), top | the other end | 66 mm apart keeps both under even a 75 mm hand |
| **F** | (0, +17, 0), front face | middle fingers | Fingers and palm are separate areas, easy to tell apart |
| **B** | (0, −17, 0), back face | heel of the palm | |

Spacing (target: at least 30 mm between any two, the rough limit below which
people start to mislocate vibration on the hand):

| Pair | Distance |
|---|---|
| L – R | 66 mm |
| L – F (closest diagonal) | √(33² + 17² + 14²) ≈ 40 mm |
| F – B | 34 mm straight, ≈ 49 mm round the surface (half the oval's perimeter, Ramanujan's approximation with semi-axes 17 and 14) |

Left/right, the most frequent instruction, gets the most room.

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
   shell ███████   ┌──────────┐   ███████   PLA, 2.5 mm, with an 18 mm hole
                   │ ▓▓motor▓▓│             TPU cup, 1 mm top, 0.5 mm proud
              ═════╧══      ══╧═════        3 thin TPU spokes to a ring
                   │  wires   │             glued under the shell
                   ▼          ▼             air gap 3 mm all round the cup
```

- **Take the motor off its module board** (adhesive pad; desolder the two
  leads, solder on longer twisted ones). Only the bare 10 mm motor goes in the
  grip; the boards stay in a small pod at the base of the handle or in the rover.
- **One TPU print per motor**: a 12 mm cup holding the motor, three spokes
  (6 x 2 x 0.6 mm), and a 24 mm ring.
- Push the motor into the cup, push the cup up through the shell's 18 mm hole
  from inside, glue the ring under the shell.
- **Cup top 0.5 mm proud of the shell.** The cup is far too soft to carry grip
  force; gripping pushes it flush and the shell takes the load, while the cup
  stays lightly pressed to the skin.
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

Needs numpy and matplotlib.

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
| 10-core cable + GX12 connector | Rover to handle, unpluggable | 150 |
| TPU (or EVA foam) for 4 cups | | — |
| Aluminium tube 12–16 mm, ~1 m; hinge mount; rubber bushing | | ~250 |
| PLA grip halves, M2 screws, foam tape | | ~100 |

**About ₹900–1,000.**

### Wiring

```
5 V buck (fused, from the 4S main leads) ──► VCC of all 4 modules
ESP32 GND ───────────────────────────────► GND of all 4 modules, buck GND
ESP32 23 / 32 / 21 / 22 ─────────────────► IN of modules F / B / L / R
each module's motor pads ── twisted pair up the rod ──► its motor in the grip
ESP32 36 ◄── 5-way switch resistor ladder (3V3 / GND)
ESP32 39 ◄── hold lever, 10k pull-up to 3V3, LOW = held
```

- **Never power the modules from the ESP32's 3V3**: four motors start at up to
  360 mA together.
- Pins are in `rover_microros/pins.h`. 21/22 are free because the MPU6050 is on
  the Pi.
- **Test one module first**: `IN` to an ESP32 pin set HIGH, `VCC` 5 V, shared
  GND. It should buzz with 3.3 V on `IN`.

### Cable cores (10)

4 motor negatives + 1 shared motor +, switch signal + 3V3 + GND, hold-lever
signal, 1 spare.

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
