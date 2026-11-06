# Shaft extender + encoder disc: fit iterations

Made by `python encoder_extender.py stl` (from `cad/`). Print all three sizes,
try each on the motor shaft / wheel / extender body, and keep the one that fits.
Sizes can be mixed: e.g. Extender B with Disc C.

All mm. Extender length is wall end to the tip of the wheel pin.

| Size | Motor-shaft bore | Wheel pin Ø | Wheel pin length | Extender length | Encoder disc hole |
|------|-----------------:|------------:|-----------------:|----------------:|------------------:|
| old  | 5.9 | 6.3 | 15 | 44 | 17.0 |
| A    | 6.2 | 6.0 | 18 | 47 | 17.3 |
| B    | 6.5 | 5.8 | 20 | 49 | 17.6 |
| C    | 6.8 | 5.6 | 22 | 51 | 17.9 |

Unchanged on all sizes: body Ø16, disc shoulder Ø20, disc Ø46 x 2 with 32 slots,
0.5 mm flat on the wheel pin, two M1.7 set-screw pilots (Ø1.4) at 5 and 9 mm from
the wall end. The wheel hub still seats at 30 mm from the wall, so the track
width (290 mm) does not change; the longer pin only goes deeper into the hub.

Print: extender standing on its wall end, PETG, 100 % infill; disc flat, black.
The encoder holders did not change (`Encoder Holder Left/Right.stl`).
