# Haptic handle: print guide

All parts in **PETG**, 0.4 mm nozzle, **0.2 mm layers**, **gyroid infill**. STLs in
`STL (print-ready orientation)` are already the right way up: drop them in and slice.

| Part | Qty | Walls | Top/bottom | Infill | Supports | Brim | Est. weight | Why |
|---|---|---|---|---|---|---|---|---|
| Grip Top Part A (wire hole) | 1 | 3 | 4 | 15% | tree, inside the channel only | no | ~33 g (solid 33 g) | your grip; its 2.5 mm walls print solid at 3 walls anyway |
| Grip Top Part B | 1 | 3 | 4 | 15% | tree, minimal | no | ~10 g (solid 10 g) | your back panel |
| Frame | 1 | 4 | 5 | 20% | tree / organic, touching build plate only where possible | no | ~74 g (solid 121 g) | carries the rod's pull and twist; gyroid infill is light and strong in every direction |
| Cap Right (switch) | 1 | 3 | 4 | 15% | tree, only under the switch housing | no | ~6 g (solid 7 g) | light clamping part |
| Cap Left | 1 | 3 | 4 | 15% | tree, minimal | no | ~5 g (solid 6 g) | light clamping part |
| Bail | 1 | 4 | 4 | 20% | none or minimal | no | ~7 g (solid 8 g) | squeezed thousands of times; PETG flexes instead of snapping |
| Pod Lid | 1 | 2 | 3 | 15% | none | no | ~2 g (solid 3 g) | a cover, carries no load |
| Rod Plug | 1 | 4 | 5 | 40% | none (its 6 and 12 mm side holes bridge fine) | 5 mm | ~8 g (solid 9 g) | small and tall; holds the cross-bolt |
| Hinge Bracket | 1 | 4 | 5 | 30% | none needed (base down) | no | ~14 g (solid 14 g) | takes the most force, at the pivot bolt |

**Total: about 158 g of PETG** for all printed parts (about 211 g if printed solid). Add roughly 10-15 % for supports. One 1 kg spool is plenty.

## Same for every part

- Nozzle 240 °C, bed 75-80 °C (PETG). Glue stick or textured PEI so PETG releases cleanly.
- Seam: rear or aligned. No ironing, no fuzzy skin.
- Supports: tree/organic, 45° threshold, "on build plate only" where the slicer allows.
- Heat-set inserts: set them after printing with a soldering iron at about 220 °C.

## Keeping it light

- Gyroid at 15-30 % instead of high infill: it is strong in every direction for its weight.
- Strength comes from walls, not infill: 4 walls on the load-bearing parts, 2-3 on covers.
- 0.2 mm layers; finer layers add time, not strength.
- Do not print anything at 100 %. The rod plug (40 %) is the densest part.

## Hardware

| Item | Qty | Where |
|---|---|---|
| M3 x 12 socket screw | 4 | caps to cradles |
| M3 heat-set insert (hole 4.0) | 4 | cradle ears |
| M3 x 25 screw + M3 nyloc nut | 2 + 2 | bail pivots through the legs |
| M2.5 x 6 screw + M2.5 heat-set insert (hole 3.6) | 2 + 2 | pod lid |
| M4 x 6 grub screw + M4 heat-set insert (hole 5.6) | 1 + 1 | rod socket |
| M3 x 20 screw + M3 nyloc nut | 1 + 1 | cross-bolt, rod plug in tube (drill the tube 3.2) |
| M6 x 30 bolt, 2 washers, M6 nyloc nut | 1 set | hinge pivot |
| Rubber grommet, ID 6, OD 12 | 1 | in the rod plug tongue |
| M4 x 25 bolt + M4 nyloc nut | 2 + 2 | hinge stops |
| M3 x 12 screw, nyloc nut, large washers | 4 | bracket to rover body |
| Aluminium tube 16 x 1 mm | ~605 mm | rod (length by user height, see CANE.md) |
| Subminiature lever microswitch (D2F size) | 1 | right leg pocket, pressed by the bail |
| 5-way navigation switch | 1 | right cap pocket |
| Thin foam tape (~1 mm) | 1 strip | inside each cradle, so the caps clamp the grip |
| Epoxy | - | rod plug into the tube |
