"""Wheel shaft extender with a built-in encoder disc, and a holder for the slot sensor.

    python encoder_extender.py <out folder>

Needs CadQuery (see _cane_later/cad/build_handle.py for the setup). Writes STEP
and print-ready STL files for both parts, plus a render.

Why: the wheel used to sit straight on the motor's 6 mm shaft, leaving no room
for the encoder. The extender grips the motor shaft with set screws, carries
the slotted disc right in front of the motor, and gives the wheel a new 6 mm
shaft SHIFT mm further out. Moving each wheel out by SHIFT widens the track
by 2 x SHIFT: update TRACK_WIDTH everywhere (firmware, Pi odometry, URDF).

Coordinates (mm): X along the shaft, out of the motor face (motor face at X = 0,
motor body at X < 0), Z up, shaft axis on X.

MEASURE before printing (the defaults are typical, not measured):
    MOTOR_D        gearbox body diameter, where the holder clamps
    SENSOR_*       your LM393 slot-sensor module
"""
import math
import sys
from pathlib import Path

import cadquery as cq

# ----------------------------------------------------------------- motor and wheel (from the user)
SHAFT_D = 6.0          # motor shaft, round
SHAFT_LEN = 21.0       # how far it sticks out of the motor
HUB_DEPTH = 15.0       # wheel hub hole is 10-15 mm deep: give it the full 15
SHIFT = 25.0           # how much further out the wheel sits (2-3 cm asked)
MOTOR_D = 37.0         # MEASURE: gearbox diameter (37 mm is the common 12 V gearmotor)

# ----------------------------------------------------------------- extender
GAP = 5.0              # extender starts this far off the motor face (room for the sensor's inner post)
BORE = 6.3             # 6 mm shaft + print clearance (FDM holes come out small)
BODY_D = 16.0          # thick body so the set screws have plastic to bite
OUT_D = 5.9            # new shaft for the wheel hub (6 mm hole; FDM pins come out fat)
OUT_FLAT = 0.5         # flat on the new shaft for the wheel's set screw
SET_PILOT = 1.4        # M1.7 set screws: pilot hole to tap/self-tap into
SET_X = (11.0, 17.0)   # set screw positions along the motor shaft

# encoder disc, printed as part of the extender (flat on the bed)
DISC_D, DISC_T = 40.0, 2.0
SLOTS = 32             # 32 x 1.47 mm slots at mid-radius: prints cleanly with a 0.4 nozzle
SLOT_IN_D, SLOT_OUT_D = 24.0, 36.0

# ----------------------------------------------------------------- sensor holder
SENSOR_LEN, SENSOR_W, SENSOR_PCB = 32.0, 14.0, 1.6    # MEASURE your module
SENSOR_SLOT = 5.5      # gap between the two posts
SENSOR_POST = (3.0, 6.0, 10.0)                        # each post: along X, across, height
SENSOR_BEAM_H = 7.0    # MEASURE: beam height above the PCB top
FORK_FROM_END = 6.0    # fork centre from the module's end
FORK_TO_HOLE = 17.0    # fork centre to the module's mounting hole
RING_W, RING_T = 10.0, 3.0
RING_X0 = -34.0        # clamp ring sits on the gearbox, clear of the motor's mounting bracket
SHELF_T = 3.0
CL = 0.4               # clearance around the motor body

# ----------------------------------------------------------------- derived
X0 = GAP                               # extender start
# The hub used to sit on the last HUB_DEPTH mm of the motor shaft, its face at SHAFT_LEN - HUB_DEPTH.
# It now starts SHIFT further out, on the extender's own shaft.
X_BODY_END = SHAFT_LEN - HUB_DEPTH + SHIFT
assert X_BODY_END >= SHAFT_LEN + 5, 'SHIFT too small: need 5 mm of solid body past the motor shaft'
X_END = X_BODY_END + HUB_DEPTH
DISC_X = X0 + DISC_T / 2               # disc mid-plane
BEAM_Z = -(SLOT_IN_D + SLOT_OUT_D) / 4                    # beam crosses the middle of the slot ring
PCB_TOP = BEAM_Z - SENSOR_BEAM_H
SHELF_TOP = PCB_TOP - SENSOR_PCB


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(d, length, start, direction=(1, 0, 0)):
    return cq.Workplane(obj=cq.Solid.makeCylinder(d / 2, length, cq.Vector(*start), cq.Vector(*direction)))


def extender():
    e = cyl(BODY_D, X_BODY_END - X0, (X0, 0, 0))
    e = e.union(cyl(DISC_D, DISC_T, (X0, 0, 0)))
    # 2 mm cone where the thick body steps down to the 6 mm shaft: no sharp, crack-starting corner
    cone = cq.Workplane(obj=cq.Solid.makeCone(5.0, OUT_D / 2, 2.0, cq.Vector(X_BODY_END, 0, 0), cq.Vector(1, 0, 0)))
    e = e.union(cone).union(cyl(OUT_D, HUB_DEPTH, (X_BODY_END, 0, 0)))
    e = e.cut(box(X_BODY_END + 2, X_END + 1, OUT_D / 2 - OUT_FLAT, 10, -10, 10))        # flat for the wheel screw
    # slots
    pitch = 360 / SLOTS
    for i in range(SLOTS):
        a0, a1 = math.radians(i * pitch - pitch / 4), math.radians(i * pitch + pitch / 4)
        pts = []
        for r, (s, t) in ((SLOT_OUT_D / 2, (a0, a1)), (SLOT_IN_D / 2, (a1, a0))):
            for k in range(7):
                a = s + (t - s) * k / 6
                pts.append((r * math.cos(a), r * math.sin(a)))
        e = e.cut(cq.Workplane('YZ', origin=(X0 - 1, 0, 0)).polyline(pts).close().extrude(DISC_T + 2))
    # socket for the motor shaft, open on the motor side
    e = e.cut(cyl(BORE, SHAFT_LEN - X0 + 0.5, (X0 - 0.1, 0, 0)))
    for x, (dy, dz) in zip(SET_X, ((0, 1), (1, 0))):          # two set screws, 90 deg apart
        e = e.cut(cyl(SET_PILOT, BODY_D, (x, 0, 0), (0, dy, dz)))
    return e


def holder():
    r_in = MOTOR_D / 2 + CL
    ring = cyl(2 * (r_in + RING_T), RING_W, (RING_X0, 0, 0)).cut(cyl(2 * r_in, RING_W + 2, (RING_X0 - 1, 0, 0)))
    ring = ring.cut(box(RING_X0 - 1, RING_X0 + RING_W + 1, -1, 1, r_in - 1, r_in + RING_T + 1))   # split at the top
    top = r_in + RING_T
    ears = box(RING_X0, RING_X0 + RING_W, -7, -1, top - 2, top + 6).union(box(RING_X0, RING_X0 + RING_W, 1, 7, top - 2, top + 6))
    ears = ears.cut(cyl(3.4, 20, (RING_X0 + RING_W / 2, -10, top + 3), (0, 1, 0)))       # M3 clamp bolt
    fork_x = DISC_X
    shelf_x1 = fork_x + FORK_FROM_END + 2
    shelf = box(RING_X0, shelf_x1, -SENSOR_W / 2 - 3, SENSOR_W / 2 + 3, SHELF_TOP - SHELF_T, SHELF_TOP)
    web = box(RING_X0, RING_X0 + RING_W, -8, 8, SHELF_TOP - SHELF_T, -r_in - RING_T + 1.5)
    # low fences either side of the PCB so it drops into place square to the disc
    fence = box(RING_X0 + RING_W, shelf_x1, -SENSOR_W / 2 - 3, -SENSOR_W / 2 - 0.3, SHELF_TOP, SHELF_TOP + 1.2) \
        .union(box(RING_X0 + RING_W, shelf_x1, SENSOR_W / 2 + 0.3, SENSOR_W / 2 + 3, SHELF_TOP, SHELF_TOP + 1.2))
    h = ring.union(ears).union(shelf).union(web).union(fence)
    # slot for the module's M3 screw, +/-3 mm along X to fine-tune the fork onto the disc
    hx = fork_x - FORK_TO_HOLE
    slot = cq.Workplane('XY', origin=(hx, 0, SHELF_TOP - SHELF_T - 1)).slot2D(6 + 3.4, 3.4).extrude(SHELF_T + 2)
    return h.cut(slot)


def proxies():
    motor = cyl(MOTOR_D, 60, (-60, 0, 0)).union(cyl(SHAFT_D, SHAFT_LEN, (0, 0, 0)))
    fx = DISC_X
    pcb = box(fx + FORK_FROM_END - SENSOR_LEN, fx + FORK_FROM_END, -SENSOR_W / 2, SENSOR_W / 2, PCB_TOP - SENSOR_PCB, PCB_TOP)
    pcb = pcb.union(box(fx - 12, fx - 4, -3, 3, PCB_TOP, PCB_TOP + 2.5))                # chip / LEDs, 2.5 mm tall
    px, py, pz = SENSOR_POST
    for s in (-1, 1):
        x0 = fx + s * (SENSOR_SLOT / 2) + (0 if s > 0 else -px)
        pcb = pcb.union(box(x0, x0 + px, -py / 2, py / 2, PCB_TOP, PCB_TOP + pz))
    wheel = cyl(110, 45, (X_BODY_END - 2, 0, 0)).cut(cyl(OUT_D + 0.2, 50, (X_BODY_END - 5, 0, 0)))
    return motor, pcb, wheel


def upright(wp, axis_up):
    """Rotate so axis_up -> +Z and sit on Z = 0 (print orientation)."""
    s = wp.val()
    if axis_up == '+X': s = s.rotate(cq.Vector(), cq.Vector(0, 1, 0), -90)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def main(out_dir):
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    ext, hol = extender(), holder()
    motor, sensor, wheel = proxies()
    for name, wp in (('Shaft Extender', ext), ('Encoder Holder', hol)):
        cq.exporters.export(wp, str(out / f'{name}.step'))
    # print orientation: extender stands on its disc; holder on its shelf (as modelled, flat bottom down)
    cq.exporters.export(cq.Workplane(obj=upright(ext, '+X')), str(out / 'Shaft Extender.stl'), tolerance=0.02, angularTolerance=0.05)
    cq.exporters.export(cq.Workplane(obj=upright(hol, 'Z')), str(out / 'Encoder Holder.stl'), tolerance=0.05, angularTolerance=0.1)

    print(f'extender: starts {X0} mm off the motor face, body to X={X_BODY_END:.1f}, wheel shaft to X={X_END:.1f}')
    print(f'          wheel moves out by about {SHIFT} mm -> track width + {2 * SHIFT:.0f} mm')
    print(f'          {SLOTS} slots, slot width at mid-radius {math.pi * (SLOT_IN_D + SLOT_OUT_D) / 2 / SLOTS / 2:.2f} mm')
    print(f'          volume {ext.val().Volume() / 1000:.1f} cm3, holder {hol.val().Volume() / 1000:.1f} cm3')
    print('clash check (overlap, mm3):')
    for (a, sa), (b, sb) in ((('extender', ext), ('sensor', sensor)), (('extender', ext), ('holder', hol)),
                             (('holder', hol), ('motor', motor)), (('sensor', sensor), ('motor', motor)),
                             (('extender', ext), ('motor body', cyl(MOTOR_D, 60, (-60, 0, 0)))),
                             (('holder', hol), ('sensor', sensor)), (('wheel', wheel), ('sensor', sensor)),
                             (('wheel', wheel), ('holder', hol))):
        v = sa.val().intersect(sb.val()).Volume()
        print(f'   {a:9s} x {b:10s}: {v:7.2f}')
    # the disc must pass between the posts with the beam inside the slot ring
    print(f'disc mid-plane X={DISC_X:.1f}; fork gap X={DISC_X - SENSOR_SLOT / 2:.2f}..{DISC_X + SENSOR_SLOT / 2:.2f}; '
          f'beam at Z={BEAM_Z:.1f} (slot ring {-SLOT_OUT_D / 2:.0f}..{-SLOT_IN_D / 2:.0f})')
    return ext, hol, motor, sensor, wheel


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'extender_out')
