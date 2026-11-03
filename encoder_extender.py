"""Wheel shaft extender, encoder disc, and a wall bracket for the slot sensor.

    python encoder_extender.py <out folder>

Needs CadQuery (see _cane_later/cad/build_handle.py for the setup). Writes STEP
and print-ready STL files to <out folder>, and the URDF visual meshes to
ros/rover_description/meshes/.

Why: the wheel used to sit straight on the motor's 5 mm shaft, leaving no room
for the encoder. The extender grips the motor shaft with set screws, carries
the slotted disc, and gives the wheel a new 7 mm shaft SHIFT mm further out.
Moving each wheel out by SHIFT widens the track by 2 x SHIFT: update
TRACK_WIDTH everywhere (firmware, Pi odometry, URDF).

The motors sit inside the body; only the shaft comes out through the side
wall. So the sensor bracket bolts to the outside of the wall (2 x M3), above
the shaft, and the sensor hangs under its shelf with its fork open downwards
over the top of the disc, so dirt falls out of the fork instead of into it.

The sensor is the LM393 module in CAD Files/lm393-motor-speed-sensor-1.snapshot.3
(LM393.stp): its sizes below are measured from that model, and the model itself
is used for the clash check and the URDF mesh when the file is there.

Coordinates (mm): X along the shaft, out of the body (outer face of the side
wall at X = 0, motor inside at X < 0), Z up, shaft axis on X.

MEASURE before printing:
    SHAFT_OUT      how far the motor shaft sticks out past the outer wall face
                   (measured 20 mm: the set screws must land on it)
    LM_BEAM_H      height of the IR beam in the fork (the plate's slotted
                   holes give +/-2 mm to put it on the slot ring)
    the motor's own screws round the shaft: the bracket keeps clear of a
    radius of WALL_KEEPOUT_R
"""
import math
import sys
from pathlib import Path

import cadquery as cq
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf

HERE = Path(__file__).parent
LM393_STEP = HERE.parent / 'CAD Files' / 'lm393-motor-speed-sensor-1.snapshot.3' / 'LM393.stp'

# ----------------------------------------------------------------- motor and wheel (from the user)
SHAFT_D = 5.0          # motor shaft, round (measured: about 5 mm)
SHAFT_OUT = 20.0       # motor shaft length outside the wall's outer face (measured: about 2 cm)
HUB_DEPTH = 15.0       # wheel hub hole is 10-15 mm deep: give it the full 15
SHIFT = 25.0           # how much further out the wheel sits (2-3 cm asked)
WALL_T = 7.6           # body side wall (CAD), only used for the stand-ins
MOTOR_D = 37.0         # stand-in only: the motor is inside the body (diameter not measured)
MOTOR_LEN = 30.0       # motor body length (measured: about 3 cm)

# ----------------------------------------------------------------- extender
GAP = 1.0              # extender starts this far off the wall: it must not rub
BORE = SHAFT_D + 0.9   # 5 mm shaft + print clearance (FDM holes come out small; +0.3 and +0.6 were too tight in PLA)
BODY_D = 16.0          # thick body so the set screws have plastic to bite
OUT_D = 6.3            # new shaft for the wheel hub (measured hole: about 7 mm; FDM pins come out fat: 6.9 and 6.6 were too fat)
OUT_FLAT = 0.5         # flat on the new shaft for the wheel's set screw
SET_PILOT = 1.4        # M1.7 set screws: pilot hole to tap/self-tap into
# The motor shaft ends before the disc's far side, so the set screws sit between the wall and
# the disc's shoulder. One from below and one from the side: the bracket's shelf covers the top,
# but a hex key reaches both through the gap between the wall and the disc.
SET_SCREWS = ((5.0, (0, 0, -1)), (9.0, (0, 1, 0)))

# encoder disc: a separate flat print, slid on from the wheel end and glued against a shoulder
DISC_X0 = 14.0         # disc's inner face off the wall: open space between the wall and the encoder
DISC_D, DISC_T = 46.0, 2.0
DISC_BORE = BODY_D + 1.0   # slides over the 16 body (+0.3 and +0.8 were too tight)
SHOULDER_D = 20.0
SLOTS = 32             # 32 x 1.77 mm slots at mid-radius: prints cleanly with a 0.4 nozzle
SLOT_IN_D, SLOT_OUT_D = 30.0, 42.0

# ----------------------------------------------------------------- LM393 module (from LM393.stp)
# Module frame: board corner at the origin, Mx along the board's 23.1 mm side (fork at the
# Mx = 0 end, header and holes at the far end), My across (the fork's gap runs along My),
# Mz up from the back of the board.
LM_BOARD = (23.1, 20.2, 1.5)
LM_FORK_MX = 3.8       # fork centre
LM_GAP_MY = 10.1       # centre of the gap between the posts (gap 12.2 wide)
LM_FORK_TOP = 16.8     # post tips, from the board's back
LM_U_BOTTOM = 4.5      # bottom of the U between the posts, from the board's back
LM_BEAM_H = 9.0        # MEASURE: IR beam height above the component face
LM_HOLES = ((20.45, 2.65), (20.45, 17.55))   # 3.1 mm holes (M3)
LM_HEADER = ((20.0, 22.3), (6.65, 13.55))    # pin header footprint: its pin tails poke out the back

# ----------------------------------------------------------------- sensor bracket
PLATE_T, SHELF_T = 3.0, 3.5
WALL_KEEPOUT_R = 19.0  # motor screw heads round the shaft on the wall: stay outside this radius
PLATE_Y = (-12.0, 30.0)
RIB_T = 3.0
ADJ = 2.0              # wall bolt slots: +/- this much up/down to put the beam on the slot ring

# ----------------------------------------------------------------- derived
X0 = GAP
# The hub used to sit on the last HUB_DEPTH mm of the motor shaft, its face at SHAFT_OUT - HUB_DEPTH.
# It now starts SHIFT further out, on the extender's own shaft.
X_BODY_END = SHAFT_OUT - HUB_DEPTH + SHIFT
X_END = X_BODY_END + HUB_DEPTH
DISC_X = DISC_X0 + DISC_T / 2          # disc mid-plane
BEAM_Z = (SLOT_IN_D + SLOT_OUT_D) / 4  # beam crosses the middle of the slot ring, above the shaft
FACE_Z = BEAM_Z + LM_BEAM_H            # module hangs upside down: component face towards the disc
SHELF_Z0 = FACE_Z + LM_BOARD[2]        # shelf underside = back of the board
SHELF_Z1 = SHELF_Z0 + SHELF_T
PCB_X = (DISC_X - LM_GAP_MY, DISC_X - LM_GAP_MY + LM_BOARD[1])
PCB_Y = (-LM_FORK_MX, LM_BOARD[0] - LM_FORK_MX)
SHELF_X1 = PCB_X[1] + 0.5              # shelf stops at the board's outer edge: the wheel hub is close
PLATE_Z = (WALL_KEEPOUT_R + 0.5, SHELF_Z1 + 20.0)

assert X_BODY_END >= SHAFT_OUT + 5, 'SHIFT too small: need 5 mm of solid body past the motor shaft'
SHOULDER_X0 = DISC_X0 - 1 - (SHOULDER_D - BODY_D) / 2   # where the shoulder's cone starts
assert all(X0 + SET_PILOT < x < min(SHOULDER_X0, SHAFT_OUT) - SET_PILOT for x, _ in SET_SCREWS), \
    'set screws: over the motor shaft, clear of the wall end and the shoulder'
assert SHELF_Z0 - LM_U_BOTTOM > DISC_D / 2 + 0.5, 'disc edge would hit the bottom of the fork'
assert SHELF_Z0 - LM_FORK_TOP > BODY_D / 2 + 2, 'post tips too close to the extender body'
assert PCB_X[0] > PLATE_T + 1, 'board would hit the wall plate: move the disc out'
assert SHELF_X1 < X_BODY_END - 3, 'bracket too close to the wheel hub'


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(d, length, start, direction=(1, 0, 0)):
    return cq.Workplane(obj=cq.Solid.makeCylinder(d / 2, length, cq.Vector(*start), cq.Vector(*direction)))


def cone(d0, d1, length, start):
    return cq.Workplane(obj=cq.Solid.makeCone(d0 / 2, d1 / 2, length, cq.Vector(*start), cq.Vector(1, 0, 0)))


def extender():
    e = cyl(BODY_D, X_BODY_END - 2 - X0, (X0, 0, 0))
    # shoulder the disc is glued against; 45 deg underside so it prints standing up without support
    sh = (SHOULDER_D - BODY_D) / 2
    e = e.union(cone(BODY_D, SHOULDER_D, sh, (SHOULDER_X0, 0, 0))).union(cyl(SHOULDER_D, 1, (DISC_X0 - 1, 0, 0)))
    # body tapers to a 12 mm face for the wheel hub to sit on. The taper stays inside the body:
    # anything wider than the shaft past X_BODY_END would stop the hub short of its full depth.
    e = e.union(cone(BODY_D, 12, 2.0, (X_BODY_END - 2, 0, 0))).union(cyl(OUT_D, HUB_DEPTH, (X_BODY_END, 0, 0)))
    e = e.cut(box(X_BODY_END + 2, X_END + 1, OUT_D / 2 - OUT_FLAT, 10, -10, 10))        # flat for the wheel screw
    # socket for the motor shaft, open on the wall side
    e = e.cut(cyl(BORE, SHAFT_OUT - X0 + 0.5, (X0 - 0.1, 0, 0)))
    for x, d in SET_SCREWS:                                    # two set screws, 90 deg apart
        e = e.cut(cyl(SET_PILOT, BODY_D, (x, 0, 0), d))
    return e


def disc():
    d = cyl(DISC_D, DISC_T, (DISC_X0, 0, 0)).cut(cyl(DISC_BORE, DISC_T + 2, (DISC_X0 - 1, 0, 0)))
    pitch = 360 / SLOTS
    for i in range(SLOTS):
        a0, a1 = math.radians(i * pitch - pitch / 4), math.radians(i * pitch + pitch / 4)
        pts = []
        for r, (s, t) in ((SLOT_OUT_D / 2, (a0, a1)), (SLOT_IN_D / 2, (a1, a0))):
            for k in range(7):
                a = s + (t - s) * k / 6
                pts.append((r * math.cos(a), r * math.sin(a)))
        d = d.cut(cq.Workplane('YZ', origin=(DISC_X0 - 1, 0, 0)).polyline(pts).close().extrude(DISC_T + 2))
    return d


def to_ext(mx, my):
    """Module (Mx, My) -> extender (X, Y)."""
    return DISC_X - LM_GAP_MY + my, mx - LM_FORK_MX


def holder():
    """L bracket: plate on the wall above the shaft, shelf out over the disc, module screwed underneath."""
    y0, y1 = PLATE_Y
    plate = box(0, PLATE_T, y0, y1, PLATE_Z[0], PLATE_Z[1])
    shelf = box(0, SHELF_X1, y0, y1, SHELF_Z0, SHELF_Z1)
    # triangular ribs at both ends, plate to shelf tip
    rib = lambda ya: cq.Workplane('XZ', origin=(0, ya + RIB_T, 0)).polyline(
        [(PLATE_T, SHELF_Z1), (SHELF_X1, SHELF_Z1), (PLATE_T, PLATE_Z[1])]).close().extrude(RIB_T)
    h = plate.union(shelf).union(rib(y0)).union(rib(y1 - RIB_T))
    # 1.2 mm lip along the board's wall-side edge so it hangs square to the disc (no room for
    # one on the wheel side; the two M3 screws along X hold that end)
    h = h.union(box(PCB_X[0] - 2.3, PCB_X[0] - 0.3, PCB_Y[0], PCB_Y[1], SHELF_Z0 - 1.2, SHELF_Z0))
    # 2 x M3 through the wall (nuts inside the body), slotted up/down to set the beam height
    for y in (y0 + 7, y1 - 7):
        h = h.cut(cq.Workplane('YZ', origin=(-1, y, PLATE_Z[1] - 8)).slot2D(2 * ADJ + 3.4, 3.4, angle=90).extrude(PLATE_T + 2))
    # the module's two M3 holes, and a pocket for the header's pin tails on the back of the board
    for mx, my in LM_HOLES:
        x, y = to_ext(mx, my)
        h = h.cut(cyl(3.4, SHELF_T + 4, (x, y, SHELF_Z0 - 2), (0, 0, 1)))
    (mx0, mx1), (my0, my1) = LM_HEADER
    (xa, ya), (xb, yb) = to_ext(mx0, my0), to_ext(mx1, my1)
    h = h.cut(box(xa - 0.5, xb + 0.5, ya - 0.5, yb + 0.5, SHELF_Z0 - 0.1, SHELF_Z0 + 2))
    # ... and for the IR LED / phototransistor legs, which come through the board under each post
    for my in (LM_GAP_MY - 7.3, LM_GAP_MY + 7.3):
        x, _ = to_ext(0, my)
        h = h.cut(box(x - 1.5, x + 1.5, -2.5, 2.5, SHELF_Z0 - 0.1, SHELF_Z0 + 2))
    return h


def lm393():
    """The real module from LM393.stp placed on the left-hand bracket, or a block stand-in."""
    if LM393_STEP.exists():
        m = cq.importers.importStep(str(LM393_STEP))
        shape = cq.Compound.makeCompound(m.solids().vals()).translate(cq.Vector(0, -100, 0))   # board corner to origin
        t = gp_Trsf()
        # (Mx, My, Mz) -> X = My + DISC_X - LM_GAP_MY, Y = Mx - LM_FORK_MX, Z = SHELF_Z0 - Mz
        t.SetValues(0, 1, 0, DISC_X - LM_GAP_MY,
                    1, 0, 0, -LM_FORK_MX,
                    0, 0, -1, SHELF_Z0)
        return cq.Workplane(obj=cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape()))
    pcb = box(PCB_X[0], PCB_X[1], PCB_Y[0], PCB_Y[1], FACE_Z, SHELF_Z0)
    for s in (-1, 1):
        x0 = DISC_X + s * 6.1 + (0 if s > 0 else -3.2)
        pcb = pcb.union(box(x0, x0 + 3.2, -3, 3, SHELF_Z0 - LM_FORK_TOP, FACE_Z))
    return pcb


def proxies():
    motor = cyl(MOTOR_D, MOTOR_LEN, (-WALL_T - MOTOR_LEN, 0, 0)).union(cyl(SHAFT_D, SHAFT_OUT + WALL_T, (-WALL_T, 0, 0)))
    wheel = cyl(110, 45, (X_BODY_END, 0, 0)).cut(cyl(OUT_D + 0.2, 50, (X_BODY_END - 5, 0, 0)))
    wall = box(-WALL_T, 0, -70, 70, -15, 90)
    return motor, lm393(), wheel, wall


def right_hand(holder_wp, sensor_wp):
    """Right-hand wheel: bracket mirrored (board runs the other way, clear of the body's corner
    pillar); the real module is turned round, not mirrored."""
    return holder_wp.mirror('XZ'), sensor_wp.rotate((DISC_X, 0, 0), (DISC_X, 0, 1), 180)


def upright(wp, axis_up):
    """Rotate so axis_up -> +Z and sit on Z = 0 (print orientation)."""
    s = wp.val()
    if axis_up == '+X': s = s.rotate(cq.Vector(), cq.Vector(0, 1, 0), -90)
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def main(out_dir):
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    ext, dsc, hol = extender(), disc(), holder()
    motor, sensor, wheel, wall = proxies()
    hol_r, sensor_r = right_hand(hol, sensor)
    for name, wp in (('Shaft Extender', ext), ('Encoder Disc', dsc), ('Encoder Holder Left', hol), ('Encoder Holder Right', hol_r)):
        cq.exporters.export(wp, str(out / f'{name}.step'))
        # print orientation, all with +X up: extender stands on its wall end, disc lies flat,
        # bracket lies on its plate with the shelf and ribs standing up
        cq.exporters.export(cq.Workplane(obj=upright(wp, '+X')), str(out / f'{name}.stl'), tolerance=0.02, angularTolerance=0.05)

    print(f'sensor model: {"LM393.stp" if LM393_STEP.exists() else "NOT FOUND, block stand-in"}')
    print(f'extender: starts {X0} mm off the wall, body to X={X_BODY_END:.1f}, wheel shaft to X={X_END:.1f}')
    print(f'          wheel moves out by {SHIFT} mm -> track width + {2 * SHIFT:.0f} mm')
    print(f'disc:     X={DISC_X0}..{DISC_X0 + DISC_T} ({DISC_X0} mm of open space off the wall), D{DISC_D}, {SLOTS} slots, '
          f'slot width at mid-radius {math.pi * (SLOT_IN_D + SLOT_OUT_D) / 2 / SLOTS / 2:.2f} mm')
    print(f'volumes cm3: extender {ext.val().Volume() / 1000:.1f}, disc {dsc.val().Volume() / 1000:.1f}, '
          f'holder {hol.val().Volume() / 1000:.1f}')
    print('clash check (overlap, mm3):')
    pairs = (('extender', ext), ('disc', dsc), ('holder', hol), ('sensor', sensor), ('motor', motor), ('wheel', wheel), ('wall', wall))
    skip = {('extender', 'disc'), ('extender', 'motor'), ('motor', 'wall')}
    for i, (a, sa) in enumerate(pairs):
        for b, sb in pairs[i + 1:]:
            if (a, b) in skip: continue
            print(f'   {a:9s} x {b:8s}: {sa.val().intersect(sb.val()).Volume():7.2f}')
    print(f'   right holder x right sensor: {hol_r.val().intersect(sensor_r.val()).Volume():.2f}')
    print(f'beam at Z={BEAM_Z:.1f} (slot ring {SLOT_IN_D / 2:.0f}..{SLOT_OUT_D / 2:.0f}); disc edge {SHELF_Z0 - LM_U_BOTTOM - DISC_D / 2:.1f} mm '
          f'below the fork bottom; post tips {SHELF_Z0 - LM_FORK_TOP - BODY_D / 2:.1f} mm over the extender')
    print(f'board X={PCB_X[0]:.1f}..{PCB_X[1]:.1f}; wheel hub face X={X_BODY_END:.0f}, bracket tip X={SHELF_X1:.1f}: '
          f'{X_BODY_END - SHELF_X1:.1f} mm clear')
    urdf_meshes(ext, dsc, hol, hol_r, sensor, sensor_r)
    return ext, dsc, hol, motor, sensor, wheel


def urdf_meshes(ext, dsc, hol, hol_r, sensor, sensor_r):
    """Visual meshes for rover.urdf.xacro, in this file's frame (mm; the URDF scales by 0.001).
    Coarser than the print STLs: they are only drawn in RViz."""
    mesh_dir = HERE / 'ros' / 'rover_description' / 'meshes'
    for name, wp in (('shaft_extender', ext), ('encoder_disc', dsc),
                     ('encoder_holder_left', hol), ('encoder_holder_right', hol_r),
                     ('encoder_sensor_left', sensor), ('encoder_sensor_right', sensor_r)):
        cq.exporters.export(wp, str(mesh_dir / f'{name}.stl'), tolerance=0.1, angularTolerance=0.3)
    print(f'URDF meshes written to {mesh_dir}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'extender_out')
