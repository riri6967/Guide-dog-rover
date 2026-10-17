"""Build the haptic handle around the user's grip (Grip Top Part A / B) with CadQuery.

    python build_handle.py <folder with Grip Top Part A.STEP / B.STEP> <out folder>

Writes one STEP per part (for SolidWorks), an assembly STEP with the user's
grip included, and preview renders.

Coordinates are the grip's own (Part A's SolidWorks origin), all mm:
    X along the bar (L/R motor pockets on the end caps at X = -/+60)
    Y up            (+Y is the top, under the palm)
    Z forward       (+Z front, toward the rover; -Z back, Part B)
Every part is modelled in place, so in SolidWorks all origins coincide.

Design (see CANE.md):
  - Frame (one print): a cradle under each end of the bar, two legs, a
    cross-piece 50 mm below the bar (finger room), a junction pod and the rod
    socket at ROD_ANGLE. Wire channel: right cradle -> right leg -> cross-piece
    -> pod.
  - Two caps screw onto the cradles over the bar ends (2 x M3 each). Each
    cradle/cap end wall has a window over the user's end-cap motor pocket.
    The right cap carries the 5-way thumb switch.
  - Bail: a bar under the grip, pivoting on M3 pins in the legs; squeezing it
    with the fingers works a microswitch in the right leg (released = stop).
  - Pod lid, rod end plug, rover hinge bracket.
"""
import math
import sys
from pathlib import Path

import cadquery as cq

# ----------------------------------------------------------------- the user's grip (measured)
GRIP_X = 60.0          # bar ends at X = +/-60
GRIP_Y, GRIP_Z = 35.0, 36.0    # height (Y) and depth (Z)
GRIP_R = 8.4           # corner radius
END_POCKET = (21.0, 23.0)      # board pocket in each end cap (Y, Z)

# ----------------------------------------------------------------- handle parameters
CL = 0.3               # clearance around the bar
WALL = 3.0             # cradle / cap wall
CRADLE_X0 = 48.0       # cradle covers the bar from |X| = 48 to the end
END_T = 3.0            # end wall thickness beyond the bar end
WINDOW = (25.0, 27.0)  # opening in the end wall over the pocket (Y, Z)
SPLIT_Y = 4.0          # cradle below, cap above
EAR_W, EAR_H = 8.0, 6.0        # screw ears either side (Z width, height each half)

LEG_X = (48.5, 62.5)   # leg spans this |X| range
LEG_Z = 12.0           # legs and cross-piece are 24 deep (Z -12..12)
GAP = 50.0             # finger room: bar underside to cross-piece top
XP_T = 12.0            # cross-piece thickness
WIRE_D = 6.0           # wire channel

POD = (50.0, 22.0, 30.0)       # junction pod outside (X, Y, Z)
POD_WALL = 2.5
ROD_ANGLE = 50.0       # rod angle above horizontal
ROD_OD, BORE = 16.0, 16.2
BOSS_D, BOSS_L = 24.0, 40.0
TUBE_LEN = 605.0       # cut length for a ~1.70 m user (see CANE.md table)

PIVOT = (-30.0, -9.0)  # bail pivot (Y, Z), pin along X through both legs
BAIL_BAR = (-26.0, -20.5, -2.0, 8.0)   # crossbar y0, y1, z0, z1 (3 mm under the bar at rest)

M3_CLEAR, M3_INSERT, M4_INSERT = 3.2, 4.0, 5.6

BAR_BOT = -GRIP_Y / 2                  # -17.5
XP_TOP = BAR_BOT - GAP                 # -67.5
XP_BOT = XP_TOP - XP_T                 # -79.5
POD_TOP, POD_BOT = XP_BOT, XP_BOT - POD[1]
ROD_DIR = cq.Vector(0, -math.sin(math.radians(ROD_ANGLE)), math.cos(math.radians(ROD_ANGLE)))
SOCKET_BASE = cq.Vector(0, POD_BOT + 6, 0)


# ----------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def profile_prism(x0, length, w_y, h_z, r):
    """Rounded-rectangle prism along +X, section w_y (Y) x h_z (Z)."""
    sk = cq.Sketch().rect(w_y, h_z).vertices().fillet(r)
    return cq.Workplane('YZ', origin=(x0, 0, 0)).placeSketch(sk).extrude(length)


def cyl(d, length, start, direction):
    return cq.Workplane(obj=cq.Solid.makeCylinder(d / 2, length, cq.Vector(*start), cq.Vector(*direction)))


# ----------------------------------------------------------------- end collar (cradle + cap), right side
def right_collar():
    x0, x_end = CRADLE_X0, GRIP_X + CL
    out_y, out_z = GRIP_Y + 2 * (CL + WALL), GRIP_Z + 2 * (CL + WALL)
    in_y, in_z = GRIP_Y + 2 * CL, GRIP_Z + 2 * CL
    sleeve = profile_prism(x0, x_end - x0, out_y, out_z, GRIP_R + CL + WALL) \
        .cut(profile_prism(x0 - 1, x_end - x0 + 1, in_y, in_z, GRIP_R + CL))
    wall = profile_prism(x_end, END_T, out_y, out_z, GRIP_R + CL + WALL) \
        .cut(box(x_end - 1, x_end + END_T + 1, -WINDOW[0] / 2, WINDOW[0] / 2, -WINDOW[1] / 2, WINDOW[1] / 2))
    zc = in_z / 2 + WALL
    ears = box(x0, x_end, SPLIT_Y - EAR_H, SPLIT_Y + EAR_H, zc - 1, zc + EAR_W) \
        .union(box(x0, x_end, SPLIT_Y - EAR_H, SPLIT_Y + EAR_H, -zc - EAR_W, -zc + 1))
    collar = sleeve.union(wall).union(ears)
    xs = (x0 + x_end) / 2
    for z in (zc + EAR_W / 2 - 0.5, -(zc + EAR_W / 2 - 0.5)):
        collar = collar.cut(cyl(M3_CLEAR, 2 * EAR_H + 2, (xs, SPLIT_Y - EAR_H - 1, z), (0, 1, 0)))
    return collar


def split(collar):
    big = 300
    cradle = collar.intersect(box(-big, big, -big, SPLIT_Y, -big, big))
    cap = collar.intersect(box(-big, big, SPLIT_Y, big, -big, big))
    return cradle, cap


# ----------------------------------------------------------------- frame
def frame():
    rc_cradle, _ = split(right_collar())
    lc_cradle = rc_cradle.mirror('YZ')
    out_bot = BAR_BOT - CL - WALL                       # underside of the cradle
    legs = box(LEG_X[0], LEG_X[1], XP_TOP, out_bot + 1, -LEG_Z, LEG_Z)
    legs = legs.union(legs.mirror('YZ'))
    xp = box(-LEG_X[1], LEG_X[1], XP_BOT, XP_TOP, -LEG_Z, LEG_Z)
    pod = box(-POD[0] / 2, POD[0] / 2, POD_BOT, POD_TOP + 0.5, -POD[2] / 2, POD[2] / 2)
    boss = cyl(BOSS_D, BOSS_L, SOCKET_BASE, ROD_DIR)
    f = rc_cradle.union(lc_cradle).union(legs).union(xp).union(pod).union(boss)

    # hollow the pod, open at the back for the lid
    f = f.cut(box(-POD[0] / 2 + POD_WALL, POD[0] / 2 - POD_WALL, POD_BOT + POD_WALL, POD_TOP - 0.01,
                  -POD[2] / 2 - 1, POD[2] / 2 - POD_WALL))
    # rod bore + grub screw insert hole
    end = SOCKET_BASE + ROD_DIR * BOSS_L
    f = f.cut(cyl(BORE, BOSS_L - 6, end, -ROD_DIR))
    f = f.cut(cyl(WIRE_D, BOSS_L, SOCKET_BASE - ROD_DIR * 2, ROD_DIR))
    g = end - ROD_DIR * 15
    f = f.cut(cyl(M4_INSERT, 14, (0, g.y, g.z), (1, 0, 0)))
    # wire channel: right cradle floor -> down the right leg -> along the cross-piece -> into the pod
    xw = 54.0
    f = f.cut(cyl(WIRE_D, BAR_BOT - (XP_BOT + XP_T / 2) + 4, (xw, BAR_BOT + 2, 0), (0, -1, 0)))
    f = f.cut(cyl(WIRE_D, xw, (xw, XP_BOT + XP_T / 2, 0), (-1, 0, 0)))
    f = f.cut(cyl(WIRE_D, XP_T / 2 + 2, (0, XP_BOT + XP_T / 2, 0), (0, -1, 0)))
    # cap screw inserts in the cradle ears
    zc = (GRIP_Z + 2 * CL) / 2 + WALL
    xs = (CRADLE_X0 + GRIP_X + CL) / 2
    for sx in (1, -1):
        for z in (zc + EAR_W / 2 - 0.5, -(zc + EAR_W / 2 - 0.5)):
            f = f.cut(cyl(M3_INSERT, 7, (sx * xs, SPLIT_Y - 7, z), (0, 1, 0)))
    # bail pivot holes and the microswitch pocket in the right leg's inner face
    # (subminiature lever switch, D2F size 12.8 x 6.5 x 5.8, lever facing the bail's tail)
    f = f.cut(cyl(M3_CLEAR, 2 * LEG_X[1] + 2, (-LEG_X[1] - 1, PIVOT[0], PIVOT[1]), (1, 0, 0)))
    f = f.cut(box(LEG_X[0] - 0.01, LEG_X[0] + 6.2, -48, -34, PIVOT[1] - 5, PIVOT[1] + 2))
    # lid screw inserts (M2.5) in the pod's back corners
    for x in (-POD[0] / 2 + 4, POD[0] / 2 - 4):
        f = f.cut(cyl(3.6, 6, (x, POD_BOT + 4, -POD[2] / 2 - 0.01), (0, 0, 1)))
    return f


def caps():
    _, cap = split(right_collar())
    # thumb switch housing on the right cap's top (left-hand use: thumb points to +X)
    top = GRIP_Y / 2 + CL + WALL
    sw = box(CRADLE_X0 + 1, GRIP_X + CL, top - 1, top + 6, -9, 9)
    cap_r = cap.union(sw).cut(box(54 - 6, 54 + 6, top + 1, top + 6.1, -6, 6)) \
        .cut(cyl(WIRE_D, 10, (54, top + 1.5, 0), (0, -1, 0)))
    cap_l = cap.mirror('YZ')
    return cap_r, cap_l


def bail():
    t = 3.5
    arm_pts = [(PIVOT[0] - 4, PIVOT[1]), (PIVOT[0], PIVOT[1] - 4), (PIVOT[0] + 4, PIVOT[1] - 1),
               (BAIL_BAR[1], BAIL_BAR[2] + 1), (BAIL_BAR[1], BAIL_BAR[3]), (BAIL_BAR[0], BAIL_BAR[3]),
               (PIVOT[0] - 3, PIVOT[1] + 4)]
    tail = [(PIVOT[0] - 3, PIVOT[1] - 2), (-42, PIVOT[1] - 2), (-42, PIVOT[1] + 2), (PIVOT[0] - 3, PIVOT[1] + 2)]
    x_in = LEG_X[0] - 0.5 - t
    b = box(-x_in, x_in, BAIL_BAR[0], BAIL_BAR[1], BAIL_BAR[2], BAIL_BAR[3])
    for sx in (1, -1):
        x0 = x_in if sx > 0 else -x_in - t
        arm = cq.Workplane('YZ', origin=(x0, 0, 0)).polyline(arm_pts).close().extrude(t)
        b = b.union(arm)
        if sx > 0:      # tail on the right presses the microswitch in the right leg
            b = b.union(cq.Workplane('YZ', origin=(x0, 0, 0)).polyline(tail).close().extrude(t))
    return b.cut(cyl(M3_CLEAR, 2 * LEG_X[1], (-LEG_X[1], PIVOT[0], PIVOT[1]), (1, 0, 0)))


def pod_lid():
    w, h = POD[0], POD[1]
    lid = box(-w / 2, w / 2, POD_BOT, POD_TOP, -POD[2] / 2 - 2.5, -POD[2] / 2)
    for x in (-w / 2 + 4, w / 2 - 4):
        lid = lid.cut(cyl(2.8, 4, (x, POD_BOT + 4, -POD[2] / 2 - 3), (0, 0, 1)))
    return lid.cut(cyl(8, 4, (12, POD_BOT + h / 2, -POD[2] / 2 - 3), (0, 0, 1)))   # cable grommet hole


# ----------------------------------------------------------------- rod, plug, hinge (placed for the assembly)
def rod_parts():
    end = SOCKET_BASE + ROD_DIR * BOSS_L
    tube_start = end - ROD_DIR * (BOSS_L - 6)
    tube = cyl(ROD_OD, TUBE_LEN, tube_start, ROD_DIR).cut(cyl(ROD_OD - 2, TUBE_LEN + 2, tube_start - ROD_DIR, ROD_DIR))
    t_end = tube_start + ROD_DIR * TUBE_LEN
    # plug: 25 mm inside the tube + 30 mm tongue (12 thick in X) with the grommet hole
    plug = cyl(ROD_OD - 2.1, 25, t_end - ROD_DIR * 25, ROD_DIR)
    perp = cq.Vector(0, ROD_DIR.z, -ROD_DIR.y)          # in the Y-Z plane, square to the rod
    pivot = t_end + ROD_DIR * 18
    tongue = (cq.Workplane('YZ', origin=(-6, 0, 0))
              .polyline([(p.y, p.z) for p in (t_end + perp * 8, t_end - perp * 8,
                                                pivot - perp * 9 + ROD_DIR * 9, pivot + perp * 9 + ROD_DIR * 9)])
              .close().extrude(12))
    plug = plug.union(tongue).cut(cyl(12, 14, (-7, pivot.y, pivot.z), (1, 0, 0)))
    # rover hinge bracket: base plate on the rover top, two 3 mm plates, 13 mm gap
    top_y = pivot.y - 25
    br = box(-25, 25, top_y, top_y + 4, pivot.z - 15, pivot.z + 15)
    for x0 in (-6.5 - 3, 6.5):
        br = br.union(box(x0, x0 + 3, top_y + 4, top_y + 35, pivot.z - 15, pivot.z + 15))
    br = br.cut(cyl(6.4, 40, (-20, pivot.y, pivot.z), (1, 0, 0)))
    for a in (35, 75):          # stop-bolt holes so the rod stays within 35-75 deg
        r = math.radians(a + 180 - 25)
        p = pivot + cq.Vector(0, math.sin(r), -math.cos(r)) * 17
        br = br.cut(cyl(4.3, 40, (-20, p.y, p.z), (1, 0, 0)))
    for x in (-19, 19):
        for dz in (-10, 10):
            br = br.cut(cyl(3.3, 6, (x, top_y - 1, pivot.z + dz), (0, 1, 0)))
    return tube, plug, br, pivot, top_y


def rover_proxy(pivot, top_y):
    """Rover body and wheels for context only (rover forward = +Z here)."""
    L, W, H, clear, wr = 274, 208, 130, 40, 55
    z_rear = pivot.z - 20
    body = box(-W / 2, W / 2, top_y - H, top_y, z_rear, z_rear + L)
    wy = top_y - H + clear - 15
    for x in (-120, 120):
        for z in (z_rear + 63, z_rear + 211):
            body = body.union(cyl(2 * wr, 41, (x - 20.5, wy, z), (1, 0, 0)))
    lidar = cyl(72, 55, (0, top_y, z_rear + 225), (0, 1, 0))
    return body, lidar


def main(grip_dir, out_dir):
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    gA = cq.importers.importStep(str(Path(grip_dir) / 'Grip Top Part A.STEP'))
    gB = cq.importers.importStep(str(Path(grip_dir) / 'Grip Top Part B.STEP'))
    # Part B is modelled in its own frame; this places it on Part A's open back (measured fit)
    # (x, y, z)_A = (-z, y - 0.44, x - 0.07)_B : a -90 deg turn about Y, then a small shift
    gB = gB.rotate((0, 0, 0), (0, 1, 0), -90).translate((0, -0.44, -0.07))

    parts = {}
    parts['Frame'] = frame()
    parts['Cap Right (switch)'], parts['Cap Left'] = caps()
    parts['Bail'] = bail()
    parts['Pod Lid'] = pod_lid()
    tube, plug, bracket, pivot, top_y = rod_parts()
    parts['Rod Plug'] = plug
    parts['Hinge Bracket'] = bracket

    for name, wp in parts.items():
        cq.exporters.export(wp, str(out / f'{name}.step'))
        bb = wp.val().BoundingBox()
        print(f'{name:20s} vol {wp.val().Volume():9.0f} mm3   size {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f}')
    cq.exporters.export(tube, str(out / 'Rod Tube (aluminium, reference).step'))

    colors = {'Frame': (0.55, 0.57, 0.6), 'Cap Right (switch)': (0.45, 0.47, 0.5), 'Cap Left': (0.45, 0.47, 0.5),
              'Bail': (0.5, 0.45, 0.85), 'Pod Lid': (0.35, 0.55, 0.85), 'Rod Plug': (0.85, 0.55, 0.3),
              'Hinge Bracket': (0.3, 0.3, 0.3)}
    asm = cq.Assembly(name='Haptic handle')
    asm.add(gA, name='Grip Top Part A (yours)', color=cq.Color(0.82, 0.8, 0.75))
    asm.add(gB, name='Grip Top Part B (yours)', color=cq.Color(0.6, 0.75, 0.9))
    for name, wp in parts.items():
        asm.add(wp, name=name, color=cq.Color(*colors[name]))
    asm.add(tube, name='Rod tube', color=cq.Color(0.7, 0.7, 0.72))
    asm.save(str(out / 'Handle Assembly.step'))

    body, lidar = rover_proxy(pivot, top_y)
    return gA, gB, parts, tube, body, lidar


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
