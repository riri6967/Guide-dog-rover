"""Dimensioned drawings of the haptic handle, all from one set of numbers.

    python handle_drawings.py [out_dir]

Writes handle_views.png (top, front, left, right) and handle_details.png
(motor cup, board pod, rod and hinge). Every dimension below is in mm and is
the value to use in CAD. Change a number here, re-run, and the drawings follow.

Handle frame (used in CANE.md):  origin = centre of the grip bar,
+X = to the person's right, +Y = forward (toward the rover), +Z = up.
"""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle

# ------------------------------------------------------------------ dimensions (mm)
BAR_LEN = 110          # grip length between the legs (X)
BAR_D = 36             # front-to-back depth (Y)
BAR_H = 30             # height (Z)
BAR_R = 6              # corner radius of the rounded-rectangle section
WALL = 2.5             # shell wall thickness

LEG_T = 15             # leg thickness (X)
LEG_D = 24             # leg depth (Y)
YOKE_Z = -65           # top of the cross-member under the bar
YOKE_T = 10            # cross-member thickness (Z)
POD_W, POD_D, POD_H = 60, 55, 23     # board pod outside (X, Y, Z), hangs under the yoke

ROD_OD = 16            # aluminium tube
ROD_ANGLE = 55         # rod angle above horizontal at the nominal hand height
BOSS_OD, BOSS_L = 24, 40             # rod socket boss on the pod
BORE = 16.2            # socket bore

MOTOR_D = 10           # coin motor
HOLE_D = 16            # hole in the shell for each cup
CUP_OD, CUP_ID, CUP_TOP = 12, 10.2, 1.0
CUP_PROUD = 0.5
RING_OD, RING_T = 21, 1.0
RING_FB = (21, 17)     # F/B ring is a stadium, flats top and bottom, to fit the 18 mm face
L_X = 33               # L/R motor centres at X = -/+33 on the top face

SW_POCKET = 12         # 5-way switch pocket, square, top centre
SW_DEPTH = 5
PADDLE = (60, 20, 3)   # hold lever paddle under the bar (X, Y, Z)

BOARD = (23.5, 21)     # vibration motor module
BOARD_HOLE_PITCH = 15  # its two 3 mm mounting holes

HINGE_H = 25           # pivot above the rover's top surface
CLEVIS_GAP, CLEVIS_PLATE, CLEVIS_H = 13, 3, 35

FLAT_TB = BAR_D - 2 * BAR_R      # flat width on top/bottom = 24
FLAT_FB = BAR_H - 2 * BAR_R      # flat height on front/back = 18
ZT, ZB = BAR_H / 2, -BAR_H / 2   # top / bottom faces
YF, YB = BAR_D / 2, -BAR_D / 2   # front / back faces
POD_TOP = YOKE_Z - YOKE_T
POD_BOT = POD_TOP - POD_H
LEG_IN, LEG_OUT = BAR_LEN / 2, BAR_LEN / 2 + LEG_T

SKIN = '#E8B48A'; MOT = '#D85A30'; TPU = '#1D9E75'; PLA = '#B4B2A9'; POD = '#85B7EB'; ROD = '#888780'
DIM = '#333333'


# ------------------------------------------------------------------ drawing helpers
def rrect(ax, x0, y0, w, h, r, fc=PLA, ec='k', lw=1.0, ls='-', alpha=1.0, z=1):
    ax.add_patch(FancyBboxPatch((x0 + r, y0 + r), w - 2 * r, h - 2 * r, boxstyle=f'round,pad={r}',
                                fc=fc, ec=ec, lw=lw, ls=ls, alpha=alpha, zorder=z))


def rect(ax, x0, y0, w, h, fc=PLA, ec='k', lw=1.0, ls='-', alpha=1.0, z=1):
    ax.add_patch(Rectangle((x0, y0), w, h, fc=fc, ec=ec, lw=lw, ls=ls, alpha=alpha, zorder=z))


def hidden(ax, x0, y0, w, h, z=5):
    rect(ax, x0, y0, w, h, fc='none', ec='#555', lw=0.8, ls=(0, (4, 3)), z=z)


def circle(ax, x, y, r, fc='none', ec='k', lw=1.0, ls='-', z=4):
    ax.add_patch(Circle((x, y), r, fc=fc, ec=ec, lw=lw, ls=ls, zorder=z))


def dim(ax, p1, p2, text, off=0.0, side=1, fs=7):
    """Dimension line between p1 and p2, offset perpendicular by `off`."""
    (x1, y1), (x2, y2) = p1, p2
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy) or 1
    nx, ny = -dy / L * off, dx / L * off
    a, b = (x1 + nx, y1 + ny), (x2 + nx, y2 + ny)
    if off:
        ax.plot([x1, a[0]], [y1, a[1]], color=DIM, lw=0.4, zorder=9)
        ax.plot([x2, b[0]], [y2, b[1]], color=DIM, lw=0.4, zorder=9)
    ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='<->', color=DIM, lw=0.6, shrinkA=0, shrinkB=0), zorder=9)
    mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
    ang = math.degrees(math.atan2(dy, dx))
    if ang > 90 or ang < -90: ang += 180
    sgn = (1 if off >= 0 else -1) * side
    ux, uy = -dy / L * sgn, dx / L * sgn            # text sits a fixed 7 pt off the line
    ax.annotate(text, (mx, my), xytext=(ux * 7, uy * 7), textcoords='offset points', fontsize=fs,
                ha='center', va='center', rotation=ang, color=DIM, zorder=10,
                bbox=dict(fc='white', ec='none', pad=0.6))


def label(ax, xy, xytext, text, fs=7):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=fs, color='#222', zorder=11,
                arrowprops=dict(arrowstyle='-', color='#777', lw=0.5), ha='left', va='center')


def setup(ax, title, xlim, ylim):
    ax.set_aspect('equal'); ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.axis('off')
    ax.set_title(title, fontsize=10, fontweight='bold', loc='left')


def rod_band(ax, start, direction, length, width, fc=ROD, z=2, ls='-'):
    sx, sy = start; dx, dy = direction
    nx, ny = -dy * width / 2, dx * width / 2
    pts = [(sx + nx, sy + ny), (sx - nx, sy - ny), (sx - nx + dx * length, sy - ny + dy * length),
           (sx + nx + dx * length, sy + ny + dy * length)]
    ax.add_patch(Polygon(pts, closed=True, fc=fc, ec='k', lw=0.8, ls=ls, zorder=z))
    return (sx + dx * length, sy + dy * length)


# ------------------------------------------------------------------ views
def top_view(ax):
    setup(ax, 'TOP VIEW  (looking down; rover is up the page)', (-110, 125), (-60, 125))
    # rod (below everything, visible in front of the bar)
    ry0 = POD_D / 2
    rod_band(ax, (0, 0), (0, 1), 115, ROD_OD, z=0)
    hidden(ax, -POD_W / 2, -POD_D / 2, POD_W, POD_D)                      # pod underneath
    hidden(ax, -LEG_OUT, -LEG_D / 2, 2 * LEG_OUT, LEG_D)                   # yoke underneath
    rrect(ax, -BAR_LEN / 2, YB, BAR_LEN, BAR_D, 1.5, z=2)                  # bar
    for s in (-1, 1):
        rrect(ax, s * LEG_IN if s > 0 else -LEG_OUT, -LEG_D / 2, LEG_T, LEG_D, 2, z=3)   # legs
    for s in (-1, 1):                                                       # L / R cups
        circle(ax, s * L_X, 0, HOLE_D / 2, fc='white', z=4)
        circle(ax, s * L_X, 0, CUP_OD / 2, fc=TPU, z=5)
        circle(ax, s * L_X, 0, MOTOR_D / 2, ec=MOT, ls='--', z=6)
    rect(ax, -SW_POCKET / 2, -SW_POCKET / 2, SW_POCKET, SW_POCKET, fc='#CECBF6', z=4)
    circle(ax, 0, 0, 3.5, fc='#7F77DD', z=5)
    for y in (YF, YB):                                                      # F / B holes edge-on
        ax.plot([-HOLE_D / 2, HOLE_D / 2], [y, y], color=TPU, lw=3, zorder=6)
    ax.text(-L_X, -13, 'L', ha='center', fontsize=9, fontweight='bold', zorder=8)
    ax.text(L_X, -13, 'R', ha='center', fontsize=9, fontweight='bold', zorder=8)
    ax.text(0, YF + 4, 'F', ha='center', fontsize=9, fontweight='bold', zorder=8)
    ax.text(0, YB - 7, 'B', ha='center', fontsize=9, fontweight='bold', zorder=8)
    dim(ax, (-LEG_OUT, YB), (LEG_OUT, YB), f'{2 * LEG_OUT:.0f} overall', off=-22)
    dim(ax, (-BAR_LEN / 2, YB), (BAR_LEN / 2, YB), f'{BAR_LEN} grip', off=-12)
    dim(ax, (-L_X, 0), (L_X, 0), f'{2 * L_X} L to R', off=13)
    dim(ax, (LEG_OUT, YB), (LEG_OUT, YF), f'{BAR_D}', off=-8)
    dim(ax, (-POD_W / 2, POD_D / 2), (POD_W / 2, POD_D / 2), f'pod {POD_W} (hidden)', off=10)
    label(ax, (0, 0), (40, 52), f'5-way switch, pocket {SW_POCKET}×{SW_POCKET}×{SW_DEPTH} deep')
    label(ax, (L_X + 5, 4), (40, 66), f'cup holes Ø{HOLE_D}, cups Ø{CUP_OD}')
    label(ax, (0, 100), (14, 100), f'rod Ø{ROD_OD} to rover (runs below)')


def front_view(ax):
    """Seen from the rover, looking back at the person: the handle's RIGHT appears on the LEFT."""
    setup(ax, 'FRONT VIEW  (from the rover side; person behind)', (-110, 160), (-185, 45))
    u = lambda x: -x
    ry = POD_BOT
    end = rod_band(ax, (0, ry), (0, -1), BOSS_L * math.cos(math.radians(90 - ROD_ANGLE)), BOSS_OD, fc=PLA, z=1)
    rod_band(ax, end, (0, -1), 45, ROD_OD, z=1)
    rect(ax, -LEG_OUT, YOKE_Z - YOKE_T, 2 * LEG_OUT, YOKE_T, z=2)                 # yoke
    rect(ax, -POD_W / 2, POD_BOT, POD_W, POD_H, fc=POD, z=2)                      # pod
    for s in (-1, 1):
        rect(ax, s * LEG_IN if s > 0 else -LEG_OUT, YOKE_Z - YOKE_T, LEG_T, ZT - (YOKE_Z - YOKE_T), z=2)
    rrect(ax, -BAR_LEN / 2, ZB, BAR_LEN, BAR_H, 1.5, z=3)
    ax.plot([-BAR_LEN / 2, BAR_LEN / 2], [FLAT_FB / 2] * 2, color='#666', lw=0.5, zorder=4)
    ax.plot([-BAR_LEN / 2, BAR_LEN / 2], [-FLAT_FB / 2] * 2, color='#666', lw=0.5, zorder=4)
    rrect(ax, -RING_FB[0] / 2, -RING_FB[1] / 2, RING_FB[0], RING_FB[1], 3, fc='none', ec=TPU, ls='--', z=4)
    circle(ax, 0, 0, HOLE_D / 2, fc='white', z=5)
    circle(ax, 0, 0, CUP_OD / 2, fc=TPU, z=6)
    rect(ax, -PADDLE[0] / 2, ZB - PADDLE[2], PADDLE[0], PADDLE[2], fc='#CECBF6', z=4)
    for s in (-1, 1):
        rect(ax, u(s * L_X) - HOLE_D / 2, ZT - 0.5, HOLE_D, 1.2, fc=TPU, ec=TPU, z=6)
    ax.text(0, -12, 'F', ha='center', fontsize=9, fontweight='bold', zorder=8)
    ax.text(u(-L_X), ZT + 4, 'L', ha='center', fontsize=9, fontweight='bold')
    ax.text(u(L_X), ZT + 4, 'R', ha='center', fontsize=9, fontweight='bold')
    dim(ax, (-LEG_OUT, ZT), (LEG_OUT, ZT), f'{2 * LEG_OUT:.0f}', off=20)
    dim(ax, (-BAR_LEN / 2, ZT), (BAR_LEN / 2, ZT), f'{BAR_LEN}', off=12)
    dim(ax, (-LEG_OUT, ZT), (-LEG_OUT, POD_BOT), f'{ZT - POD_BOT:.0f}', off=-12)
    dim(ax, (LEG_OUT, ZB), (LEG_OUT, ZT), f'{BAR_H}', off=-8)
    dim(ax, (-20, ZB), (-20, POD_TOP + YOKE_T), f'{ZB - POD_TOP - YOKE_T:.0f} hand gap', off=0)
    dim(ax, (-POD_W / 2, POD_BOT), (POD_W / 2, POD_BOT), f'pod {POD_W}×{POD_H}', off=-8)
    dim(ax, (LEG_IN, YOKE_Z - YOKE_T), (LEG_OUT, YOKE_Z - YOKE_T), f'{LEG_T}', off=-8)
    label(ax, (0, ZB - PADDLE[2]), (88, -32), f'hold paddle {PADDLE[0]}×{PADDLE[1]}')
    label(ax, (5, 0), (88, -8), f'F cup, stadium ring {RING_FB[0]}×{RING_FB[1]}')


def side_view(ax, right=False):
    """Left view: from the person's left (-X); the rover (+Y) is on the page's LEFT. Right view mirrors."""
    t = 'RIGHT VIEW  (from the right; rover on the right)' if right else 'LEFT VIEW  (from the left; rover on the left)'
    setup(ax, t, (-150, 115) if not right else (-115, 150), (-205, 45))
    h = (lambda y: y) if right else (lambda y: -y)
    a = math.radians(ROD_ANGLE)
    d = (h(math.cos(a)), -math.sin(a))                 # rod direction, forward and down
    start = (0, POD_BOT)
    end = rod_band(ax, start, d, BOSS_L, BOSS_OD, fc=PLA, z=1)
    rod_band(ax, end, d, 70, ROD_OD, z=1)
    rect(ax, -POD_D / 2, POD_BOT, POD_D, POD_H, fc=POD, z=2)
    rrect(ax, h(YB) if right else h(YF), ZB, BAR_D, BAR_H, BAR_R, z=2)            # bar section (beyond the leg)
    rect(ax, -LEG_D / 2, YOKE_Z - YOKE_T, LEG_D, ZT - (YOKE_Z - YOKE_T), z=3)      # leg, nearest
    rrect(ax, h(YB) if right else h(YF), ZB, BAR_D, BAR_H, BAR_R, fc='none', ec='#555', ls=(0, (4, 3)), z=4)
    for y, nm in ((YF, 'F'), (YB, 'B')):
        ax.plot([h(y)] * 2, [-HOLE_D / 2, HOLE_D / 2], color=TPU, lw=3, zorder=6)
        ax.text(h(y) + h(5 if y > 0 else -5) * 1.0, 0, nm, ha='center', va='center', fontsize=9, fontweight='bold', zorder=8)
    ax.plot([h(-HOLE_D / 2) if False else -HOLE_D / 2, HOLE_D / 2], [ZT, ZT], color=TPU, lw=3, zorder=6)
    rect(ax, min(h(-12), h(8)), ZB - PADDLE[2], 20, PADDLE[2], fc='#CECBF6', z=5)
    circle(ax, h(-12), ZB - 1.5, 1.5, fc='k', z=6)
    if right:
        ax.plot([0, 0], [ZB, POD_TOP], color=MOT, lw=1.2, ls=(0, (2, 2)), zorder=7)
        label(ax, (0, -40), (40, -40), 'motor wires down this leg')
    # angle annotation
    ax.plot([0, h(60)], [POD_BOT - 0, POD_BOT], color=DIM, lw=0.4, ls=':')
    ax.text(h(48), POD_BOT - 12, f'{ROD_ANGLE}°', fontsize=8, color=DIM)
    dim(ax, (h(YF), ZT), (h(YB), ZT), f'{BAR_D}', off=10 if right else -10)
    dim(ax, (h(YF), ZB), (h(YF), ZT), f'{BAR_H}', off=-8 if right else 8)
    dim(ax, (-POD_D / 2, POD_BOT), (POD_D / 2, POD_BOT), f'pod {POD_D}', off=-8)
    label(ax, (h(-12), ZB - 1.5), (h(-75) if not right else h(-75), -35), 'paddle pivot (rear)')
    ax.annotate('to rover', xy=(h(100), -175), xytext=(h(60), -175), fontsize=7,
                arrowprops=dict(arrowstyle='->', lw=0.6), ha='center', va='center')


# ------------------------------------------------------------------ details sheet
def cup_detail(ax):
    setup(ax, f'MOTOR CUP (TPU) — section, L/R version', (-20, 24), (-14, 9))
    z_skin_top = CUP_PROUD
    rect(ax, -18, -WALL, 18 - HOLE_D / 2, WALL, z=1); rect(ax, HOLE_D / 2, -WALL, 18 - HOLE_D / 2, WALL, z=1)   # shell
    rect(ax, -RING_OD / 2, -WALL - RING_T, (RING_OD - HOLE_D) / 2, RING_T, fc=TPU, z=2)                        # ring
    rect(ax, HOLE_D / 2, -WALL - RING_T, (RING_OD - HOLE_D) / 2, RING_T, fc=TPU, z=2)
    cup_bot = -WALL - RING_T - 1.0
    rect(ax, -CUP_OD / 2, cup_bot, CUP_OD, z_skin_top - cup_bot, fc=TPU, z=2)                                  # cup
    rect(ax, -CUP_ID / 2, cup_bot, CUP_ID, z_skin_top - CUP_TOP - cup_bot, fc='white', z=3)                    # cavity
    rect(ax, -MOTOR_D / 2, z_skin_top - CUP_TOP - 2.7, MOTOR_D, 2.7, fc=MOT, z=4)                               # motor
    for s in (-1, 1):
        rect(ax, s * CUP_OD / 2 if s > 0 else -HOLE_D / 2, -WALL - RING_T + 0.2, (HOLE_D - CUP_OD) / 2, 0.6, fc=TPU, z=2)
    ax.plot([-1, -1], [cup_bot, cup_bot - 4], color='#D85A30', lw=1); ax.plot([1, 1], [cup_bot, cup_bot - 4], color='#185FA5', lw=1)
    dim(ax, (-HOLE_D / 2, 4), (HOLE_D / 2, 4), f'hole Ø{HOLE_D}', off=2)
    dim(ax, (-CUP_OD / 2, cup_bot), (CUP_OD / 2, cup_bot), f'cup Ø{CUP_OD}', off=-4)
    dim(ax, (-RING_OD / 2, -8.5), (RING_OD / 2, -8.5), f'ring Ø{RING_OD} × {RING_T}', off=-2)
    label(ax, (CUP_OD / 2, CUP_PROUD), (14.5, 3), f'{CUP_PROUD} proud of shell')
    label(ax, (3, z_skin_top - 0.5), (14.5, 5.5), f'top {CUP_TOP}')
    label(ax, (14, -1.2), (14.5, -1.2), f'shell {WALL}')
    label(ax, (6.5, -WALL - RING_T + 0.5), (14.5, -5), '3 spiral spokes 0.6 thick,\n2 wide, ~6 long, in a 2 mm gap')
    label(ax, (0, z_skin_top - CUP_TOP - 1.3), (14.5, -10.5), f'cavity Ø{CUP_ID}: motor Ø{MOTOR_D}×2.7')


def cup_top(ax):
    setup(ax, 'CUP from above (L/R round ring; F/B stadium ring)', (-14, 42), (-14, 14))
    circle(ax, 0, 0, RING_OD / 2, fc=TPU)
    circle(ax, 0, 0, HOLE_D / 2, fc='white', z=5)
    for k in range(3):
        th0 = math.radians(90 + 120 * k)
        pts = []
        for i in range(12):                                     # spiral spoke from r=6 to r=8 over 70 deg
            f = i / 11; r = CUP_OD / 2 + f * (HOLE_D - CUP_OD) / 2; th = th0 + math.radians(70) * f
            pts.append((r * math.cos(th), r * math.sin(th)))
        ax.plot(*zip(*pts), color='#0F6E56', lw=2.0, zorder=6)
    circle(ax, 0, 0, CUP_OD / 2, fc=TPU, z=7)
    circle(ax, 0, 0, MOTOR_D / 2, ec=MOT, ls='--', z=8)
    rrect(ax, 14, -RING_FB[1] / 2, RING_FB[0], RING_FB[1], 4, fc=TPU, z=2)
    circle(ax, 14 + RING_FB[0] / 2, 0, HOLE_D / 2 - 0.01, fc='white', z=5)
    circle(ax, 14 + RING_FB[0] / 2, 0, CUP_OD / 2, fc=TPU, z=7)
    dim(ax, (14, -RING_FB[1] / 2), (14 + RING_FB[0], -RING_FB[1] / 2), f'{RING_FB[0]}', off=-2.5)
    dim(ax, (14 + RING_FB[0], -RING_FB[1] / 2), (14 + RING_FB[0], RING_FB[1] / 2), f'{RING_FB[1]}', off=-2.5)


def pod_detail(ax):
    setup(ax, f'BOARD POD — inside, from above ({POD_W}×{POD_D}×{POD_H} outside, 2.5 wall)', (-40, 62), (-36, 36))
    rect(ax, -POD_W / 2, -POD_D / 2, POD_W, POD_D, fc=POD, z=1)
    rect(ax, -POD_W / 2 + 2.5, -POD_D / 2 + 2.5, POD_W - 5, POD_D - 5, fc='white', z=2)
    names = [('F', -1, 1), ('B', 1, 1), ('L', -1, -1), ('R', 1, -1)]
    for nm, sx, sy in names:
        x0 = sx * 1.5 if sx > 0 else -1.5 - BOARD[0]
        y0 = sy * 1.5 if sy > 0 else -1.5 - BOARD[1]
        rect(ax, x0, y0, BOARD[0], BOARD[1], fc='#3a3a3a', z=3)
        cx, cy = x0 + BOARD[0] / 2, y0 + BOARD[1] / 2
        for hx in (-BOARD_HOLE_PITCH / 2, BOARD_HOLE_PITCH / 2):
            circle(ax, cx + hx, cy + 6, 1.5, fc='white', z=4)
        circle(ax, cx - 4, cy - 2, 5, fc='none', ec='#bbb', ls='--', z=4)
        ax.text(cx + 6, cy - 3, nm, color='white', fontsize=9, fontweight='bold', zorder=5)
    circle(ax, 0, 0, BORE / 2, fc='none', ec='k', ls='--', z=6)
    dim(ax, (-POD_W / 2, POD_D / 2), (POD_W / 2, POD_D / 2), f'{POD_W}', off=4)
    dim(ax, (POD_W / 2, -POD_D / 2), (POD_W / 2, POD_D / 2), f'{POD_D}', off=-4)
    label(ax, (-BOARD[0] - 1.5 + BOARD[0] / 2 - BOARD_HOLE_PITCH / 2, 1.5 + BOARD[1] / 2 + 6), (34, 26),
          f'M2.5 standoffs at the boards\' Ø3 holes, {BOARD_HOLE_PITCH} apart')
    label(ax, (BORE / 2, 0), (34, 8), 'rod socket below (cable exits here)')
    label(ax, (-15, 27), (34, -10), 'motor wires in from the right leg')
    label(ax, (20, -20), (34, -24), f'boards {BOARD[0]}×{BOARD[1]}, headers and motors removed\n'
                                    '(dashed = where the motor was)')


def rod_detail(ax):
    setup(ax, 'ROD AND HINGE — side elevation (schematic)', (-60, 520), (-40, 820))
    a = math.radians(ROD_ANGLE)
    # rover body
    rect(ax, -40, -40, 300, 40, fc='#D3D1C7', z=1); ax.text(110, -22, 'rover top', ha='center', fontsize=7)
    rect(ax, 0, 0, CLEVIS_PLATE * 3, CLEVIS_H, fc='#888', z=2)
    circle(ax, 4.5, HINGE_H, 4, fc='white', z=3)
    rod_len = 640
    sx, sy = 4.5, HINGE_H
    ex, ey = sx + rod_len * math.cos(a), sy + rod_len * math.sin(a)
    rod_band(ax, (sx, sy), (math.cos(a), math.sin(a)), rod_len, ROD_OD * 1.6, z=2)
    rrect(ax, ex - 20, ey - 10, 90, 40, 6, fc=PLA, z=3)
    ax.text(ex + 25, ey + 10, 'handle', ha='center', fontsize=7, zorder=4)
    ax.plot([sx, sx + 160], [sy, sy], color=DIM, lw=0.5, ls=':')
    ax.text(sx + 110, sy + 30, f'{ROD_ANGLE}°', fontsize=9, color=DIM)
    label(ax, (4.5, HINGE_H), (60, 120), f'M6 pivot, {HINGE_H} above rover top,\nrubber grommet in the rod plug')
    label(ax, (sx + 40, sy + 60), (200, 260), 'tube Ø16×1 aluminium\ncable runs inside')
    ax.text(250, 520, 'Rod length (hinge to grip centre):\n'
                      'L = √((H_grip − 155)² + 450²)\n'
                      'H_grip ≈ 0.45 × user height\n\n'
                      'height   H_grip   L      angle   tube cut\n'
                      '1.55 m   700     707   50.5°    ~565\n'
                      '1.70 m   765     758   53.6°    ~620\n'
                      '1.85 m   830     811   56.3°    ~670',
            fontsize=7, family='monospace', va='top',
            bbox=dict(fc='#F1EFE8', ec='#888', lw=0.5, pad=4))


def main(out):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(2, 2, figsize=(15, 12))
    top_view(axs[0, 0]); front_view(axs[0, 1]); side_view(axs[1, 0]); side_view(axs[1, 1], right=True)
    fig.suptitle('Haptic handle — all dimensions in mm (origin: grip centre; +X right, +Y toward rover, +Z up)', fontsize=12)
    fig.tight_layout(); fig.savefig(out / 'handle_views.png', dpi=130); plt.close(fig)

    fig, axs = plt.subplots(2, 2, figsize=(15, 11))
    cup_detail(axs[0, 0]); cup_top(axs[0, 1]); pod_detail(axs[1, 0]); rod_detail(axs[1, 1])
    fig.suptitle('Haptic handle — details', fontsize=12)
    fig.tight_layout(); fig.savefig(out / 'handle_details.png', dpi=130); plt.close(fig)
    print(f'wrote {out / "handle_views.png"} and {out / "handle_details.png"}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'handle_drawings_out')
