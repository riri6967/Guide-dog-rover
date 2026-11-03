"""Dimensions sheet for the parts in encoder_extender.py, drawn from its own numbers.

    python encoder_extender_drawing.py [out.png]

Default output: CAD Files/Wheel Extender/Wheel Extender dimensions.png.
Re-run after changing encoder_extender.py so the sheet and the parts agree.
"""
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

from encoder_extender import *          # noqa: F403 - the sheet uses the part's constants as they are

OUT = HERE.parent / 'CAD Files' / 'Wheel Extender' / 'Wheel Extender dimensions.png'
DIM, PART, EDGE = '#b03030', '#c9d6e3', '#1a1a2e'


def dim(ax, p0, p1, text, off=(0, 0), rot=0, size=8.5):
    """Dimension arrow from p0 to p1 with text at the middle (+ off)."""
    ax.annotate('', p1, p0, arrowprops=dict(arrowstyle='<->', color=DIM, lw=0.9, shrinkA=0, shrinkB=0))
    ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, color=DIM, fontsize=size,
            ha='center', va='center', rotation=rot)


def rect(ax, x0, x1, y0, y1, **kw):
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **kw))


def note(ax, x, y, text, **kw):
    ax.text(x, y, text, **{'fontsize': 8, 'ha': 'center', 'va': 'center', **kw})


def setup(ax, title, xlim, ylim):
    ax.set_title(title, fontsize=12, loc='left')
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect('equal'); ax.axis('off')


def extender_section(ax):
    setup(ax, f'1  Shaft Extender - section through the axis (print 4, PETG 100 %)', (-14, X_END + 9), (-58, 32))
    rect(ax, -WALL_T, 0, -24, 28, fc='#e8e8e8', ec='#888')
    note(ax, -WALL_T / 2, 30, 'body wall', color='#666')
    rect(ax, -WALL_T - 4, SHAFT_OUT, -SHAFT_D / 2, SHAFT_D / 2, fc='#bbb', ec='none', zorder=2.6)
    note(ax, -WALL_T - 2, -8, f'motor shaft D{SHAFT_D:g}\n{SHAFT_OUT:g} out of the wall', color='#555', fontsize=7.5)
    r_b, r_s, r_o = BODY_D / 2, SHOULDER_D / 2, OUT_D / 2
    top = [(X0, r_b), (SHOULDER_X0, r_b), (DISC_X0 - 1, r_s), (DISC_X0, r_s), (DISC_X0, r_b),
           (X_BODY_END - 2, r_b), (X_BODY_END, 6), (X_BODY_END, r_o), (X_END, r_o)]
    outline = top + [(x, -z) for x, z in reversed(top)]
    ax.add_patch(Polygon(outline, closed=True, fc=PART, ec=EDGE, lw=1.4, hatch='///', zorder=2))
    bore_end = SHAFT_OUT + 0.4
    rect(ax, X0, bore_end, -BORE / 2, BORE / 2, fc='white', ec=EDGE, lw=1.2, zorder=2.5)
    x_low, x_side = (x for x, _ in SET_SCREWS)
    rect(ax, x_low - SET_PILOT / 2, x_low + SET_PILOT / 2, -r_b, -BORE / 2, fc='white', ec=EDGE, lw=1, zorder=2.5)
    ax.plot([x_side] * 2, [-r_b - 1, r_b + 1], ls='--', color=EDGE, lw=0.8, zorder=4)
    note(ax, x_low, -r_b - 3.2, 'M1.7 set screw\n(from below)', fontsize=7.5)
    note(ax, x_side + 1, r_b + 9, 'M1.7 set screw\n(from the side)', fontsize=7.5)
    ax.plot([x_side, x_side], [r_b + 1, r_b + 6.5], color=EDGE, lw=0.6)
    rect(ax, DISC_X0, DISC_X0 + DISC_T, -DISC_D / 2, DISC_D / 2, fc='#9a9a9a', ec='none', alpha=0.75, zorder=3)
    note(ax, DISC_X + 3, DISC_D / 2 + 2.5, 'disc (part 2)')
    ax.plot([-WALL_T - 6, X_END + 6], [0, 0], ls='-.', color='#888', lw=0.6, zorder=1)
    note(ax, X_BODY_END + 1, r_b + 12, f'taper {BODY_D:g} -> 12\nover 2 (hub seat)', fontsize=7.5)
    dim(ax, (X_BODY_END, 11), (X_END, 11), f'wheel pin {HUB_DEPTH:g} long', off=(0, 2))
    # diameters
    dim(ax, (X0 + 2.3, -BORE / 2), (X0 + 2.3, BORE / 2), f'bore {BORE:g}', off=(-1.4, 0), rot=90, size=7.5)
    dim(ax, (SHOULDER_X0 - 0.5, -r_s), (SHOULDER_X0 - 0.5, r_s), f'shoulder {SHOULDER_D:g}\n(45 deg)', off=(-0.5, r_s + 2.5), size=7.5)
    dim(ax, (X_BODY_END - 6, -r_b), (X_BODY_END - 6, r_b), f'body {BODY_D:g}', off=(-1.4, 0), rot=90, size=7.5)
    dim(ax, (X_END + 2, -r_o), (X_END + 2, r_o), f'pin {OUT_D:g}\n(flat {OUT_FLAT:g})', off=(3.5, 0), rot=90, size=7.5)
    # positions from the wall's outer face
    stops = [(X0, f'{X0:g}'), (x_low, f'{x_low:g}'), (x_side, f'{x_side:g}'), (DISC_X0, f'{DISC_X0:g}'),
             (bore_end, f'bore ends {bore_end:g}'), (X_BODY_END, f'{X_BODY_END:g}'), (X_END, f'{X_END:g}')]
    for i, (x, t) in enumerate(stops):
        y = -27 - 4.2 * i
        ax.plot([x, x], [y, -DISC_D / 2 if DISC_X0 <= x <= DISC_X0 + DISC_T else -r_b], color=DIM, lw=0.3, alpha=0.6)
        if i == 0:
            ax.annotate('', (x, y), (x - 3, y), arrowprops=dict(arrowstyle='->', color=DIM, lw=0.9))
            ax.text(x + 1, y, t, color=DIM, fontsize=8.5, va='center')
        else:
            dim(ax, (0, y), (x, y), t, off=(0, 1.6))
    ax.plot([0, 0], [-24, -27 - 4.2 * (len(stops) - 1)], color=DIM, lw=0.3, alpha=0.6)


def disc_face(ax):
    setup(ax, f'2  Encoder Disc - face (print 2, BLACK, {DISC_T:g} thick)', (-30, 30), (-40, 30))
    ax.add_patch(Circle((0, 0), DISC_D / 2, fc=PART, ec=EDGE, lw=1.4))
    ax.add_patch(Circle((0, 0), DISC_BORE / 2, fc='white', ec=EDGE, lw=1.2))
    pitch = 360 / SLOTS
    for i in range(SLOTS):
        a0, a1 = math.radians(i * pitch - pitch / 4), math.radians(i * pitch + pitch / 4)
        pts = [(r * math.cos(s + (t - s) * k / 6), r * math.sin(s + (t - s) * k / 6))
               for r, (s, t) in ((SLOT_OUT_D / 2, (a0, a1)), (SLOT_IN_D / 2, (a1, a0))) for k in range(7)]
        ax.add_patch(Polygon(pts, closed=True, fc='white', ec=EDGE, lw=0.6))
    for z in (0,):
        ax.plot([-27, 27], [z, z], ls='-.', color='#888', lw=0.6); ax.plot([z, z], [-27, 27], ls='-.', color='#888', lw=0.6)
    dim(ax, (-DISC_BORE / 2, 1.5), (DISC_BORE / 2, 1.5), f'bore {DISC_BORE:g}', off=(0, 1.3))
    mid = math.pi * (SLOT_IN_D + SLOT_OUT_D) / 2 / SLOTS / 2
    note(ax, 0, 27, f'{SLOTS} slots, {pitch:g} deg pitch, slot = bar = {pitch / 2:.2f} deg ({mid:.2f} at mid-radius)')
    for y, d, t in ((-27, DISC_D, f'D {DISC_D:g}'), (-32, SLOT_OUT_D, f'slots end D {SLOT_OUT_D:g}'),
                    (-37, SLOT_IN_D, f'slots start D {SLOT_IN_D:g}')):
        dim(ax, (-d / 2, y), (d / 2, y), t, off=(0, 1.3))


def holder_side(ax):
    setup(ax, '3  Encoder Holder - side view along the wall (print 1 Left + 1 Right)', (-22, 40), (-30, 62))
    rect(ax, -WALL_T, 0, -24, PLATE_Z[1] + 4, fc='#e8e8e8', ec='#888')
    rect(ax, X0, X_BODY_END, -BODY_D / 2, BODY_D / 2, fc='#f2c9a0', ec='#999', alpha=0.6)
    rect(ax, X_BODY_END, X_BODY_END + 8, -24, PLATE_Z[1] + 4, fc='#d8d8d8', ec='#aaa', alpha=0.5, hatch='..')
    note(ax, X_BODY_END + 4, -14, 'wheel', color='#777', rotation=90)
    rect(ax, DISC_X0, DISC_X0 + DISC_T, -DISC_D / 2, DISC_D / 2, fc='#9a9a9a', ec='none')
    h = dict(fc=PART, ec=EDGE, lw=1.2)
    rect(ax, 0, PLATE_T, PLATE_Z[0], PLATE_Z[1], **h)
    rect(ax, 0, SHELF_X1, SHELF_Z0, SHELF_Z1, **h)
    ax.add_patch(Polygon([(PLATE_T, SHELF_Z1), (SHELF_X1, SHELF_Z1), (PLATE_T, PLATE_Z[1])], closed=True, **h))
    rect(ax, PCB_X[0] - 2.3, PCB_X[0] - 0.3, SHELF_Z0 - 1.2, SHELF_Z0, **h)
    rect(ax, PCB_X[0], PCB_X[1], FACE_Z, SHELF_Z0, fc='#2e8b57', ec='#1d5c39')
    for s in (-1, 1):
        x0 = DISC_X + s * 6.1 + (0 if s > 0 else -3.2)
        rect(ax, x0, x0 + 3.2, SHELF_Z0 - LM_FORK_TOP, FACE_Z, fc='#333', ec='#111')
    ax.plot([DISC_X - 4, DISC_X + 4], [BEAM_Z, BEAM_Z], color='red', lw=1.4)
    ax.text(DISC_X + 4.5, BEAM_Z, 'IR beam', color='red', fontsize=8, va='center')
    ax.plot([-6, X_BODY_END], [0, 0], ls='-.', color='#888', lw=0.6)
    dim(ax, (0, PLATE_Z[1] + 7), (SHELF_X1, PLATE_Z[1] + 7), f'{SHELF_X1:g}', off=(0, 1.8))
    dim(ax, (0, PLATE_Z[1] + 2.5), (PLATE_T, PLATE_Z[1] + 2.5), f'{PLATE_T:g}', off=(0, 1.6))
    for x, z, t in ((-17, PLATE_Z[1], f'{PLATE_Z[1]:g}'), (-13, SHELF_Z0, f'{SHELF_Z0:g}'), (-9, PLATE_Z[0], f'{PLATE_Z[0]:g}')):
        dim(ax, (x, 0), (x, z), t, off=(-1.6, 0), rot=90)
    dim(ax, (SHELF_X1 + 1.5, SHELF_Z0), (SHELF_X1 + 1.5, SHELF_Z1), f'{SHELF_T:g}', off=(1.6, 0), rot=90)
    dim(ax, (DISC_X + 9.5, 0), (DISC_X + 9.5, BEAM_Z), f'beam {BEAM_Z:g}', off=(1.6, 0), rot=90)
    note(ax, 9, PLATE_Z[1] - 6, f'2 x M3 slots\n+/-{ADJ:g} up/down', ha='left')
    dim(ax, (0, -26), (DISC_X, -26), f'disc centre {DISC_X:g}', off=(0, 1.8))
    dim(ax, (SHELF_X1, SHELF_Z1 + 1.5), (X_BODY_END, SHELF_Z1 + 1.5), f'{X_BODY_END - SHELF_X1:g} clear', off=(0, 1.8), size=7.5)


def holder_below(ax):
    y0, y1 = PLATE_Y
    setup(ax, '4  Encoder Holder (Left) - from below', (-6, 40), (y0 - 8, y1 + 8))
    rect(ax, 0, SHELF_X1, y0, y1, fc=PART, ec=EDGE, lw=1.2)
    rect(ax, 0, PLATE_T, y0, y1, fc='#9fb3c8', ec=EDGE, lw=1.2)
    for y in (y0 + 7, y1 - 7):
        ax.add_patch(Circle((PLATE_T / 2, y), 1.0, fc='white', ec=EDGE))
    rect(ax, PCB_X[0], PCB_X[1], PCB_Y[0], PCB_Y[1], fc='none', ec='#2e8b57', ls='--', lw=1)
    note(ax, DISC_X, PCB_Y[0] + 1.6, 'fork', color='#2e8b57')
    for mx, my in LM_HOLES:
        x, y = to_ext(mx, my)
        ax.add_patch(Circle((x, y), 1.7, fc='white', ec=EDGE))
        note(ax, x, y + 3, f'M3 ({x:.1f}, {y:.2f})', fontsize=7)
    ax.plot([DISC_X, DISC_X], [y0 - 3, y1 + 3], ls='-.', color='#555', lw=0.7)
    note(ax, DISC_X, y1 + 4.5, 'disc plane')
    dim(ax, (PCB_X[0], y1 - 3), (PCB_X[1], y1 - 3), f'board {LM_BOARD[1]:g}', off=(0, 1.3))
    dim(ax, (0, y0 - 3), (SHELF_X1, y0 - 3), f'{SHELF_X1:g}', off=(0, -1.6))
    dim(ax, (SHELF_X1 + 3, y0), (SHELF_X1 + 3, y1), f'{y1 - y0:g}  (Y {y0:g} .. {y1:g}, shaft at 0)', off=(1.6, 0), rot=90)


def notes(fig):
    text = f"""MEASURED (from you)
  motor shaft D {SHAFT_D:g}, {SHAFT_OUT:g} out of the wall
  wheel hub hole about 7 -> pin {OUT_D:g}
  wheel hub depth {HUB_DEPTH:g}

PRINT CLEARANCES
  bore {BORE:g} on a {SHAFT_D:g} shaft (+{BORE - SHAFT_D:.1f})
  pin {OUT_D:g} for a 7 hole (-{7 - OUT_D:.1f})
  (opened up after the first print was too tight)
  disc bore {DISC_BORE:g} on the {BODY_D:g} body
  set screw pilots {SET_PILOT:g} (M1.7)

RESULT
  extender {X_END - X0:g} long, starts {X0:g} off the wall
  set screws at {SET_SCREWS[0][0]:g} and {SET_SCREWS[1][0]:g}: on the shaft,
    between the wall and the disc
  wheel moves out {SHIFT:g} -> track 240 + 2 x {SHIFT:g} = {240 + 2 * SHIFT:g}
  {SLOTS} slots = {SLOTS} ticks per wheel turn

SENSOR
  your LM393 module (LM393.stp):
  board {LM_BOARD[0]:g} x {LM_BOARD[1]:g} x {LM_BOARD[2]:g}
  hangs upside down, fork over the disc
  beam assumed {LM_BEAM_H:g} above the board:
  slide the bracket up/down to suit
  solder wires to the header (a Dupont
  plug would hit the disc)"""
    fig.text(0.76, 0.43, text, family='monospace', fontsize=9.5, va='top')


def main(out):
    fig = plt.figure(figsize=(20, 14.2), dpi=124)
    fig.text(0.02, 0.975, 'Shaft extender + encoder  (all mm)   -   encoder_extender.py', fontsize=15, va='top')
    extender_section(fig.add_axes([0.03, 0.47, 0.6, 0.46]))
    disc_face(fig.add_axes([0.66, 0.5, 0.32, 0.45]))
    holder_side(fig.add_axes([0.03, 0.02, 0.36, 0.43]))
    holder_below(fig.add_axes([0.42, 0.03, 0.3, 0.42]))
    notes(fig)
    fig.savefig(out, facecolor='white')
    print(f'wrote {out}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else OUT)
