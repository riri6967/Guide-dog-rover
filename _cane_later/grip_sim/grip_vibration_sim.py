"""Vibration model of the haptic grip: can the hand tell the four motors apart,
and how much of the rover's own vibration reaches the hand?

    python grip_vibration_sim.py [out_dir]

A linear frequency-domain model (mass-spring-damper network + beam elements),
solved as  (K(1 + i*eta) + i*w*C - w^2 M) X = F  at each frequency.

Parts (all motion is in one direction, normal to the skin):
  - Rod: 1 m aluminium tube, 16 x 1 mm, 10 Euler-Bernoulli beam elements.
    Bottom end on the rover through the mount (rigid hinge or rubber bushing),
    free to pivot. Top end pinned to the grip.
  - Grip bar: rigid body (its first bending mode is ~5 kHz, far above 150 Hz),
    3 DOF: up/down, pitch (tilting along its length), roll (across it).
  - 4 coin motors, each a small mass on its mount (glued rigid / PLA tongue /
    soft TPU spokes / foam), at L, R (top, either end) and F, B (front, back).
  - Hand: one rigid mass on the arm (spring-damper to the body). Skin is a
    spring-damper between the hand and each motor, plus 8 contact patches
    between the hand and the rest of the bar.

What the person feels at a motor site = skin indentation = motor motion minus
hand motion.

Every number marked ASSUMED is an estimate: order-of-magnitude values from the
haptics / biomechanics literature or typical part data, not measurements.
The sensitivity run varies them to see whether the conclusions survive.

Not modelled (limitations): vibration spreading through the skin itself, the
rotating (2-axis) nature of the ERM force, the rod's angle, nonlinear foam,
grip force changes. Linear and one-axis: treat results as comparisons between
designs, not exact predictions. The eyes-shut test is still the real check.
"""
import sys
from pathlib import Path

import numpy as np

rng = np.random.default_rng(1)

# ------------------------------------------------------------------ geometry
# Grip (revised): 110 mm long, 34 x 28 mm oval. Motor centres (x along the
# bar, y forward, z up), metres. L/R 66 mm apart so they stay under an
# 80 mm-wide hand.
SITES = {'L': (-0.033, 0.0), 'R': (0.033, 0.0), 'F': (0.0, 0.017), 'B': (0.0, -0.017)}
NAMES = list(SITES)
GRIP_LEN = 0.110

# ------------------------------------------------------------------ parts
F_MOTOR = 0.55        # N, ASSUMED: 10 mm coin ERM ~1 G on a 100 g sled at 12000 rpm,
                      # scaled by (9000/12000)^2 for this module's 9000 rpm
F_HZ = 9000 / 60      # 150 Hz
M_MOTOR = 1.3e-3      # kg, ASSUMED: ~1.0 g motor + ~0.3 g of cup/tongue that moves with it

M_BAR = 0.040         # kg, ASSUMED: printed PLA shell ~33 g + switch, wires
I_PITCH = M_BAR * GRIP_LEN ** 2 / 12
I_ROLL = M_BAR * (0.017 ** 2 + 0.014 ** 2) / 2

ROD = dict(E=70e9, rho=2700.0, od=0.016, id=0.014, L=1.0, n=10, eta=0.02)

M_HAND = 0.45         # kg, ASSUMED effective hand mass
K_ARM, C_ARM = 3000.0, 30.0       # ASSUMED hand-arm support: ~13 Hz, well damped
K_SKIN, C_SKIN = 1000.0, 1.5      # per motor contact, ASSUMED (finger/palm pad impedance)
K_PAD, C_PAD = 600.0, 1.0         # per bar contact patch, ASSUMED
PADS = [(x, y) for x in (-0.045, -0.015, 0.015, 0.045) for y in (0.012, -0.012)]

# Motor mounts: stiffness k (N/m) and loss factor eta (material damping).
MOUNTS = {
    'Glued straight to the bar': dict(k=1e7, eta=0.02),
    'PLA tongue (1 mm)': dict(k=3900.0, eta=0.03),     # cantilever calc, PLA E = 3.5 GPa
    'TPU spokes': dict(k=180.0, eta=0.10),             # 3 spokes 6 x 2 x 0.6 mm, TPU E ~30 MPa
    'Foam pad': dict(k=150.0, eta=0.30),               # EVA / foam tape, ASSUMED
}

DETECT_UM = 0.2       # skin indentation (um) just detectable at ~150 Hz, ASSUMED
                      # (Pacinian threshold, roughly 0.1-0.3 um for a ~1 cm contact)

# Rover mount options for the rod's bottom end.
ROVER_MOUNTS = {
    'Rigid hinge': dict(k=1e8, eta=0.02),
    'Rubber bushing': dict(k=4.0e3, eta=0.25),   # ASSUMED soft rubber grommet
}


# ------------------------------------------------------------------ model build
class Model:
    def __init__(self, mount, rover_mount, p=None):
        p = dict(dict(k_skin=K_SKIN, c_skin=C_SKIN, k_pad=K_PAD, c_pad=C_PAD,
                      m_hand=M_HAND, k_arm=K_ARM, c_arm=C_ARM, m_motor=M_MOTOR), **(p or {}))
        n_rod = ROD['n'] + 1
        self.i_rod = lambda node, dof: 2 * node + dof           # dof 0 = w, 1 = slope
        o = 2 * n_rod
        self.i_bar = {'z': o, 'pitch': o + 1, 'roll': o + 2}
        self.i_mot = {nm: o + 3 + k for k, nm in enumerate(NAMES)}
        self.i_hand = o + 7
        n = o + 8
        K = np.zeros((n, n)); Kh = np.zeros((n, n)); C = np.zeros((n, n)); M = np.zeros((n, n))

        # Rod: Euler-Bernoulli elements with consistent mass. Hysteretic damping.
        E, rho, L = ROD['E'], ROD['rho'], ROD['L'] / ROD['n']
        I = np.pi / 64 * (ROD['od'] ** 4 - ROD['id'] ** 4)
        A = np.pi / 4 * (ROD['od'] ** 2 - ROD['id'] ** 2)
        ke = E * I / L ** 3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L],
                                        [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]])
        me = rho * A * L / 420 * np.array([[156, 22 * L, 54, -13 * L], [22 * L, 4 * L * L, 13 * L, -3 * L * L],
                                           [54, 13 * L, 156, -22 * L], [-13 * L, -3 * L * L, -22 * L, 4 * L * L]])
        for e in range(ROD['n']):
            idx = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
            K[np.ix_(idx, idx)] += ke; Kh[np.ix_(idx, idx)] += ROD['eta'] * ke
            M[np.ix_(idx, idx)] += me

        def spring(a, b, k, c=0.0, eta=0.0):
            """Spring between two linear combinations of DOFs: a, b = {dof: weight}."""
            v = np.zeros(n)
            for d, w in a.items(): v[d] += w
            for d, w in (b or {}).items(): v[d] -= w
            outer = np.outer(v, v)
            K[:] += k * outer; Kh[:] += k * eta * outer; C[:] += c * outer

        # Rover mount: bottom node to the (moving) ground. Hinge: slope free.
        rm = ROVER_MOUNTS[rover_mount]
        self.k_base, self.eta_base = rm['k'], rm['eta']
        spring({self.i_rod(0, 0): 1}, None, rm['k'], eta=rm['eta'])

        # Grip bar rigid body, pinned to the rod's top node at its centre.
        b = self.i_bar
        M[b['z'], b['z']] += M_BAR; M[b['pitch'], b['pitch']] += I_PITCH; M[b['roll'], b['roll']] += I_ROLL
        spring({self.i_rod(ROD['n'], 0): 1}, {b['z']: 1}, 1e7)

        def bar_point(x, y):
            return {b['z']: 1.0, b['pitch']: x, b['roll']: y}

        mt = MOUNTS[mount]
        h = self.i_hand
        M[h, h] += p['m_hand']
        spring({h: 1}, None, p['k_arm'], c=p['c_arm'])
        for nm, (x, y) in SITES.items():
            m = self.i_mot[nm]
            M[m, m] += p['m_motor']
            spring({m: 1}, bar_point(x, y), mt['k'], eta=mt['eta'])         # mount
            spring({m: 1}, {h: 1}, p['k_skin'], c=p['c_skin'])             # skin over motor
        for x, y in PADS:
            spring(bar_point(x, y), {h: 1}, p['k_pad'], c=p['c_pad'])      # rest of the hand

        self.K, self.Kh, self.C, self.M, self.n = K, Kh, C, M, n

    def solve(self, f, force=None, base=0.0):
        """Complex amplitudes at frequency f. force = {dof: N}; base = rover displacement (m)."""
        w = 2 * np.pi * f
        A = self.K + 1j * self.Kh + 1j * w * self.C - w * w * self.M
        F = np.zeros(self.n, complex)
        for d, v in (force or {}).items(): F[d] += v
        F[self.i_rod(0, 0)] += self.k_base * (1 + 1j * self.eta_base) * base
        return np.linalg.solve(A, F)

    def skin(self, X):
        """Skin indentation amplitude (m) at each motor site."""
        return {nm: abs(X[self.i_mot[nm]] - X[self.i_hand]) for nm in NAMES}


def db(r):
    return 20 * np.log10(r)


# ------------------------------------------------------------------ experiments
def crosstalk(mount, f=F_HZ, p=None, rover='Rubber bushing'):
    """For each active motor: indentation at all four sites (um)."""
    m = Model(mount, rover, p)
    out = {}
    for act in NAMES:
        X = m.solve(f, {m.i_mot[act]: F_MOTOR})
        out[act] = {k: v * 1e6 for k, v in m.skin(X).items()}
    return out


def worst_separation(ct):
    """Smallest gap (dB) between the active site and the loudest other site."""
    return min(db(ct[a][a] / max(v for k, v in ct[a].items() if k != a)) for a in NAMES)


def main(out_dir):
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True)
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    print(f'Motor: {F_HZ:.0f} Hz, force {F_MOTOR} N (assumed). Detection threshold ~{DETECT_UM} um.\n')

    # 1. Crosstalk at 150 Hz for each mount
    results = {}
    for mount in MOUNTS:
        ct = crosstalk(mount); results[mount] = ct
        print(f'== {mount}')
        print('   active ->  ' + '  '.join(f'{k:>8}' for k in NAMES) + '   (skin indentation, um)')
        for a in NAMES:
            print(f'   {a:>6}   ' + '  '.join(f'{ct[a][k]:8.2f}' for k in NAMES)
                  + f'    gap to next loudest {db(ct[a][a] / max(v for k, v in ct[a].items() if k != a)):5.1f} dB')
        print(f'   worst-case gap: {worst_separation(ct):.1f} dB\n')

    # 2. Sensitivity: vary every assumed hand / material number at random
    print('== Sensitivity (500 random variations of the assumed numbers, factor 0.5x-2x each)')
    sens = {}
    for mount in MOUNTS:
        gaps = []
        for _ in range(500):
            s = lambda: float(np.exp(rng.uniform(np.log(0.5), np.log(2.0))))
            p = dict(k_skin=K_SKIN * s(), c_skin=C_SKIN * s(), k_pad=K_PAD * s(), c_pad=C_PAD * s(),
                     m_hand=M_HAND * s(), k_arm=K_ARM * s(), c_arm=C_ARM * s(), m_motor=M_MOTOR * s())
            saved = dict(MOUNTS[mount]); MOUNTS[mount]['k'] *= s()
            gaps.append(worst_separation(crosstalk(mount, p=p)))
            MOUNTS[mount].update(saved)
        gaps = np.array(gaps); sens[mount] = gaps
        print(f'   {mount:28s} worst-case gap: median {np.median(gaps):5.1f} dB, '
              f'5th percentile {np.percentile(gaps, 5):5.1f} dB, min {gaps.min():5.1f} dB')

    # 3. Frequency sweep, L active
    f = np.linspace(20, 500, 481)
    fig, axs = plt.subplots(1, len(MOUNTS), figsize=(16, 4), sharey=True)
    for ax, mount in zip(axs, MOUNTS):
        m = Model(mount, 'Rubber bushing')
        resp = {k: [] for k in NAMES}
        for fi in f:
            sk = m.skin(m.solve(fi, {m.i_mot['L']: F_MOTOR}))
            for k in NAMES: resp[k].append(sk[k] * 1e6)
        for k in NAMES:
            ax.semilogy(f, resp[k], lw=2.2 if k == 'L' else 1.2, label=f'{k}' + (' (on)' if k == 'L' else ''))
        ax.axvline(F_HZ, color='k', ls=':', lw=1)
        ax.axhline(DETECT_UM, color='gray', ls='--', lw=1)
        ax.set_title(mount, fontsize=10); ax.set_xlabel('frequency (Hz)'); ax.grid(alpha=.3)
    axs[0].set_ylabel('skin indentation (um)'); axs[0].legend(fontsize=8)
    fig.suptitle('Left motor on: what each site feels (dotted = 150 Hz, dashed = detection threshold)')
    fig.tight_layout(); fig.savefig(out_dir / 'sweep_left_motor.png', dpi=110); plt.close(fig)

    # 4. Bar chart at 150 Hz, L and F active
    fig, axs = plt.subplots(1, 2, figsize=(12, 4))
    for ax, act in zip(axs, ('L', 'F')):
        x = np.arange(len(MOUNTS)); w = 0.2
        for j, k in enumerate(NAMES):
            ax.bar(x + (j - 1.5) * w, [results[mt][act][k] for mt in MOUNTS], w, label=k)
        ax.set_yscale('log'); ax.axhline(DETECT_UM, color='gray', ls='--', lw=1)
        ax.set_xticks(x); ax.set_xticklabels([mt.replace(' (1 mm)', '').replace(' straight', '\nstraight') for mt in MOUNTS], fontsize=8)
        ax.set_title(f'{act} motor on, 150 Hz'); ax.set_ylabel('skin indentation (um)'); ax.grid(alpha=.3, axis='y')
    axs[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out_dir / 'crosstalk_150Hz.png', dpi=110); plt.close(fig)

    # 5. Rover vibration reaching the skin
    print('\n== Rover vibration: skin indentation per 1 m/s^2 of vibration at the rod mount')
    f2 = np.linspace(2, 500, 997)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for rover in ROVER_MOUNTS:
        m = Model('TPU spokes', rover)
        tr = []
        for fi in f2:
            base = 1.0 / (2 * np.pi * fi) ** 2          # displacement for 1 m/s^2 acceleration
            tr.append(max(m.skin(m.solve(fi, base=base)).values()) * 1e6)
        tr = np.array(tr)
        ax.loglog(f2, tr, label=rover)
        for band in ((5, 30), (30, 100), (100, 200), (200, 500)):
            sel = (f2 >= band[0]) & (f2 <= band[1])
            print(f'   {rover:15s} {band[0]:>3}-{band[1]:<3} Hz: up to {tr[sel].max():8.2f} um per m/s^2')
    ax.axhline(DETECT_UM, color='gray', ls='--', lw=1, label='detection threshold (150 Hz)')
    ax.axvspan(100, 200, color='orange', alpha=.12, label='band that masks the haptics')
    ax.set_xlabel('rover vibration frequency (Hz)'); ax.set_ylabel('skin indentation (um) per 1 m/s^2')
    ax.set_title('How much rover vibration reaches the hand'); ax.grid(alpha=.3, which='both'); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out_dir / 'rover_transmission.png', dpi=110); plt.close(fig)

    # 6. Sensitivity histogram
    fig, ax = plt.subplots(figsize=(8, 4))
    for mount, g in sens.items():
        ax.hist(g, bins=40, alpha=.55, label=mount)
    ax.axvline(10, color='k', ls='--', lw=1); ax.text(10.5, ax.get_ylim()[1] * 0.9, '10 dB target', fontsize=8)
    ax.set_xlabel('worst-case gap between the active motor and the next loudest site (dB)')
    ax.set_ylabel('count (of 500)'); ax.legend(fontsize=8); ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(out_dir / 'sensitivity.png', dpi=110); plt.close(fig)
    print(f'\nPlots written to {out_dir}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'grip_sim_out')
