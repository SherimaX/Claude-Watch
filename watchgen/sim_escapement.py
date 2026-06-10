"""Render escapement frames + advance function so geometry can be tuned.

Usage: python -m watchgen.sim_escapement
Outputs docs/escapement_sim.png and prints lock/drop numbers.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from . import params as P
from .escapement import (escape_outline, lever_outline, lever_world,
                         pins_world, wheel_world, max_wheel_advance)


def show(ax, p, color, lw=1.0):
    geoms = p.geoms if hasattr(p, "geoms") else [p]
    for g in geoms:
        if g.is_empty:
            continue
        ax.plot(*g.exterior.xy, color, lw=lw)
        for i in g.interiors:
            ax.plot(*i.xy, color, lw=lw)


def main():
    outline = escape_outline()
    body, pins = lever_outline(with_pin_holes=False)

    # advance function: lever angle -> how far wheel can turn
    phis = np.linspace(-P.LEVER_SWING, P.LEVER_SWING, 33)
    theta = 0.0
    fig, axs = plt.subplots(2, 3, figsize=(17, 11))

    # find a free starting wheel angle at the first lever position
    cp0 = pins_world(phis[0], pins)
    import numpy as _np
    for t in _np.arange(0, 18, 0.2):
        from .escapement import wheel_world as _ww
        if not _ww(t, outline).intersects(cp0):
            theta = t
            break
    print('start theta', theta)

    # sweep: simulate one full back-and-forth, tracking wheel angle
    states = []
    for phi in list(phis) + list(phis[::-1]):
        cp = pins_world(phi, pins)
        adv = max_wheel_advance(cp, theta, outline=outline)
        theta += adv
        states.append((phi, theta, adv))

    ax = axs[0][0]
    s = np.array(states)
    ax.plot(s[:len(phis), 0], s[:len(phis), 1], ".-", label="swing +")
    ax.plot(s[len(phis):, 0], s[len(phis):, 1], ".-", label="swing -")
    ax.set_xlabel("lever angle (deg)")
    ax.set_ylabel("wheel angle (deg)")
    ax.legend(); ax.grid(True)
    ax.set_title("wheel advance vs lever swing (pitch=18)")

    per_half = s[-1, 1] / 2.0
    print(f"total wheel advance over one full lever cycle: {s[-1,1]:.2f} deg"
          f" (target = pitch {360/P.ESC_T:.0f})")

    # key frames
    frames = [(-P.LEVER_SWING, "banked entry"),
              (-2.0, "mid"),
              (0.0, "line of centers"),
              (2.0, "mid+"),
              (P.LEVER_SWING, "banked exit")]
    theta = s[-1,1] % 360
    for k, (phi, label) in enumerate(frames):
        ax = axs.flat[k + 1]
        cp = pins_world(phi, pins)
        adv = max_wheel_advance(cp, theta, outline=outline)
        theta += adv
        show(ax, wheel_world(theta, outline), "C0")
        show(ax, lever_world(phi, body, pins), "C3")
        ax.plot(*P.P_ESCAPE, "k+"); ax.plot(*P.P_LEVER, "k+")
        ax.plot(*P.P_BALANCE, "kx")
        c = 0.55 * (P.P_ESCAPE + P.P_LEVER) - 0.05*P.P_BALANCE
        ax.set_xlim(c[0] - 11, c[0] + 11); ax.set_ylim(c[1] - 10, c[1] + 12)
        ax.set_aspect("equal")
        ax.set_title(f"phi={phi:+.0f}  wheel={theta:.2f}  ({label})")

    plt.tight_layout()
    plt.savefig("docs/escapement_sim.png", dpi=130)
    print("wrote docs/escapement_sim.png")


if __name__ == "__main__":
    main()
