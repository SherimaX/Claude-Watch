"""Build every part, verify layout clearances, export STLs + drawings.

Usage: python -m watchgen.generate
"""
import os
import numpy as np
import trimesh

from . import params as P
from . import parts_train as T
from . import parts_frame as F
from . import escapement as E


def _flip(m):
    """Rotate 180 deg about X (print the other face down)."""
    m.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi, [1, 0, 0]))
    return m


def _drop(m):
    m.apply_translation([0, 0, -m.bounds[0][2]])
    return m


# name -> (builder, flip_for_printing)
PARTS = {
    "back_plate":      (F.back_plate, False),
    "front_plate":     (F.front_plate, False),
    "balance_cock":    (F.balance_cock, False),
    "center_wheel":    (T.center_wheel, False),
    "center_arbor":    (T.center_arbor, False),
    "third_wheel":     (T.third_wheel, False),
    "fourth_wheel":    (T.fourth_wheel, True),
    "escape_wheel":    (T.escape_wheel, True),
    "barrel_drum":     (T.barrel_drum, False),
    "barrel_lid":      (T.barrel_lid, True),
    "barrel_arbor":    (T.barrel_arbor, False),
    "mainspring":      (T.mainspring, False),
    "mainspring_strong": (lambda: T.mainspring(band=1.6), False),
    "ratchet_wheel":   (T.ratchet_wheel, False),
    "click":           (T.click, False),
    "winding_key":     (T.winding_key, True),
    "lever":           (T.lever, True),
    "roller_main":     (T.roller_main, True),
    "roller_safety":   (T.roller_safety, False),
    "balance_wheel":   (T.balance_wheel, False),
    "balance_arbor":   (T.balance_arbor, False),
    "hairspring":      (T.hairspring, False),
    "hairspring_soft": (lambda: T.hairspring(band=0.45), False),
    "hairspring_stiff": (lambda: T.hairspring(band=0.6), False),
    "cannon_pinion":   (F.cannon_pinion, False),
    "minute_wheel":    (F.minute_wheel, False),
    "hour_wheel":      (F.hour_wheel, False),
    "dial":            (F.dial, True),
    "minute_hand":     (F.minute_hand, False),
    "hour_hand":       (F.hour_hand, False),
    "case_ring":       (F.case_ring, False),
    "case_back":       (F.case_back, False),
}


# --------------------------------------------------------------- checks
def _d(a, b):
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


def run_checks():
    ok = True

    def chk(name, val, need):
        nonlocal ok
        good = val >= need
        ok &= good
        print(f"  [{'ok' if good else 'XX'}] {name}: {val:.2f} >= {need}")

    print("plan-view clearances:")
    wt, pt_ = P.WHEEL_TIP_R, P.PINION_TIP_R
    # wheels inside the plate
    chk("third wheel inside plate", P.PLATE_R - (_d(P.P_THIRD, (0, 0)) + wt), 0.2)
    chk("fourth wheel inside plate", P.PLATE_R - (_d(P.P_FOURTH, (0, 0)) + wt), 0.2)
    chk("barrel gear inside plate", P.PLATE_R - (_d(P.P_BARREL, (0, 0)) + wt), 0.2)
    chk("escape wheel inside plate", P.PLATE_R - (_d(P.P_ESCAPE, (0, 0)) + P.ESC_R), 0.2)
    # same-z neighbours
    chk("4th-pinion hub vs 3rd wheel", _d(P.P_FOURTH, P.P_THIRD) - wt - 2.0, 0.3)
    chk("esc-pinion hub vs 4th wheel", _d(P.P_ESCAPE, P.P_FOURTH) - wt - 2.0, 0.3)
    chk("escape wheel vs center pinion", _d(P.P_ESCAPE, (0, 0)) - P.ESC_R - pt_, 0.5)
    chk("4th wheel vs center pinion", _d(P.P_FOURTH, (0, 0)) - wt - pt_, 0.3)
    chk("drum vs center pinion", _d(P.P_BARREL, (0, 0)) - P.DRUM_OR - pt_, 0.5)
    chk("drum vs 3rd pinion", _d(P.P_BARREL, P.P_THIRD) - P.DRUM_OR - pt_, 0.5)
    chk("drum vs balance arbor", _d(P.P_BARREL, P.P_BALANCE) - P.DRUM_OR - 2.2, 0.5)
    chk("lever pivot vs center wheel", _d(P.P_LEVER, (0, 0)) - wt - 1.0, 0.4)
    chk("balance rim inside case", P.CASE_ID / 2 - (_d(P.P_BALANCE, (0, 0)) + P.BAL_RIM_OD / 2), 0.5)
    chk("balance rim vs ratchet", _d(P.P_BALANCE, P.P_BARREL) - P.BAL_RIM_OD / 2 - P.RATCHET_R, 0.5)
    pivot, ang, abut = F.click_geo()
    chk("click pivot vs balance rim", _d(pivot, P.P_BALANCE) - P.BAL_RIM_OD / 2 - 3.0, 0.5)
    # pillars vs everything
    objs = [((0, 0), wt, "center wheel"), (P.P_BARREL, wt, "barrel gear"),
            (P.P_THIRD, wt, "third wheel"), (P.P_FOURTH, wt, "fourth wheel"),
            (P.P_ESCAPE, P.ESC_R, "escape wheel"),
            (P.P_BARREL, P.DRUM_OR, "drum")]
    for a in P.PILLAR_ANGLES:
        c = P.PILLAR_R * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        worst = min(_d(c, o) - r for o, r, _ in objs) - P.PILLAR_RAD
        chk(f"pillar@{a:.0f}", worst, 0.3)
    # banking posts vs roller/balance/escape
    for c in F.banking_posts_world():
        chk("banking post vs roller", _d(c, P.P_BALANCE) - P.ROLLER_MAIN_R - 1.5, 0.3)
        chk("banking post vs escape", _d(c, P.P_ESCAPE) - P.ESC_R - 1.5, 0.3)
    # dial feet vs minute wheel
    for a in P.DIAL_FEET_ANGLES:
        c = P.DIAL_FEET_R * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        chk(f"dial foot@{a:.0f} vs minute wheel",
            _d(c, P.P_MINUTE) - 0.7 * (P.MINUTE_W_T / 2 + 1) - 2.5, 0.3)

    print("z-stack mesh overlaps:")
    def overlap(name, a, b):
        nonlocal ok
        o = min(a[1], b[1]) - max(a[0], b[0])
        good = o >= 1.0
        ok &= good
        print(f"  [{'ok' if good else 'XX'}] {name}: {o:.2f} mm")
    overlap("barrel gear / center pinion", P.Z_BARREL_G, P.Z_CENTER_P)
    overlap("center wheel / third pinion", P.Z_CENTER_W, P.Z_THIRD_P)
    overlap("third wheel / fourth pinion", P.Z_THIRD_W, P.Z_FOURTH_P)
    overlap("fourth wheel / escape pinion", P.Z_FOURTH_W, P.Z_ESC_P)
    overlap("escape wheel / pallet pins", P.Z_ESC_W, P.PALLET_PIN_Z)
    overlap("cannon / minute wheel", P.Z_CANNON_G, P.Z_MINUTE_W)
    overlap("minute pinion / hour wheel", P.Z_MINUTE_P, P.Z_HOUR_W)
    return ok


def layout_png():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(15, 7.5))

    def C(ax, c, r, color, label=None):
        th = np.linspace(0, 2 * np.pi, 90)
        ax.plot(c[0] + r * np.cos(th), c[1] + r * np.sin(th), color, lw=1)
        if label:
            ax.annotate(label, c, fontsize=7, ha="center")

    ax = axs[0]
    ax.set_title("train side (between plates, front view)")
    C(ax, (0, 0), P.PLATE_R, "k")
    C(ax, (0, 0), P.WHEEL_TIP_R, "C0", "center")
    C(ax, P.P_BARREL, P.WHEEL_TIP_R, "C1", "barrel gear")
    C(ax, P.P_BARREL, P.DRUM_OR, "C1")
    C(ax, P.P_THIRD, P.WHEEL_TIP_R, "C2", "third")
    C(ax, P.P_FOURTH, P.WHEEL_TIP_R, "C3", "fourth")
    C(ax, P.P_ESCAPE, P.ESC_R, "C4", "escape")
    C(ax, P.P_LEVER, 1.5, "C5", "lever")
    C(ax, P.P_BALANCE, P.ROLLER_MAIN_R, "C6", "roller")
    for a in P.PILLAR_ANGLES:
        c = P.PILLAR_R * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        C(ax, c, P.PILLAR_RAD, "k")
    for c in F.banking_posts_world():
        C(ax, c, 1.5, "C5")
    from .escapement import lever_world
    lw = lever_world(0.0)
    for g in (lw.geoms if hasattr(lw, "geoms") else [lw]):
        ax.plot(*g.exterior.xy, "C5", lw=0.8)

    ax2 = axs[1]
    ax2.set_title("back side (balance, ratchet) + front (motion works)")
    C(ax2, (0, 0), P.PLATE_R, "k")
    C(ax2, P.P_BALANCE, P.BAL_RIM_OD / 2, "C6", "balance")
    C(ax2, P.P_BALANCE, P.HS_R0 + P.HS_PITCH * P.HS_TURNS, "C6")
    C(ax2, P.P_BARREL, P.RATCHET_R, "C1", "ratchet")
    pivot, ang, abut = F.click_geo()
    C(ax2, pivot, 2.2, "C1", "click")
    C(ax2, (0, 0), 0.7 * (P.CANNON_T / 2 + 1), "C7", "cannon")
    C(ax2, P.P_MINUTE, 0.7 * (P.MINUTE_W_T / 2 + 1), "C7", "minute wh")
    C(ax2, (0, 0), 0.7 * (P.HOUR_W_T / 2 + 1), "C8", "hour wh")
    for a in P.DIAL_FEET_ANGLES:
        c = P.DIAL_FEET_R * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        C(ax2, c, 2.5, "k")
    for ax_ in axs:
        ax_.set_aspect("equal")
        ax_.set_xlim(-44, 44); ax_.set_ylim(-44, 44)
    plt.tight_layout()
    plt.savefig("docs/layout.png", dpi=110)
    print("wrote docs/layout.png")


def main():
    os.makedirs("stl", exist_ok=True)
    ok = run_checks()
    layout_png()
    if not ok:
        print("\n*** CLEARANCE CHECKS FAILED — fix before trusting STLs ***")
    total = 0
    for name, (builder, flip) in PARTS.items():
        m = builder()
        if flip:
            _flip(m)
        _drop(m)
        path = f"stl/{name}.stl"
        m.export(path)
        total += 1
        print(f"  {path:32s} tris={len(m.faces):6d} "
              f"size={np.round(m.extents, 1)}")
    print(f"exported {total} STLs")


if __name__ == "__main__":
    main()
