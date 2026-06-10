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
    "crown_wheel":     (F.crown_wheel, True),
    "setting_shaft":   (F.setting_shaft, True),
    "stem":            (F.stem, False),
    "crown":           (F.crown, False),
    "crown_tube":      (F.crown_tube, False),
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
    "seconds_pinion":  (F.seconds_pinion, True),
    "seconds_idler_a": (F.seconds_idler_a, True),
    "seconds_idler_b": (F.seconds_idler_b, False),
    "seconds_wheel":   (F.seconds_wheel, False),
    "seconds_hand":    (F.seconds_hand, False),
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
    bal_sweep = (P.BAL_RIM_OD + P.BAL_RIM_ID) / 4 + P.BAL_BOSS_R  # 16.3
    chk("balance sweep inside case",
        P.CASE_ID / 2 - (_d(P.P_BALANCE, (0, 0)) + bal_sweep), 0.5)
    u = P.P_BALANCE / np.linalg.norm(P.P_BALANCE)
    chk("cock legs clear balance sweep", P.COCK_LEG_D - 3.0 - bal_sweep, 0.3)
    chk("cock legs inside plate",
        P.PLATE_R - (_d(P.P_BALANCE - P.COCK_LEG_D * u, (0, 0)) + 7.5), 0.3)
    chk("cock legs clear hairspring",
        P.COCK_LEG_D - 3.0 - (P.HS_R0 + P.HS_PITCH * P.HS_TURNS + 2.4), 0.3)
    chk("crown-whl bars vs balance sweep",
        _d(P.P_BALANCE, P.KW_CROWN_WHEEL) - 5.4 - bal_sweep, 0.4)
    pivot, ang, abut = F.click_geo()
    chk("click pins vs balance sweep",
        _d(pivot, P.P_BALANCE) - bal_sweep - 3.0, 0.5)
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

    print("keyless works:")
    S, CW, B = P.KW_SETTING, P.KW_CROWN_WHEEL, P.P_BARREL
    bar_out = P.KW_BAR_R + P.KW_BAR_D / 2
    cw_tip = 0.7 * (P.KW_CW_SPUR_T / 2 + 1.3)
    chk("setting shaft vs drum (plan)", _d(S, B) - P.DRUM_OR - 1.7, 0.3)
    chk("setting bars vs CW flange", _d(CW, S) - bar_out - P.KW_FLANGE_R, 0.15)
    err = abs(_d(CW, B) - 0.35 * (P.KW_CW_SPUR_T + P.RATCHET_T))
    chk("crown spur/ratchet mesh (err)", 0.2 - err, 0.0)
    err2 = abs(_d(S, P.P_MINUTE) - 0.35 * (P.KW_SET_PIN_T + P.MINUTE_W_T))
    chk("setting pinion/minute wheel mesh (err)", 0.25 - err2, 0.0)
    chk("setting pinion vs hour wheel",
        _d(S, (0, 0)) - 0.7 * (P.HOUR_W_T / 2 + 1)
        - 0.7 * (P.KW_SET_PIN_T / 2 + 1.3), 0.3)
    for a in P.DIAL_FEET_ANGLES:
        c = P.DIAL_FEET_R * np.array([np.cos(np.radians(a)),
                                      np.sin(np.radians(a))])
        chk(f"dial foot@{a:.0f} vs setting pinion",
            _d(c, S) - 0.7 * (P.KW_SET_PIN_T / 2 + 1.3) - 2.5, 0.3)
    # stem fin sweep (cylinder r=OD/2 about y-axis at z=KW_STEM_Z)
    fr = P.KW_PINION_OD / 2
    chk("fins clear pocket floor", P.POCKET_FLOOR - (P.KW_STEM_Z + fr), 0.2)
    chk("fins clear CW flange", (P.KW_STEM_Z - fr) - P.Z_CW_FLANGE[1], 0.1)
    chk("fins clear setting disc", (P.KW_STEM_Z - fr) - P.Z_SET_DISC[1], 0.1)
    chk("ratchet vs cover", P.Z_RATCHET[0] - (-9.4), 0.5)

    print("small seconds:")
    W, IA, IB = P.P_SECONDS, P.P_SEC_IA, P.P_SEC_IB
    sec_tip = 0.7 * (P.SEC_W_T / 2 + 1)
    chk("seconds wheel inside plate", P.PLATE_R - (_d(W, (0, 0)) + sec_tip), 0.3)
    chk("seconds wheel vs balance arbor", _d(W, P.P_BALANCE) - sec_tip - 1.5, 0.4)
    chk("seconds wheel vs pillar@-18",
        _d(W, P.PILLAR_R * np.array([np.cos(np.radians(-18)),
                                     np.sin(np.radians(-18))]))
        - sec_tip - P.PILLAR_RAD, 0.3)
    chk("idler B vs escape arbor pin",
        _d(IB, P.P_ESCAPE) - 0.7 * (P.SEC_IB_T / 2 + 1) - 1.0, 0.4)
    chk("idler A wheel vs escape pin",
        _d(IA, P.P_ESCAPE) - 0.7 * (P.SEC_IA_W_T / 2 + 1) - 1.0, 0.4)
    chk("idler A vs roller (plan)", _d(IA, P.P_BALANCE) - 5.95 - P.ROLLER_MAIN_R, 0.4)
    chk("hour hand clears seconds sweep",
        (_d(W, (0, 0)) - 5.0) - 21.5, 0.5)
    for nm, az, bz in (("esc pin2/idler A wheel", P.Z_SEC_PIN2, P.Z_SEC_IA_W),
                       ("idler A pinion/idler B", P.Z_SEC_IA_P, P.Z_SEC_IB),
                       ("idler B/seconds wheel", P.Z_SEC_IB, P.Z_SEC_W)):
        o = min(az[1], bz[1]) - max(az[0], bz[0])
        chk(f"z-mesh {nm}", o, 1.0)
    # parked pinion strip (x +/-2.5, y +/-w/2) vs the OTHER wheel's bar
    # annulus (outer 5.4): corner distance must exceed bar reach
    halfw = P.KW_PINION_W / 2 + 0.2
    c1 = np.array([-fr, P.KW_PUSH_Y + halfw])      # pushed, vs setting
    chk("pushed fins vs setting bars", _d(S, c1) - bar_out, 0.2)
    c2 = np.array([fr, P.KW_PULL_Y - halfw])       # pulled, vs crown whl
    chk("pulled fins vs crown bars", _d(CW, c2) - bar_out, 0.2)
    chk("setting bars vs case wall",
        P.CASE_ID / 2 - (_d(S, (0, 0)) + bar_out), 0.15)
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
