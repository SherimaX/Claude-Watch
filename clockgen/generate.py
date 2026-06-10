"""Clearance checks, STL export and the beauty render for the clock.

Usage: python -m clockgen.generate [render-only]
"""
import os
import sys
import numpy as np
import trimesh

from . import params as P
from . import parts
from .geometry3d import rotm


# --------------------------------------------------------------- checks
def _seg_point_dist(a, b, p=(0, 0, 0)):
    a, b, p = map(np.asarray, (a, b, p))
    d = b - a
    t = np.clip(-(a - p) @ d / (d @ d), 0, 1)
    return float(np.linalg.norm(a + t * d - p))


def run_checks(meshes):
    ok = True

    def chk(name, val, need):
        nonlocal ok
        good = val >= need
        ok &= good
        print(f"  [{'ok' if good else 'XX'}] {name}: {val:.2f} >= {need}")

    print("tourbillon clearances:")
    # everything riding in the inner carriage, measured from its centre
    M_bal = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0],
                      [0, 0, 0, 1.0]])  # local Z -> +Y (chain pose)
    r_max = 0.0
    for name, off in (("inner", None), ("balance", (0, P.BAL_Y, 0)),
                      ("dome", (0, P.DOME_Y0, 0)),
                      ("escape", (P.ESC_POS, 1.8, 0)),
                      ("lever", (P.LEVER_POS, 4.0, 0))):
        v = meshes[name].vertices
        if off is not None:
            v = v @ M_bal[:3, :3].T + off
        keep = v if name != "inner" else v[np.abs(v[:, 2]) < P.INNER_R + 1]
        r_max = max(r_max, float(np.linalg.norm(keep, axis=1).max()))
    chk("carriage assembly inside outer cage",
        (P.CAGE_R - P.CAGE_TUBE) - r_max, 2.0)
    chk("lay rod clears carriage sweep",
        _seg_point_dist(P.LAY_A, P.LAY_B) - P.LAY_ROD_R - r_max, 0.5)
    chk("balance rim inside carriage ring",
        (P.INNER_R - P.INNER_TUBE) - (P.BAL_R + 1.6), 1.0)
    chk("escape wheel inside carriage ring",
        (P.INNER_R - P.INNER_TUBE) - (P.ESC_POS + P.ESC_TIP_R), 1.0)
    chk("dome under the cock arc", 15.0 - 1.3 - P.DOME_Y1, 0.8)
    chk("escapement under balance rim (z gap)", 4.0 - 3.3, 0.5)
    chk("lever reaches the roller",
        (P.LEVER_POS - 3.0) - (3.2 - 1.0), 0.0)

    print("totem stack:")
    cage_lo = P.TOURB_C[2] - P.CAGE_R - P.CAGE_TUBE
    chk("cage clears the base sphere",
        cage_lo - (P.SPH_C + P.SPH_R - 8.0), 6.0)   # trumpet zone overlap
    a = np.radians(P.DIAL_TILT)
    dial_lo = P.DIAL_C + np.array([0, -np.sin(a), -np.cos(a)]) * P.DIAL_OR
    cage_top = P.TOURB_C[2] + P.CAGE_R + P.CAGE_TUBE
    chk("dial ring clears the finial", dial_lo[2] - (cage_top + 3.4 * 2),
        1.5)
    chk("hands inside the minute track",
        (P.DIAL_OR - 1.0) - P.HAND_SEC_L, 0.5)
    chk("minute hand reaches the markers",
        P.HAND_MIN_L - (P.DIAL_IR + 2.0), 0.0)
    # swan-neck columns + both shaft segments vs the tourbillon sphere
    from .parts import column_shaft_segs, column_curve
    sweep = P.CAGE_R + P.CAGE_TUBE
    for s, nm in ((-1, "left"), (1, "right")):
        pth = column_curve(s * P.COL_X)
        d = float(np.linalg.norm(pth - P.TOURB_C, axis=1).min())
        chk(f"{nm} column clears the cage sweep",
            d - P.COL_ROD_R - sweep, 1.0)
    for (a_s, b_s), nm in zip(column_shaft_segs(), ("lower", "upper")):
        d = _seg_point_dist(a_s, b_s, P.TOURB_C)
        chk(f"{nm} shaft segment clears the cage", d - 1.2 - sweep, 1.0)
    chk("barrel inside the shell",
        (P.SPH_R - P.SPH_T) - np.sqrt(29.5 ** 2 + 9.5 ** 2), 0.5)
    chk("drum gear meshes transfer pinion (err)",
        0.3 - abs((P.TRANSFER_C[2] - P.SPH_C) - 1.8 * (30 + 8) / 2), 0.0)
    chk("key clears the back bezel", 10.0 - 1.5 - 4.4, 0.3)
    return ok


# ----------------------------------------------------------------- STLs
# print pose: rotate so the natural flat face lies on the bed
PRINT_ROT = {
    "barrel": None, "key": ([1, 0, 0], 0), "transfer": None,
    "inner": ([1, 0, 0], 90), "cage": None, "lay": None,
    "balance": None, "dome": None, "escape": None, "lever": None,
    "dial": None, "markers": None, "canister": ([1, 0, 0], 180),
    "hand_h": None, "hand_m": None, "hand_s": None, "cap": None,
}


def export_stls(meshes):
    os.makedirs("stl_clock", exist_ok=True)
    total = 0
    for name, m in meshes.items():
        mm = m.copy()
        pr = PRINT_ROT.get(name)
        if pr:
            mm.apply_transform(rotm(pr[0], pr[1]))
        mm.apply_translation([0, 0, -mm.bounds[0][2]])
        path = f"stl_clock/{name}.stl"
        mm.export(path)
        total += 1
        print(f"  {path:34s} tris={len(mm.faces):6d} "
              f"size={np.round(mm.extents, 1)}")
    print(f"exported {total} STLs")


# --------------------------------------------------------------- render
def beauty_render(meshes, chain, path="docs/clock_render.png"):
    import matplotlib.pyplot as plt
    from . import render as R
    angles = {"cage": 18.0, "inner": 35.0, "lay": 60.0, "balance": 25.0,
              "sec": 40.0,
              "hand_h": -(10 + 9.5 / 60) * 30, "hand_m": -9.5 * 6,
              "hand_s": -180.0}
    fig = plt.figure(figsize=(13.2, 9.2))
    # left: full clock, front three-quarter
    ax = fig.add_subplot(121, projection="3d")
    ax.set_xlim(-72, 72); ax.set_ylim(-72, 72); ax.set_zlim(-4, 312)
    ax.set_box_aspect((1, 1, 316 / 144))
    ax.view_init(elev=12, azim=-72)
    ax.set_axis_off()
    ax.set_proj_type("persp", focal_length=0.45)
    R.draw(ax, R.pose_scene(meshes, chain, angles))
    ax.set_position([-0.27, -0.075, 0.95, 1.08])
    # right: tourbillon close-up
    ax2 = fig.add_subplot(122, projection="3d")
    c = P.TOURB_C
    w = 47
    ax2.set_xlim(c[0] - w, c[0] + w)
    ax2.set_ylim(c[1] - w, c[1] + w)
    ax2.set_zlim(c[2] - w, c[2] + w)
    ax2.set_box_aspect((1, 1, 1))
    ax2.view_init(elev=16, azim=-58)
    ax2.set_axis_off()
    R.draw(ax2, R.pose_scene(meshes, chain, angles, only=R.SPHERE_ONLY))
    ax2.set_position([0.36, 0.00, 0.66, 0.94])
    fig.text(0.22, 0.972, "spherical-tourbillon table clock",
             ha="center", fontsize=13)
    fig.text(0.70, 0.972, "two-axis flying tourbillon (60 s / 12 s)",
             ha="center", fontsize=13)
    fig.savefig(path, dpi=128, facecolor="white")
    plt.close(fig)
    print(f"wrote {path}")


def layout_png(path="docs/clock_layout.png"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.2, 11.0))

    def C(c, r, color, label=None, lw=1.2):
        th = np.linspace(0, 2 * np.pi, 120)
        ax.plot(c[0] + r * np.cos(th), c[1] + r * np.sin(th), color, lw=lw)
        if label:
            ax.annotate(label, c, fontsize=8, ha="center")

    ax.add_patch(plt.Rectangle((-P.PLINTH_R, 0), 2 * P.PLINTH_R,
                               P.PLINTH_H, fc="#d8dadd", ec="k", lw=0.8))
    C((0, P.SPH_C), P.SPH_R, "C0", "base sphere\n(mainspring barrel)")
    C((3, P.SPH_C), P.DRUM_R, "C1", "", lw=0.8)
    C((0, P.TOURB_C[2]), P.CAGE_R, "C8")
    ax.annotate("outer cage 60 s",
                (0, P.TOURB_C[2] + P.CAGE_R + 4), fontsize=8, ha="center")
    C((0, P.TOURB_C[2]), P.INNER_R, "C9", "")
    ax.annotate("inner carriage 12 s\n(balance inside)",
                (0, P.TOURB_C[2] - P.INNER_R - 11), fontsize=8, ha="center")
    C((0, P.TOURB_C[2]), P.BAL_R, "C3", "")
    a = np.radians(P.DIAL_TILT)
    yz = P.DIAL_C[1:]
    u = np.array([np.sin(a), np.cos(a)])
    lo, hi = yz - u * P.DIAL_OR, yz + u * P.DIAL_OR
    ax.plot([lo[0], hi[0]], [lo[1], hi[1]], "C2", lw=6, alpha=0.6)
    ax.annotate("dial ring (tilted 12 deg)", (hi[0] + 5, hi[1] - 4),
                fontsize=8)
    ax.plot([P.POD_C[1], P.RELAY_C[1], 14.1],
            [P.POD_C[2], P.RELAY_C[2], 234.6], "k--", lw=1)
    ax.plot(*P.RELAY_C[1:], "ko", ms=4)
    ax.annotate("seconds line\n(relay at the swan-neck)", (50, 178),
                fontsize=8)
    ax.annotate("stalk + sun crown", (14, 110), fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-80, 95)
    ax.set_ylim(-5, 315)
    ax.set_title("side elevation (mm)")
    plt.tight_layout()
    plt.savefig(path, dpi=110)
    plt.close(fig)
    print(f"wrote {path}")


def main():
    os.makedirs("docs", exist_ok=True)
    meshes, chain = parts.build()
    ntris = sum(len(m.faces) for m in meshes.values())
    print(f"built {len(meshes)} parts, {ntris} triangles total")
    ok = run_checks(meshes)
    layout_png()
    beauty_render(meshes, chain)
    if "render-only" in sys.argv:
        return
    if not ok:
        print("\n*** CLEARANCE CHECKS FAILED -- fix before trusting STLs ***")
    export_stls(meshes)


if __name__ == "__main__":
    main()
