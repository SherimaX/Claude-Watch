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
    sweep = P.CAGE_R + P.CAGE_TUBE          # the spinning sphere's radius
    chk("sphere clears the pedestal plate",
        (P.TOURB_C[2] - sweep) - P.PED_PLATE_Z[1], 6.0)
    z_loc = P.TRUMPET_Z1 - P.TOURB_C[2]     # trumpet rim, sphere-local
    r_ring = np.sqrt(P.CAGE_R ** 2 - z_loc ** 2) - P.CAGE_TUBE
    chk("trumpet clears the spinning sphere", r_ring - 9.3, 1.0)
    chk("centre shaft inside the trumpet bore", 7.2 - P.PIPE_R, 1.0)
    # gold hoops ride the cage outside the tumbling carriage
    chk("hoops clear the carriage sweep",
        (P.CAGE_R + P.HOOP_R_OFF) - P.HOOP_TUBE - r_max, 2.0)
    # the dial floats in front: canister back vs the spinning sphere
    n = P.dial_normal()
    q = P.DIAL_C + n * (-(P.DIAL_T / 2 + P.CAN_GAP + P.CAN_D))
    v = P.TOURB_C - q
    along = abs(float(v @ n))
    inpl = np.sqrt(max(float(v @ v) - along ** 2, 0.0))
    d_can = np.hypot(along, max(inpl - P.CAN_R, 0.0))
    chk("canister clears the spinning sphere", d_can - sweep, 1.0)
    chk("sphere fills the dial aperture", sweep - (P.DIAL_IR - 2.0), 0.0)
    chk("hands inside the minute track",
        (P.DIAL_OR - 1.0) - P.HAND_SEC_L, 0.5)
    chk("minute hand reaches the markers",
        P.HAND_MIN_L - (P.DIAL_IR + 2.0), 0.0)
    # swan-neck columns + seconds shaft vs the sphere and the dial back
    from .parts import column_shaft_seg, column_curve
    for s, nm in ((-1, "left"), (1, "right")):
        pth = column_curve(s * P.COL_X)
        d = float(np.linalg.norm(pth - P.TOURB_C, axis=1).min())
        chk(f"{nm} column clears the sphere", d - P.COL_ROD_R - sweep, 1.0)
        proj = float(((pth - P.DIAL_C) @ n).max())
        chk(f"{nm} column stays behind the dial",
            -proj - (P.COL_ROD_R + P.DIAL_T / 2), 0.5)
    a_s, b_s = column_shaft_seg()
    d = _seg_point_dist(a_s, b_s, P.TOURB_C)
    chk("seconds shaft clears the sphere", d - 1.2 - sweep, 1.0)
    chk("y rod clears the plate", (P.POD_C[2] - 2.6) - P.PED_PLATE_Z[1],
        0.3)
    # barrel inside the pedestal
    tip_r = P.DRUM_M * (P.DRUM_T / 2 + 1)
    chk("barrel inside the pedestal wall",
        (P.PED_WALL_R - P.PED_WALL_T) - tip_r, 0.5)
    chk("barrel under the plate",
        P.PED_PLATE_Z[0] - (P.BARREL_C[2] + tip_r), 0.5)
    chk("barrel over the foot",
        (P.BARREL_C[2] - tip_r) - (P.PLINTH_H + 4.5), 0.3)
    chk("drum gear meshes transfer pinion (err)",
        0.3 - abs((P.TRANSFER_C[2] - P.BARREL_C[2])
                  - P.DRUM_M * (P.DRUM_T + P.TRANS_T) / 2), 0.0)
    chk("key clears the back bezel", P.PORT_B_R - 1.5 - 4.4, 0.3)
    return ok


# ----------------------------------------------------------------- STLs
# print pose: rotate so the natural flat face lies on the bed
PRINT_ROT = {
    "barrel": None, "key": ([1, 0, 0], 0), "transfer": None,
    "inner": ([1, 0, 0], 90), "cage": None, "hoops": None, "lay": None,
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
    ax.set_xlim(-80, 80); ax.set_ylim(-80, 80); ax.set_zlim(-4, 196)
    ax.set_box_aspect((1, 1, 200 / 160))
    ax.view_init(elev=12, azim=-72)
    ax.set_axis_off()
    ax.set_proj_type("persp", focal_length=0.45)
    R.draw(ax, R.pose_scene(meshes, chain, angles))
    ax.set_position([-0.33, -0.13, 1.12, 1.30])
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
    fig, ax = plt.subplots(figsize=(7.6, 8.6))

    def C(c, r, color, label=None, lw=1.2):
        th = np.linspace(0, 2 * np.pi, 120)
        ax.plot(c[0] + r * np.cos(th), c[1] + r * np.sin(th), color, lw=lw)
        if label:
            ax.annotate(label, c, fontsize=8, ha="center")

    ax.add_patch(plt.Rectangle((-P.PLINTH_R, 0), 2 * P.PLINTH_R,
                               P.PLINTH_H, fc="#d8dadd", ec="k", lw=0.8))
    ax.add_patch(plt.Rectangle((-P.PED_WALL_R, P.PED_WALL_Z[0]),
                               2 * P.PED_WALL_R,
                               P.PED_PLATE_Z[1] - P.PED_WALL_Z[0],
                               fc="none", ec="k", lw=0.8))
    ax.annotate("pedestal", (P.PED_WALL_R + 3, 30), fontsize=8)
    C((0, P.BARREL_C[2]), P.DRUM_R, "C1",
      "mainspring\nbarrel", lw=0.8)
    C((0, P.TOURB_C[2]), P.CAGE_R, "C8")
    ax.annotate("the sphere = outer cage, 60 s\n(gold armillary hoops)",
                (0, P.TOURB_C[2] + P.CAGE_R + 8), fontsize=8, ha="center")
    C((0, P.TOURB_C[2]), P.INNER_R, "C9", "")
    ax.annotate("inner carriage 12 s\n(balance inside)",
                (12, P.TOURB_C[2] - P.INNER_R - 12), fontsize=8,
                ha="center")
    C((0, P.TOURB_C[2]), P.BAL_R, "C3", "")
    a = np.radians(P.DIAL_TILT)
    yz = P.DIAL_C[1:]
    u = np.array([np.sin(a), np.cos(a)])
    lo, hi = yz - u * P.DIAL_OR, yz + u * P.DIAL_OR
    ax.plot([lo[0], hi[0]], [lo[1], hi[1]], "C2", lw=6, alpha=0.6)
    ax.annotate("dial ring floats in front\n(tilted 12 deg, open centre"
                "\nframes the sphere)", (lo[0] - 2, lo[1] - 22),
                fontsize=8, ha="left")
    from .parts import column_shaft_seg
    a_s, b_s = column_shaft_seg()
    ax.plot([P.GBOX_C[1], a_s[1], b_s[1]],
            [P.POD_C[2], a_s[2] - 2, b_s[2]], "k--", lw=1)
    ax.plot(P.POD_C[1], P.POD_C[2], "ko", ms=4)
    ax.annotate("seconds line\n(front bevel shaft)", (-62, 72),
                fontsize=8)
    ax.annotate("stalk + sun crown", (22, P.STALK_Z0 - 2), fontsize=8)
    ax.annotate("viewer", (-86, 6), fontsize=8, color="0.4")
    ax.set_aspect("equal")
    ax.set_xlim(-95, 80)
    ax.set_ylim(-5, 200)
    ax.set_title("side elevation (mm) -- viewer at left")
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
