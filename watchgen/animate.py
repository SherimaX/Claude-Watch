"""Animated GIFs of the movement running.

  python -m watchgen.animate          # both animations
  python -m watchgen.animate esc      # 2D escapement close-up only
  python -m watchgen.animate 3d       # 3D movement only

Kinematics are scripted from the verified geometry: balance swings
sinusoidally; the lever follows the impulse pin (ratio -r_ip/fork_len)
and rests on its banking during overswing; each crossing of the release
angle steps the escape wheel half a tooth (9 deg) with a short ramp;
the train creeps at the real gear ratios.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import trimesh

from . import params as P
from . import parts_train as T
from . import parts_frame as F
from . import escapement as E
from .geometry import spiral_ribbon, circle, rect, zspan
import shapely.affinity as sa

FPS = 18
PERIODS = 5                  # loops cleanly: 5*18deg = 90deg = 12 fourth-
N = int(round(PERIODS / 1.2 * FPS))          # wheel teeth exactly
AMP = 40.0                   # balance amplitude (deg)
RELEASE = 2.5                # lever angle where the wheel is released
RAMP = 0.08                  # escape step ramp time (s)
HALF_PITCH = 360.0 / P.ESC_T / 2.0           # 9 deg per beat


def smooth(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def kinematics():
    """Return arrays t, psi (balance), phi (lever), th_esc (escape)."""
    t = np.arange(N) / FPS
    psi = AMP * np.sin(2 * np.pi * 1.2 * t)
    phi_free = -psi * P.ROLLER_R_IP / P.FORK_LEN
    phi = np.clip(phi_free, -P.LEVER_SWING, P.LEVER_SWING)
    # release events: phi crosses +/-RELEASE moving outward
    events = []
    for i in range(1, N):
        if phi[i - 1] < RELEASE <= phi[i]:
            events.append(t[i])
        if phi[i - 1] > -RELEASE >= phi[i]:
            events.append(t[i])
    th = np.zeros(N)
    for ev in events:
        th += HALF_PITCH * smooth((t - ev) / RAMP)
    return t, psi, phi, th


def hairspring_mesh(psi_deg):
    """Hairspring with the inner end rotated by psi (coils breathe)."""
    dpsi = np.radians(psi_deg)
    turns = P.HS_TURNS - dpsi / (2 * np.pi)
    pitch = P.HS_PITCH * P.HS_TURNS / turns
    sp = spiral_ribbon(P.HS_R0, pitch, turns, P.HS_BAND, ccw=True,
                       n_per_turn=70)
    sp = sa.rotate(sp, psi_deg, origin=(0, 0))
    hub = circle(P.HS_R0 + P.HS_BAND / 2)
    r_out = P.HS_R0 + P.HS_PITCH * P.HS_TURNS
    tab = rect(3.0, 2.4, c=(r_out + 1.2, 0))
    return zspan(sp.union(hub).union(tab), P.Z_HSPRING)


# ------------------------------------------------------------------- 3D
def _rotz(m, ang_deg, p=(0.0, 0.0)):
    m.apply_transform(trimesh.transformations.rotation_matrix(
        np.radians(ang_deg), [0, 0, 1], [p[0], p[1], 0]))
    return m


def _at(m, p):
    m.apply_translation([p[0], p[1], 0])
    return m


def animate_3d(path="docs/watch_animation.gif"):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    print("building meshes...")
    base = {
        "plate":  (F.back_plate(), "#90a0b0", 0.22, None),
        "cock":   (F.balance_cock(), "#557755", 0.45, None),
        "drum":   (_at(T.barrel_drum(), P.P_BARREL), "#b89030", 0.9, None),
        "ratchet": (_at(T.ratchet_wheel(), P.P_BARREL), "#b89030", 1.0, None),
        "center": (_at(T.center_wheel(), (0, 0)), "#cc6633", 1.0, None),
        "arbor":  (_at(T.center_arbor(), (0, 0)), "#cc6633", 1.0, None),
        "third":  (_at(T.third_wheel(), P.P_THIRD), "#33aa66", 1.0, None),
        "fourth": (_at(T.fourth_wheel(), P.P_FOURTH), "#3366cc", 1.0, None),
        "escape": (_at(T.escape_wheel(), P.P_ESCAPE), "#cc3366", 1.0, None),
        "lever":  (T.lever(), "#dd2222", 1.0, None),
        "roller": (_at(T.roller_main(), P.P_BALANCE), "#9933cc", 1.0, None),
        "safety": (_at(T.roller_safety(), P.P_BALANCE), "#9933cc", 1.0, None),
        "balarb": (_at(T.balance_arbor(), P.P_BALANCE), "#444444", 1.0, None),
        "balance": (_at(T.balance_wheel(), P.P_BALANCE), "#505860", 1.0, None),
    }
    t, psi, phi, th = kinematics()
    frames = []
    for k in range(N):
        moving = {
            "escape": _rotz(base["escape"][0].copy(), th[k], P.P_ESCAPE),
            "fourth": _rotz(base["fourth"][0].copy(), -th[k] / 6.0 + 3.75,
                            P.P_FOURTH),
            "lever":  _rotz(base["lever"][0].copy(), phi[k], P.P_LEVER),
            "roller": _rotz(base["roller"][0].copy(), psi[k], P.P_BALANCE),
            "safety": _rotz(base["safety"][0].copy(), psi[k], P.P_BALANCE),
            "balance": _rotz(base["balance"][0].copy(), psi[k], P.P_BALANCE),
            "hspr":   _at(hairspring_mesh(psi[k]), P.P_BALANCE),
        }
        fig = plt.figure(figsize=(7.2, 6.4))
        ax = fig.add_subplot(111, projection="3d")
        for name, (m, color, alpha, _) in base.items():
            mm = moving.get(name, m)
            v, f = mm.vertices, mm.faces
            ax.add_collection3d(Poly3DCollection(
                v[f], alpha=alpha, facecolor=color, edgecolor="none"))
        m = moving["hspr"]
        ax.add_collection3d(Poly3DCollection(
            m.vertices[m.faces], alpha=1.0, facecolor="#3344aa",
            edgecolor="none"))
        ax.set_xlim(-40, 40), ax.set_ylim(-40, 40), ax.set_zlim(-26, 42)
        ax.set_box_aspect((1, 1, 68 / 80))
        ax.view_init(elev=26, azim=-55)
        ax.set_axis_off()
        ax.set_position([-0.18, -0.22, 1.36, 1.44])
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
        frames.append(img)
        plt.close(fig)
        if k % 10 == 0:
            print(f"  3d frame {k+1}/{N}")
    # crop common white margins
    stackmin = np.min(np.stack([f.min(axis=2) for f in frames]), axis=0)
    ys, xs = np.where(stackmin < 250)
    y0, y1 = max(ys.min() - 8, 0), ys.max() + 8
    x0, x1 = max(xs.min() - 8, 0), xs.max() + 8
    frames = [f[y0:y1, x0:x1] for f in frames]
    _save_gif(frames, path)


# ------------------------------------------------------------------- 2D
def animate_escapement(path="docs/escapement_animation.gif"):
    outline = E.escape_outline()
    body, pins = E.lever_outline(with_pin_holes=False)
    guard = E.guard_outline()
    t, psi, phi, th = kinematics()
    A = P.P_BALANCE
    ang_AL = np.degrees(np.arctan2(*(P.P_LEVER - A)[::-1]))
    frames = []

    def fill(ax, p, color, alpha=1.0, lw=0.6):
        geoms = p.geoms if hasattr(p, "geoms") else [p]
        for g in geoms:
            if g.is_empty:
                continue
            xy = np.array(g.exterior.coords)
            ax.fill(xy[:, 0], xy[:, 1], color, alpha=alpha, lw=lw,
                    edgecolor="k", zorder=2)
            for i in g.interiors:
                xy = np.array(i.coords)
                ax.fill(xy[:, 0], xy[:, 1], "white", lw=0.4,
                        edgecolor="k", zorder=3)

    # base offset chosen so a tooth rests on a pin at phi = -8 (sim start)
    cp0 = E.pins_world(-P.LEVER_SWING, pins)
    th0 = next(x for x in np.arange(0, 18, 0.2)
               if not E.wheel_world(x, outline).intersects(cp0))
    th0 += E.max_wheel_advance(cp0, th0, outline=outline)

    for k in range(N):
        fig, ax = plt.subplots(figsize=(6.4, 6.4))
        # balance is behind the back plate: draw first, faint
        rim = circle(P.BAL_RIM_OD / 2, tuple(A)).difference(
            circle(P.BAL_RIM_ID / 2, tuple(A)))
        spk = sa.rotate(rect(P.BAL_RIM_OD - 2, 2.4, c=(0, 0)), psi[k],
                        origin=(0, 0))
        spk = spk.union(sa.rotate(spk, 90, origin=(0, 0)))
        fill(ax, rim.union(sa.translate(spk, *A)), "#dde1e6", alpha=0.55)
        fill(ax, E.wheel_world(th0 + th[k], outline), "#f2b6c6")
        fill(ax, E.to_world(guard, phi[k]), "#9fd49f")
        fill(ax, E.to_world(body, phi[k]), "#f0908a")
        fill(ax, E.pins_world(phi[k], pins), "#803030")
        # roller + impulse pin + safety disc + balance spokes
        a = np.radians(ang_AL + psi[k])
        pin_c = A + P.ROLLER_R_IP * np.array([np.cos(a), np.sin(a)])
        fill(ax, circle(P.ROLLER_MAIN_R, tuple(A)), "#d8c4ec", alpha=0.7)
        fill(ax, circle(P.PALLET_PIN_D / 2, tuple(pin_c)), "#5a3580")
        saf = sa.rotate(E.roller_safety_outline(), ang_AL + psi[k],
                        origin=(0, 0))
        fill(ax, sa.translate(saf, *A), "#b9a0d8", alpha=0.7)
        for c in F.banking_posts_world():
            fill(ax, circle(1.5, tuple(c)), "#777777")
        ax.plot(*P.P_ESCAPE, "k+", ms=4)
        ax.plot(*P.P_LEVER, "k+", ms=4)
        ax.set_xlim(P.P_ESCAPE[0] - 16, A[0] + 18)
        ax.set_ylim(P.P_ESCAPE[1] - 15, A[1] + 17)
        ax.set_aspect("equal")
        ax.set_axis_off()
        ax.set_title("pin-pallet escapement, real time (1.2 Hz)",
                     fontsize=10)
        fig.tight_layout(pad=0.2)
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
        frames.append(img)
        plt.close(fig)
        if k % 15 == 0:
            print(f"  2d frame {k+1}/{N}")
    _save_gif(frames, path)


def _save_gif(frames, path):
    from PIL import Image
    imgs = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE,
                                       colors=128) for f in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / FPS), loop=0, optimize=True)
    print(f"wrote {path} ({os.path.getsize(path)//1024} kB, "
          f"{len(frames)} frames)")




# ----------------------------------------------------------- full sequence
def animate_full(path="docs/watch_full_animation.gif"):
    """Running -> winding (crown) -> pull crown & set hands -> push back."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from . import parts_frame as PF
    print("building meshes...")
    Z_MIN_HAND, Z_HR_HAND = 26.6, 24.9

    def at(m, p):
        m.apply_translation([p[0], p[1], 0]); return m

    hands_min = at(F.minute_hand().apply_translation([0, 0, Z_MIN_HAND])
                   if False else F.minute_hand(), (0, 0))
    hands_min.apply_translation([0, 0, Z_MIN_HAND])
    hands_hr = F.hour_hand(); hands_hr.apply_translation([0, 0, Z_HR_HAND])

    static = [
        (F.back_plate(), "#90a0b0", 0.20),
        (F.front_plate(), "#90a0b0", 0.14),
        (F.case_ring(), "#aab4be", 0.10),
        (F.case_back(), "#aab4be", 0.12),
        (F.balance_cock(), "#557755", 0.40),
        (at(T.barrel_drum(), P.P_BARREL), "#b89030", 0.85),
        (F.dial(), "#f5f0e6", 0.30),
    ]
    moving_base = {
        "center": (at(T.center_wheel(), (0, 0)), "#cc6633", 1.0, (0, 0)),
        "arbor":  (at(T.center_arbor(), (0, 0)), "#cc6633", 1.0, (0, 0)),
        "third":  (at(T.third_wheel(), P.P_THIRD), "#33aa66", 1.0, P.P_THIRD),
        "fourth": (at(T.fourth_wheel(), P.P_FOURTH), "#3366cc", 1.0, P.P_FOURTH),
        "escape": (at(T.escape_wheel(), P.P_ESCAPE), "#cc3366", 1.0, P.P_ESCAPE),
        "lever":  (T.lever(), "#dd2222", 1.0, P.P_LEVER),
        "roller": (at(T.roller_main(), P.P_BALANCE), "#9933cc", 1.0, P.P_BALANCE),
        "balance": (at(T.balance_wheel(), P.P_BALANCE), "#505860", 1.0, P.P_BALANCE),
        "ratchet": (at(T.ratchet_wheel(), P.P_BARREL), "#b89030", 1.0, P.P_BARREL),
        "click":  (at(_rotz(T.click(), PF.click_geo()[1]),
                      PF.click_geo()[0]), "#888888", 1.0,
                   PF.click_geo()[0]),
        "crownw": (at(F.crown_wheel(), P.KW_CROWN_WHEEL), "#d4a017", 1.0,
                   P.KW_CROWN_WHEEL),
        "setshaft": (at(F.setting_shaft(), P.KW_SETTING), "#7a5cc4", 1.0,
                     P.KW_SETTING),
        "minutew": (at(F.minute_wheel(), P.P_MINUTE), "#5c8a8a", 1.0,
                    P.P_MINUTE),
        "hourw":  (at(F.hour_wheel(), (0, 0)), "#8a5c5c", 1.0, (0, 0)),
        "cannon": (at(F.cannon_pinion(), (0, 0)), "#8a5c5c", 1.0, (0, 0)),
        "minhand": (hands_min, "#101010", 1.0, (0, 0)),
        "hrhand": (hands_hr, "#28404f", 1.0, (0, 0)),
    }
    stem_base, crown_base = F.stem(), F.crown()

    # ---------------- timeline -----------------------------------------
    NA, NB, NC = 36, 36, 52
    NT = NA + NB + NC
    psi = np.zeros(NT); phi = np.zeros(NT); th_esc = np.zeros(NT)
    minute = np.zeros(NT); stem_rot = np.zeros(NT); pull = np.zeros(NT)
    cam = np.zeros((NT, 2)); label = [""] * NT
    # balance ticks throughout (visualised ~1 osc / 8 frames)
    tt = np.arange(NT)
    psi = 40.0 * np.sin(2 * np.pi * tt / 8.0)
    phi = np.clip(-psi * P.ROLLER_R_IP / P.FORK_LEN,
                  -P.LEVER_SWING, P.LEVER_SWING)
    beats = np.zeros(NT)
    for i in range(1, NT):
        beats[i] = beats[i - 1] + (
            1 if (phi[i - 1] < 2.5 <= phi[i]) or
                 (phi[i - 1] > -2.5 >= phi[i]) else 0)
    th_esc = beats * 9.0
    for k in range(NT):
        T0 = 3654.0     # 10:09 - minute-hand degrees since 12:00
        if k < NA:                              # A: running, time-lapse
            minute[k] = T0 + 360.0 * k / NA * 0.25   # 15 min sweep
            cam[k] = (52, -55)
            label[k] = "running (time-lapse)"
        elif k < NA + NB:                       # B: winding
            j = k - NA
            minute[k] = minute[NA - 1]
            stem_rot[k] = stem_rot[k - 1] + 40.0
            cam[k] = (52 - 90 * min(j / 8.0, 1), -55 - 20 * min(j / 8.0, 1))
            label[k] = "winding: turn crown (click holds the ratchet)"
        else:                                   # C: pull & set, push back
            j = k - NA - NB
            cam[k] = (-38 + 68 * min(j / 10.0, 1), -75)
            if j < 6:
                pull[k] = P.KW_TRAVEL * j / 5.0
                stem_rot[k] = stem_rot[k - 1]
                minute[k] = minute[k - 1]
                label[k] = "pull crown out ..."
            elif j < NC - 6:
                pull[k] = P.KW_TRAVEL
                stem_rot[k] = stem_rot[k - 1] + 55.0
                minute[k] = minute[k - 1] + 55.0 * 0.75
                label[k] = "setting: crown drives the hands"
            else:
                pull[k] = P.KW_TRAVEL * (NC - 1 - j) / 5.0
                stem_rot[k] = stem_rot[k - 1]
                minute[k] = minute[k - 1]
                label[k] = "push crown back in"

    frames = []
    for k in range(NT):
        wind = stem_rot[k] if k < NA + NB else stem_rot[NA + NB - 1]
        setr = stem_rot[k] - wind                  # setting-phase rotation
        spin = {
            "escape": th_esc[k], "fourth": -th_esc[k] / 6.0 + 3.75,
            "lever": phi[k], "roller": psi[k], "balance": psi[k],
            "ratchet": wind * (6.0 / 8.0) * (12.0 / 18.0),
            "crownw": -wind * 6.0 / 8.0,
            "setshaft": -setr * 6.0 / 8.0,
            "minutew": setr * 6.0 / 8.0 * 12.0 / 48.0 + 7.5,
            "cannon": -minute[k] + 180.0,
            "minhand": -minute[k] + 180.0,
            "hourw": -minute[k] / 12.0 + 180.0,
            "hrhand": -minute[k] / 12.0 + 180.0,
        }
        fig = plt.figure(figsize=(7.0, 6.6))
        ax = fig.add_subplot(111, projection="3d")
        for m, col, al in static:
            ax.add_collection3d(Poly3DCollection(
                m.vertices[m.faces], alpha=al, facecolor=col,
                edgecolor="none"))
        for name, (m, col, al, ctr) in moving_base.items():
            mm = m.copy()
            if name in spin:
                _rotz(mm, spin[name], ctr)
            ax.add_collection3d(Poly3DCollection(
                mm.vertices[mm.faces], alpha=al, facecolor=col,
                edgecolor="none"))
        hs = _at(hairspring_mesh(psi[k]), P.P_BALANCE)
        ax.add_collection3d(Poly3DCollection(
            hs.vertices[hs.faces], alpha=1.0, facecolor="#3344aa",
            edgecolor="none"))
        for part, col in ((F.stem_world(pull[k], stem_base), "#444444"),
                          (F.crown_world(pull[k], crown_base), "#222222")):
            part.apply_transform(trimesh.transformations.rotation_matrix(
                np.radians(stem_rot[k]), [0, 1, 0],
                [0, 0, P.KW_STEM_Z]))
            ax.add_collection3d(Poly3DCollection(
                part.vertices[part.faces], alpha=1.0, facecolor=col,
                edgecolor="none"))
        ax.set_xlim(-44, 44); ax.set_ylim(-40, 48); ax.set_zlim(-30, 44)
        ax.set_box_aspect((1, 1, 74 / 88))
        ax.view_init(elev=cam[k][0], azim=cam[k][1])
        ax.set_axis_off()
        ax.set_position([-0.18, -0.2, 1.36, 1.4])
        fig.text(0.5, 0.04, label[k], ha="center", fontsize=11)
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
        plt.close(fig)
        if k % 10 == 0:
            print(f"  full frame {k+1}/{NT}")
    stackmin = np.min(np.stack([f.min(axis=2) for f in frames]), axis=0)
    ys, xs = np.where(stackmin < 250)
    y0, y1 = max(ys.min() - 6, 0), ys.max() + 6
    x0, x1 = max(xs.min() - 6, 0), xs.max() + 6
    frames = [f[y0:y1, x0:x1] for f in frames]
    _save_gif(frames, path)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "both"
    if which in ("esc", "both"):
        animate_escapement()
    if which in ("3d", "both"):
        animate_3d()
    if which in ("full", "both"):
        animate_full()
