"""Animated GIFs of the table clock running.

  python -m clockgen.animate          # both animations
  python -m clockgen.animate full     # whole clock, swaying camera
  python -m clockgen.animate sphere   # tourbillon close-up, orbiting camera

One loop = exactly 60 s of clock time: the outer cage makes one
revolution, the inner carriage five, the second hand one; the minute
hand creeps 6 degrees (invisible at the loop seam).  The balance is
visualised at one oscillation per 10 frames, and every release steps
the escape wheel half a tooth -- the same stylisation the watch uses.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from . import params as P
from . import parts
from . import render as R

BAL_AMP = 150.0          # balance amplitude, deg
BAL_PER = 10             # frames per oscillation
LEVER_MAX = 9.0
RELEASE = 2.5


def timeline(N):
    """Per-frame angle dicts for one 60 s loop."""
    k = np.arange(N)
    prog = k / N
    psi = BAL_AMP * np.sin(2 * np.pi * k / BAL_PER)
    phi = np.clip(-psi * 0.74, -LEVER_MAX, LEVER_MAX)
    beats = np.zeros(N)
    for i in range(1, N):
        beats[i] = beats[i - 1] + (
            1 if (phi[i - 1] < RELEASE <= phi[i]) or
                 (phi[i - 1] > -RELEASE >= phi[i]) else 0)
    th_esc = beats * P.BEAT_DEG
    h0, m0, s0 = P.START_HMS
    sec = s0 + prog * 60.0
    out = []
    for i in range(N):
        out.append({
            "cage": 360.0 * prog[i],
            "inner": P.INNER_RATIO * 360.0 * prog[i],
            "lay": P.LAY_RATIO * 360.0 * prog[i],
            "sec": 360.0 * prog[i],
            "slow": -360.0 / 48.0 * prog[i],
            "drum": 360.0 / 48.0 * (8.0 / 30.0) * prog[i],
            "balance": psi[i],
            "lever": phi[i],
            "escape": th_esc[i],
            "hand_s": -6.0 * sec[i],
            "hand_m": -6.0 * (m0 + sec[i] / 60.0),
            "hand_h": -30.0 * (h0 + (m0 + sec[i] / 60.0) / 60.0),
        })
    return out


def _save_gif(frames, path, fps):
    from PIL import Image
    imgs = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE,
                                       colors=128) for f in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / fps), loop=0, optimize=True)
    print(f"wrote {path} ({os.path.getsize(path) // 1024} kB, "
          f"{len(frames)} frames)")


def animate_full(path="docs/clock_animation.gif", N=120, fps=16):
    meshes, chain = parts.build()
    tl = timeline(N)
    frames = []
    for i in range(N):
        prog = i / N
        fig = plt.figure(figsize=(6.6, 8.4))
        ax = R.setup_axes(fig, lim_xy=80, z0=-4, z1=196,
                          elev=12.0 + 3.0 * np.sin(4 * np.pi * prog),
                          azim=-90.0 + 26.0 * np.sin(2 * np.pi * prog))
        R.draw(ax, R.pose_scene(meshes, chain, tl[i],
                                dome_fn=parts.hairspring_dome))
        ax.set_position([-0.30, -0.10, 1.60, 1.22])
        fig.text(0.5, 0.022,
                 "spherical tourbillon: sphere 60 s / carriage 12 s "
                 "(8x speed)", ha="center", fontsize=9)
        frames.append(R.grab(fig))
        plt.close(fig)
        if i % 10 == 0:
            print(f"  full frame {i + 1}/{N}")
    frames = R.autocrop(frames)
    _save_gif(frames, path, fps)


def animate_sphere(path="docs/clock_tourbillon_animation.gif",
                   N=140, fps=14):
    meshes, chain = parts.build()
    tl = timeline(N)
    c = P.TOURB_C
    w = 46
    frames = []
    for i in range(N):
        prog = i / N
        fig = plt.figure(figsize=(6.6, 6.6))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlim(c[0] - w, c[0] + w)
        ax.set_ylim(c[1] - w, c[1] + w)
        ax.set_zlim(c[2] - w + 4, c[2] + w + 4)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=15.0, azim=-90.0 - 360.0 * prog)
        ax.set_axis_off()
        R.draw(ax, R.pose_scene(meshes, chain, tl[i],
                                dome_fn=parts.hairspring_dome,
                                only=R.SPHERE_ONLY))
        ax.set_position([-0.16, -0.16, 1.32, 1.32])
        fig.text(0.5, 0.03,
                 "two-axis flying tourbillon, orbit view (6x speed)",
                 ha="center", fontsize=10)
        frames.append(R.grab(fig))
        plt.close(fig)
        if i % 10 == 0:
            print(f"  sphere frame {i + 1}/{N}")
    frames = R.autocrop(frames)
    _save_gif(frames, path, fps)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "both"
    if which in ("full", "both"):
        animate_full()
    if which in ("sphere", "both"):
        animate_sphere()
