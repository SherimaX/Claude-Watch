"""Shared renderer: poses the kinematic chain and rasterises it.

All triangles go into ONE Poly3DCollection with per-face lambert
shading, so matplotlib can depth-sort the whole scene at once --
essential for nested cages (separate collections never interleave).
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from . import params as P
from . import parts
from .geometry3d import rotm

_L1 = np.array([-0.45, -0.75, 0.55])
_L1 /= np.linalg.norm(_L1)
_L2 = np.array([0.7, 0.4, 0.25])
_L2 /= np.linalg.norm(_L2)


def _hex2rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4)]) / 255.0


def world_matrices(chain, angles):
    """Compose parent transforms + per-joint Z spins."""
    out = {}

    def W(name):
        if name in out:
            return out[name]
        parent, M0, tag = chain[name]
        Mp = W(parent) if parent else np.eye(4)
        spin = angles.get(tag, 0.0) if tag else 0.0
        out[name] = Mp @ M0 @ rotm([0, 0, 1], spin)
        return out[name]

    for n in chain:
        W(n)
    return out


def pose_scene(meshes, chain, angles, dome_fn=None, only=None):
    """Return [(verts_world, faces, rgb)] for every (selected) part."""
    mats = world_matrices(chain, angles)
    out = []
    for name, mesh in meshes.items():
        if only is not None and name not in only:
            continue
        m = mesh
        if name == "dome" and dome_fn is not None:
            m = dome_fn(angles.get("balance", 0.0))
        M = mats[name]
        v = m.vertices @ M[:3, :3].T + M[:3, 3]
        out.append((v, m.faces, _hex2rgb(parts.COLORS[name])))
    return out


def draw(ax, scene):
    """One shaded, depth-sortable collection for the whole scene."""
    tris, cols = [], []
    for v, f, rgb in scene:
        t = v[f]                       # (n, 3, 3)
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        nn = np.linalg.norm(n, axis=1)
        ok = nn > 1e-12
        t, n = t[ok], n[ok] / nn[ok, None]
        lam = (0.42 + 0.50 * np.clip(n @ _L1, 0, None)
               + 0.22 * np.clip(n @ _L2, 0, None)
               + 0.18 * np.clip(-n @ _L1, 0, None))   # faces are unordered
        c = np.clip(rgb[None, :] * lam[:, None], 0, 1)
        tris.append(t)
        cols.append(np.concatenate([c, np.ones((len(c), 1))], axis=1))
    pc = Poly3DCollection(np.concatenate(tris), facecolors=np.concatenate(cols),
                          edgecolor="none")
    ax.add_collection3d(pc)
    return pc


def setup_axes(fig, lim_xy=72, z0=-4, z1=312, elev=13, azim=-90):
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-lim_xy, lim_xy)
    ax.set_ylim(-lim_xy, lim_xy)
    ax.set_zlim(z0, z1)
    ax.set_box_aspect((1, 1, (z1 - z0) / (2 * lim_xy)))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_proj_type("persp", focal_length=0.45)
    return ax


def grab(fig):
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()


def autocrop(frames, pad=8):
    stackmin = np.min(np.stack([f.min(axis=2) for f in frames]), axis=0)
    ys, xs = np.where(stackmin < 250)
    y0, y1 = max(ys.min() - pad, 0), ys.max() + pad
    x0, x1 = max(xs.min() - pad, 0), xs.max() + pad
    return [f[y0:y1, x0:x1] for f in frames]


SPHERE_ONLY = {"cage", "cage_rubies", "lay", "inner", "inner_rubies",
               "balance", "dome", "escape", "lever", "stalk",
               "center_shaft"}
