"""2D outline generators + extrusion helpers.

Every part is built as stacked extrusions of shapely polygons, then
concatenated.  Overlapping sub-solids are fine: slicers union them.
"""
import numpy as np
import shapely.geometry as sg
import shapely.ops as so
import trimesh


# --------------------------------------------------------------- helpers
def rot(pts, ang):
    """Rotate Nx2 points by ang (radians)."""
    c, s = np.cos(ang), np.sin(ang)
    m = np.array([[c, -s], [s, c]])
    return np.asarray(pts) @ m.T


def circle(r, c=(0.0, 0.0), n=96):
    return sg.Point(c).buffer(r, quad_segs=max(8, n // 4))


def ring(r_out, r_in, c=(0.0, 0.0), n=96):
    return circle(r_out, c, n).difference(circle(r_in, c, n))


def poly(pts):
    p = sg.Polygon(pts)
    if not p.is_valid:
        p = p.buffer(0)
    return p


def union(*polys):
    return so.unary_union(list(polys))


def extrude(poly2d, h, z0=0.0):
    """Extrude a shapely (Multi)Polygon to a trimesh between z0 and z0+h."""
    meshes = []
    geoms = poly2d.geoms if hasattr(poly2d, "geoms") else [poly2d]
    for g in geoms:
        if g.is_empty or g.area < 1e-6:
            continue
        m = trimesh.creation.extrude_polygon(g, height=h)
        m.apply_translation([0, 0, z0])
        meshes.append(m)
    return trimesh.util.concatenate(meshes)


def zspan(poly2d, span, z0extra=0.0):
    """Extrude between span=(z_lo, z_hi)."""
    lo, hi = span
    return extrude(poly2d, hi - lo, lo + z0extra)


def stack(*meshes):
    return trimesh.util.concatenate([m for m in meshes if m is not None])


# ---------------------------------------------------------- involute gear
def _inv(a):
    return np.tan(a) - a


def gear_outline(m, z, shift=0.0, pa_deg=20.0, backlash=0.15,
                 ad=1.0, ded=1.25, flank_pts=10):
    """Involute spur gear outline (CCW) centred at origin."""
    pa = np.radians(pa_deg)
    rp = m * z / 2.0
    rb = rp * np.cos(pa)
    ra = rp + m * (ad + shift)
    rf = rp - m * (ded - shift)
    rf = max(rf, 0.30 * rp)
    # tooth thickness at pitch circle, minus backlash
    s = m * (np.pi / 2 + 2 * shift * np.tan(pa)) - backlash
    psi = s / (2 * rp)              # half tooth angle at pitch radius
    pitch_ang = 2 * np.pi / z

    # involute flank from max(rf, rb) to ra (right flank of a tooth
    # centred on angle 0; flank lies at negative angles)
    r_lo = max(rf, rb)
    rs = np.linspace(r_lo, ra, flank_pts)
    ang = []
    for r in rs:
        if r < rb:
            a_off = psi + _inv(pa)            # radial below base circle
        else:
            a_off = psi + _inv(pa) - _inv(np.arccos(np.clip(rb / r, 0, 1)))
        ang.append(-a_off)
    right = np.column_stack([rs * np.cos(ang), rs * np.sin(ang)])
    if rf < rb - 1e-9:
        # radial segment + root point
        a0 = ang[0]
        right = np.vstack([[rf * np.cos(a0), rf * np.sin(a0)], right])
    left = right[::-1].copy()
    left[:, 1] *= -1.0

    tip_a = -ang[-1]                 # half angular width of tip
    pts = []
    for i in range(z):
        base = i * pitch_ang
        pts.extend(rot(right, base))
        # tip arc
        for t in np.linspace(-tip_a, tip_a, 4)[1:-1]:
            pts.append([ra * np.cos(base + t), ra * np.sin(base + t)])
        pts.extend(rot(left, base))
        # root arc to next tooth
        a_root_start = base - right[0][1] / abs(right[0][1] + 1e-12) * 0
        a0 = base + np.arctan2(left[-1][1], left[-1][0]) - base
        a0 = base + abs(np.arctan2(right[0][1], right[0][0]))
        a1 = base + pitch_ang - abs(np.arctan2(right[0][1], right[0][0]))
        for t in np.linspace(a0, a1, 5)[1:-1]:
            pts.append([rf * np.cos(t), rf * np.sin(t)])
    return poly(pts)


def spoked_wheel(gear_poly, hub_r, rim_r, n_spokes=4, spoke_w=2.6,
                 bore=0.0):
    """Cut spoke windows into a gear blank; keep hub and rim."""
    cuts = []
    gap = 1.2
    r0, r1 = hub_r + gap, rim_r - gap
    if r1 - r0 > 3.0:
        for i in range(n_spokes):
            a0 = 2 * np.pi * i / n_spokes
            a1 = 2 * np.pi * (i + 1) / n_spokes
            half_w0 = (spoke_w / 2) / r0
            half_w1 = (spoke_w / 2) / r1
            arc_o = [(r1 * np.cos(t), r1 * np.sin(t))
                     for t in np.linspace(a0 + half_w1, a1 - half_w1, 12)]
            arc_i = [(r0 * np.cos(t), r0 * np.sin(t))
                     for t in np.linspace(a1 - half_w0, a0 + half_w0, 12)]
            cuts.append(poly(arc_o + arc_i))
    out = gear_poly.difference(union(*cuts)) if cuts else gear_poly
    if bore > 0:
        out = out.difference(circle(bore / 2))
    return out


# ------------------------------------------------------------ escape wheel
def escape_outline(z, r_tip, r_root, lean_deg=10.0, tip_w=0.45,
                   back_frac=0.55):
    """Clock-style escape wheel, teeth lean *forward* into CCW rotation.

    Locking face: from tip, leaning lean_deg from radial, down to root.
    Back of tooth: shallow slope to the root circle.
    """
    pitch = 2 * np.pi / z
    lean = np.radians(lean_deg)
    pts = []
    for i in range(z):
        a = i * pitch
        tip1 = np.array([r_tip * np.cos(a), r_tip * np.sin(a)])
        # tip flat (small, for printability) trailing edge
        a2 = a - tip_w / r_tip
        tip2 = np.array([r_tip * np.cos(a2), r_tip * np.sin(a2)])
        # locking face foot: rotate forward (CCW) by lean while descending
        foot_a = a + lean * (r_tip - r_root) / r_tip
        foot = np.array([r_root * np.cos(foot_a), r_root * np.sin(foot_a)])
        # back slope foot
        back_a = a - back_frac * pitch
        back = np.array([r_root * np.cos(back_a), r_root * np.sin(back_a)])
        pts.extend([back, tip2, tip1, foot])
    return poly(pts)


# ---------------------------------------------------------------- spirals
def spiral_ribbon(r0, pitch, turns, band, ccw=True, n_per_turn=90):
    """Archimedean spiral ribbon polygon (centerline r = r0 + pitch*th/2pi)."""
    th_end = 2 * np.pi * turns
    th = np.linspace(0.0, th_end, int(n_per_turn * turns) + 1)
    r = r0 + pitch * th / (2 * np.pi)
    sgn = 1.0 if ccw else -1.0
    xo = (r + band / 2) * np.cos(sgn * th)
    yo = (r + band / 2) * np.sin(sgn * th)
    xi = (r - band / 2) * np.cos(sgn * th)
    yi = (r - band / 2) * np.sin(sgn * th)
    outer = np.column_stack([xo, yo])
    inner = np.column_stack([xi, yi])[::-1]
    return poly(np.vstack([outer, inner]))


def sector(r_out, a0, a1, r_in=0.0, c=(0.0, 0.0), n=24):
    aa = np.linspace(a0, a1, n)
    pts = [(c[0] + r_out * np.cos(t), c[1] + r_out * np.sin(t)) for t in aa]
    if r_in > 0:
        pts += [(c[0] + r_in * np.cos(t), c[1] + r_in * np.sin(t))
                for t in aa[::-1]]
    else:
        pts.append(c)
    return poly(pts)


def rect(w, h, c=(0.0, 0.0), ang=0.0):
    pts = np.array([[-w / 2, -h / 2], [w / 2, -h / 2],
                    [w / 2, h / 2], [-w / 2, h / 2]])
    pts = rot(pts, ang) + np.asarray(c)
    return poly(pts)


def square_hole(af, c=(0.0, 0.0), ang=0.0):
    """Square hole polygon, af = across flats."""
    return rect(af, af, c, ang)


def rounded_bar(p0, p1, w):
    """Stadium shape from p0 to p1, width w."""
    return sg.LineString([tuple(p0), tuple(p1)]).buffer(w / 2, quad_segs=12)


def d_profile(r, flat_x):
    """Circle of radius r with a flat at x=flat_x (D-shaft/D-bore)."""
    return circle(r).intersection(
        poly([(-r * 1.2, -r * 1.2), (flat_x, -r * 1.2),
              (flat_x, r * 1.2), (-r * 1.2, r * 1.2)]))
