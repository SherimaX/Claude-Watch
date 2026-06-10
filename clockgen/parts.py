"""Every part of the table clock + the kinematic chain that places it.

Each part is built in its own LOCAL frame: +Z = its spin axis, origin =
its pivot (static parts are built in world coordinates directly).  The
CHAIN maps part -> (parent, rest pose in parent frame, spin tag); the
animation inserts a Z-rotation at every joint, so nested motion (outer
cage carrying the inner carriage carrying the balance) falls out of
plain matrix composition.
"""
import numpy as np
import shapely.geometry as sg
import shapely.affinity as sa

from watchgen.geometry import (gear_outline, escape_outline, spiral_ribbon,
                               circle, ring, rect, poly, union, rounded_bar)
from . import params as P
from .geometry3d import (lathe, tube, arc_path, ring_torus, sphere_panel,
                         sphere_arc, cyl, ball, box, crown_teeth,
                         extrude_flat, frame, rotm, transm, put, merge)


# =================================================================== base
def plinth():
    pr = [(0, 0), (62, 0), (62, 6.5), (59, 8.5), (47, 9.5), (41.5, 13),
          (32, 14.8), (24, 15.5), (0, 15.5)]
    return lathe(pr, n=56)


def cradle():
    m = ring_torus(19.0, 3.0, n_main=40, n_tube=8)
    m.apply_translation([0, 0, 17.0])
    return m


def base_sphere_steel():
    """Openwork armillary shell: solid cap + collar + meridian ribs."""
    parts = []
    sp = lambda *a, **k: put(sphere_panel(*a, **k), transm([0, 0, P.SPH_C]))
    parts.append(sp(P.SPH_R, P.SPH_T, -85.0, P.RIB_LAT[0], n_lat=8))
    parts.append(sp(P.SPH_R, P.SPH_T, P.RIB_LAT[1], 74.5, n_lat=6))
    for lon in P.RIB_LONS:
        m = sphere_arc(P.SPH_R - 0.4, P.RIB_LAT[0] + 1, P.RIB_LAT[1] - 1,
                       lon, P.RIB_R, n=26, n_tube=6)
        parts.append(put(m, transm([0, 0, P.SPH_C])))
    # partial equator band, open at the front (front = -Y = -90 deg lon)
    eq = tube(arc_path(P.SPH_R - 0.4, -55.0, 235.0, n=56), 2.0, n=6)
    parts.append(put(eq, transm([0, 0, P.SPH_C])))
    return merge(*parts)


def base_sphere_gold():
    """Latitude hoops + porthole bezels (the jewellery layer)."""
    parts = []
    for lat in P.HOOP_LATS:
        r = (P.SPH_R - 0.2) * np.cos(np.radians(lat))
        z = P.SPH_C + (P.SPH_R - 0.2) * np.sin(np.radians(lat))
        parts.append(put(ring_torus(r, 1.6, n_main=48, n_tube=6),
                         transm([0, 0, z])))
    # front porthole bezel (frames the mainspring view)
    yf = -np.sqrt(P.SPH_R ** 2 - 15.0 ** 2)
    parts.append(put(ring_torus(15.0, 1.5, n_main=40, n_tube=6),
                     frame((0, yf, P.SPH_C), (0, -1, 0))))
    # back porthole bezel (the winding key passes through)
    yb = np.sqrt(P.SPH_R ** 2 - 10.0 ** 2)
    parts.append(put(ring_torus(10.0, 1.5, n_main=36, n_tube=6),
                     frame((0, yb, P.SPH_C), (0, 1, 0))))
    return merge(*parts)


# ------------------------------------------------------- barrel and key
def barrel_drum():
    """Drum + ring gear + mainspring, local +Z = drum axis (-> +Y world).
    The open front ring leaves the spiral visible through the porthole."""
    parts = [
        lathe([(20.0, 0), (25.5, 0), (25.5, 2.0), (20.0, 2.0)], n=48),
        lathe([(25.0, 0), (27.0, 0), (27.0, 19.0), (25.0, 19.0)], n=48),
        lathe([(4.5, 17.2), (27.0, 17.2), (27.0, 19.0), (4.5, 19.0)], n=48),
    ]
    g = gear_outline(1.8, 30, 0.0, backlash=0.3, flank_pts=5)
    g = g.difference(circle(24.5))
    parts.append(extrude_flat(g, 2.5, 0.0))
    return merge(*parts)


def mainspring_clock():
    """Blued spiral, visible through the front porthole (rides with the
    drum; its slow unwinding is invisible at animation timescales)."""
    sp = spiral_ribbon(6.0, 2.7, 6.2, 1.5, ccw=True, n_per_turn=36)
    return extrude_flat(sp, 12.0, 3.0)


def winding_key():
    """Key + barrel arbor + ratchet, one static piece (the click holds it).
    Local +Z = key axis (-> +Y world, out the back porthole)."""
    parts = [
        cyl(2.2, 39.0, -37.0, n=12),            # arbor through the drum
        cyl(4.2, 4.0, -36.0, n=12),             # spring-hook core
        cyl(2.6, 44.0, 0.0, n=12),              # key shaft
        cyl(4.4, 3.0, 3.5, n=12),               # collar at the bezel
    ]
    g = gear_outline(1.0, 14, 0.0, backlash=0.25, flank_pts=4)
    parts.append(extrude_flat(g.difference(circle(2.0)), 2.2, 8.0))  # ratchet
    bow = ring_torus(6.5, 1.7, n_main=32, n_tube=8)
    bow.apply_transform(rotm([1, 0, 0], 90))
    bow.apply_translation([0, 0, 50.5])
    parts.append(bow)
    parts.append(cyl(1.4, 6.0, 43.0, n=8))      # bow stem
    return merge(*parts)


def click_pawl():
    """Tiny pawl resting on the ratchet (static, world coords)."""
    m = box((1.8, 6.5, 2.4))
    m.apply_transform(rotm([1, 0, 0], -38))
    m.apply_translation([0, P.KEY_Y0 + 11.5, P.SPH_C + 8.6])
    return m


# ------------------------------------------------ drive line in the base
def gearbox():
    """Collar gearbox drum: encloses the 48:1 reduction (static, world).
    The transfer arbor pierces its front wall; the centre shaft and the
    Y rod leave through the top and the back."""
    m = lathe([(0, -P.GBOX_H / 2), (P.GBOX_R, -P.GBOX_H / 2),
               (P.GBOX_R, P.GBOX_H / 2), (0, P.GBOX_H / 2)], n=32)
    m.apply_translation(P.GBOX_C)
    return m


def transfer_arbor():
    """Pinion meshing the drum ring gear, slow.  Local +Z -> +Y world."""
    g = gear_outline(1.8, 8, 0.4, backlash=0.3, flank_pts=4)
    m = extrude_flat(g, 2.2, 0.0)
    return merge(m, cyl(1.4, 8.5, -1.0, n=10))


def center_shaft():
    """1 rpm vertical shaft: gearbox -> tourbillon cage pipe."""
    m = cyl(1.6, 26.0, 0.0, n=12)
    m = merge(m, cyl(3.1, 1.6, 24.4, n=10))            # cage drive key
    return m


def y_rod():
    """1 rpm take-off to the rear pod.  Local +Z -> +Y world."""
    m = cyl(1.2, 21.0, 0.0, n=10)
    m = merge(m, crown_teeth(2.6, 10, 1.1, 1.6, 2.0, 0.5, tilt=40),
              cyl(2.6, 0.8, 0.0, n=16),
              crown_teeth(2.6, 10, 1.1, 1.6, 2.0, 19.0, tilt=-40),
              cyl(2.6, 0.8, 20.2, n=16))
    return m


def column_pod():
    """Bevel pods: collar shoulder + swan-neck relay knuckle (static)."""
    return merge(ball(4.8, P.POD_C, sub=2), ball(4.5, P.RELAY_C, sub=2))


def column_shaft_segs():
    """The seconds line bows around the tourbillon in two straight
    spinning segments meeting at the relay knuckle."""
    a = P.POD_C + np.array([0.0, 0.0, 2.0])
    b = np.array([0.0, 14.1, 234.6])      # canister inlet boss
    return (a, P.RELAY_C), (P.RELAY_C, b)


def _shaft(a, b):
    L = np.linalg.norm(np.asarray(b) - np.asarray(a))
    m = cyl(1.2, L, 0.0, n=10)
    m = merge(m, crown_teeth(2.4, 10, 1.0, 1.5, 1.8, 0.6, tilt=40),
              cyl(2.4, 0.7, 0.4, n=16),
              crown_teeth(2.4, 10, 1.0, 1.5, 1.8, L - 2.2, tilt=-40),
              cyl(2.4, 0.7, L - 2.6, n=16))
    return m


def col_shaft_a():
    (a, b), _ = column_shaft_segs()
    return _shaft(a, b)


def col_shaft_b():
    _, (a, b) = column_shaft_segs()
    return _shaft(a, b)


def column_curve(s_x, n=40):
    """Centreline of one swan-neck column (quadratic bezier)."""
    p0 = np.array([s_x, 30.0, 97.0])
    pm = np.array([s_x * 1.4, 74.0, 150.0])
    p1 = np.array([s_x, 15.5, 237.0])
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 2 * p0 + 2 * (1 - t) * t * pm + t ** 2 * p1


def columns():
    """Twin swan-neck columns bowing around the tourbillon (static)."""
    parts = []
    for s in (-1, 1):
        pth = column_curve(s * P.COL_X)
        parts.append(tube(pth, P.COL_ROD_R, n=10))
        # collars
        for tt in (0.18, 0.5, 0.82):
            i = int(tt * (len(pth) - 1))
            d = pth[min(i + 1, len(pth) - 1)] - pth[max(i - 1, 0)]
            parts.append(put(lathe([(1.0, -1.6), (P.COL_ROD_R + 1.3, -1.6),
                                    (P.COL_ROD_R + 1.3, 1.6), (1.0, 1.6)],
                                   n=8), frame(pth[i], d)))
    # foot pad on the sphere collar + bracket to the canister back
    parts.append(put(cyl(8.5, 5.0, -2.5, n=24),
                     frame((0, 30.0, 97.0), (0, -0.25, 1))))
    parts.append(put(cyl(7.0, 3.0, -1.5, n=24),
                     frame((0, 15.5, 236.0), P.dial_normal())))
    # cross-strut between the columns at the relay
    parts.append(tube(np.linspace((-P.COL_X * 1.18, 51.5, 152.0),
                                  (P.COL_X * 1.18, 51.5, 152.0), 2),
                      1.4, n=8))
    return merge(*parts)


# ============================================================ tourbillon
def stalk():
    """Fixed vase + trumpet around the cage pipe + the sun crown."""
    vase = lathe([(0, P.STALK_Z0), (11.5, P.STALK_Z0),
                  (10.0, P.STALK_Z0 + 2.0), (5.4, P.STALK_Z0 + 4.5),
                  (6.6, P.STALK_Z0 + 7.0), (0, P.STALK_Z0 + 7.0)], n=40)
    trumpet = lathe([(6.6, 109.0), (6.6, 116.0), (9.3, 117.8),
                     (9.3, P.TRUMPET_Z1), (7.2, P.TRUMPET_Z1),
                     (7.2, 109.0)], n=40)
    sun = crown_teeth(P.SUN_R, 24, 1.4, 1.8, 2.2, P.TRUMPET_Z1, tilt=35)
    return merge(vase, trumpet, sun)


def outer_cage():
    """Meridian rings at 45/135 + equator + hubs; local Z = spin axis.
    The XZ plane stays free for the planetary lay shaft."""
    mer = put(ring_torus(P.CAGE_R, P.CAGE_TUBE, 56, 8), rotm([1, 0, 0], 90))
    parts = [put(mer, rotm([0, 0, 1], lon)) for lon in P.MERIDIAN_LONS]
    parts.append(ring_torus(P.CAGE_R, P.EQ_TUBE, 56, 8))
    # top hub + finial
    parts.append(lathe([(0, 33.5), (5.0, 33.5), (2.2, 38.5), (2.2, 40.0),
                        (0, 40.0)], n=24))
    parts.append(ball(3.4, (0, 0, 41.5), sub=2))
    # bottom pipe (spins inside the trumpet, keyed to the centre shaft)
    parts.append(lathe([(3.2, -41.0), (5.0, -41.0), (5.0, -33.5),
                        (6.4, -32.4), (6.4, -30.8), (3.2, -30.8)], n=32))
    # pivot bosses for the inner carriage (on the +/-X equator nodes)
    for s in (-1, 1):
        parts.append(put(cyl(4.0, 7.0, 0, n=20),
                         frame((s * 35.5, 0, 0), (-s, 0, 0))))
        parts.append(put(cyl(4.8, 1.8, 0, n=20),
                         frame((s * 36.9, 0, 0), (-s, 0, 0))))
    # lay-shaft steady struts (from the pipe flange and the -X boss)
    d = P.LAY_B - P.LAY_A
    p1 = P.LAY_A + d * 0.14
    p2 = P.LAY_A + d * 0.86
    parts.append(tube(np.linspace((-5.8, 0, -30.6), p1, 2), 0.8, n=6))
    parts.append(tube(np.linspace((-30.5, 0, -3.8), p2, 2), 0.8, n=6))
    for p in (p1, p2):
        parts.append(put(cyl(1.6, 2.2, -1.1, n=10), frame(p, d)))
    return merge(*parts)


def cage_rubies():
    """Jewel dots on the pivot bosses + top hub (parent: outer cage)."""
    return merge(ball(1.25, (28.9, 0, 0), 1), ball(1.25, (-28.9, 0, 0), 1),
                 ball(1.6, (0, 0, 38.9), 1))


def lay_shaft():
    """Planetary rod: sun crown -> inner-carriage bevel.  Local +Z = A->B."""
    L = np.linalg.norm(P.LAY_B - P.LAY_A)
    m = cyl(P.LAY_ROD_R, L + 1.0, -0.5, n=8)
    for z0, tilt in ((-2.4, -38), (L - 0.4, 38)):
        m = merge(m, cyl(3.1, 0.9, z0 + 1.0, n=14),
                  crown_teeth(2.6, 9, 1.0, 1.6, 1.9, z0 + 1.6, tilt=tilt))
    return m


def inner_cage():
    """Carriage: ring + bar + hub + cock + stubs; local Z = its spin axis
    (maps to the outer cage's X)."""
    parts = [put(ring_torus(P.INNER_R, P.INNER_TUBE, 48, 8),
                 rotm([1, 0, 0], 90))]          # ring in local XZ plane
    parts.append(box((2 * P.INNER_R, 2.6, 7.0)))            # bar along X
    parts.append(put(cyl(4.0, 5.6, -2.8, n=20), frame((0, 0, 0), (0, 1, 0))))
    # pivot stubs along local Z into the outer bosses + drive bevel on -Z
    parts.append(cyl(1.8, 9.5, P.INNER_R - 0.5, n=10))
    parts.append(cyl(1.8, 9.5, -P.INNER_R - 9.0, n=10))
    parts.append(merge(cyl(3.4, 1.0, -26.5, n=14),
                       crown_teeth(2.8, 12, 1.0, 1.5, 1.8, -25.5, tilt=42)))
    # balance cock: arc over the hairspring dome, in the local XY plane
    cc, rr = 5.107, 9.893
    pth = arc_path(rr, -24.5, 204.5, n=22, c=(0, cc, 0))
    parts.append(tube(pth, 1.3, n=8))
    parts.append(put(cyl(2.0, 2.6, -1.3, n=10),
                     frame((0, 14.4, 0), (0, 1, 0))))
    # escape-wheel lower bearing stub on the bar
    parts.append(put(cyl(1.2, 3.4, -1.2, n=8),
                     frame((P.ESC_POS, 0, 0), (0, 1, 0))))
    # counterpoise
    parts.append(ball(3.2, (-16.0, 0, 0), 2))
    return merge(*parts)


def inner_rubies():
    return merge(ball(1.0, (0, 15.0, 0), 1),          # staff top jewel
                 ball(0.9, (P.ESC_POS, 2.4, 0), 1))


def balance_wheel():
    """Rim + 3 spokes + roller/impulse pin + staff; local Z = staff.
    The mesh sits around z=0; the chain lifts it to BAL_Y on the staff."""
    w = ring(P.BAL_R, P.BAL_R - P.BAL_RIM_W)
    for a in (90, 210, 330):
        w = w.union(sa.rotate(rounded_bar((0, 0), (P.BAL_R - 1.0, 0), 2.2),
                              a, origin=(0, 0)))
    w = w.union(circle(3.0)).difference(circle(1.0))
    m = extrude_flat(w, P.BAL_T, -P.BAL_T / 2)
    # six radial timing screws on the rim
    for i in range(6):
        a = np.radians(60 * i + 30)
        c = (P.BAL_R + 0.7) * np.array([np.cos(a), np.sin(a), 0])
        m = merge(m, put(cyl(0.9, 2.0, -1.0, n=8),
                         frame(c, (np.cos(a), np.sin(a), 0))))
    m = merge(m, cyl(1.2, 12.6, -3.4, n=10),          # staff
              cyl(2.2, 1.4, P.BAL_T / 2, n=10),       # collet
              cyl(3.2, 1.0, -2.4, n=14),              # roller disc
              put(cyl(0.55, 1.7, 0, n=6), frame((2.3, 0, -3.1), (0, 0, 1))))
    return m


def hairspring_dome(phase_deg=0.0):
    """Spherical hairspring: spiral climbing a dome; breathes with the
    balance like the flat one in the watch.  Local Z = staff axis."""
    h = P.DOME_Y1 - P.DOME_Y0
    turns = P.DOME_TURNS - np.radians(phase_deg) / (2 * np.pi)
    n = 110
    s = np.linspace(0.0, 1.0, n)
    th = 2 * np.pi * turns * s + np.radians(phase_deg)
    r = P.DOME_R_IN + (P.DOME_R_OUT - P.DOME_R_IN) * np.cos(
        s * np.pi / 2) ** 0.85
    z = h * np.sin(s * np.pi / 2) ** 1.1
    pth = np.column_stack([r * np.cos(th), r * np.sin(th), z])
    m = tube(pth, 0.5, n=6)
    # outer stud foot + inner collet grip
    m = merge(m, put(cyl(0.9, 2.4, -0.6, n=8), frame(pth[0], (0, 0, 1))),
              cyl(1.4, 1.4, h - 0.7, n=8))
    return m


def escape_wheel_clock():
    """15-tooth escape wheel + pinion; local Z = its axis."""
    w = escape_outline(P.ESC_T, P.ESC_TIP_R, P.ESC_ROOT_R, lean_deg=12,
                       tip_w=0.4)
    for a in (0, 120, 240):
        c = 2.6 * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        w = w.difference(circle(0.95, tuple(c)))
    w = w.difference(circle(0.65))
    m = extrude_flat(w, 1.5, 0.0)
    g = gear_outline(0.8, 7, 0.4, backlash=0.2, flank_pts=4)
    m = merge(m, extrude_flat(g.difference(circle(0.65)), 1.6, -1.8))
    return m


def lever_clock():
    """Pin-pallet lever; local Z = pivot, fork at -X, pallets at +X."""
    body = rounded_bar((-2.0, 0), (2.0, 0), 2.0)
    fork = poly([(-1.6, 1.6), (-3.0, 1.35), (-3.0, 0.45), (-1.9, 0.55),
                 (-1.9, -0.55), (-3.0, -0.45), (-3.0, -1.35), (-1.6, -1.6)])
    arms = union(rounded_bar((1.0, 0), (1.5, 2.0), 1.3),
                 rounded_bar((1.0, 0), (1.5, -2.0), 1.3))
    w = union(body, fork, arms, circle(1.5)).difference(circle(0.55))
    m = extrude_flat(w, 1.2, -0.1)
    for sy in (1, -1):
        m = merge(m, put(cyl(0.5, 2.2, 0, n=6),
                         frame((1.6, sy * 2.1, -2.2), (0, 0, 1))))
    return m


# ================================================================== dial
def dial_frame():
    return frame(P.DIAL_C, P.dial_normal(), (1, 0, 0))


def dial():
    """Chapter ring + S-spokes + hub (local XY, face +Z)."""
    w = ring(P.DIAL_OR, P.DIAL_IR).union(circle(P.DIAL_HUB_R))
    for k in range(4):
        a0 = np.radians(45 + 90 * k)
        a1 = a0 + np.radians(46)
        am = a0 + np.radians(25)
        p0 = (P.DIAL_HUB_R - 1) * np.array([np.cos(a0), np.sin(a0)])
        pm = 27.5 * np.array([np.cos(am), np.sin(am)])
        p1 = (P.DIAL_IR + 1.5) * np.array([np.cos(a1), np.sin(a1)])
        t = np.linspace(0, 1, 16)[:, None]
        bez = ((1 - t) ** 2 * p0 + 2 * (1 - t) * t * pm + t ** 2 * p1)
        w = w.union(sg.LineString(bez).buffer(1.9, quad_segs=6))
    w = w.difference(circle(3.6))
    return extrude_flat(w, P.DIAL_T, -P.DIAL_T / 2)


def dial_markers():
    """Raised batons + minute ticks (parent: dial frame)."""
    rm = (P.DIAL_OR + P.DIAL_IR) / 2
    w = None
    for h in range(12):
        a = 90 - 30 * h
        if h == 0:
            b = union(*[sa.rotate(rect(2.2, 7.0, (dx, rm)), a - 90,
                                  origin=(0, 0)) for dx in (-2.1, 2.1)])
        else:
            b = sa.rotate(rect(2.8, 7.0, (0, rm)), a - 90, origin=(0, 0))
        w = b if w is None else w.union(b)
    for t in range(60):
        if t % 5 == 0:
            continue
        w = w.union(sa.rotate(rect(0.9, 2.2, (0, P.DIAL_OR - 1.6)),
                              -6.0 * t, origin=(0, 0)))
    return extrude_flat(w, 1.2, P.DIAL_T / 2)


def canister():
    """Motion-works drum + pipes + inlet bevel boss (parent: dial frame)."""
    z0 = -P.DIAL_T / 2 - P.CAN_GAP - P.CAN_D
    parts = [
        lathe([(0, z0), (P.CAN_R, z0), (P.CAN_R, z0 + P.CAN_D),
               (9.0, z0 + P.CAN_D), (0, z0 + P.CAN_D)], n=36),
        lathe([(P.CAN_R, z0 + 1.0), (P.CAN_R + 1.2, z0 + 2.5),
               (P.CAN_R + 1.2, z0 + 4.0), (P.CAN_R, z0 + 5.5)], n=36),
        cyl(3.4, -z0 + 3.2, z0, n=16),       # hour pipe
        cyl(2.3, -z0 + 4.4, z0, n=14),       # minute pipe
        cyl(1.25, -z0 + 6.0, z0, n=10),      # seconds pipe
    ]
    # inlet boss at the drum's lower edge (column shaft arrives there)
    parts.append(put(cyl(2.6, 6.5, -3.2, n=14),
                     frame((0, -P.CAN_R, z0 + P.CAN_D / 2), (0, -1, -0.3))))
    return merge(*parts)


def _breguet_hand(L, w, ring_r, ring_at, hub_r, bore, tail=6.0):
    bar = rounded_bar((0, -tail), (0, ring_at - ring_r + 0.6), w)
    moon = ring(ring_r, ring_r - 1.3, c=(0, ring_at))
    tip = poly([(-w * 1.5, ring_at + ring_r - 0.6), (0, L),
                (w * 1.5, ring_at + ring_r - 0.6)])
    w2 = union(bar, moon, tip, circle(hub_r)).difference(circle(bore))
    return extrude_flat(w2, 1.1, 0.0)


def hand_hour():
    return _breguet_hand(P.HAND_HOUR_L, 2.6, 4.2, P.HAND_HOUR_L * 0.60,
                         5.2, 3.5)


def hand_minute():
    return _breguet_hand(P.HAND_MIN_L, 2.0, 3.4, P.HAND_MIN_L * 0.64,
                         4.2, 2.4)


def hand_second():
    w = union(rounded_bar((0, -13.0), (0, P.HAND_SEC_L), 0.9),
              circle(3.0, (0, -9.0)), circle(2.2))
    w = w.difference(circle(2.2, (0, -9.0)).difference(
        circle(1.4, (0, -9.0))))
    w = w.difference(circle(1.05))
    return extrude_flat(w, 1.0, 0.0)


def hub_cap():
    return lathe([(0, 0), (2.2, 0), (1.6, 1.6), (0, 2.2)], n=16)


# ====================================================== assembly / chain
def _f(origin, zdir, xhint=(1, 0, 0)):
    return frame(origin, zdir, xhint)


def build():
    """Return (meshes, chain).  chain: name -> (parent, M0, spin_tag)."""
    df = dial_frame()
    seg_a, seg_b = column_shaft_segs()
    d_lay = (P.LAY_B - P.LAY_A) / np.linalg.norm(P.LAY_B - P.LAY_A)

    meshes = {
        "plinth": plinth(), "cradle": cradle(),
        "sphere_steel": base_sphere_steel(),
        "sphere_gold": base_sphere_gold(),
        "barrel": barrel_drum(), "spring": mainspring_clock(),
        "key": winding_key(),
        "click": click_pawl(), "gearbox": gearbox(),
        "transfer": transfer_arbor(), "center_shaft": center_shaft(),
        "y_rod": y_rod(), "pod": column_pod(),
        "col_shaft_a": col_shaft_a(), "col_shaft_b": col_shaft_b(),
        "columns": columns(),
        "stalk": stalk(),
        "cage": outer_cage(), "cage_rubies": cage_rubies(),
        "lay": lay_shaft(),
        "inner": inner_cage(), "inner_rubies": inner_rubies(),
        "balance": balance_wheel(), "dome": hairspring_dome(0.0),
        "escape": escape_wheel_clock(), "lever": lever_clock(),
        "dial": dial(), "markers": dial_markers(), "canister": canister(),
        "hand_h": hand_hour(), "hand_m": hand_minute(),
        "hand_s": hand_second(), "cap": hub_cap(),
    }

    I = np.eye(4)
    chain = {
        # statics (world frame)
        "plinth": (None, I, None), "cradle": (None, I, None),
        "sphere_steel": (None, I, None), "sphere_gold": (None, I, None),
        "click": (None, I, None), "gearbox": (None, I, None),
        "pod": (None, I, None), "columns": (None, I, None),
        "stalk": (None, I, None),
        # base mechanism
        "barrel": (None, _f((0, -6.5, P.SPH_C), (0, 1, 0)), "drum"),
        "spring": ("barrel", np.eye(4), None),
        "key": (None, _f((0, P.KEY_Y0, P.SPH_C), (0, 1, 0)), None),
        "transfer": (None, _f(tuple(P.TRANSFER_C), (0, 1, 0)), "slow"),
        "center_shaft": (None, _f((0, 0, 92.0), (0, 0, 1)), "sec"),
        "y_rod": (None, _f((0, 8.0, 98.5), (0, 1, 0)), "sec"),
        "col_shaft_a": (None, _f(tuple(seg_a[0]),
                                 tuple(seg_a[1] - seg_a[0])), "sec"),
        "col_shaft_b": (None, _f(tuple(seg_b[0]),
                                 tuple(seg_b[1] - seg_b[0])), "sec"),
        # tourbillon chain
        "cage": (None, transm(P.TOURB_C), "cage"),
        "cage_rubies": ("cage", I, None),
        "lay": ("cage", _f(tuple(P.LAY_A), tuple(d_lay)), "lay"),
        "inner": ("cage", _f((0, 0, 0), (1, 0, 0), (0, 0, 1)), "inner"),
        "inner_rubies": ("inner", I, None),
        "balance": ("inner", _f((0, P.BAL_Y, 0), (0, 1, 0)), "balance"),
        "dome": ("inner", _f((0, P.DOME_Y0, 0), (0, 1, 0)), None),
        "escape": ("inner", _f((P.ESC_POS, 1.8, 0), (0, 1, 0)), "escape"),
        "lever": ("inner", _f((P.LEVER_POS, 4.0, 0), (0, 1, 0)), "lever"),
        # dial group
        "dial": (None, df, None),
        "markers": (None, df, None),
        "canister": (None, df, None),
        "hand_h": (None, df @ transm([0, 0, 4.0]), "hand_h"),
        "hand_m": (None, df @ transm([0, 0, 5.3]), "hand_m"),
        "hand_s": (None, df @ transm([0, 0, 6.7]), "hand_s"),
        "cap": (None, df @ transm([0, 0, 7.9]), None),
    }
    return meshes, chain


COLORS = {
    "plinth": "#2e3138", "cradle": "#c9a227",
    "sphere_steel": "#a7b0b9", "sphere_gold": "#c9a227",
    "barrel": "#a8842c", "spring": "#2b4bb3",
    "key": "#71757b", "click": "#71757b",
    "gearbox": "#9c7c2c", "transfer": "#888c92",
    "center_shaft": "#6a6e74", "y_rod": "#84888f",
    "pod": "#9c7c2c", "col_shaft_a": "#84888f", "col_shaft_b": "#84888f",
    "columns": "#3a3e45",
    "stalk": "#8f99a3",
    "cage": "#d4af37", "cage_rubies": "#c0182c", "lay": "#b8743a",
    "inner": "#9aa6b2", "inner_rubies": "#c0182c",
    "balance": "#b87333", "dome": "#2b4bb3",
    "escape": "#cc3366", "lever": "#dd2222",
    "dial": "#f4efe3", "markers": "#15161a", "canister": "#9c7c2c",
    "hand_h": "#1b2f6e", "hand_m": "#1b2f6e", "hand_s": "#b01818",
    "cap": "#c9a227",
}
