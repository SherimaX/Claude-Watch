"""Static parts (plates, cock, case) + motion works + dial + hands."""
import numpy as np
import shapely.affinity as sa

from . import params as P
from .geometry import (gear_outline, spoked_wheel, circle, ring, union,
                       extrude, zspan, stack, square_hole, rect, poly,
                       rounded_bar, rot)
from . import escapement as E


def _pt(c):
    return (float(c[0]), float(c[1]))


def banking_posts_world():
    """Banking post centers (world), lever at neutral, local (7, +/-4)."""
    _, ang = E._lever_geo()
    return [P.P_LEVER + rot(np.array([[7.0, 4.0]]), ang)[0],
            P.P_LEVER + rot(np.array([[7.0, -4.0]]), ang)[0]]


def click_geo():
    """Click pivot, orientation angle (deg), spring abutment pin (world)."""
    a = np.radians(145.0)
    pivot = P.P_BARREL + 16.8 * np.array([np.cos(a), np.sin(a)])
    ang = np.degrees(np.arctan2(*(P.P_BARREL - pivot)[::-1]))
    abut = pivot + rot(np.array([[-6.5, -5.8]]), np.radians(ang))[0]
    return pivot, ang, abut


# ----------------------------------------------------------------- plates
def back_plate():
    d = circle(P.PLATE_R, n=200)
    holes = [
        (P.P_CENTER, P.CENTER_STUB_D + 0.3),       # center stub bearing
        (P.P_BARREL, P.BARREL_ARBOR_BEAR_D + 0.4),
        (P.P_THIRD, P.PIN_HOLE_BEAR),
        (P.P_FOURTH, P.PIN_HOLE_BEAR),
        (P.P_ESCAPE, P.PIN_HOLE_BEAR),
        (P.P_LEVER, P.PIN_HOLE_PRESS),             # lever stud pressed
        (P.P_BALANCE, 4.2),                        # arbor clearance
    ]
    pivot, ang, abut = click_geo()
    holes += [(pivot, P.PIN_HOLE_PRESS), (abut, P.PIN_HOLE_PRESS)]
    # balance cock screw bosses (self-tap M3 from the back)
    u = P.P_BALANCE / np.linalg.norm(P.P_BALANCE)
    v = np.array([-u[1], u[0]])
    leg0 = P.P_BALANCE + 13.5 * u
    for s in (+1, -1):
        holes.append((leg0 + 4.5 * s * v, 2.5))
    for c, dia in holes:
        d = d.difference(circle(dia / 2, _pt(c)))
    m = extrude(d, P.PLATE_T, -P.PLATE_T)
    # pillars with M3 pilot holes
    for a in P.PILLAR_ANGLES:
        c = P.PILLAR_R * np.array([np.cos(np.radians(a)),
                                   np.sin(np.radians(a))])
        pil = circle(P.PILLAR_RAD, _pt(c)).difference(circle(1.25, _pt(c)))
        m = stack(m, extrude(pil, P.PILLAR_H, 0.0))
    # banking posts
    for c in banking_posts_world():
        m = stack(m, extrude(circle(1.5, _pt(c)), 11.4, 0.0))
    return m


def front_plate():
    d = circle(P.PLATE_R, n=200)
    holes = [
        (P.P_CENTER, P.CENTER_PIPE_OD + 0.4),
        (P.P_BARREL, 4.4),
        (P.P_THIRD, P.PIN_HOLE_BEAR),
        (P.P_FOURTH, P.PIN_HOLE_BEAR),
        (P.P_ESCAPE, P.PIN_HOLE_BEAR),
        (P.P_LEVER, 1.8),
        (P.P_BALANCE, P.BAL_PIVOT_D + 0.4),
        (P.P_MINUTE, P.PIN_HOLE_PRESS),
    ]
    for a in P.PILLAR_ANGLES:
        c = P.PILLAR_R * np.array([np.cos(np.radians(a)),
                                   np.sin(np.radians(a))])
        holes.append((c, 3.4))
    for c, dia in holes:
        d = d.difference(circle(dia / 2, _pt(c)))
    m = extrude(d, P.FPLATE_T, P.PILLAR_H)
    # dial posts with locating pegs
    for a in P.DIAL_FEET_ANGLES:
        c = P.DIAL_FEET_R * np.array([np.cos(np.radians(a)),
                                      np.sin(np.radians(a))])
        m = stack(m, extrude(circle(2.5, _pt(c)),
                             P.DIAL_Z - (P.PILLAR_H + P.FPLATE_T),
                             P.PILLAR_H + P.FPLATE_T))
        m = stack(m, extrude(circle(0.9, _pt(c)), P.DIAL_T + 0.2, P.DIAL_Z))
    return m


def balance_cock():
    A = P.P_BALANCE
    u = A / np.linalg.norm(A)
    leg0 = A + 13.5 * u
    v = np.array([-u[1], u[0]])
    stud = A + np.array([P.HS_R0 + P.HS_PITCH * P.HS_TURNS + 1.2, 0.0])
    body = rounded_bar(_pt(A), _pt(leg0), 7.0)
    body = body.union(rounded_bar(_pt(A), _pt(stud), 6.0))
    body = body.union(circle(4.5, _pt(A)))
    body = body.union(rounded_bar(_pt(leg0 + 4.5 * v), _pt(leg0 - 4.5 * v),
                                  6.5))
    body = body.difference(circle(1.7, _pt(stud)))     # hairspring stud pin
    for s in (+1, -1):
        body = body.difference(circle(1.7, _pt(leg0 + 4.5 * s * v)))
    # pivot cup: leave floor, model as ring + cap
    m = extrude(body.difference(circle(1.2, _pt(A))), P.Z_COCK[1] - P.Z_COCK[0],
                P.Z_COCK[0])
    # close pivot hole bottom (cup floor 0.7)
    m = stack(m, extrude(circle(2.6, _pt(A)), 0.7, P.Z_COCK[0]))
    # legs rising to back plate
    for s in (+1, -1):
        leg = circle(3.0, _pt(leg0 + 4.5 * s * v)).difference(
            circle(1.7, _pt(leg0 + 4.5 * s * v)))
        m = stack(m, extrude(leg, -3.0 - P.Z_COCK[1], P.Z_COCK[1]))
    return m


# ------------------------------------------------------------ motion works
def cannon_pinion():
    g = gear_outline(P.MODULE, P.CANNON_T, 0.3, backlash=P.BACKLASH)
    g = g.difference(circle(P.CANNON_BORE / 2))
    # two compliance slits
    for a in (0, 180):
        g = g.difference(sa.rotate(rect(0.5, 2.6, c=(P.CANNON_BORE / 2 + 0.9,
                                                     0)), a, origin=(0, 0)))
    m = zspan(g, P.Z_CANNON_G)
    m = stack(m, extrude(ring(2.2, 1.3), 24.2 - P.Z_CANNON_G[1],
                         P.Z_CANNON_G[1]))
    m = stack(m, extrude(ring(1.6, 0.9), P.CANNON_PIPE_TOP - 24.2, 24.2))
    return m


def minute_wheel():
    g = gear_outline(P.MODULE, P.MINUTE_W_T, -0.3, backlash=P.BACKLASH)
    g = spoked_wheel(g, hub_r=4.2, rim_r=14.0, bore=P.PIN_HOLE_BEAR)
    m = zspan(g, P.Z_MINUTE_W)
    p = gear_outline(P.MODULE, P.MINUTE_P_T, 0.0, backlash=P.BACKLASH)
    m = stack(m, zspan(p.difference(circle(P.PIN_HOLE_BEAR / 2)),
                       P.Z_MINUTE_P))
    return m


def hour_wheel():
    g = gear_outline(P.MODULE, P.HOUR_W_T, 0.0, backlash=P.BACKLASH)
    g = spoked_wheel(g, hub_r=P.HOUR_PIPE_OD / 2 + 1.0, rim_r=13.0,
                     bore=P.HOUR_PIPE_ID)
    m = zspan(g, P.Z_HOUR_W)
    m = stack(m, extrude(ring(P.HOUR_PIPE_OD / 2, P.HOUR_PIPE_ID / 2),
                         25.0 - P.Z_HOUR_W[1], P.Z_HOUR_W[1]))
    return m


# ------------------------------------------------------------- dial, hands
def dial():
    d = circle(38.5, n=200).difference(circle(3.8))
    for a in P.DIAL_FEET_ANGLES:
        c = P.DIAL_FEET_R * np.array([np.cos(np.radians(a)),
                                      np.sin(np.radians(a))])
        d = d.difference(circle(1.0, _pt(c)))
    m = extrude(d, P.DIAL_T, P.DIAL_Z)
    # raised batons
    for i in range(12):
        a = np.radians(90 - i * 30)
        L = 5.0 if i % 3 else 7.0
        c = (34.5 - L / 2) * np.array([np.cos(a), np.sin(a)])
        bat = sa.rotate(rect(L, 1.8 if i % 3 else 2.4, c=_pt(c)),
                        np.degrees(a), origin=_pt(c))
        m = stack(m, extrude(bat, 0.6, P.DIAL_Z + P.DIAL_T))
    return m


def _hand(length, tail, bore, hub_od):
    h = ring(hub_od / 2, bore / 2)
    blade = poly([(hub_od / 2 - 0.5, 1.3), (length, 0.45), (length, -0.45),
                  (hub_od / 2 - 0.5, -1.3)])
    tailp = poly([(-hub_od / 2 + 0.5, 1.2), (-tail, 0.8), (-tail, -0.8),
                  (-hub_od / 2 + 0.5, -1.2)])
    return union(h, blade, tailp)


def minute_hand():
    return extrude(_hand(35.0, 8.0, 3.0, 6.0), 1.2, 0.0)


def hour_hand():
    return extrude(_hand(24.0, 7.0, 6.8, 9.8), 1.2, 0.0)


# ---------------------------------------------------------------- case
CASE_BOSS_ANGLES = [-10.0, 120.0, 240.0]


def case_ring():
    z0, z1 = -11.0, 29.4
    wall = ring(P.CASE_OD / 2, P.CASE_ID / 2, n=240)
    m = extrude(wall, z1 - z0, z0)
    # front shoulder (movement stop) + bezel
    m = stack(m, extrude(ring(P.CASE_ID / 2 + 0.01, 39.5, n=240),
                         29.4 - 17.2, 17.2))
    m = stack(m, extrude(ring(39.5, 38.0, n=240), 29.4 - 26.0, 26.0))
    m = stack(m, extrude(ring(39.5, 36.0, n=240), 29.4 - 28.2, 28.2))
    # back-cover screw bosses
    for a in CASE_BOSS_ANGLES:
        c = (P.CASE_ID / 2 - 2.2) * np.array([np.cos(np.radians(a)),
                                              np.sin(np.radians(a))])
        b = circle(2.8, _pt(c)).difference(circle(1.25, _pt(c)))
        b = b.intersection(circle(P.CASE_ID / 2 - 0.05, n=240))
        m = stack(m, extrude(b, -3.0 - (-9.6), -9.6))
    # lugs with strap slots (NATO-style pass-through)
    for s in (+1, -1):
        block = rect(28.0, 6.5, c=(0, s * (P.CASE_OD / 2 + 2.2)))
        block = block.difference(rect(P.LUG_W + 1.0, 2.6,
                                      c=(0, s * (P.CASE_OD / 2 + 3.4))))
        block = block.difference(circle(P.CASE_OD / 2 - 0.1, n=240))
        m = stack(m, extrude(block, 8.0, -6.0))
    return m


def case_back():
    d = circle(P.CASE_ID / 2 - 0.3, n=200)
    d = d.difference(circle(5.0, _pt(P.P_BARREL)))           # winding access
    d = d.difference(circle(16.5, _pt(P.P_BALANCE)))         # balance window
    for a in CASE_BOSS_ANGLES:
        c = (P.CASE_ID / 2 - 2.2) * np.array([np.cos(np.radians(a)),
                                              np.sin(np.radians(a))])
        d = d.difference(circle(1.7, _pt(c)))
    m = extrude(d, 1.4, -11.0)
    return m
