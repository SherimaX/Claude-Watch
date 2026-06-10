"""Rotating parts: train wheels, barrel, mainspring, escapement, balance."""
import numpy as np
import shapely.affinity as sa

from . import params as P
from .geometry import (gear_outline, spoked_wheel, circle, ring, union,
                       extrude, zspan, stack, spiral_ribbon, square_hole,
                       rect, poly, rounded_bar, d_profile)
from . import escapement as E


def wheel48(backlash=None):
    return gear_outline(P.MODULE, P.WHEEL_T, P.WHEEL_SHIFT,
                        backlash=P.BACKLASH if backlash is None else backlash)


def pinion8():
    return gear_outline(P.MODULE, P.PINION_T, P.PINION_SHIFT,
                        backlash=P.BACKLASH, flank_pts=8)


RIM_IN = 14.0     # spoke window outer limit for 48T wheels


# ------------------------------------------------------------ train wheels
CENTER_SQ = 3.4       # center arbor square (wheel keyed on it)


def center_wheel():
    """48T wheel + 8T pinion, square-bored onto the printed center arbor."""
    w = spoked_wheel(wheel48(), hub_r=4.6, rim_r=RIM_IN)
    m = zspan(w.difference(square_hole(CENTER_SQ + 0.1)), P.Z_CENTER_W)
    pin = pinion8().difference(square_hole(CENTER_SQ + 0.1))
    m = stack(m, zspan(pin, P.Z_CENTER_P))
    hub = circle(P.CENTER_PIPE_OD / 2).difference(
        square_hole(CENTER_SQ + 0.1))
    m = stack(m, extrude(hub, P.Z_CENTER_P[0] - P.Z_CENTER_W[1] + 0.2,
                         P.Z_CENTER_W[1] - 0.1))
    return m


def center_arbor():
    """Vertical print: pivot stub, square (wheel seat), pipe, cannon grip."""
    m = extrude(circle(P.CENTER_STUB_D / 2), 5.0, -2.0)
    m = stack(m, extrude(square_hole(CENTER_SQ), 8.6 - 3.0, 3.0))
    m = stack(m, extrude(circle(P.CENTER_PIPE_OD / 2), 17.2 - 8.6, 8.6))
    m = stack(m, extrude(circle(3.8 / 2), 19.0 - 17.2, 17.2))
    return m


def third_wheel():
    w = spoked_wheel(wheel48(), hub_r=P.HUB_R + 1.0, rim_r=RIM_IN,
                     bore=P.PIN_HOLE_PRESS)
    m = zspan(w, P.Z_THIRD_W)
    pin = pinion8().difference(circle(P.PIN_HOLE_PRESS / 2))
    m = stack(m, zspan(pin, P.Z_THIRD_P))
    return m


def fourth_wheel():
    pin = pinion8().difference(circle(P.PIN_HOLE_PRESS / 2))
    m = zspan(pin, P.Z_FOURTH_P)
    hub = ring(2.0, P.PIN_HOLE_PRESS / 2)
    m = stack(m, extrude(hub, P.Z_FOURTH_W[0] - P.Z_FOURTH_P[1] + 0.2,
                         P.Z_FOURTH_P[1] - 0.1))
    w = spoked_wheel(wheel48(), hub_r=P.HUB_R + 1.0, rim_r=RIM_IN,
                     bore=P.PIN_HOLE_PRESS)
    m = stack(m, zspan(w, P.Z_FOURTH_W))
    return m


def escape_wheel():
    pin = pinion8().difference(circle(P.PIN_HOLE_PRESS / 2))
    m = zspan(pin, P.Z_ESC_P)
    hub = ring(2.0, P.PIN_HOLE_PRESS / 2)
    m = stack(m, extrude(hub, P.Z_ESC_W[0] - P.Z_ESC_P[1] + 0.2,
                         P.Z_ESC_P[1] - 0.1))
    w = E.escape_outline()
    w = w.difference(circle(P.PIN_HOLE_PRESS / 2))
    # lightening: keep a solid disc, teeth need stiffness; just bore + 4 holes
    for a in (45, 135, 225, 315):
        c = 5.6 * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        w = w.difference(circle(1.8, tuple(c)))
    m = stack(m, zspan(w, P.Z_ESC_W))
    return m


# ---------------------------------------------------------------- barrel
def barrel_drum():
    z0, z1 = P.Z_DRUM[0], 12.6
    m = None
    # bottom disc with arbor hole
    m = stack(m, extrude(ring(P.DRUM_OR, P.BARREL_ARBOR_BEAR_D / 2 + 0.25),
                         1.0, z0))
    # gear ring around the bottom
    g = wheel48().difference(circle(P.DRUM_OR - 0.6))
    m = stack(m, zspan(g, P.Z_BARREL_G))
    # wall with outer-end spring hook slot
    wall = ring(P.DRUM_OR, P.DRUM_IR)
    hook = rect(2.4, 2.0, c=(P.DRUM_IR - 0.4, 0))
    wall = wall.union(hook.difference(rect(2.4, 1.1, c=(P.DRUM_IR - 1.0, 0))))
    m = stack(m, extrude(wall, 11.8 - (z0 + 1.0), z0 + 1.0))
    # lid seat: thinner wall section at the top
    seat = ring(P.DRUM_OR, P.DRUM_IR + 0.8)
    m = stack(m, extrude(seat, 12.6 - 11.8, 11.8))
    return m


def barrel_lid():
    m = extrude(ring(P.DRUM_IR + 0.65, P.BARREL_ARBOR_BEAR_D / 2 + 0.25),
                0.8, 11.8)
    m = stack(m, extrude(ring(P.DRUM_IR - 0.15, P.DRUM_IR - 1.0), 1.4, 10.4))
    return m


def barrel_arbor():
    m = extrude(square_hole(P.BARREL_SQ), 5.6, -8.8)            # ratchet sq
    m = stack(m, extrude(circle(P.BARREL_ARBOR_BEAR_D / 2), 3.6, -3.2))
    core = circle(P.ARBOR_CORE_R)
    # inner spring hook: radial slot in the core
    core = core.difference(rect(1.5, 1.6, c=(P.ARBOR_CORE_R - 0.7, 0)))
    m = stack(m, extrude(core, 11.9 - 0.4, 0.4))
    m = stack(m, extrude(circle(2.0), 17.0 - 11.5, 11.5))       # top stub
    # M3 pilot for ratchet screw
    return m


def mainspring(band=None):
    band = band or P.MS_BAND
    r0 = P.ARBOR_CORE_R + 0.8 + band / 2
    turns = (P.DRUM_IR - 1.2 - r0 - band) / 2.6
    sp = spiral_ribbon(r0, 2.6, turns, band, ccw=True)
    # inner end: C-ring around the core with an inward hook tab
    inner = ring(P.ARBOR_CORE_R + 0.2 + band, P.ARBOR_CORE_R + 0.2)
    tab = rect(1.3, 1.2, c=(P.ARBOR_CORE_R - 0.45, 0))
    sp = sp.union(inner).union(tab)
    # outer end: outward hook tab (engages drum wall slot)
    th_end = 2 * np.pi * turns
    r_end = r0 + 2.6 * turns / (2 * np.pi) * 2 * np.pi / (2 * np.pi)
    r_end = r0 + 2.6 * turns
    # outer tab placed at spiral end angle
    a = th_end % (2 * np.pi)
    c = (r_end + 0.4) * np.array([np.cos(a), np.sin(a)])
    tab2 = sa.rotate(rect(1.2, 2.6, c=tuple(c)), np.degrees(a), origin=(0, 0))
    sp = sp.union(tab2)
    return extrude(sp, P.MS_H - 0.2, P.Z_DRUM[0] + 1.2)


def ratchet_wheel():
    """18T m0.7 spur: meshed by the crown wheel for winding, held by the
    click (the click beak angle makes involute teeth one-way)."""
    w = gear_outline(P.MODULE, P.RATCHET_T, 0.0, backlash=P.BACKLASH)
    w = w.difference(square_hole(P.BARREL_SQ + 0.15))
    return zspan(w, P.Z_RATCHET)


def click():
    """Pawl with integrated spring arm; pivots on a filament pin."""
    body = rounded_bar((0, 0), (8.0, 0), 3.0)
    beak = poly([(8.0, 1.4), (10.6, 0.4), (8.0, -1.2)])
    spring = rounded_bar((0.0, -1.0), (-7.0, -4.4), 1.2)
    w = union(body, beak, spring, circle(2.2))
    w = w.difference(circle(P.PIN_HOLE_BEAR / 2))
    return zspan(w, P.Z_RATCHET)


# ------------------------------------------------------------- escapement
def lever():
    body, pins = E.lever_outline(with_pin_holes=True)
    m = zspan(body, P.Z_LEVER)
    g = E.guard_outline()
    m = stack(m, zspan(g, P.Z_GUARD))
    return m


D_BORE = (1.6, 1.05)     # roller bores: D-profile over the arbor flat


def roller_main():
    d = E.roller_main_outline().difference(d_profile(*D_BORE))
    m = zspan(d, P.Z_ROLLER)
    neck = circle(2.1).difference(d_profile(*D_BORE))
    m = stack(m, extrude(neck, P.Z_ROLLER[0] - 9.4, 9.4))
    return m


def roller_safety():
    d = E.roller_safety_outline().difference(d_profile(*D_BORE))
    return zspan(d, P.Z_SAFETY)


# ---------------------------------------------------------------- balance
def balance_wheel():
    rim = ring(P.BAL_RIM_OD / 2, P.BAL_RIM_ID / 2)
    spokes = union(*[sa.rotate(rect(P.BAL_RIM_OD - 2, 2.4, c=(0, 0)), a,
                               origin=(0, 0)) for a in (0, 90)])
    hub = circle(4.0)
    w = union(rim, spokes, hub)
    # M3 self-tap holes for tuning screws/nuts: each hole gets a boss so
    # the 2.5 mm bore cannot sever the 2 mm-wide rim
    n = P.BAL_WEIGHT_HOLES
    rmid = (P.BAL_RIM_OD + P.BAL_RIM_ID) / 4
    for i in range(n):
        a = 2 * np.pi * (i + 0.5) / n
        c = (rmid * np.cos(a), rmid * np.sin(a))
        w = w.union(circle(P.BAL_BOSS_R, c))
    w = w.difference(square_hole(P.BAL_SQUARE + 0.1))
    for i in range(n):
        a = 2 * np.pi * (i + 0.5) / n
        w = w.difference(circle(1.25, (rmid * np.cos(a),
                                       rmid * np.sin(a))))
    assert w.geom_type == "Polygon", \
        "balance wheel outline must stay one connected piece"
    return zspan(w, P.Z_BAL_RIM)


def hairspring(pitch=None, band=None):
    pitch = pitch or P.HS_PITCH
    band = band or P.HS_BAND
    sp = spiral_ribbon(P.HS_R0, pitch, P.HS_TURNS, band, ccw=True,
                       n_per_turn=140)
    hub = circle(P.HS_R0 + band / 2).difference(
        square_hole(P.BAL_SQUARE + 0.1))
    r_out = P.HS_R0 + pitch * P.HS_TURNS
    a = (2 * np.pi * P.HS_TURNS) % (2 * np.pi)
    c = (r_out + 1.2) * np.array([np.cos(a), np.sin(a)])
    tab = sa.rotate(rect(3.0, 2.4, c=tuple(c)), np.degrees(a), origin=(0, 0))
    tab = tab.difference(circle(0.8, tuple(
        (r_out + 1.2) * np.array([np.cos(a), np.sin(a)]))))
    sp = sp.union(hub).union(tab)
    return zspan(sp, P.Z_HSPRING)


def balance_arbor():
    """Printed vertically: pivots, squares and rounds in one shaft."""
    m = extrude(circle(P.BAL_PIVOT_D / 2), 2.0, -9.8)
    m = stack(m, extrude(square_hole(P.BAL_SQUARE), -3.2 - (-7.8), -7.8))
    m = stack(m, extrude(circle(P.BAL_ARBOR_D / 2), 8.2 - (-3.4), -3.4))
    m = stack(m, extrude(d_profile(1.5, 0.95), 12.8 - 8.2, 8.2))
    m = stack(m, extrude(circle(P.BAL_ARBOR_D / 2), 14.4 - 12.8, 12.8))
    m = stack(m, extrude(circle(P.BAL_PIVOT_D / 2), 2.0, 14.4))
    return m
