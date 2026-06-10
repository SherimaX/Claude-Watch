"""Pin-pallet (Roskopf) lever escapement: outlines + 2D kinematic check.

Frames:
 * world frame = movement frame (escape wheel at P_ESCAPE etc.)
 * lever local frame: pivot at origin, +X pointing at the balance.

The escape wheel turns CCW (front view).  The lever carries two 1.75 mm
filament pallet pins; teeth lock on the pins (draw angle pulls the lever
onto its banking), the sloped tooth back gives impulse as it slides
under a pin.
"""
import numpy as np
import shapely.affinity as sa
import shapely.geometry as sg

from . import params as P
from .geometry import poly, circle, union, rot, rounded_bar, rect


# ------------------------------------------------------------ escape wheel
def escape_outline():
    """20 pin-pallet teeth, CCW rotation: locking face leads (+angle side),
    impulse plane trails from the tip."""
    z, R = P.ESC_T, P.ESC_R
    pitch = 2 * np.pi / z
    imp = np.radians(P.ESC_IMP_SPAN)
    draw = np.radians(P.ESC_DRAW_DEG)
    r_imp = R - P.ESC_IMP_DROP        # radius at end of impulse face
    r_root = P.ESC_ROOT
    lock_len = R - r_root
    foot_off = lock_len * np.tan(draw) / R     # lock foot leads the tip
    pts = []
    for i in range(z):
        a = i * pitch                 # tip angle
        # ascending-angle order within one pitch:
        # root step -> impulse-face end -> tip -> lock foot -> root arc
        pts.append([r_root * np.cos(a - imp - 0.04),
                    r_root * np.sin(a - imp - 0.04)])
        pts.append([r_imp * np.cos(a - imp), r_imp * np.sin(a - imp)])
        pts.append([R * np.cos(a), R * np.sin(a)])              # tip
        foot_a = a + foot_off
        pts.append([r_root * np.cos(foot_a), r_root * np.sin(foot_a)])
        for t in np.linspace(foot_a + 0.02, a + pitch - imp - 0.08, 4):
            pts.append([r_root * np.cos(t), r_root * np.sin(t)])
    return poly(pts)


# ----------------------------------------------------------------- lever
def _lever_geo():
    """Key lever-local geometry (lever pivot at origin, +X to balance)."""
    # escape center in lever frame
    ang_AB = np.arctan2(*(P.P_BALANCE - P.P_LEVER)[::-1])
    ang_E = np.arctan2(*(P.P_ESCAPE - P.P_LEVER)[::-1])
    e_local_ang = ang_E - ang_AB
    E = P.LEVER_D * np.array([np.cos(e_local_ang), np.sin(e_local_ang)])
    return E, ang_AB


def pallet_pin_centers(r_orbit_in=None):
    """Pin centers in lever-local frame.  Pins sit at escape-frame polar
    radius PIN_SEAT, +/- half the span angle about the E->pivot line."""
    E, _ = _lever_geo()
    half = np.radians(P.LEVER_SPAN_T * 360.0 / P.ESC_T / 2.0)   # 22.5 deg
    seat = P.ESC_R + P.PALLET_PIN_D / 2 - P.PIN_LOCK_DEPTH
    aL = np.arctan2(-E[1], -E[0])      # direction escape -> lever pivot
    pins = []
    for s in (+1, -1):
        a = aL + s * half
        pins.append(E + seat * np.array([np.cos(a), np.sin(a)]))
    return pins                         # [entry?, exit?] resolved by sim


def lever_outline(with_pin_holes=True):
    """Lever local-frame polygon (body plane).  Fork at +X."""
    pins = pallet_pin_centers()
    body = rounded_bar((0, 0), pins[0], 3.0)
    body = body.union(rounded_bar((0, 0), pins[1], 3.0))
    body = body.union(rounded_bar((0, 0), (P.FORK_LEN - 1.0, 0), 3.0))
    body = body.union(circle(3.0))                       # pivot boss
    for c in pins:
        body = body.union(circle(P.PALLET_PIN_D / 2 + 1.0, c))
    # fork head: block with slot + short horns.  In the fork plane only
    # the impulse pin and the balance arbor exist (roller disc is above,
    # safety disc below), so the head only clears the arbor itself.
    head = rect(3.4, P.FORK_SLOT_W + 2 * 1.6, c=(11.5, 0))
    horn = rect(1.8, P.FORK_SLOT_W + 2 * 1.6 + 1.8, c=(12.4, 0))
    body = body.union(head).union(horn)
    body = body.difference(circle(2.5, (P.LB_DIST, 0)))    # arbor/neck clearance
    # fork slot
    slot = rect(3.4, P.FORK_SLOT_W, c=(P.LB_DIST - P.ROLLER_R_IP + 0.6, 0))
    body = body.difference(slot)
    if with_pin_holes:
        for c in pins:
            body = body.difference(circle(P.PIN_HOLE_PRESS / 2, c))
        body = body.difference(circle(P.PIN_HOLE_PRESS / 2))   # pivot
    return body, pins


def guard_outline():
    """Guard pin footprint (below-lever plane), local frame."""
    gx = P.LB_DIST - P.ROLLER_SAFETY_R - 0.35
    return rect(1.6, P.GUARD_W, c=(gx - 0.8, 0)).union(
        rounded_bar((P.FORK_LEN - 2.0, 0), (gx - 0.8, 0), 2.2))


# ----------------------------------------------------------------- roller
def roller_main_outline(crescent=False):
    """Main roller disc, balance-local frame; impulse pin hole at +X."""
    d = circle(P.ROLLER_MAIN_R)
    d = d.difference(circle(P.PIN_HOLE_PRESS / 2, (P.ROLLER_R_IP, 0)))
    return d


def roller_safety_outline():
    d = circle(P.ROLLER_SAFETY_R)
    cres = circle(1.7, (P.ROLLER_SAFETY_R, 0))
    return d.difference(cres)


# ------------------------------------------------------- placed polygons
def to_world(local_poly, phi_deg):
    """Place a lever local-frame polygon into the world at swing phi."""
    _, ang_AB = _lever_geo()
    g = sa.rotate(local_poly, phi_deg, origin=(0, 0))
    g = sa.rotate(g, np.degrees(ang_AB), origin=(0, 0))
    return sa.translate(g, *P.P_LEVER)


def pins_world(phi_deg, pins=None):
    """Only the pallet pins (the wheel-plane collision solids)."""
    if pins is None:
        pins = pallet_pin_centers()
    return to_world(union(*[circle(P.PALLET_PIN_D / 2, c) for c in pins]),
                    phi_deg)


def lever_world(phi_deg, body=None, pins=None):
    """Full lever (body + pins) for drawing."""
    if body is None:
        body, pins = lever_outline(with_pin_holes=False)
    pin_solids = union(*[circle(P.PALLET_PIN_D / 2, c) for c in pins])
    return to_world(body.union(pin_solids), phi_deg)


def wheel_world(theta_deg, outline=None):
    if outline is None:
        outline = escape_outline()
    g = sa.rotate(outline, theta_deg, origin=(0, 0))
    return sa.translate(g, *P.P_ESCAPE)


def max_wheel_advance(lever_poly, theta0, span=25.0, outline=None):
    """Max CCW wheel rotation from theta0 before hitting the lever."""
    if outline is None:
        outline = escape_outline()
    if wheel_world(theta0 + 1e-3, outline).intersects(lever_poly):
        return 0.0
    # linear scan (cannot binary-search: blocked zones are narrow)
    step = 0.2
    a = step
    while a <= span:
        if wheel_world(theta0 + a, outline).intersects(lever_poly):
            lo, hi = a - step, a
            for _ in range(18):
                mid = 0.5 * (lo + hi)
                if wheel_world(theta0 + mid, outline).intersects(lever_poly):
                    hi = mid
                else:
                    lo = mid
            return lo
        a += step
    return span
