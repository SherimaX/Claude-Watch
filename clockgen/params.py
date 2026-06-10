"""Layout of the spherical-tourbillon table clock, in millimetres.

Coordinate system: Z up, the viewer (front) is at -Y.  The clock is a
totem of three elements:

      dial ring (tilted 12 deg, hour/minute/second from the canister)
        |            ... twin rear columns carry the seconds line up
      tourbillon sphere (two-axis flying gyrotourbillon, 60 s / 12 s)
        |            ... stalk + trumpet with the stationary sun crown
      base sphere (armillary openwork; mainspring barrel inside)
        |
      plinth

Every rotating part is modelled in its own "spin frame" (+Z = its
rotation axis, origin = its pivot) and placed by a rigid kinematic
chain (parts.CHAIN), so the same data drives the STL export, the
clearance checks and the animations.

Mechanical story (ratios are real, one reduction is enclosed):
  barrel (1 rev/3 h) -> drum ring gear 30T -> transfer pinion 8T
  -> [48:1 reduction inside the collar gearbox] -> centre shaft at
  1 rev/min.  The centre shaft drives the tourbillon cage directly
  (cage = 60 s) and, through 1:1 bevels (gearbox -> Y rod -> column
  pod -> column shaft -> canister), the seconds pipe.  Minute = /60
  and hour = /12 motion works live inside the canister.
  Inside the cage, the stationary sun crown on the trumpet walks the
  lay-shaft planetary (sun 24T -> lay 9T bevels -> inner 12T): the
  inner carriage turns 5x per cage turn = 1 rev/12 s.  The
  escapement (15-tooth wheel, 2.5 Hz balance with a spherical
  hairspring) rides in the inner carriage.
"""
import numpy as np

# ------------------------------------------------------------- the totem
PLINTH_R = 62.0
PLINTH_H = 15.5
SPH_C = 60.0          # base-sphere centre height
SPH_R = 46.0          # base-sphere radius
SPH_T = 3.0           # shell thickness of solid panels
RIB_R = 2.2           # meridian rib tube radius
RIB_LONS = [30, 90, 150, 210, 270, 330]   # none at 0 (front) / 180 (back)
RIB_LAT = (-52.0, 56.0)                   # ribs span cap to collar
HOOP_LATS = [-28.0, 28.0]

TOURB_C = np.array([0.0, 0.0, 152.0])     # tourbillon sphere centre
CAGE_R = 36.0         # outer cage ring radius
CAGE_TUBE = 2.6
EQ_TUBE = 2.4         # equator ring (carries the carriage bosses)
MERIDIAN_LONS = [45.0, 135.0]             # cage ring planes (deg about Z)
INNER_R = 21.5        # inner carriage ring radius
INNER_TUBE = 2.4
BAL_R = 12.0          # balance rim outer radius
BAL_RIM_W = 2.1
BAL_T = 2.2
BAL_Y = 5.1           # balance plane offset along the staff (inner frame)
DOME_Y0, DOME_Y1 = 6.6, 12.4              # spherical hairspring span
DOME_R_OUT, DOME_R_IN, DOME_TURNS = 8.0, 2.4, 3.5
ESC_POS = 12.0        # escape-wheel centre along the carriage bar
ESC_TIP_R = 5.0
ESC_ROOT_R = 3.8
ESC_T = 15            # teeth
LEVER_POS = 5.4

# stalk / trumpet / sun / cage pipe (world Z unless noted)
STALK_Z0 = 102.0      # vase foot on the collar
TRUMPET_Z1 = 119.4    # trumpet rim (sun crown sits on it)
SUN_R = 8.2
PIPE_R = 5.0          # cage bottom pipe outer radius

# lay shaft (planetary rod riding on the outer cage), cage-local coords;
# the cage meridian rings live at 45/135 deg so the XZ plane is free
LAY_A = np.array([-8.0, 0.0, -31.4])      # lower bevel (at the sun)
LAY_B = np.array([-27.2, 0.0, -6.2])      # upper bevel (at inner pivot)
LAY_ROD_R = 0.9

# ------------------------------------------------------------------ dial
DIAL_C = np.array([0.0, 2.0, 250.0])
DIAL_TILT = 12.0      # deg, top leans back
DIAL_OR = 52.0        # chapter ring outer radius
DIAL_IR = 39.0        # chapter ring inner radius (open centre)
DIAL_T = 3.0
DIAL_HUB_R = 12.0
HAND_MIN_L = 47.5
HAND_HOUR_L = 31.0
HAND_SEC_L = 50.0

def dial_normal():
    """Unit normal of the dial face (points at the viewer, tilted up)."""
    a = np.radians(DIAL_TILT)
    return np.array([0.0, -np.cos(a), np.sin(a)])

def dial_up():
    a = np.radians(DIAL_TILT)
    return np.array([0.0, np.sin(a), np.cos(a)])

# canister (motion works drum) behind the dial
CAN_R = 12.5
CAN_D = 15.0
CAN_GAP = 6.0         # dial back face -> canister front face

# ---------------------------------------------------------------- column
POD_C = np.array([0.0, 29.5, 98.5])       # bevel pod on the collar
RELAY_C = np.array([0.0, 46.0, 158.0])    # swan-neck bevel knuckle
COL_X = 5.0           # twin rods at x = +/- COL_X
COL_ROD_R = 2.2

# ------------------------------------------------------- base mechanism
BARREL_C = np.array([0.0, 3.0, SPH_C])    # drum axis along Y
DRUM_R = 27.0
GBOX_C = np.array([0.0, 3.5, 99.0])       # collar gearbox (encloses 48:1)
GBOX_R, GBOX_H = 6.0, 11.0
TRANSFER_C = np.array([0.0, -6.3, 94.2])  # transfer arbor (along Y);
                                          # 34.2 = m1.8*(30+8)/2 mesh dist
KEY_Y0 = 40.0         # winding key frame origin (along +Y, back porthole)

# ------------------------------------------------------------ animation
CAGE_PERIOD = 60.0    # s per outer-cage rev (= seconds line, 1 rpm)
INNER_RATIO = 5.0     # inner-carriage revs per cage rev (12 s)
LAY_RATIO = 8.0       # lay-shaft revs per cage rev (visual)
BEAT_DEG = 360.0 / ESC_T / 2.0            # escape step per beat (12 deg)
START_HMS = (10, 9, 30)                   # hands at first frame
