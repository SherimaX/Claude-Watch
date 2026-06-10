"""Layout of the spherical-tourbillon table clock, in millimetres.

Coordinate system: Z up, the viewer (front) is at -Y.  The clock is ONE
armillary sphere that visibly rotates, floating over a pedestal, with
the dial hovering IN FRONT of it:

      dial ring (tilted 12 deg, floating at -Y, the sphere behind it)
        |            ... twin front columns carry the canister; the
        |                seconds line spins up a front bevel shaft
      armillary tourbillon sphere (the WHOLE sphere = the rotating
        |    outer cage, 60 s/rev; inner carriage somersaults, 12 s)
        |            ... stalk + trumpet with the stationary sun crown
      pedestal (openwork drum; mainspring barrel inside, porthole)
        |
      plinth

Every rotating part is modelled in its own "spin frame" (+Z = its
rotation axis, origin = its pivot) and placed by a rigid kinematic
chain (parts.CHAIN), so the same data drives the STL export, the
clearance checks and the animations.

Mechanical story (ratios are real, one reduction is enclosed):
  barrel (1 rev/3 h) -> drum ring gear 30T -> transfer pinion 8T
  -> [48:1 reduction inside the collar gearbox] -> centre shaft at
  1 rev/min.  The centre shaft drives the whole armillary sphere
  directly (sphere/cage = 60 s) and, through 1:1 bevels (gearbox ->
  Y rod -> front pod -> column shaft -> canister), the seconds pipe.
  Minute = /60 and hour = /12 motion works live inside the canister.
  Inside the sphere, the stationary sun crown on the trumpet walks the
  lay-shaft planetary (sun 24T -> lay 9T bevels -> inner 12T): the
  inner carriage turns 5x per sphere turn = 1 rev/12 s.  The
  escapement (15-tooth wheel, 2.5 Hz balance with a spherical
  hairspring) rides in the inner carriage.
"""
import numpy as np

# ------------------------------------------------------------- the totem
PLINTH_R = 48.0
PLINTH_H = 15.5

# pedestal (openwork drum housing the mainspring barrel)
PED_BASE_R = 31.0     # foot disc radius
PED_WALL_R = 24.0     # side wall outer radius
PED_WALL_T = 3.0
PED_WALL_Z = (20.0, 57.0)
PED_WALL_LONS = [(-50.0, 50.0), (130.0, 230.0)]   # side panels only:
                                                  # front (-Y) / back open
PED_PLATE_Z = (57.0, 60.5)                        # top plate ring
PED_PLATE_IR, PED_PLATE_OR = 12.0, 31.0
PORT_F_R = 14.0       # front porthole bezel (frames the mainspring)
PORT_B_R = 10.0       # back porthole bezel (winding key passes through)

# the sphere = the tourbillon (two-axis flying gyrotourbillon)
TOURB_C = np.array([0.0, 0.0, 118.0])     # sphere centre
CAGE_R = 36.0         # outer cage ring radius
CAGE_TUBE = 2.6
EQ_TUBE = 2.4         # equator ring (carries the carriage bosses)
MERIDIAN_LONS = [45.0, 135.0]             # cage ring planes (deg about Z)
HOOP_LATS = [-28.0, 28.0]                 # gold armillary hoops ON the cage
HOOP_R_OFF = 0.6      # hoops ride just outside the cage radius
HOOP_TUBE = 1.6
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
STALK_Z0 = 68.0       # vase foot on the pedestal collar
TRUMPET_Z1 = 85.4     # trumpet rim (sun crown sits on it)
SUN_R = 8.2
PIPE_R = 5.0          # cage bottom pipe outer radius

# lay shaft (planetary rod riding on the outer cage), cage-local coords;
# the cage meridian rings live at 45/135 deg so the XZ plane is free
LAY_A = np.array([-8.0, 0.0, -31.4])      # lower bevel (at the sun)
LAY_B = np.array([-27.2, 0.0, -6.2])      # upper bevel (at inner pivot)
LAY_ROD_R = 0.9

# ------------------------------------------------------------------ dial
# the dial floats IN FRONT of the sphere (viewer side); its open centre
# frames the spinning sphere behind the hands
DIAL_C = np.array([0.0, -66.0, 118.0])
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
POD_C = np.array([0.0, -23.0, 63.7])      # bevel pod on the plate rim
COL_X = 15.0          # column feet at x = +/- COL_X on the plinth
COL_ROD_R = 2.2

# ------------------------------------------------------- base mechanism
BARREL_C = np.array([0.0, 0.0, 38.5])     # drum axis along Y
DRUM_M, DRUM_T, TRANS_T = 1.1, 30, 8      # ring gear module / teeth
DRUM_R = 16.5         # drum wall outer radius (gear pitch r = 16.5)
GBOX_C = np.array([0.0, 3.5, 64.2])       # collar gearbox (encloses 48:1)
GBOX_R, GBOX_H = 6.0, 11.0
TRANSFER_C = np.array([0.0, -6.3, 59.4])  # transfer arbor (along Y);
                                          # 20.9 = m1.1*(30+8)/2 mesh dist
KEY_Y0 = 26.0         # winding key frame origin (along +Y, back porthole)

# ------------------------------------------------------------ animation
CAGE_PERIOD = 60.0    # s per sphere rev (= seconds line, 1 rpm)
INNER_RATIO = 5.0     # inner-carriage revs per sphere rev (12 s)
LAY_RATIO = 8.0       # lay-shaft revs per sphere rev (visual)
BEAT_DEG = 360.0 / ESC_T / 2.0            # escape step per beat (12 deg)
START_HMS = (10, 9, 30)                   # hands at first frame
