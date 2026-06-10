"""All dimensions for the 3D-printed mechanical watch, in millimetres.

Coordinate system: movement axis = Z. Back-plate front face is Z=0;
the train lives in Z+ between the plates, the balance/ratchet live in Z-
behind the back plate, the motion works / dial / hands in Z+ in front of
the front plate.  "Front view" = looking at the dial (+Z toward viewer).

Architecture: going-barrel -> center wheel (minutes) -> third -> fourth
-> escape pinion; 20-tooth pin-pallet escape wheel; Roskopf-style lever
with 1.75 mm filament pallet pins; roller + impulse pin on the balance
arbor; balance + printed hairspring behind the back plate.

Train: (48/8)^3 = 216 escape revs per center rev.
Beat: 216 * 20 teeth / 3600 s = 1.2 Hz balance, 2.4 ticks/s.
"""
import numpy as np

# ---------------------------------------------------------------- gearing
MODULE = 0.7          # train + motion works module
PA_DEG = 20.0
BACKLASH = 0.18       # circumferential backlash per mesh (FDM tolerance)
PINION_T = 8
WHEEL_T = 48
PINION_SHIFT = 0.40   # profile shift avoids undercut on 8T pinion
WHEEL_SHIFT = -0.40

MESH_DIST = MODULE * (WHEEL_T + PINION_T) / 2.0      # 19.6 every mesh
WHEEL_TIP_R = MODULE * (WHEEL_T / 2 + 1 + WHEEL_SHIFT)     # 17.2
PINION_TIP_R = MODULE * (PINION_T / 2 + 1 + PINION_SHIFT)  # 3.78

# ------------------------------------------------------------- escapement
ESC_T = 20            # escape wheel teeth (pin-pallet profile)
ESC_R = 12.0          # tip radius
ESC_IMP_DROP = 1.55   # radial fall of tooth impulse face
ESC_IMP_SPAN = 11.0   # impulse face angular span (wheel degrees)
ESC_DRAW_DEG = 14.0   # locking face draw angle (from radial)
ESC_ROOT = 9.2
LEVER_SPAN_T = 2.5    # pallet pins span 2.5 tooth pitches (45 deg)
LEVER_D = ESC_R / np.cos(np.radians(45.0 / 2))       # 12.99 pivot dist
PALLET_PIN_D = 1.75   # filament pins
PIN_LOCK_DEPTH = 0.3  # pin depth inside tip circle at neutral; swing
                      # alternates engagement, windows must overlap
LEVER_SWING = 8.0     # +/- degrees between bankings
FORK_LEN = 11.0       # lever pivot -> fork slot bottom region
ROLLER_R_IP = 4.0     # impulse-pin radius on roller
LB_DIST = FORK_LEN + ROLLER_R_IP   # lever pivot to balance = 15.0
FORK_SLOT_W = 2.10
ROLLER_MAIN_R = 5.3
ROLLER_SAFETY_R = 3.2
GUARD_W = 1.2

# ------------------------------------------------------------- positions
def _pol(r, deg):
    a = np.radians(deg)
    return np.array([r * np.cos(a), r * np.sin(a)])

P_CENTER = np.array([0.0, 0.0])
P_BARREL = _pol(MESH_DIST, 90.0)
P_THIRD  = _pol(MESH_DIST, 215.0)
P_FOURTH = P_THIRD + _pol(MESH_DIST, -34.0)
P_ESCAPE = P_FOURTH + _pol(MESH_DIST, 15.0)
P_LEVER  = P_ESCAPE + _pol(LEVER_D, 90.0)
P_BALANCE = P_LEVER + _pol(LB_DIST, 80.0)
P_MINUTE = _pol(21.0, 128.4)          # minute-wheel stud (front side)

PILLAR_ANGLES = [60.0, 150.0, 240.0, -18.0]
PILLAR_R = 37.0
PILLAR_RAD = 3.0
DIAL_FEET_ANGLES = [30.0, 170.0, 210.0, 300.0]
DIAL_FEET_R = 33.0

PLATE_R = 40.0
PLATE_T = 3.0
FPLATE_T = 2.5

# ---------------------------------------------------------------- Z stack
WHEEL_TH = 1.8
Z_THIRD_W   = (0.6, 2.4)
Z_FOURTH_P  = (0.2, 2.8)
Z_CENTER_W  = (3.0, 4.8)
Z_THIRD_P   = (2.6, 5.2)
Z_FOURTH_W  = (5.2, 7.0)
Z_ESC_P     = (4.8, 7.4)
Z_ESC_W     = (7.8, 9.6)
Z_LEVER     = (9.8, 11.2)    # lever body rides ABOVE the wheel plane
PALLET_PIN_Z = (7.6, 11.2)   # filament pins glued in lever, dip into wheel
Z_GUARD     = (8.4, 9.8)     # guard pin on lever underside
Z_SAFETY    = (8.4, 9.4)     # safety roller disc (guard plane)
Z_ROLLER    = (11.4, 12.6)   # main roller disc; impulse pin hangs down
IMP_PIN_Z   = (9.7, 12.6)    # impulse pin through fork-slot plane
Z_BARREL_G  = (5.6, 8.0)
Z_CENTER_P  = (5.2, 8.4)
Z_DRUM      = (5.6, 13.8)
PILLAR_H    = 14.6
# front side (front plate top = PILLAR_H + FPLATE_T = 17.1)
Z_CANNON_G  = (17.6, 19.4)
Z_MINUTE_W  = (17.6, 19.4)
Z_MINUTE_P  = (19.6, 22.0)
Z_HOUR_W    = (20.2, 22.0)
HOUR_PIPE_TOP = 25.0
DIAL_Z      = 23.2
DIAL_T      = 1.2
CANNON_PIPE_TOP = 28.0
# back side
Z_RATCHET   = (-8.8, -7.8)   # deep: meshes the crown wheel spur
Z_BAL_RIM   = (-5.2, -3.4)
Z_HSPRING   = (-7.4, -5.6)
Z_COCK      = (-10.0, -7.8)

# ------------------------------------------------------------ arbors/pins
PIN_D = 1.75           # pivots are pieces of 1.75 mm filament
PIN_HOLE_BEAR = 2.0    # bearing hole in plates (running fit)
PIN_HOLE_PRESS = 1.6   # press-fit hole in wheel hubs
HUB_R = 3.2

CENTER_PIPE_OD = 4.0
CENTER_PIPE_ID = 2.0
CENTER_STUB_D = 2.5
CANNON_BORE = 3.55     # friction fit over 3.8 slotted pipe tip
HOUR_PIPE_OD = 7.0
HOUR_PIPE_ID = 4.6

BAL_ARBOR_D = 3.0
BAL_PIVOT_D = 2.0
BAL_SQUARE = 2.8       # across flats; mating square holes 2.9
BAL_RIM_OD = 30.0
BAL_RIM_ID = 26.0
BAL_WEIGHT_HOLES = 8   # M3 holes in rim for tuning nuts

# hairspring (separate flat part, press onto square arbor)
HS_R0 = 3.5
HS_BAND = 0.5
HS_PITCH = 2.0
HS_TURNS = 4.0
HS_H = 1.8

# motion works (12:1), same module
CANNON_T = 12
MINUTE_W_T = 48
MINUTE_P_T = 15
HOUR_W_T = 45
MW_DIST = MODULE * (CANNON_T + MINUTE_W_T) / 2.0     # 21.0 both meshes

# ----------------------------------------------------- keyless works
# Crown at movement +Y (wrist "3 o'clock"; lugs are on +/-X).  Stem axis:
# x=0, z=KW_STEM_Z, along Y.  The stem pinion (6 fins) engages lantern
# bar-rings: pushed -> crown wheel (12T spur -> ratchet = winding);
# pulled 7 mm -> setting wheel (shaft up to a 12T pinion -> minute wheel).
KW_STEM_Z = -4.5
KW_STEM_D = 4.0
KW_PINION_FINS = 6
KW_PINION_OD = 5.0       # fin tip diameter (core 2.8)
KW_PINION_W = 2.0
KW_BAR_R = 4.5           # lantern bar ring radius
KW_BARS = 8
KW_BAR_D = 1.8
KW_CW_SPUR_T = 12        # crown wheel deep spur (meshes ratchet, m=0.7)
                         # mesh dist 0.35*(12+18) = 10.5
KW_CROWN_WHEEL = np.array([KW_BAR_R,
                           19.6 + np.sqrt(10.5 ** 2 - KW_BAR_R ** 2)])
KW_SETTING = np.array([-KW_BAR_R, 35.65])
KW_SET_PIN_T = 12
KW_PUSH_Y = float(KW_CROWN_WHEEL[1])     # pinion centre, pushed (29.09)
KW_PULL_Y = float(KW_SETTING[1])         # pinion centre, pulled (36.09)
KW_TRAVEL = KW_PULL_Y - KW_PUSH_Y        # 7.0
POCKET_FLOOR = -1.7      # back-plate thinned over the keyless works
# z-bands (back side, negative z).  The two lantern wheels are fully
# z-separated: crown wheel anchors on its deep spur, the setting wheel's
# anchor disc sits one level lower, inside a recess in the case back.
Z_CW_SPUR  = (-8.8, -7.8)    # crown wheel deep gear (meshes ratchet)
Z_CW_FLANGE = (-7.8, -7.35)  # bar anchor flange (r=5.3)
Z_BARS     = (-7.35, -2.7)   # crown-wheel lantern bars
Z_SET_DISC = (-9.9, -9.2)    # setting anchor disc (in case-back recess)
Z_SET_BARS = (-9.2, -2.7)
KW_FLANGE_R = 5.3
Z_SET_PINION = (17.4, 19.6)  # setting shaft top pinion

# ---------------------------------------------------------------- barrel
DRUM_OR = 14.2
DRUM_IR = 12.5
ARBOR_CORE_R = 3.5
MS_BAND = 1.3
MS_H = 5.2
BARREL_ARBOR_BEAR_D = 5.0
BARREL_SQ = 4.0
RATCHET_T = 18        # now a m=0.7 spur gear (driven by crown wheel,
RATCHET_R = 6.65      # held by the click); tip radius

# ---------------------------------------------------------------- case
CASE_ID = 83.6
CASE_OD = 88.6
LUG_W = 24.0

TOL = 0.25
