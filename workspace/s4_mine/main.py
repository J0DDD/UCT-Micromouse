# =========================================================================
# Copy of Takudzwa Masendeke's Submission 4 code (workspace/final_task/main.py,
# shared with permission), with the HW_ hardware profile re-calibrated for
# THIS mouse. Every changed line is marked [this mouse] and keeps the
# friend's original value in its comment. Nothing else is modified.
# =========================================================================
# =========================================================================
# EEE3097S Micromouse - Submission 4: Search, Celebrate & Sprint
#
# Author : Takudzwa Masendeke
# Date   : 19 October 2026
#
# AI assistance: developed with help from Gemini Pro 3.1 and Claude.
#
# Phase 1  floodfill exploration; walls classified on the move from the
#          three ToF sensors; corridor centring by the 2-state [y, psi]
#          Kalman filter from the GA3 report (side ToF + gyro + encoders).
# Phase 2  first 2x2 block with all four interior walls seen open is the
#          target room: stop, 360 deg clockwise pirouette on the gyro.
# Phase 3  turn-weighted Dijkstra over KNOWN-open walls back to (0,0),
#          then face the start heading ("North").
# Phase 4  3.0 s pause, then the same planner from (0,0) to the room;
#          straights merged into one accel-cruise-decel profile each.
#
# One file for co-simulator and board: on_simulator() picks the profile,
# so the FNV-1a hash stays valid against the physical run log. Only the
# HW_ constants affect the real mouse.
#
# Frames: the map grid is in "mouse frame" - x along the start heading,
# y to the arena side. Dir 0 = start heading (+x, called North in the
# brief), 1 = +y, 2 = -x, 3 = -y. Headings in degrees, CCW positive.
# =========================================================================
import uct_mouse

BUILD = "v71j CENTRE 31-Oct"

try:
    import math
    _sqrt = math.sqrt
    _sin = math.sin
    _cos = math.cos
except ImportError:
    def _sqrt(x):
        if x <= 0.0:
            return 0.0
        g = x if x > 1.0 else 1.0
        for _ in range(20):
            g = 0.5 * (g + x / g)
        return g

    def _sin(x):
        while x > 3.14159265:
            x -= 6.28318531
        while x < -3.14159265:
            x += 6.28318531
        x2 = x * x
        return x * (1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0 * (1.0 - x2 / 72.0))))

    def _cos(x):
        return _sin(x + 1.57079633)

# ---------------- hardware profile (this mouse) --------------------------
HW_MOTOR_POL_L    = 1             # flipped for the Oct-2026 MicroPython firmware
HW_MOTOR_POL_R    = 1
HW_ENC_POL_L      = 1           # clean MicroPython (diag66): left counts + forwards. The first drive still checks it
HW_ENC_POL_R      = 1
HW_LEFT_ENC_OFF   = 0           # [this mouse] left encoder works (spin test: encoders 1062 vs gyro 1084 deg). Friend's: 1 (faulty encoder)
HW_LREV_SUB       = 0           # [this mouse] left encoder counts backwards fine (spin test). Friend's: 1
HW_SPIN_TRIM0_CW  = 5.0         # v71i: right spins (right wheel backwards is the faster one: demo right turns came out -11)
HW_SPIN_TRIM0     = 8.0         # v71h: spincheck - left wheel ~20-25% stronger: the right needs ~+8 to match in spins (+6 left every turn 9-24 mm left)
HW_ENC_FIX        = 1           # v65: undo wrong-way encoder counts with the gyro (v64: left encoder counted one way only)
HW_LEFT_DEADBAND  = 12.0        # [this mouse] motors() sweep, linear fit over 45-170 mm/s. Friend's: 20
HW_RIGHT_DEADBAND = 27.0        # [this mouse] motors() sweep (right wheel barely moves below ~30 PWM). Friend's: 15
HW_TICKS_PER_M    = 5670.0      # [this mouse] calibrate.ticks(): 5658 / 5678 ticks pushed over 1.000 m. Friend's: 5709
HW_GYRO_GAIN      = 1.0         # measured: 360 deg by hand reads +360 (0 = auto-measure at start)
HW_CAL_SAMPLES    = 150         # start-up gyro samples (mouse still)
HW_VMAX           = 601.0       # [this mouse] LEFT: linear-fit gain 6.8 mm/s per PWM x (100-12) (real top speed ~315 mm/s). Friend's: 498
HW_TAU            = 0.06        # s, motor lag used for accel feed-forward
HW_MAZE_ROWS      = 4           # lab maze: 4 cells to the side (0 = ask uct_mouse.get_maze_dimensions(); the board says 6x6)
HW_MAZE_COLS      = 6           # ... and 6 along the start heading (East). Known outer walls: a misread there cannot send it out
HW_ARENA_SIDE     = 0           # 0 = detect from the first side opening (+1 maze on the LEFT of the start heading, -1 RIGHT)
HW_SIDE_X         = 50.0        # mm side ToF ahead of the axle
HW_FRONT_OFF      = 57.0        # mm front ToF ahead of the axle (info only: the code uses CELL/2 - HW_FRONT_CENTER)
HW_WALL_K         = 0.0         # deg/s of steering per mm off-centre, straight from side ToF
HW_GYRO_TRIM      = 0.0         # deg/s added to gyro bias. Drifts RIGHT -> make it NEGATIVE (see straight test)
HW_TEST_STRAIGHT  = 0           # >0: just drive this many cells straight and stop (calibration)
HW_TEST_TURN      = 0           # >0: in the start cell do this many 90 deg turns left, then right, print each
HW_CALIBRATE      = 0           # 1 = calibration run, prints the values to paste below; 2 = also side-sensor offset
HW_RIGHT_SCALE    = 1.00        # straight driving: right wheel power x this, as v53. Ends up left -> 0.97
HW_KPSI           = 8.0         # heading loop (1/s), as v53 (the only settings that completed mazes). Wobbles -> 6
HW_VMAX_R         = 596.0       # [this mouse] RIGHT: 8.15 mm/s per PWM x (100-27) (real top ~295 mm/s). Friend's: 412
HW_TURN_LATFIX    = 1           # 1 = after every turn, set the sideways position from still side readings before driving
HW_PSI_MAX        = 0.10        # rad (~6 deg): max angle it steers to recentre. v69: 4 deg could not beat a 5-8 deg heading error (v67)
HW_RETURN_STOPS   = 1           # v71: kept ON - continuous returns caused 4 of 9 emulator failures; v70's stopped return had the best corners on the real mouse
HW_STOP_LAT       = 1           # v69: at every new-cell stop, set the sideways position from the still side medians and correct the gyro heading from the walls
HW_KLAT           = 0.010       # rad heading per mm off-centre (0.007 tested worse)

HW_SIDE_OFFSET    = 0.0         # [this mouse] not measured reliably (L/R offsets drifted between sessions). Friend's: 2.0

HW_STOP_EACH_CELL = 1           # 1 = STABLE: stop on every cell centre and sense walls standing still (v53 finished mazes this way)

HW_SIDE_GUARD_L   = 40.0  # v71b: steer away from the left wall from 40 mm (was 35)
HW_SIDE_GUARD     = 40.0  # v71b: same on the right

HW_FRONT_STOP     = 30.0        # mm: front reflex - blocked below this + speed * HW_FRONT_REACT (centre reads 37 now)
HW_FRONT_REACT    = 0.15        # s: ToF latency + braking time, so faster = stop earlier
HW_FRONT_CHECK    = 40  # v67: longer still look after each turn (v66: 30)
HW_TURN_LEAD      = 0.06        # s: brake on the predicted stopping heading (motor lag), stops overshoot
HW_TURN_KLIN      = 10.0        # 1/s: gentle linear approach for the last ~20 deg (no chatter at the end)
HW_TURN_WMIN      = 20.0        # deg/s: never command less while outside tolerance (beats stiction)
HW_ALIGN_MOVE     = 1           # 1 = align to the front wall with ONE encoder move from a median reading
HW_ARCS           = 0           # 0 = return/sprint stop and turn on the spot at corners (safest)
HW_KEEP_IW        = 1           # 1 = keep the learned L/R motor-imbalance trim across stops (anti-veer)
HW_STOP_FIX       = 1           # 1 = at each cell stop, a wall seen ahead (<500 mm) fixes the along position
# Real chassis: 105 mm long, front ToF 65 mm ahead of the axle, so turning on the
# spot the front corners sweep ~86 mm round the axle against ~94 mm to a wall.
HW_PIVOT_BACK     = 0.0         # mm short of centre before turning (backing off put the axle off-centre after the turn: left wheel caught the corner)
HW_REVERSE_OUT    = 0           # 0 = 180 spin in dead ends (reversing is blind: drifted right into walls behind)
HW_REV_HOME       = 0           # 0 = drive into the start forwards, then spin to face the start heading
HW_SPIN_SHIFT     = 8.0         # mm step off a wall before a risky spin (15 left it off-centre into the next corridor)
HW_V_REVERSE      = 100.0       # mm/s top speed backwards (slow: more side-wall fixes per mm)
HW_SENSE_STEPS    = 80          # 10 ms steps of standing-still wall sensing per NEW cell (v53: 40, v64: 60)
HW_SKIP_SEEN      = 1           # 1 = cells already mapped: no wall sensing, just a quick front fix
HW_WALL_STOP      = 0           # 0 = stop on the position estimate (the near front wall corrects it while approaching)
HW_ZUPT_STOP      = 1           # 1 = re-estimate the gyro bias while standing still (both wheels dead still)
# Flight log (real mouse): spinning on the spot the wheels STALL at PWM 64-70 and
# spin fast at 75; from rest the right wheel starts later than the left.
HW_TURN_BB        = 1           # 1 = spin turns at a fixed PWM, cut early, coast onto the heading
HW_TURN_PWM       = 48  # v67: slower spins, less overshoot/slide (v66: 52; wheels start ~42-46)
HW_TURN_COAST     = 0.045       # s: coast per deg/s after cutting (diag: ~7 deg at 200 deg/s); learned each turn

HW_KICK_L         = 0           # start kick off: per-wheel speed control starts both wheels together (diag.py had none)
HW_KICK_R         = 0           # 

HW_STOP_CREEP     = 45.0        # mm/s: slowest speed into a stop (70 arrived too hot); stop swing is fixed by HW_STRAIGHTEN
HW_BRAKE_HOLD     = 0           # 1 = gyro-steered roll-out at stops (tested worse in emulation: off)
HW_STRAIGHTEN     = 0.0         # deg: 0 = off (correction spins after stops looked like wiggling)
HW_EXPLORE_ALL    = 0           # 0 = brief: pirouette as soon as it is in the confirmed room (1 = map the whole maze first)
HW_EXPLORE_TMAX   = 200.0       # s: ... but stop exploring after this long once the room is known
HW_STOP_EXTRA     = 0.0         # mm past the cell centre at every stop (8 was making up for the bad far-wall fixes)
HW_WHEEL_PI       = 1           # 1 = each wheel on its OWN encoder speed loop (diag.py's controller: straight, square stops)
HW_WHEEL_KP       = 0.12        # PWM per mm/s of wheel speed error (diag.py)
HW_WHEEL_KI       = 0.8         # PWM per mm of accumulated wheel speed error (diag.py)
HW_LOOP_FIXED     = 1           # 1 = fixed 10 ms loop period (wait only the rest of the period)
HW_FRONT_CENTER   = 48.0        # [this mouse] centred front readings 45-55 mm (calibrate.tof). Friend's: 37
HW_FRONT_NEAR     = 120.0       # mm: front readings set the position only below this (longer ones read 15-30 short)
HW_FRONT_GAIN     = 0.12        # share of each near front reading's error applied while driving towards the wall
HW_POS_MODEL      = 1           # 1 = keep an along/sideways uncertainty; every position fix is weighted by it
HW_EDGE_FIX       = 1           # 1 = a side wall ending/starting (cell boundary) fixes the along position on the move
HW_EDGE_OFF       = 18.0        # mm: start value - a wall END is seen this far past the boundary, a START this far before (learned: "p" in edge lines; v46 run: 17-19)
HW_SIDE_NOM       = 55.0        # [this mouse] start-cell side references 49-69 mm, typically ~55. Friend's: 52
HW_NOM_L          = 0.0         # mm LEFT reading at the TRUE centre, from workspace/centre_test.py (0 = measure at start, re-learn)
HW_NOM_R          = 0.0         # mm RIGHT reading at the TRUE centre (set both, or neither)
HW_TOF_LAG        = 0.02        # s: age of a ToF reading (the mouse has moved speed x lag since)
HW_ALIGN_TOL      = 3.0         # mm: before a turn, move to the cell centre if further off than this
HW_ON_CONTACT     = 0           # v71j: 0 = back off and drive on after a bump (the demo allows recovering); at most 6 per run
HW_POST_BLANK     = 30.0        # mm: ignore side readings while the side sensor is this close to a cell boundary (posts, wall ends)
HW_SPIN_MATCH     = 1           # 1 = spin with EQUAL wheel speeds (right motor ~25% weaker: equal PWM shifted the axle left on every turn)
HW_SPIN_TRIM_MAX  = 12.0  # v71d: 12 (learned only from right spins now; start +6)
HW_TURN_PWM2      = 48  # v67: correction spins as slow (v66: 52)
HW_PIVOT_LEARN    = 1           # v66: learn per spin direction where to line up so the axle ENDS on the centre (v65: 3-17 mm left after every left turn)
HW_PIVOT_MAX      = 4.0           # v71e: 4 (a -6 line-up put a right spin ~5 mm past the centre before a sprint crash)
HW_PSI_TURN       = 0.105       # rad (~6 deg): max recentring steer for the first 150 mm after a turn (off tested worse)
HW_TILT_LIM       = 0.25        # climb detector: accel tilt as a fraction of g (0 = off)
HW_SIDE_WALL      = 100.0       # mm: walls read 20-70, open sides 140+ (measured)
HW_FRONT_FIX_MAX  = 150.0       # mm: trust front ToF for distance only below this
HW_FRONT_SNAP     = 30.0        # mm: front fix only if within this of a wall line
HW_TOF_SIGMA      = 20.0        # mm side ToF noise (tof_check: spread ~+-20 standing still)
HW_V_EXPLORE      = 75.0  # v71b: back to v70's 75 for the demo runs
HW_V_RETURN       = 90.0  # v67: slower for rigour (v66: 120)
HW_V_SPRINT       = 130.0  # v71: slower (v70: 160); every v70 failure was in the sprint, still 1.5x explore
HW_V_TURN         = 170.0       # mm/s through smooth 90 deg arcs
HW_ACCEL          = 300.0  # v67: gentler starts/stops (v66: 400)
HW_W_TURN         = 180.0       # deg/s in-place turn ceiling
HW_ALPHA_TURN     = 1000.0      # deg/s^2
HW_TURN_TOL       = 3.0         # deg: accept the spin within this (1.5 caused extra back-and-forth corrections)
HW_ROOM_DOORS     = 8           # max openings in the target room's rim (8 = any: the brief allows arbitrary openings)
HW_KP_V           = 0.10        # effort per mm/s (common mode)
HW_KI_V           = 0.6
HW_KP_W           = 0.08        # effort per deg/s (gyro rate loop; was 0.06 - snappier heading hold)

HW_KI_W           = 0.3         # was 0.5: with the big right dead-band it over-corrected

HW_WAIT_BUTTON    = 1           # 1 = press SW1 to start (board only)
HW_DEBUG          = 1           # 1 = print sensing/decisions (run tethered via mpremote)
HW_TPMM_AUTO      = 0           # 1 = measure encoder ticks/mm at start (spin vs gyro)
HW_KF_HEADING     = 0           # 0 = heading from the gyro only (diag: gyro-only straight was perfect; walls caused bends)
HW_SYSID          = 0           # 1 = measure dead-bands/ticks per mm with a start spin

# ---------------- runtime values (simulator defaults) --------------------
CELL = 200.0
DT_MS = 10
IS_SIM = 1
MOT_POL_L = 1
MOT_POL_R = 1
ENC_POL_L = 1
ENC_POL_R = 1
ENC_FIX = 0
DB_L = 40.0                     # first guesses only - SYSID measures them
DB_R = 40.0
TPMM = 5.0                      # encoder ticks per mm
GYRO_GAIN = 1.0
TPMM_AUTO = 0
GYRO_AUTO = 0                   # 1 = measure gyro sign/scale at start (board default)
CAL_SAMPLES = 150
VMAX = 400.0
VMAX_L = 400.0                  # mm/s per wheel at PWM 100 (identified in sim)
VMAX_R = 400.0
TAU = 0.06
SYSID = 1                       # identify dead-bands, wheel gains, ticks/mm at start
MAZE_ROWS = 0
MAZE_COLS = 0
SIDE = 1
SIDE_AUTO = 0
SIDE_X = 50.0
FRONT_OFF = 65.0
SIDE_WALL = 150.0
FRONT_FIX_MAX = 2000.0
TOF_SIGMA = 6.0
V_EXPLORE = 260.0
V_RETURN = 320.0
V_SPRINT = 360.0
V_TURN = 215.0                  # mm/s through arcs (outer wheel = 1.54x)
ACCEL = 2000.0
W_TURN = 330.0
ALPHA_TURN = 3000.0
TURN_TOL = 1.0
KP_V = 0.10
KI_V = 0.8
KP_W = 0.12
KI_W = 1.2
HALF_AXLE = 54.0                # mm
NOM_L = 60.0                    # centred reading of each side sensor (measured at start)
NOM_R = 60.0
SIDE_NOM = 60.0                 # mm side reading when centred (re-measured at start)
ROOM_MAX_DOORS = 1              # openings in the room's rim (sim generator: always 1)
WALL_K = 0.0                    # direct side-wall steering (board)
GYRO_TRIM = 0.0
TEST_STRAIGHT = 0
STOP_EACH = 0
KF_HEADING = 1
FRONT_STOP = 0.0                # sim never needs the reflexes; board values above
FRONT_REACT = 0.0
FRONT_CHECK = 0
TURN_LEAD = 0.0                 # 0/0/0 = the sim's sqrt turn profile
TURN_KLIN = 0.0
TURN_WMIN = 0.0
ALIGN_MOVE = 0
ARCS = 1
KEEP_IW = 0
STOP_FIX = 0
PIVOT_BACK = 0.0
REVERSE_OUT = 0
REV_HOME = 0
SPIN_SHIFT = 0.0
V_REVERSE = 150.0
SENSE_STEPS = 40
SKIP_SEEN = 0
WALL_STOP = 0
ZUPT_STOP = 0
TURN_BB = 0
TURN_PWM = 80
TURN_COAST = 0.08
KICK_L = 0
KICK_R = 0
LOOP_FIXED = 0
WHEEL_PI = 0
STOP_EXTRA = 0.0
STOP_CREEP = 40.0
STRAIGHTEN = 0.0
BRAKE_HOLD = 0
EXPLORE_ALL = 0
SIDE_OFFSET = 0.0
FRONT_CENTER = 35.0             # sim: 100 - FRONT_OFF
FRONT_NEAR = 2000.0
POS_MODEL = 0                   # board: weighted along/sideways fixes (sim keeps its own filter)
NOM_FIXED = 0                   # board: 1 = HW_NOM_L/R measured, never re-learned
EDGE_FIX = 0                    # board: side-wall edge fixes
TILT_LIM = 0.0
SIDE_GUARD = 0.0                # mm: steer away below this (board only; sim never needs it)
DECIDE_AHEAD = 60.0             # decide a cell this far before its centre
FRONT_SNAP = 45.0
FRONT_GAIN = 0.15
KLAT = 0.008                    # rad of heading demand per mm of offset
PSI_MAX = 0.14                  # rad
KPSI = 8.0                      # 1/s heading loop
ARC_KR = 4.0                    # deg/s per mm of radius error in arcs
ARC_KH = 8.0                    # 1/s on heading error to the arc tangent
DEBUG = 1

DX = (1, 0, -1, 0)
DY = (0, 1, 0, -1)
D2R = 0.0174532925


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def fabs(x):
    if x < 0.0:
        return -x
    return x


# Feed-forward PWM for a wheel speed v (mm/s): dead-band + v / gain, with the
# gain from VMAX = gain * (100 - dead-band) (diag.py's pwm_for).
def wheel_ff(v, db, vmax):
    kv = vmax / (100.0 - db)
    if v > 0.5:
        return db + v / kv
    if v < -0.5:
        return -db + v / kv
    return 0.0


def med3(a, b, c):
    if a > b:
        a, b = b, a
    if b > c:
        b = c
    return a if a > b else b


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


LOGBUF = None                   # board: log lines not yet written to runlog.txt
SAVE_DUE = 0                    # 1 = enough lines (or a BUMP/STALL) to write at the next stop
SAVE_OK = 1                     # 0 = no writes now (sprint); everything is written at the end


def log(*args):
    if DEBUG:
        print(*args)
        if LOGBUF is not None:
            LOGBUF.append(" ".join([str(a) for a in args])[:110])
            if len(LOGBUF) > 150:
                del LOGBUF[1]           # only if it could not write for very long
            global SAVE_DUE
            if len(LOGBUF) >= 25 or "BUMP" in LOGBUF[-1] or "STALL" in LOGBUF[-1]:
                SAVE_DUE = 1


# Board: append the pending log lines to runlog.txt on the mouse's drive (plug
# in USB afterwards and open UCT_MMOUSE/runlog.txt). Writing flash freezes the
# board for ~1 s and loses encoder counts, so this only ever runs standing
# still (settle(), mission end) - v49 wrote mid-drive, lost ~140 mm of
# odometry and stopped between two cells. Never lets an error escape.
def save_log():
    global SAVE_DUE
    SAVE_DUE = 0
    if not LOGBUF:
        return
    try:
        f = open("runlog.txt", "a")
        for line in LOGBUF:
            f.write(line)
            f.write("\n")
        f.close()
        LOGBUF.clear()
    except Exception as e:
        print("could not save runlog.txt:", e)


# A safe moment (wheels stopped and still): write the log if it is due.
def save_due():
    if SAVE_DUE and SAVE_OK:
        save_log()


# Raised on the board when the mouse touches something (HW_ON_CONTACT = 1):
# in the demo any contact fails the run, so it stops instead of backing up.
class ContactStop(Exception):
    pass


# 1 in co-simulation, 0 on the board - CPython has os.environ, MicroPython does not.
def on_simulator():
    try:
        import os
        os.environ
        return 1
    except AttributeError:
        return 0
    except Exception:
        pass
    return 0


def select_profile():
    global IS_SIM, MOT_POL_L, MOT_POL_R, ENC_POL_L, ENC_POL_R, ENC_FIX, DB_L, DB_R, TPMM
    global GYRO_GAIN, CAL_SAMPLES, VMAX, TAU, MAZE_ROWS, MAZE_COLS, SIDE, SIDE_X
    global FRONT_OFF, SIDE_WALL, FRONT_FIX_MAX, TOF_SIGMA, V_EXPLORE, V_RETURN
    global V_SPRINT, V_TURN, ACCEL, W_TURN, ALPHA_TURN, TURN_TOL, KP_V, KI_V, KP_W, KI_W
    global DEBUG, FRONT_STOP, FRONT_CENTER, TILT_LIM, KF_HEADING, STOP_EACH, WALL_K, GYRO_TRIM, TEST_STRAIGHT, SIDE_AUTO, SIDE_GUARD, TPMM_AUTO, FRONT_SNAP, GYRO_AUTO, ROOM_MAX_DOORS, VMAX_L, VMAX_R, SYSID
    global FRONT_REACT, FRONT_CHECK, TURN_LEAD, TURN_KLIN, TURN_WMIN, ALIGN_MOVE, ARCS, KEEP_IW, STOP_FIX
    global PIVOT_BACK, REVERSE_OUT, REV_HOME, SPIN_SHIFT, V_REVERSE, SENSE_STEPS, SKIP_SEEN, WALL_STOP, ZUPT_STOP
    global KPSI, KLAT, PSI_MAX, WHEEL_PI, STOP_EXTRA, FRONT_NEAR, POS_MODEL, EDGE_FIX, SIDE_NOM, NOM_L, NOM_R, NOM_FIXED
    global TURN_BB, TURN_PWM, TURN_COAST, KICK_L, KICK_R, LOOP_FIXED, SIDE_OFFSET, STOP_CREEP, STRAIGHTEN, EXPLORE_ALL, BRAKE_HOLD
    IS_SIM = on_simulator()
    if IS_SIM == 1:
        return
    DEBUG = HW_DEBUG
    FRONT_REACT = HW_FRONT_REACT
    FRONT_CHECK = HW_FRONT_CHECK
    TURN_LEAD = HW_TURN_LEAD
    TURN_KLIN = HW_TURN_KLIN
    TURN_WMIN = HW_TURN_WMIN
    ALIGN_MOVE = HW_ALIGN_MOVE
    ARCS = HW_ARCS
    KEEP_IW = HW_KEEP_IW
    STOP_FIX = HW_STOP_FIX
    PIVOT_BACK = HW_PIVOT_BACK
    REVERSE_OUT = HW_REVERSE_OUT
    REV_HOME = HW_REV_HOME
    SPIN_SHIFT = HW_SPIN_SHIFT
    V_REVERSE = HW_V_REVERSE
    SENSE_STEPS = HW_SENSE_STEPS
    SKIP_SEEN = HW_SKIP_SEEN
    WALL_STOP = HW_WALL_STOP
    ZUPT_STOP = HW_ZUPT_STOP
    TURN_BB = HW_TURN_BB
    TURN_PWM = HW_TURN_PWM
    TURN_COAST = HW_TURN_COAST
    KICK_L = HW_KICK_L
    KICK_R = HW_KICK_R
    LOOP_FIXED = HW_LOOP_FIXED
    STOP_CREEP = HW_STOP_CREEP
    STRAIGHTEN = HW_STRAIGHTEN
    BRAKE_HOLD = HW_BRAKE_HOLD
    EXPLORE_ALL = HW_EXPLORE_ALL
    MOT_POL_L = HW_MOTOR_POL_L
    MOT_POL_R = HW_MOTOR_POL_R
    ENC_POL_L = HW_ENC_POL_L
    ENC_POL_R = HW_ENC_POL_R
    ENC_FIX = HW_ENC_FIX
    DB_L = HW_LEFT_DEADBAND
    DB_R = HW_RIGHT_DEADBAND
    TPMM = HW_TICKS_PER_M / 1000.0
    GYRO_GAIN = HW_GYRO_GAIN
    if GYRO_GAIN == 0.0:
        GYRO_GAIN = 1.0
        GYRO_AUTO = 1
    CAL_SAMPLES = HW_CAL_SAMPLES
    VMAX = HW_VMAX
    VMAX_L = HW_VMAX
    VMAX_R = HW_VMAX_R if HW_VMAX_R > 0.0 else HW_VMAX
    SIDE_OFFSET = HW_SIDE_OFFSET
    KLAT = HW_KLAT
    WHEEL_PI = HW_WHEEL_PI
    STOP_EXTRA = HW_STOP_EXTRA
    PSI_MAX = HW_PSI_MAX
    KPSI = HW_KPSI
    SYSID = HW_SYSID
    TPMM_AUTO = HW_TPMM_AUTO
    TAU = HW_TAU
    MAZE_ROWS = HW_MAZE_ROWS
    MAZE_COLS = HW_MAZE_COLS
    SIDE = HW_ARENA_SIDE
    if SIDE == 0:
        SIDE = 1
        SIDE_AUTO = 1
    SIDE_X = HW_SIDE_X
    FRONT_OFF = CELL / 2.0 - HW_FRONT_CENTER    # effective: reading + this = distance to the wall
    FRONT_NEAR = HW_FRONT_NEAR
    POS_MODEL = HW_POS_MODEL
    EDGE_FIX = HW_EDGE_FIX
    # a start cell with one side wall cannot measure the centred reading:
    # the sim's 60 made one-wall centring push the mouse ~9 mm to the open side
    SIDE_NOM = HW_SIDE_NOM
    NOM_L = HW_SIDE_NOM + 0.5 * HW_SIDE_OFFSET * SIDE
    NOM_R = HW_SIDE_NOM - 0.5 * HW_SIDE_OFFSET * SIDE
    if HW_NOM_L > 0.0 and HW_NOM_R > 0.0:
        # measured at the true centre (centre_test.py): use as they are, a
        # start-of-run calibration would absorb an off-centre placement
        NOM_FIXED = 1
        SIDE_NOM = 0.5 * (HW_NOM_L + HW_NOM_R)
        NOM_L = HW_NOM_L if SIDE > 0 else HW_NOM_R      # map frame (SIDE < 0: L/R swapped)
        NOM_R = HW_NOM_R if SIDE > 0 else HW_NOM_L
    SIDE_WALL = HW_SIDE_WALL
    SIDE_GUARD = HW_SIDE_GUARD
    FRONT_STOP = HW_FRONT_STOP
    FRONT_CENTER = HW_FRONT_CENTER
    TILT_LIM = HW_TILT_LIM
    KF_HEADING = HW_KF_HEADING
    STOP_EACH = HW_STOP_EACH_CELL
    WALL_K = HW_WALL_K
    GYRO_TRIM = HW_GYRO_TRIM
    TEST_STRAIGHT = HW_TEST_STRAIGHT
    FRONT_FIX_MAX = HW_FRONT_FIX_MAX
    TOF_SIGMA = HW_TOF_SIGMA
    FRONT_SNAP = HW_FRONT_SNAP
    V_EXPLORE = HW_V_EXPLORE
    V_RETURN = HW_V_RETURN
    V_SPRINT = HW_V_SPRINT
    V_TURN = HW_V_TURN
    ACCEL = HW_ACCEL
    W_TURN = HW_W_TURN
    ALPHA_TURN = HW_ALPHA_TURN
    TURN_TOL = HW_TURN_TOL
    ROOM_MAX_DOORS = HW_ROOM_DOORS
    KP_V = HW_KP_V
    KI_V = HW_KI_V
    KP_W = HW_KP_W
    KI_W = HW_KI_W


# =========================================================================
# MazeSolver: wall map, floodfill exploration, room detection, planning.
# Grid size comes from the arguments only - nothing assumes 4x6.
# =========================================================================
class MazeSolver:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.n = rows * cols
        self.wall = [0] * self.n        # bit d set = wall on side d
        self.known = [0] * self.n       # bit d set = side d has been observed
        self.visited = [0] * self.n
        self.room = None                # list of 4 cell indices once found
        self.weak = []                  # (x, y, d) walls seen by a weak vote, re-checked if no room turns up
        x = 0
        while x < cols:
            self.set_wall(x, 0, 3, 1)
            self.set_wall(x, rows - 1, 1, 1)
            x += 1
        y = 0
        while y < rows:
            self.set_wall(0, y, 2, 1)
            self.set_wall(cols - 1, y, 0, 1)
            y += 1

    def inside(self, x, y):
        return 0 <= x < self.cols and 0 <= y < self.rows

    def set_wall(self, x, y, d, present):
        if not self.inside(x, y):
            return
        i = y * self.cols + x
        b = 1 << d
        if self.known[i] & b and (self.wall[i] & b) and not present:
            return                      # never erase a seen wall (outer walls)
        self.known[i] |= b
        if present:
            self.wall[i] |= b
        else:
            self.wall[i] &= ~b
        nx = x + DX[d]
        ny = y + DY[d]
        if self.inside(nx, ny):
            j = ny * self.cols + nx
            ob = 1 << ((d + 2) & 3)
            self.known[j] |= ob
            if present:
                self.wall[j] |= ob
            else:
                self.wall[j] &= ~ob

    # v71: exploration ended without a room (v70: a 56/80 side vote closed
    # the room's only door). Walls seen by a weak vote that face an unvisited
    # cell become unknown again: the explorer goes back, faces each one and
    # checks it with the front sensor (FRONT CHECK) before driving through.
    def unweak(self):
        n = 0
        for x, y, d in self.weak:
            nx = x + DX[d]
            ny = y + DY[d]
            if self.inside(nx, ny) and not self.visited[ny * self.cols + nx]:
                i = y * self.cols + x
                j = ny * self.cols + nx
                self.known[i] &= ~(1 << d)
                self.wall[i] &= ~(1 << d)
                ob = 1 << ((d + 2) & 3)
                self.known[j] &= ~ob
                self.wall[j] &= ~ob
                n += 1
        self.weak = []
        return n

    def is_open(self, i, d, optimistic):
        b = 1 << d
        if self.wall[i] & b:
            return False
        if self.known[i] & b:
            return True
        return optimistic

    # 2x2 blocks with bottom-left (bx, by): the four interior walls
    def _block(self, bx, by):
        c = self.cols
        i0 = by * c + bx
        return ((i0, 0), (i0, 1), (i0 + 1, 1), (i0 + c, 0))

    # The 8 walls round the outside of the block (side d of cell i).
    def _rim(self, bx, by):
        c = self.cols
        i0 = by * c + bx
        return ((i0, 3), (i0, 2), (i0 + 1, 3), (i0 + 1, 0),
                (i0 + c, 1), (i0 + c, 2), (i0 + c + 1, 1), (i0 + c + 1, 0))

    # Count (open, unknown) walls in a list of (cell, side).
    def _count(self, sides):
        nopen = 0
        nunk = 0
        for i, d in sides:
            b = 1 << d
            if not (self.known[i] & b):
                nunk += 1
            elif not (self.wall[i] & b):
                nopen += 1
        return nopen, nunk

    # Target room = 2x2 block whose 4 interior walls are all seen open and
    # whose 8 rim walls are all seen, with at most ROOM_MAX_DOORS openings.
    # (Braided mazes can contain other open 2x2 areas; they have more doors.)
    def check_room(self):
        if self.room is not None:
            return self.room
        by = 0
        while by < self.rows - 1:
            bx = 0
            while bx < self.cols - 1:
                io, iu = self._count(self._block(bx, by))
                if io == 4:
                    ro, ru = self._count(self._rim(bx, by))
                    if ru == 0 and ro <= ROOM_MAX_DOORS:
                        i0 = by * self.cols + bx
                        self.room = [i0, i0 + 1, i0 + self.cols, i0 + self.cols + 1]
                        log("ROOM found at", bx, by, "doors", ro)
                        return self.room
                bx += 1
            by += 1
        return None

    # Fallback when exploration ended without a confirmed room: the 2x2 block
    # with all 4 interior walls seen open that looks most like the room
    # (most rim walls seen as walls, then fewest openings). None if no block.
    def best_room(self):
        best = None
        bs = -99
        by = 0
        while by < self.rows - 1:
            bx = 0
            while bx < self.cols - 1:
                io, iu = self._count(self._block(bx, by))
                if io == 4:
                    ro, ru = self._count(self._rim(bx, by))
                    sc = 2 * (8 - ro - ru) - ro
                    if sc > bs:
                        bs = sc
                        i0 = by * self.cols + bx
                        best = [i0, i0 + 1, i0 + self.cols, i0 + self.cols + 1]
                bx += 1
            by += 1
        return best

    # Seeds for the exploration floodfill: cells that could complete a
    # plausible room are cheapest. v71: a cell that NO possible room can
    # contain (every 2x2 block round it has an interior wall seen closed, or
    # too many openings) is not a target at all - it is only driven through
    # when it is on the way, so the search ends as soon as the room is the
    # only place left it can be.
    def _explore_seeds(self):
        if self.room is not None and not EXPLORE_ALL:
            return [(i, 0) for i in self.room]
        seeds = []
        best = [9] * self.n
        by = 0
        while by < self.rows - 1:
            bx = 0
            while bx < self.cols - 1:
                nopen, nunk = self._count(self._block(bx, by))
                bad = 0 if nopen + nunk == 4 else 1
                if not bad:
                    ro, ru = self._count(self._rim(bx, by))
                    if ro > ROOM_MAX_DOORS:
                        bad = 1
                if not bad:
                    i0 = by * self.cols + bx
                    cost = 0 if nopen >= 3 else (1 if nopen >= 2 else 2)
                    for i in (i0, i0 + 1, i0 + self.cols, i0 + self.cols + 1):
                        if not self.visited[i] and cost < best[i]:
                            best[i] = cost
                bx += 1
            by += 1
        i = 0
        while i < self.n:
            if not self.visited[i] and best[i] < 9:
                seeds.append((i, best[i]))
            i += 1
        return seeds

    # Multi-source floodfill (Dial's algorithm on unit edges).
    def flood(self, seeds, optimistic):
        INF = 30000
        dist = [INF] * self.n
        buckets = []
        for i, c in seeds:
            if c < dist[i]:
                dist[i] = c
                while len(buckets) <= c:
                    buckets.append([])
                buckets[c].append(i)
        lvl = 0
        while lvl < len(buckets):
            for i in buckets[lvl]:
                if dist[i] != lvl:
                    continue
                x = i % self.cols
                y = i // self.cols
                for d in range(4):
                    if self.is_open(i, d, optimistic):
                        nx = x + DX[d]
                        ny = y + DY[d]
                        if self.inside(nx, ny):
                            j = ny * self.cols + nx
                            if dist[j] > lvl + 1:
                                dist[j] = lvl + 1
                                while len(buckets) <= lvl + 1:
                                    buckets.append([])
                                buckets[lvl + 1].append(j)
            lvl += 1
        return dist

    # Next heading for exploration from (x, y) facing d; -1 when done.
    def next_dir(self, x, y, d):
        seeds = self._explore_seeds()
        if not seeds:
            return -1
        dist = self.flood(seeds, 1)
        i = y * self.cols + x
        best = -1
        bestv = 30000
        for k in (0, 1, 3, 2):          # straight first, then left/right, back last
            nd = (d + k) & 3
            if self.is_open(i, nd, 0):
                nx = x + DX[nd]
                ny = y + DY[nd]
                if self.inside(nx, ny):
                    v = dist[ny * self.cols + nx]
                    if v < bestv:
                        bestv = v
                        best = nd
        if bestv >= 30000:
            return -1
        return best

    # Turn-weighted Dijkstra over KNOWN-open walls. State = cell*4 + heading.
    # Cost: 4 per cell, +2 per 90 deg turn (a smooth arc), +7 for a 180.
    # Returns the heading of every cell-to-cell move, or None.
    def plan(self, x, y, d, goals):
        n4 = self.n * 4
        INF = 30000
        dist = [INF] * n4
        par = [-1] * n4
        s0 = (y * self.cols + x) * 4 + d
        dist[s0] = 0
        buckets = [[s0]]
        goal_state = -1
        lvl = 0
        while lvl < len(buckets) and goal_state < 0:
            for s in buckets[lvl]:
                if dist[s] != lvl:
                    continue
                i = s >> 2
                h = s & 3
                if i in goals:
                    goal_state = s
                    break
                cand = []
                if self.is_open(i, h, 0):
                    nx = i % self.cols + DX[h]
                    ny = i // self.cols + DY[h]
                    if self.inside(nx, ny):
                        cand.append(((ny * self.cols + nx) * 4 + h, 4))
                cand.append((i * 4 + ((h + 1) & 3), 2))
                cand.append((i * 4 + ((h + 3) & 3), 2))
                cand.append((i * 4 + ((h + 2) & 3), 7))
                for t, c in cand:
                    nv = lvl + c
                    if nv < dist[t]:
                        dist[t] = nv
                        par[t] = s
                        while len(buckets) <= nv:
                            buckets.append([])
                        buckets[nv].append(t)
            lvl += 1
        if goal_state < 0:
            return None
        chain = []
        s = goal_state
        while s >= 0:
            chain.append(s)
            s = par[s]
        chain.reverse()
        moves = []
        k = 1
        while k < len(chain):
            if (chain[k - 1] >> 2) != (chain[k] >> 2):
                moves.append(chain[k] & 3)
            k += 1
        return moves


# =========================================================================
# Mouse: sensing, pose estimate, velocity loops and motion primitives.
# =========================================================================
class Mouse:
    def __init__(self):
        self.th = 0.0           # heading, deg, map frame, CCW+
        self.X = CELL / 2.0     # mm, map frame
        self.Y = CELL / 2.0
        self.d = 0              # current axis
        self.cx = 0
        self.cy = 0
        self.el = 0             # encoder counts, wrong-way steps undone (ENC_FIX)
        self.er = 0
        self.rwl = 0            # last encoder readings as counted
        self.rwr = 0
        self.efl = 0            # steps whose count was reversed (left, right)
        self.efr = 0
        self.ef_log = 0
        self.v = 0.0            # measured forward speed mm/s
        self.rate = 0.0         # gyro rate deg/s, map frame
        self.rate_p = 0.0       # physical-frame rate
        self.iv = 0.0
        self.iw = 0.0
        self.bias = 0.0
        self.last_ms = 0
        self.dL = 8190
        self.dC = 8190
        self.dR = 8190
        self.ds = 0.0
        self.dt = 0.01
        self.vcmd_prev = 0.0
        self.wcmd_prev = 0.0
        self.p11 = 100.0        # KF covariance: lateral mm^2
        self.p12 = 0.0
        self.p22 = 0.0003       # heading rad^2
        self.t0 = 0
        self.vc = 0.0           # commanded forward speed (profile state)
        self.nv = [0, 0, 0, 0, 0]   # valid-reading count per ToF slot
        self.dead = 0
        self.wlast = 0.0
        self.hC = 8190
        self.aC = 99
        self.bump = 0
        self.tilt_n = 0
        self.a0 = None
        self.tk = 0
        self.hL = 8190
        self.hR = 8190
        self.aL = 99
        self.aR = 99
        self.el_last = 0
        self.er_last = 0
        self.fC = 8190          # front confirmed by two valid readings (max of the last two)
        self.pC = 8190
        self.rL = [8190, 8190, 8190]    # last three valid side readings
        self.rR = [8190, 8190, 8190]
        self.mL = 8190          # their median: one stray echo cannot kick the steering
        self.mR = 8190
        self.rC = [8190, 8190, 8190]    # last three valid front readings
        self.mC = 8190
        self.rB = [0.0, 1e9, 2e9]       # ... and the wall positions they imply (along axis d)
        self.iw_fwd = 0.0       # learned L/R imbalance trim for straight driving
        self.vote = (0, 0, 0, 0)        # last stationary side vote: (vl, nl, vr, nr)
        self.fmed = 8190        # median front reading from the last stationary vote
        self.fmed_ok = 0        # 1 while the mouse has not moved since that vote
        self.wturn = 0.0
        self.solver = None
        self.lost = 0
        self.vfront = (0, 0)
        self.a_goal = 1e9       # along-position where the current move plans to stop
        self.still_l = 1        # wheel did not move during the last step (for the start kick)
        self.kf_on = 1          # 0 = ignore the side walls (calibration straight check)
        self.vlw = 0.0          # filtered speed of each wheel, mm/s
        self.vrw = 0.0
        self.ilw = 0.0          # each wheel's speed-loop integrator (PWM)
        self.stall = 0          # steps driven with the wheels hardly turning
        self.irw = 0.0
        self.kfh = 0.0          # degrees the side walls rotated the heading estimate (logged per cell)
        self.kfl = 0.0          # mm the side walls moved the sideways estimate since the last still fix
        self.sl_d = -1          # axis and along-position of the last still sideways fix (-1 = none on this axis)
        self.sl_a = 0.0
        self.pl_last = 0        # last PWM sent to each motor
        self.pr_last = 0
        self.still_r = 1
        self.tcoast = TURN_COAST
        self.sLm = 0            # median side wall readings of the last stationary look (0 = none)
        self.sRm = 0
        self.sLn = 0            # ... and how many readings each median is from
        self.sRn = 0
        self.fvals = []         # front readings of the last stationary look
        self.sa0 = 10.0         # along-track uncertainty (mm, 1 sigma) at the last position fix
        self.odo = 0.0          # mm driven since that fix (the uncertainty grows with it)
        self.ecls = [0, 0]      # side edge tracker (L, R): class 0 none, 1 wall, 2 open
        self.en = [0, 0]        # valid readings in a row of that class
        self.ea = [0.0, 0.0]    # along-position of the last one
        self.ec = [0, 0]        # readings of the other class since (a change being confirmed)
        self.eca = [0.0, 0.0]   # along-position of the first of those
        self.epend = []         # edge fixes waiting for the new cell's walls to confirm them
        self.ep = HW_EDGE_OFF   # edge offset in use (learned from the edges seen)
        self.exw = 0.0          # average sensor position minus boundary: wall ends ...
        self.exo = 0.0          # ... and wall starts
        self.enw = 0
        self.eno = 0
        self.eo = [0, 0]        # old-class readings in a row while a change is being confirmed
        self.nom_n = 0          # still looks that measured the centred side reading
        self.nom_log = 0.0
        self.pin = 0            # steps driven with the heading stuck far off (corner caught on a post)
        self.slip = 0           # steps the wheels and the gyro disagreed on the turn rate
        self.trim = [HW_SPIN_TRIM0, HW_SPIN_TRIM0_CW]  # PWM added to the right wheel in CCW / CW spins
        self.enc_chk = 0        # 1 = check both encoders count on the first drive
        self.ncontact = 0       # contacts so far (v71j: recover up to 6)
        self.ec0 = None
        self.piv = [0.0, 0.0]   # mm: line up this much short of the centre before a CCW / CW spin at a wall (learned)
        self.piv_k = -1         # which of the two the coming turn uses (-1 = none)
        self.front_turn = 0     # 1 = the last line-up was on a wall close ahead
        self.turn_side = 0      # last 90: +1 left turn (old front wall now on the right), -1 right turn
        self.boost_to = -1e9    # along-position until which the stronger post-turn steering applies
        self.hits = {}          # contacts per map edge (4 * cell + side)

    # ---- sensing --------------------------------------------------------
    def now(self):
        try:
            return uct_mouse.get_ticks_ms()
        except Exception:
            return self.last_ms + DT_MS

    def sense(self):
        t = self.now()
        dt = (t - self.last_ms) * 0.001
        self.last_ms = t
        if dt <= 0.0:
            dt = DT_MS * 0.001
        elif dt > 0.25:
            dt = 0.25
        self.dt = dt
        e = uct_mouse.get_encoders()
        l = e[0] * ENC_POL_L
        r = e[1] * ENC_POL_R
        tl = l - self.rwl               # ticks this step, as the encoders counted them
        tr = r - self.rwr
        self.rwl = l
        self.rwr = r
        g = (uct_mouse.get_gyro() - self.bias) / GYRO_GAIN
        if ENC_FIX:
            # v65: an encoder can count the WRONG WAY (v64 board: the left one
            # counted forwards while its wheel ran backwards in left spins and
            # reverses). Keep each count's size; when the counts do not fit the
            # gyro (right - left = yaw x track) and one wheel reversed does,
            # reverse that one. The gyro picks the wheel only when it is clear
            # (a spin counted as a straight); for a straight it is the wheel
            # counting against its own motor command (unequal wheel speeds
            # made the gyro pick the wrong one in emulation).
            yt = (g + self.rate_p) * D2R * dt * HALF_AXLE * TPMM      # yaw this step, in ticks
            e0 = fabs(tr - tl - yt)
            sz = fabs(tl) + fabs(tr)
            if HW_LEFT_ENC_OFF:
                tl = tr - yt                    # v71f: left encoder not used at all
                if tr == 0 and self.pl_last == 0 and self.pr_last == 0:
                    tl = 0                      # v71i: standing still (else the gyro zero error made it creep)
            elif HW_LREV_SUB and self.pl_last < 0:
                # v71d: spincheck in the air: left backwards -185/-120/-56
                # ticks at PWM 40/60/80 (right -439/-700/-873, left forwards
                # 584/826/1010). Its count is useless running backwards: take
                # the right wheel's and the gyro's word for it
                tl = tr - yt
            elif e0 > 1.5 + 0.2 * sz:
                e1 = fabs(tr + tl - yt)         # left reversed
                e2 = fabs(tr + tl + yt)         # right reversed
                f = 0
                if e1 < 0.65 * e0 or e2 < 0.65 * e0:
                    if fabs(e1 - e2) > 0.5 * sz:
                        f = 1 if e1 < e2 else 2
                    elif e1 < 0.65 * e0 and tl * self.pl_last < 0 and tr * self.pr_last >= 0:
                        f = 1
                    elif e2 < 0.65 * e0 and tr * self.pr_last < 0 and tl * self.pl_last >= 0:
                        f = 2
                    # otherwise leave both: the real gyro lags the encoders at
                    # spin starts, and "the wheel that was wrong before" then
                    # flipped a HEALTHY left encoder 177 times in the v67 run
                if f == 1:
                    tl = -tl
                    self.efl += 1
                elif f == 2:
                    tr = -tr
                    self.efr += 1
        self.still_l = 1 if fabs(tl) < 0.5 else 0
        self.still_r = 1 if fabs(tr) < 0.5 else 0
        dl = tl / TPMM
        dr = tr / TPMM
        self.el += tl                   # counts with any wrong-way steps undone
        self.er += tr
        ds = 0.5 * (dl + dr)
        self.ds = ds
        self.odo += ds if ds > 0.0 else -ds
        self.v += 0.4 * (ds / dt - self.v)
        self.vlw += 0.35 * (dl / dt - self.vlw)     # each wheel's own speed (wheel-PI mode)
        self.vrw += 0.35 * (dr / dt - self.vrw)
        self.rate_p = g
        self.rate = g * SIDE
        thm = self.th + 0.5 * self.rate * dt          # mid-step heading
        self.th += self.rate * dt
        a = thm * D2R
        self.X += ds * _cos(a)
        self.Y += ds * _sin(a)
        # Five ToF slots (left, front-left, front, front-right, right). Some
        # boards report the side sensors in the front-left/right slots, so use
        # whichever slot of each pair has actually been returning readings.
        tof = uct_mouse.get_tof()
        nv = self.nv
        k = 0
        while k < 5:
            if 0 < tof[k] < 8190:
                nv[k] += 1
            k += 1
        a = tof[0] if nv[0] >= nv[1] else tof[1]
        b = tof[4] if nv[4] >= nv[3] else tof[3]
        if a <= 0:
            a = 8190
        if b <= 0:
            b = 8190
        if SIDE > 0:
            self.dL = a
            self.dR = b
        else:
            self.dL = b
            self.dR = a
        c = tof[2]
        self.dC = c if c > 0 else 8190
        # last valid side reading, held for up to 8 steps across dropouts
        if self.dL < 8190:
            self.hL = self.dL
            self.aL = 0
            r = self.rL
            r[0] = r[1]
            r[1] = r[2]
            r[2] = self.dL
            self.mL = med3(r[0], r[1], r[2])
        else:
            self.aL += 1
        if self.dR < 8190:
            self.hR = self.dR
            self.aR = 0
            r = self.rR
            r[0] = r[1]
            r[1] = r[2]
            r[2] = self.dR
            self.mR = med3(r[0], r[1], r[2])
        else:
            self.aR += 1
        if self.dC < 8190:
            self.hC = self.dC
            self.aC = 0
            self.fC = self.dC if self.dC > self.pC else self.pC
            self.pC = self.dC
            r = self.rC
            r[0] = r[1]
            r[1] = r[2]
            r[2] = self.dC
            self.mC = med3(r[0], r[1], r[2])
            b = self.rB
            b[0] = b[1]
            b[1] = b[2]
            b[2] = self.along(self.d) + self.dC
        else:
            self.aC += 1


    # Is the cell boundary at along-position bs (axis d) known to be open?
    def known_open(self, d, bs):
        sv = self.solver
        if sv is None:
            return 0
        ac = bs - CELL / 2.0            # along-position of the cell before that boundary
        if d & 1:
            cx = int(self.X / CELL)
            cy = int(DY[d] * ac / CELL)
        else:
            cx = int(DX[d] * ac / CELL)
            cy = int(self.Y / CELL)
        if not sv.inside(cx, cy):
            return 0
        i = cy * sv.cols + cx
        b = 1 << d
        return 1 if ((sv.known[i] & b) and not (sv.wall[i] & b)) else 0

    # Front reflex: a wall the front ToF has confirmed is closer than the
    # distance needed to stop from the current speed.
    def front_block(self):
        if FRONT_STOP <= 0.0 or self.aC >= 6:
            return 0
        v = self.v if self.v > 0.0 else 0.0
        if self.fC >= FRONT_STOP + FRONT_REACT * v:
            return 0
        d = self.d
        aw = self.along(d) + FRONT_OFF + self.fC
        # A wall at the front edge of the cell the move stops in is expected:
        # the speed profile already stops for it, so only the last-resort
        # distance applies (the speed term made it brake 20-35 mm short).
        if aw > self.a_goal + CELL / 2.0 - 40.0 and self.fC >= FRONT_STOP:
            return 0
        # Walls stand only on cell boundaries, and a boundary the map already
        # knows is open cannot be one: such short readings are the ToF's
        # stray echoes (an open corridor sometimes reads 30-60 mm).
        bs = CELL * float(round(aw / CELL))
        if fabs(aw - bs) > 40.0:
            return 0
        sv = self.solver
        if sv is not None:
            ac = bs - CELL / 2.0        # along-position of the cell before that boundary
            if d & 1:
                cx = int(self.X / CELL)
                cy = int(DY[d] * ac / CELL)
            else:
                cx = int(DX[d] * ac / CELL)
                cy = int(self.Y / CELL)
            if sv.inside(cx, cy):
                i = cy * sv.cols + cx
                b = 1 << d
                if (sv.known[i] & b) and not (sv.wall[i] & b):
                    return 0
        return 1

    # ---- actuation ------------------------------------------------------
    @staticmethod
    def _pwm(eff, db):
        if eff > 0.0:
            p = db + eff * (100.0 - db) / 100.0
        elif eff < 0.0:
            p = -db + eff * (100.0 - db) / 100.0
        else:
            return 0
        return int(clamp(p, -100.0, 100.0))

    # One 10 ms control step: v in mm/s, w in deg/s (map frame).
    def step(self, v, w):
        dt = self.dt
        # FRONT REFLEX: never drive forward into a wall the front ToF can see
        if v > 0.0 and self.front_block():
            v = 0.0
        if v != 0.0 or w != 0.0:
            self.fmed_ok = 0
        # CLIMB DETECTOR: body tilting = a wheel is riding up a wall
        if TILT_LIM > 0.0 and v > 0.0:
            self.tk += 1
            if self.tk % 2 == 0 and self.tilted():
                if not self.bump:
                    log("TILT - a wheel is riding up a wall")
                self.bump = 1
                v = 0.0
                w = 0.0
        if v == 0.0 and w == 0.0:
            uct_mouse.set_motors(0, 0)
            self.pl_last = 0
            self.pr_last = 0
            self.ilw = 0.0
            self.irw = 0.0
            self.iv *= 0.5
            self.iw *= 0.5
            self.vcmd_prev = 0.0
            self.wcmd_prev = 0.0
        elif WHEEL_PI:
            # diag.py's controller: the turn rate becomes a speed difference,
            # then EACH wheel follows its own target on its own encoder
            # (feed-forward from the measured dead-band and gain + PI). Both
            # wheels speed up and slow down together: no lean, square stops.
            wl = w * SIDE * D2R * HALF_AXLE
            cl = v - wl
            cr = v + wl
            el = cl - self.vlw
            er = cr - self.vrw
            self.ilw = clamp(self.ilw + HW_WHEEL_KI * el * dt, -15.0, 15.0)
            self.irw = clamp(self.irw + HW_WHEEL_KI * er * dt, -15.0, 15.0)
            # stall detector: told to drive but the wheels hardly turn for
            # ~0.4 s = it hit something; treat it like a bump (stop, back off)
            if fabs(v) >= 50.0 and fabs(self.v) < 15.0:
                self.stall += 1
                if self.stall > 40:
                    self.stall = 0
                    self.bump = 1
                    log("STALL - wheels blocked")
            else:
                self.stall = 0
            pl = int(clamp(wheel_ff(cl, DB_L, VMAX_L) + HW_WHEEL_KP * el + self.ilw, -100.0, 100.0))
            pr = int(clamp(HW_RIGHT_SCALE * wheel_ff(cr, DB_R, VMAX_R) + HW_WHEEL_KP * er + self.irw, -100.0, 100.0))
            uct_mouse.set_motors(pl, pr)
            self.pl_last = pl
            self.pr_last = pr
        else:
            wp = w * SIDE
            ev = v - self.v
            ew = wp - self.rate_p
            self.iv = clamp(self.iv + KI_V * ev * dt, -25.0, 25.0)
            self.iw = clamp(self.iw + KI_W * ew * dt, -25.0, 25.0)
            uv = KP_V * ev + self.iv
            uw = KP_W * ew + self.iw
            wl = wp * D2R * HALF_AXLE
            cl = v - wl
            cr = v + wl
            # feed-forward of each wheel's commanded acceleration (motor lag)
            al = (cl - self.vcmd_prev) / dt
            ar = (cr - self.wcmd_prev) / dt
            self.vcmd_prev = cl
            self.wcmd_prev = cr
            el = (cl + TAU * al) * 100.0 / VMAX_L + uv - uw
            er = (cr + TAU * ar) * 100.0 / VMAX_R + uv + uw
            pl = self._pwm(el, DB_L)
            pr = self._pwm(er, DB_R)
            if fabs(v) >= 30.0 and fabs(self.v) < 15.0:
                # start kick (only while the mouse is setting off): a wheel at
                # rest needs its breakaway PWM, else one wheel starts first and
                # the mouse yaws (the right lean)
                if self.still_l and pl != 0 and abs(pl) < KICK_L:
                    pl = KICK_L if pl > 0 else -KICK_L
                if self.still_r and pr != 0 and abs(pr) < KICK_R:
                    pr = KICK_R if pr > 0 else -KICK_R
            uct_mouse.set_motors(pl, pr)
            self.pl_last = pl
            self.pr_last = pr
            # encoder watchdog: wheels commanded but no counts at all -> stop
            if v >= 80.0:
                if self.ds == 0.0 and self.el == self.el_last and self.er == self.er_last:
                    self.dead += 1
                    if self.dead > 60:
                        uct_mouse.set_motors(0, 0)
                        raise RuntimeError("encoders not counting - check battery/encoder wiring")
                else:
                    self.dead = 0
            self.el_last = self.el
            self.er_last = self.er
        self.wait_period()
        self.sense()

    # Control period: the board needs ~12 ms to compute a step, so waiting a
    # further 10 ms made it a 22 ms loop; wait only for the rest of the period.
    def wait_period(self):
        if LOOP_FIXED:
            w = DT_MS - (self.now() - self.last_ms)
            if w >= 1:
                uct_mouse.delay_ms(w)
        else:
            uct_mouse.delay_ms(DT_MS)

    # Spin on the spot at raw PWM p, direction sg (+1 = CCW in the map frame).
    def spin_raw(self, sg, p, learn=1):
        if POS_MODEL and HW_SPIN_MATCH:
            # board: equal wheel SPEEDS, not equal PWM. The right motor is
            # ~25% weaker, so at equal PWM the left wheel ran faster and the
            # mouse rotated about a point right of the axle: every 90 left
            # the axle ~4 mm to the left, every U-turn ~8 mm (v49-v52 logs,
            # and what you see). The right wheel gets the model's extra PWM
            # plus a trim learned from the encoders, kept from spin to spin.
            a = int(p)
            ff = DB_R + (p - DB_L) * (VMAX_L / (100.0 - DB_L)) / (VMAX_R / (100.0 - DB_R)) - p
            k = 0 if sg * SIDE > 0 else 1   # v71: one trim per spin direction (a shared one swung -19..+10
            #                                 and CCW spins with it >= 0 ended ~6 mm forward)
            t = self.trim[k]
            # v71b: never slow a wheel below the spin PWM (a -20 trim put the
            # right wheel at ~31 PWM, under its ~42-46 breakaway): a negative
            # trim speeds the LEFT wheel up instead
            if t < 0.0:
                a = int(clamp(p - t, 0.0, 100.0))
                t = 0.0
            b = int(clamp(p + ff + t, 0.0, 100.0))
            if sg * SIDE > 0:
                a = -a
            else:
                b = -b
            uct_mouse.set_motors(a, b)
            self.pl_last = a
            self.pr_last = b
            self.wait_period()
            self.sense()
            e = fabs(self.vlw) - fabs(self.vrw)         # > 0: left faster
            if learn and k == 1 and fabs(self.vlw) > 60.0 and not HW_LEFT_ENC_OFF:
                # v71d: only right spins (left wheel forwards: both encoders
                # valid); the left spin uses the same trim
                self.trim[1] = clamp(self.trim[1] + 0.004 * e, -HW_SPIN_TRIM_MAX, HW_SPIN_TRIM_MAX)
                self.trim[0] = self.trim[1]
            return
        q = int(p) if sg * SIDE > 0 else -int(p)
        uct_mouse.set_motors(-q, q)
        self.pl_last = -q
        self.pr_last = q
        self.wait_period()
        self.sense()

    # Motors off and wait until the spin has really stopped.
    def spin_stop(self):
        uct_mouse.set_motors(0, 0)
        self.pl_last = 0
        self.pr_last = 0
        n = 0
        quiet = 0
        while n < 50 and quiet < 3:
            self.step(0.0, 0.0)
            quiet = quiet + 1 if fabs(self.rate) < 5.0 else 0
            n += 1

    # Spin turn for motors that stall below ~75 PWM on the spot: drive the
    # spin at a fixed PWM and cut it where the coast (learned: deg per deg/s)
    # carries the heading onto the target; then at most two correction spins.
    def turn_bb(self, delta):
        target = self.th + delta
        t0 = self.now()
        ovs = 0.0
        tries = 0
        while tries < 3:
            err = target - self.th
            if tries > 0 and fabs(err) < TURN_TOL:
                break
            sg = 1.0 if err > 0.0 else -1.0
            p = TURN_PWM if (tries == 0 or not POS_MODEL) else HW_TURN_PWM2
            th0 = self.th
            n = 0
            while n < 300:
                rem = sg * (target - self.th)
                r = sg * self.rate
                if r < 0.0:
                    r = 0.0
                if rem - r * self.tcoast <= 0.0:
                    break               # the coast from here reaches the target
                if n > 15 and fabs(self.th - th0) < 2.0 and p >= 95:
                    log("turn blocked - giving up (wheels stuck or touching)")
                    if POS_MODEL:
                        self.contact()
                    tries = 3
                    break
                if n > 15 and fabs(self.th - th0) < 2.0 and p < 95:
                    p += 5              # not turning: wheels stuck, push harder
                    th0 = self.th
                    n = 0
                self.spin_raw(sg, p)
                n += 1
            rc = sg * self.rate
            thc = self.th
            if rc > 80.0:
                # active brake: a short opposite pulse stops the spin crisply
                # (free coasting varied 58-115 ms and overshot 8-16 deg)
                k = 0
                while k < 4 and sg * self.rate > 0.33 * rc:
                    self.spin_raw(-sg, TURN_PWM, 0)     # brake pulse: no trim learning
                    k += 1
            self.spin_stop()
            if rc > 60.0:
                c = sg * (self.th - thc) / rc
                if 0.0 < c < 0.3:
                    self.tcoast += 0.5 * (c - self.tcoast)
            err = target - self.th
            if delta * err < 0.0 and fabs(err) > ovs:
                ovs = fabs(err)
            tries += 1
        err = target - self.th
        if POS_MODEL and fabs(err) > 45.0:
            log("TURN FAILED err", int(err))     # wedged or spun away
            if HW_ON_CONTACT:
                self.contact()                  # (recovering: the next straight's HEADING LOST check straightens it)
        if DEBUG:
            log("turn", int(delta), "err", int(10 * err) / 10.0, "ovs", int(10 * ovs) / 10.0,
                "ms", self.now() - t0, "tries", tries, "coast", int(1000 * self.tcoast),
                "trim", int(self.trim[0]), int(self.trim[1]))
        self.p22 += 0.0005
        self.p11 += 25.0

    def hold(self, steps):
        n = 0
        while n < steps:
            self.step(0.0, 0.0)
            n += 1

    # Accelerometer tilt vs the resting gravity vector (board only).
    def tilted(self):
        try:
            t = uct_mouse.get_telemetry()
        except Exception:
            return 0
        if self.a0 is None:
            return 0
        dx = t[0] - self.a0[0]
        dy = t[1] - self.a0[1]
        if dx * dx + dy * dy > (TILT_LIM * self.a0[2]) ** 2:
            self.tilt_n += 1
        else:
            self.tilt_n = 0
        return 1 if self.tilt_n >= 3 else 0

    # Board: something was touched. In the demo that already fails the run,
    # so stop dead and end it (HW_ON_CONTACT = 1) rather than backing up and
    # carrying on with a doubtful position; HW_ON_CONTACT = 0 just logs it.
    def contact(self):
        uct_mouse.set_motors(0, 0)
        self.ncontact += 1
        if HW_ON_CONTACT or self.ncontact > 6:
            self.bump = 0
            log("CONTACT - run stopped")
            raise ContactStop()
        # v71j: keep the bump flag so the caller backs off and replans
        self.bump = 1
        log("CONTACT", self.ncontact, "- backing off, driving on")

    # Climbed / bumped: stop, reverse gently off the obstacle, stop.
    def back_off(self):
        log("BUMP - backing off")
        self.bump = 0
        self.tilt_n = 0
        if POS_MODEL:
            self.sa0 = 40.0             # wheels may have slipped: along position unknown
            self.odo = 0.0
        n = 0
        while n < (60 if POS_MODEL else 35):
            self.step(-70.0 if POS_MODEL else -60.0, 0.0)     # board: ~40 mm back
            n += 1
        self.settle()
        self.vc = 0.0

    # Before turning on the spot: if there is a wall ahead, creep until the
    # front ToF reads the centred distance, so the corners cannot touch while
    # spinning, and use that wall to reset the along-track position.
    def align_front(self, nd=-1):
        if ALIGN_MOVE:
            # v71: a line-up more than one 30 mm move away (after a long
            # sprint straight) measures and moves again; smaller ones move once
            # (repeating those pushed into a blocked spot in emulation)
            k = 0
            while k < 2 and self.align_pos(nd) > 30.0 and POS_MODEL:
                k += 1
            return
        d = self.d
        n = 0
        good = 0
        seen = 0
        t0 = self.now()
        while n < 150:
            n += 1
            if self.aC < 6 and self.hC < 130:
                seen = 1
                e = self.hC - FRONT_CENTER
                if fabs(e) < 4.0:
                    good += 1
                    if good >= 4:
                        break
                    v = 0.0
                else:
                    good = 0
                    v = clamp(3.0 * e, -60.0, 60.0)
                    if 0.0 < v < 25.0:
                        v = 25.0
                    elif -25.0 < v < 0.0:
                        v = -25.0
            else:
                v = 0.0
                if n > 25 and not seen:
                    break
            psi = wrap180(self.th - 90.0 * d)
            self.step(v, clamp(-4.0 * psi, -40.0, 40.0))
        self.settle()
        if DEBUG and seen:
            log("align front", self.hC, "good", good, "ms", self.now() - t0)
        if good >= 4:
            # axle is on the cell centre along this axis: correct the pose
            a = self.cell_along(self.cx, self.cy, d) - self.along(d)
            self.X += DX[d] * a
            self.Y += DY[d] * a

    # Board: after a stop, a front median 45-110 mm longer than at a cell
    # centre means the mouse is between two cells - no centre is that far
    # from a wall. (v49 stopped like that and mapped a wall at the wrong edge.)
    def between(self):
        return POS_MODEL and self.fmed_ok and len(self.fvals) >= 8 \
            and FRONT_CENTER + 45.0 < self.fmed < FRONT_CENTER + 110.0

    # ... so creep up to that wall (it is really there), then take the
    # position from it: the axle is now on the centre of the cell in front of
    # the nearest boundary. Leaves a fresh front median for stop_fix.
    def creep_to_wall(self):
        d = self.d
        log("front", self.fmed, "- between cells: creeping to the wall")
        self.move_to(d, self.along(d) + clamp(self.fmed - FRONT_CENTER, 0.0, 75.0))
        self.front_median(30)
        f = self.fmed
        if fabs(f - FRONT_CENTER) > 25.0:
            # it did not get there (blocked, or not a wall across): leave the
            # estimate alone (v50 snapped anyway and jumped two cells)
            log("re-centre failed: front still", f)
            return
        b = self.along(d) + FRONT_OFF + f           # the wall line, in the estimate
        e = CELL * float(round(b / CELL)) - b
        nx = int((self.X + DX[d] * e) / CELL)
        ny = int((self.Y + DY[d] * e) / CELL)
        if abs(nx - self.cx) + abs(ny - self.cy) > 1:
            log("re-centre rejected: would jump to", nx, ny)
            return
        self.sa0 = 40.0                             # the wall decides, not the old estimate
        self.odo = 0.0
        self.abs_fix(e, self.front_sig(len(self.fvals)))
        self.cx = int(self.X / CELL)
        self.cy = int(self.Y / CELL)
        log("re-centred in", self.cx, self.cy, "front", f)

    # Board, before every spin: put the axle on the cell centre along the
    # current axis. Spinning, the front corners sweep ~86 mm round the axle
    # and the walls are ~94 mm away, so it must be within a few mm. A wall
    # close ahead is measured (up to 25 readings standing still, median) and
    # weighted against the estimate; without one the estimate (encoders +
    # edge fixes) is all there is. Then one short encoder move to the centre.
    def align_pos(self, nd=-1):
        d = self.d
        t0 = self.now()
        k = -1
        dd = (nd - d) & 3 if nd >= 0 else 0
        if dd == 1 or dd == 3:
            k = 0 if (dd == 1) == (SIDE > 0) else 1     # physical spin: 0 = CCW (left turn), 1 = CW
        self.piv_k = k
        vals = list(self.fvals) if self.fmed_ok else []
        near = 0
        for v in vals:
            if v < FRONT_NEAR + 30.0:
                near += 1
        n = 0
        while n < 80 and len(vals) < 25 and (near > 0 or (n < 20 and len(vals) < 5)):
            self.step(0.0, 0.0)
            if self.dC < 8190:
                vals.append(self.dC)
                if self.dC < FRONT_NEAR + 30.0:
                    near += 1
            n += 1
        f = 8190
        if len(vals) >= 5:
            vals.sort()
            f = vals[len(vals) // 2]
        c = self.cell_along(self.cx, self.cy, d)
        fx = 0.0
        self.front_turn = 0
        if f < FRONT_NEAR:
            e = c + FRONT_CENTER - f - self.along(d)
            if fabs(e) < (95.0 if self.sa() > 20.0 else 60.0):     # after a long straight a wall this near is this cell's
                fx = self.abs_fix(e, self.front_sig(len(vals)))
                self.front_turn = 1
        elif f < 8190:
            fx = self.far_fix(f)
        if self.front_turn and HW_PIVOT_LEARN and k >= 0:
            c -= self.piv[k]            # spins in this direction end this far forward: start that much short
        mv = c - self.along(d)
        if fabs(mv) > 60.0 and not self.front_turn:
            log("align skipped, move", int(mv))     # pose estimate is off: do not drive blind
            return 0.0
        if not self.front_turn:
            mv = clamp(mv, -15.0, 15.0)     # v71e: no wall close ahead: never move far on the estimate alone
        if fabs(mv) > HW_ALIGN_TOL:
            self.move_to(d, self.along(d) + clamp(mv, -30.0, 30.0))
        log("align front", f, "fix", int(fx), "move", int(mv), "sa", int(self.sa()), "piv", int(self.piv[k]) if k >= 0 else 0,
            "ms", self.now() - t0)
        return fabs(mv) if self.front_turn else 0.0

    # Gyro bias from samples taken with BOTH wheels dead still (a turn that
    # is still unwinding has zero forward motion but is still rotating).
    def zupt_add(self, gs, gn):
        if ZUPT_STOP and gn >= 12:
            e = gs / gn
            # board: wheels dead still for 12+ samples, so the mouse is not
            # turning - a start zero off by more than 2 deg/s gets fixed too
            if fabs(e) < 2.0:          # as v53 (the start calibration now checks the mouse is still)
                self.bias += 0.5 * e * GYRO_GAIN

    # Median of the valid front readings standing still (for stop_fix).
    def front_median(self, steps):
        vals = []
        gs = 0.0
        gn = 0
        pel = self.rwl
        per = self.rwr
        n = 0
        while n < steps:
            self.step(0.0, 0.0)
            if n >= 6 and self.rwl == pel and self.rwr == per:     # v71i: raw readings (the left count is computed from the gyro now)
                gs += self.rate_p
                gn += 1
            pel = self.rwl
            per = self.rwr
            if self.dC < 8190:
                vals.append(self.dC)
            n += 1
        self.zupt_add(gs, gn)
        self.fmed = 8190
        if len(vals) >= 3:
            vals.sort()
            self.fmed = vals[len(vals) // 2]
        self.fmed_ok = 1
        self.fvals = vals


    # Before the pirouette: step away from this cell's walls so the corners
    # (sweeping ~86 mm round the axle) clear them. The 90 deg turn used for
    # a side wall faces an open side, so nothing is in reach while it turns.
    def spin_spot(self, sv):
        if SPIN_SHIFT <= 0.0:
            return -1
        i = self.cy * sv.cols + self.cx
        d = self.d
        b = (d + 2) & 3
        wf = sv.wall[i] & (1 << d)
        wb = sv.wall[i] & (1 << b)
        c = self.cell_along(self.cx, self.cy, d)
        if wf and not wb:
            self.move_to(d, c - SPIN_SHIFT)
        elif wb and not wf:
            self.move_to(d, c + SPIN_SHIFT)
        lt = (d + 1) & 3
        rt = (d + 3) & 3
        s = -1
        if (sv.wall[i] & (1 << lt)) and not (sv.wall[i] & (1 << rt)):
            s = rt
        elif (sv.wall[i] & (1 << rt)) and not (sv.wall[i] & (1 << lt)):
            s = lt
        if s >= 0:
            self.turn_to(s)
            self.move_to(s, self.cell_along(self.cx, self.cy, s) + SPIN_SHIFT)
        log("spin spot walls", sv.wall[i], "d", self.d)
        return s

    # Short straight move to along-position a_end on the encoders only
    # (forwards or backwards), holding the heading of axis d.
    def move_to(self, d, a_end):
        if fabs(a_end - self.along(d)) > 80.0:
            return                      # short alignment moves only
        self.a_goal = a_end
        n = 0
        lim = min(120, 40 + int(5.0 * fabs(a_end - self.along(d))))    # ~1.5x the time it needs (v64 backed into a wall)
        while n < lim:
            rem = a_end - self.along(d)
            if fabs(rem) < 2.0:
                break
            v = clamp(4.0 * rem, -80.0, 80.0)
            if 0.0 < v < 30.0:
                v = 30.0
            elif -30.0 < v < 0.0:
                v = -30.0
            psi = wrap180(self.th - 90.0 * d)
            self.step(v, clamp(-4.0 * psi, -40.0, 40.0))
            if v > 0.0 and self.front_block():
                break
            n += 1
        self.settle()

    # Stand still and look ahead: 1 if the front ToF says wall (majority of
    # the valid readings within reach of this cell's front edge).
    def front_check(self, steps):
        fs = 0
        fo = 0
        sl = []
        sr = []
        n = 0
        while n < steps:
            self.step(0.0, 0.0)
            if self.dC < 8190:
                if self.dC + FRONT_OFF < CELL / 2.0 + 60.0:
                    fs += 1
                else:
                    fo += 1
            if self.dL < SIDE_WALL:
                sl.append(self.dL)
            if self.dR < SIDE_WALL:
                sr.append(self.dR)
            n += 1
        self.side_medians(sl, sr)
        return 1 if (fs >= 3 and fs > 2 * fo) else 0     # clear majority: strays cannot fake a wall

    def side_medians(self, sl, sr):
        sl.sort()
        sr.sort()
        self.sLm = sl[len(sl) // 2] if len(sl) >= 3 else 0
        self.sRm = sr[len(sr) // 2] if len(sr) >= 3 else 0
        self.sLn = len(sl)
        self.sRn = len(sr)

    # At a cell stop, right after the stationary vote: a wall seen ahead
    # (median of ~20 readings, so stray echoes are outvoted) fixes the
    # along-track position. Without it the start-up wheel slip of every
    # cell adds up along open corridors, and the next 90 deg turn turns
    # that along error into a sideways one (into a post).
    def stop_fix(self):
        f = self.fmed
        if not STOP_FIX or not self.fmed_ok or f > 500:
            return 0
        if POS_MODEL:
            # only this cell's own front wall (longer readings read short)
            if f >= FRONT_NEAR:
                return self.far_fix(f)
            d = self.d
            e = self.cell_along(self.cx, self.cy, d) + FRONT_CENTER - f - self.along(d)
            if fabs(e) > 60.0:
                return 0
            return int(self.abs_fix(e, self.front_sig(len(self.fvals))))
        return 0


    # Along-position of the first known wall straight ahead of cell (cx, cy)
    # on axis d, if every edge before it is known open; else None.
    def mapped_wall(self, d):
        sv = self.solver
        if sv is None:
            return None
        x = self.cx
        y = self.cy
        b = 1 << d
        k = 0
        while k < 5 and sv.inside(x, y):
            i = y * sv.cols + x
            if not (sv.known[i] & b):
                return None
            if sv.wall[i] & b:
                return self.cell_along(x, y, d) + CELL / 2.0
            x += DX[d]
            y += DY[d]
            k += 1
        return None

    # End of a drive: the wheels never stop at exactly the same moment, which
    # swung the nose a few degrees. Keep the gyro heading loop running with
    # zero forward speed while the wheels coast down (one brakes, the other
    # eases), and only cut the motors once speed and rotation are both ~0.
    def stop_hold(self, d):
        if not BRAKE_HOLD:
            self.settle()
            return
        # both wheels keep driving FORWARD while the speed ramps down fast, so
        # the gyro can steer to the end; never opposite PWMs (sticky wheels
        # turn those into jerks)
        vc = self.v if self.v > 0.0 else 0.0
        n = 0
        while n < 20 and vc > 12.0:
            vc *= 0.7
            psi = wrap180(self.th - 90.0 * d)
            self.step(vc, clamp(-KPSI * psi, -40.0, 40.0))
            n += 1
        self.settle()

    # Stop and wait until the wheels are really still.
    def settle(self):
        n = 0
        quiet = 0
        while n < 40 and quiet < 4:
            self.step(0.0, 0.0)
            if fabs(self.v) < 5.0 and fabs(self.rate) < 8.0:
                quiet += 1
            else:
                quiet = 0
            n += 1
        if quiet >= 4:
            save_due()           # wheels still: a safe moment to write the log

    def calibrate(self):
        uct_mouse.set_motors(0, 0)
        n = 0
        while n < 20:
            uct_mouse.delay_ms(DT_MS)
            n += 1
        if IS_SIM:
            tot = 0.0
            n = 0
            while n < CAL_SAMPLES:
                uct_mouse.delay_ms(DT_MS)
                tot += uct_mouse.get_gyro()
                n += 1
            self.bias = tot / CAL_SAMPLES + GYRO_TRIM * GYRO_GAIN
        else:
            # board: only a truly still mouse gives the gyro's zero. A hand on
            # it / rocking after SW1 gave a wrong zero, and the mouse then
            # curved into a wall within a few cells (ends the run on contact)
            best = None
            k = 0
            while k < 4:
                e0 = uct_mouse.get_encoders()
                tot = 0.0
                sq = 0.0
                n = 0
                while n < CAL_SAMPLES:
                    uct_mouse.delay_ms(DT_MS)
                    g = uct_mouse.get_gyro()
                    tot += g
                    sq += g * g
                    n += 1
                e1 = uct_mouse.get_encoders()
                mean = tot / CAL_SAMPLES
                var = sq / CAL_SAMPLES - mean * mean
                sd = _sqrt(var) if var > 0.0 else 0.0
                moved = abs(e1[0] - e0[0]) > 2 or abs(e1[1] - e0[1]) > 2
                if best is None or sd < best[1]:
                    best = (mean, sd)
                if not moved and sd < 1.0:     # still mouse ~0.03 deg/s; a hand / rocking is several
                    break
                log("gyro cal: moving - retrying (spread", int(100 * sd) / 100.0, "wheels moved", 1 if moved else 0, ")")
                uct_mouse.delay_ms(500)
                k += 1
            self.bias = best[0] + GYRO_TRIM * GYRO_GAIN
            log("gyro zero", int(100 * best[0]) / 100.0, "deg/s spread", int(100 * best[1]) / 100.0)
        try:
            ax = ay = az = 0.0
            n = 0
            while n < 10:
                t = uct_mouse.get_telemetry()
                ax += t[0]
                ay += t[1]
                az += t[2]
                uct_mouse.delay_ms(DT_MS)
                n += 1
            g = _sqrt(ax * ax + ay * ay + az * az) / 10.0
            if g > 0.1:
                self.a0 = (ax / 10.0, ay / 10.0, g)
        except Exception:
            self.a0 = None
        e = uct_mouse.get_encoders()
        self.el = e[0] * ENC_POL_L
        self.er = e[1] * ENC_POL_R
        self.rwl = self.el
        self.rwr = self.er
        self.last_ms = self.now()
        self.t0 = self.last_ms
        self.sense()
        self.th = 0.0
        self.X = CELL / 2.0
        self.Y = CELL / 2.0



    # Board: on the first drive, once one wheel has moved ~20 mm, check that
    # the other encoder counted too (v61: the LEFT encoder read ~0, the left
    # wheel ran away, the mouse curved right). Dead -> stop with a clear
    # message. Counting backwards -> reverse it from here on. v64 did that
    # after its estimates had used the wrong counts (latfix fixed 33-78 mm
    # after every left turn, then it hit a wall); now sense() has undone the
    # wrong-way steps all along, so the position does not jump.
    def first_drive_check(self):
        global ENC_POL_L, ENC_POL_R
        if self.ec0 is None:
            self.ec0 = (self.el, self.er, self.rwl, self.rwr)
            return
        dl = self.el - self.ec0[0]
        dr = self.er - self.ec0[1]
        if abs(dl) <= 120 and abs(dr) <= 120:
            return                      # judge once one wheel has done ~20 mm (wheels start from rest unevenly)
        self.enc_chk = 0
        ql = self.rwl - self.ec0[2]     # as counted
        qr = self.rwr - self.ec0[3]
        log("encoder check L", int(ql), "R", int(qr), "ticks counted, L", int(dl), "R", int(dr), "after the gyro check")
        if abs(ql) < 10 and not HW_LEFT_ENC_OFF:
            uct_mouse.set_motors(0, 0)
            log("LEFT ENCODER NOT COUNTING - reseat its cable, charge the battery")
            raise RuntimeError("ENC L DEAD")
        if abs(qr) < 10:
            uct_mouse.set_motors(0, 0)
            log("RIGHT ENCODER NOT COUNTING - reseat its cable, charge the battery")
            raise RuntimeError("ENC R DEAD")
        if ql * dl < 0 and not HW_LEFT_ENC_OFF:
            ENC_POL_L = -ENC_POL_L
            self.rwl = -self.rwl
            log("LEFT encoder counts backwards - reversed (position unaffected)")
        if qr * dr < 0:
            ENC_POL_R = -ENC_POL_R
            self.rwr = -self.rwr
            log("RIGHT encoder counts backwards - reversed (position unaffected)")

    # Start-up system identification (simulator profile): spin on the spot
    # at two PWM levels. Each wheel's steady tick rate is linear in PWM above
    # its dead-band, so two points give the dead-band (x-intercept) and the
    # gain; ticks counted against gyro-measured rotation give ticks per mm.
    def sysid(self):
        global DB_L, DB_R, TPMM, VMAX_L, VMAX_R
        pa = 72
        pb = 92
        rl = [0.0, 0.0]
        rr = [0.0, 0.0]
        tk = 0.0
        ang = 0.0
        ph = 0
        while ph < 2:
            p = pa if ph == 0 else pb
            n = 0
            while n < 34:
                uct_mouse.set_motors(p, -p)
                uct_mouse.delay_ms(DT_MS)
                l0 = self.el
                r0 = self.er
                th0 = self.th
                self.sense()
                if n >= 18:
                    rl[ph] += self.el - l0
                    rr[ph] += r0 - self.er
                    tk += fabs(self.el - l0) + fabs(self.er - r0)
                    ang += fabs(self.th - th0)
                n += 1
            ph += 1
        self.settle()
        if rl[1] > rl[0] + 10 and rr[1] > rr[0] + 10 and ang > 20.0:
            DB_L = clamp(pa - rl[0] * (pb - pa) / (rl[1] - rl[0]), 5.0, 80.0)
            DB_R = clamp(pa - rr[0] * (pb - pa) / (rr[1] - rr[0]), 5.0, 80.0)
            TPMM = clamp(0.5 * tk / (HALF_AXLE * ang * D2R), 2.0, 12.0)
            t = 16.0 * DT_MS * 0.001
            VMAX_L = (rl[1] - rl[0]) / (pb - pa) * (100.0 - DB_L) / TPMM / t
            VMAX_R = (rr[1] - rr[0]) / (pb - pa) * (100.0 - DB_R) / TPMM / t
        log("SYSID db", DB_L, DB_R, "tpmm", TPMM, "vmax", VMAX_L, VMAX_R)
        self.turn_by(wrap180(0.0 - self.th))
        self.X = CELL / 2.0
        self.Y = CELL / 2.0

    # Re-estimate gyro bias while parked (used during the 3 s pause).
    def zupt(self, steps):
        tot = 0.0
        n = 0
        while n < steps:
            uct_mouse.set_motors(0, 0)
            uct_mouse.delay_ms(DT_MS)
            tot += uct_mouse.get_gyro()
            n += 1
        est = tot / steps
        if fabs(est - self.bias) < 1.0:
            self.bias += 0.7 * (est - self.bias)
        self.last_ms = self.now()
        self.sense()

    # ---- geometry helpers ----------------------------------------------
    def along(self, d):
        return self.X * DX[d] + self.Y * DY[d]

    def lateral(self, d):
        return -self.X * DY[d] + self.Y * DX[d]

    def cell_along(self, x, y, d):
        return (x * CELL + CELL / 2.0) * DX[d] + (y * CELL + CELL / 2.0) * DY[d]

    def cell_lat(self, x, y, d):
        return -(x * CELL + CELL / 2.0) * DY[d] + (y * CELL + CELL / 2.0) * DX[d]

    # ---- Kalman corridor filter: state [lat (mm), psi (rad)] -------------
    def kf(self, latc):
        if not self.kf_on:
            return
        d = self.d
        ds = self.ds
        p11 = self.p11 + 2.0 * ds * self.p12 + ds * ds * self.p22 + 0.02 * fabs(ds)
        p12 = self.p12 + ds * self.p22
        p22 = self.p22 + 4e-8 + 2e-7 * fabs(ds)
        zs = 0.0
        nz = 0
        ok = 1
        if POS_MODEL:
            # board: level with a cell boundary the side sensor sees the post
            # or prong (~10 mm closer) or a wall end (half the beam on the
            # wall): those readings push the estimate the wrong way
            a = self.along(d) + SIDE_X
            ok = 1 if fabs(a - CELL * float(round(a / CELL))) >= HW_POST_BLANK else 0
        if ok and self.dL < NOM_L + 35.0 and self.dL < SIDE_WALL:
            zs += NOM_L - self.dL
            nz += 1
        if ok and self.dR < NOM_R + 35.0 and self.dR < SIDE_WALL:
            zs += self.dR - NOM_R
            nz += 1
        if nz > 0:
            z = zs / nz
            psi = wrap180(self.th - 90.0 * d) * D2R
            lat = self.lateral(d) - latc
            r = TOF_SIGMA * TOF_SIGMA / nz
            h1 = p11 + SIDE_X * p12
            h2 = p12 + SIDE_X * p22
            s = h1 + SIDE_X * h2 + r
            nu = z - (lat + SIDE_X * psi)
            if not KF_HEADING:
                # heading is trusted from the gyro: scalar update of the
                # sideways offset only
                h1 = p11
                h2 = 0.0
                s = p11 + r
            if nu * nu < 9.0 * s + 25.0:
                k1 = h1 / s
                k2 = h2 / s
                dl = k1 * nu
                self.X -= DY[d] * dl
                self.Y += DX[d] * dl
                self.kfl += dl
                self.th += k2 * nu / D2R
                self.kfh += k2 * nu / D2R
                p11 -= k1 * h1
                p12 -= k1 * h2
                p22 -= k2 * h2
        self.p11 = p11
        self.p12 = p12
        self.p22 = p22

    # Front ToF: every wall across the corridor sits on a multiple of CELL,
    # so snap the measured wall position and pull the along estimate onto it.
    def front_fix(self):
        if POS_MODEL:
            self.front_fuse()
            return
        if self.dC <= 0 or self.dC > FRONT_FIX_MAX:
            return
        if fabs(wrap180(self.th - 90.0 * self.d)) > 6.0:
            return
        d = self.d
        b = self.along(d) + FRONT_OFF + self.dC
        bs = CELL * float(round(b / CELL))
        err = bs - b
        if fabs(err) < FRONT_SNAP:
            c = FRONT_GAIN * err
            self.X += DX[d] * c
            self.Y += DY[d] * c

    # Board: a wall across the corridor close ahead (front reading below
    # FRONT_NEAR, the range it was calibrated in) pulls the along estimate a
    # little with each fresh reading (median of the last three, error
    # clipped so the ToF's long tail cannot drag it). The stop then lands on
    # the averaged position instead of on the first low reading, which
    # stopped it 10-25 mm short. Longer readings are not used: they catch
    # the floor and read 15-30 mm short.
    def front_fuse(self):
        if self.aC != 0 or self.mC >= FRONT_NEAR:
            return
        d = self.d
        if fabs(wrap180(self.th - 90.0 * d)) > 6.0:
            return
        b = self.along(d) + FRONT_OFF + self.mC     # implied wall line
        bs = CELL * float(round(b / CELL))
        e = bs - b
        if fabs(e) > 30.0 or self.known_open(d, bs):
            return                      # no wall line there / map says open: stray echo
        c = HW_FRONT_GAIN * clamp(e, -15.0, 15.0)
        self.shift(c)
        s = self.sa()
        self.sa0 = s - 0.1 * (s - 4.0) if s > 4.0 else s
        self.odo = 0.0

    # ---- along-track uncertainty and absolute fixes (board) -------------
    # 1-sigma along error: the last fix's, plus 5% of the distance driven.
    def sa(self):
        s = self.sa0 + 0.05 * self.odo
        return s if s < 40.0 else 40.0

    # Absolute along-track measurement saying the estimate is e mm short
    # (sigma sm): apply the share the two uncertainties call for.
    def abs_fix(self, e, sm):
        s = self.sa()
        s2 = s * s
        m2 = sm * sm
        c = e * s2 / (s2 + m2)
        self.shift(c)
        s = _sqrt(s2 * m2 / (s2 + m2))
        self.sa0 = s if s > 3.0 else 3.0
        self.odo = 0.0
        return c

    # Move the along estimate by c mm, and the edge tracker's stored
    # positions with it: they are in the same frame (v49: a wall starting on
    # both sides at once got the same correction applied twice).
    def shift(self, c):
        d = self.d
        self.X += DX[d] * c
        self.Y += DY[d] * c
        self.ea[0] += c
        self.ea[1] += c
        self.eca[0] += c
        self.eca[1] += c

    # Board, position lost (sa >= 30: after a contact or a slip): a mapped
    # wall straight ahead beyond FRONT_NEAR still gives a coarse along fix.
    # Those readings are ~10% short (the beam catches the floor), so this is
    # only used when the estimate is worse than that.
    def far_fix(self, f):
        if self.sa() < 30.0 or f < FRONT_NEAR or f > 400:
            return 0
        bw = self.mapped_wall(self.d)
        if bw is None:
            return 0
        e = bw - FRONT_OFF - (f + 0.1 * (f - FRONT_CENTER)) - self.along(self.d)
        if fabs(e) > 150.0:
            return 0
        # v71e: at most 30 mm, weighted lightly (v71d: a 269 reading - long,
        # not short - moved the estimate -52 mm and the line-up drove 41 mm
        # into a corner)
        c = self.abs_fix(clamp(e, -30.0, 30.0), 25.0)
        log("far fix", int(c), "front", f)
        return int(c)

    # Sigma of a median of n front readings standing still (+-16 mm each).
    @staticmethod
    def front_sig(n):
        return _sqrt(400.0 / (n if n > 0 else 1) + 4.0)

    # After a spin: a 90 swaps the along and sideways uncertainties (along
    # error before the turn is sideways error after it), a 180 keeps them;
    # the spin itself adds ~3 mm of axle slip. Never below ~8 mm sideways:
    # a calibration error (FRONT_CENTER, side offset) is not in the model,
    # and latfix right after the turn is the check that catches it.
    # Edge trackers restart.
    def swap_axes(self, delta):
        self.sl_d = -1
        sa = self.sa()
        if fabs(fabs(wrap180(delta)) - 90.0) < 45.0:
            p = self.p11
            self.p11 = (sa * sa if sa > 15.0 else 225.0) + 9.0
            self.sa0 = _sqrt((p if p > 25.0 else 25.0) + 9.0)
        else:
            if self.p11 < 225.0:
                self.p11 = 225.0
            self.sa0 = _sqrt(sa * sa + 9.0)
        self.odo = 0.0
        self.p12 = 0.0
        self.rC = [8190, 8190, 8190]    # front readings of the old axis no longer apply
        self.mC = 8190
        self.edge_reset()

    def edge_reset(self):
        self.ecls[0] = 0
        self.ecls[1] = 0
        self.ec[0] = 0
        self.ec[1] = 0
        self.eo[0] = 0
        self.eo[1] = 0

    # ---- side-wall edges: absolute along fixes on the move (board) -------
    # A side reading switching between wall (< SIDE_WALL) and open (>= 140)
    # means the side sensor just passed a cell boundary. Confirmed only once
    # the new class has lasted 40 mm (a lone post between two open sides
    # reads "wall" for ~30 mm: every edge undo in the v46 run was one), and
    # cancelled by two old-class readings in a row (one is a stray). Placed
    # midway between the last old and the first new reading (corrected for
    # ToF age and the beam slant). Dropouts and 100-140 readings are ignored.
    def edge_track(self):
        if self.v < 40.0:
            return
        d = self.d
        psi = wrap180(self.th - 90.0 * d) * D2R
        if fabs(psi) > 0.1:
            self.ec[0] = 0
            self.ec[1] = 0
            self.eo[0] = 0
            self.eo[1] = 0
            return
        s = 0
        while s < 2:
            a = self.along(d) + SIDE_X - self.v * HW_TOF_LAG
            r = self.dL if s == 0 else self.dR
            if r < SIDE_WALL:
                k = 1
            elif 140.0 <= r < 8190:
                k = 2
            else:
                s += 1
                continue
            ah = a - SIDE_NOM * psi if s == 0 else a + SIDE_NOM * psi
            if self.ecls[s] == 0:
                self.ecls[s] = k
                self.en[s] = 1
                self.ea[s] = ah
            elif k == self.ecls[s]:
                if self.ec[s] > 0:
                    self.eo[s] += 1
                    if self.eo[s] >= 2:
                        self.ec[s] = 0  # back to the old class: a post or strays
                        self.eo[s] = 0
                        self.en[s] += 1
                        self.ea[s] = ah
                else:
                    self.en[s] += 1
                    self.ea[s] = ah
            else:
                self.eo[s] = 0
                if self.ec[s] == 0:
                    self.eca[s] = ah
                self.ec[s] += 1
                if self.ec[s] >= 2 and ah - self.eca[s] >= 40.0:
                    gap = self.eca[s] - self.ea[s]
                    if self.en[s] >= 3 and 0.0 <= gap < 25.0:
                        self.edge_event(s, self.ecls[s], 0.5 * (self.ea[s] + self.eca[s]))
                    self.ecls[s] = k
                    self.en[s] = self.ec[s]
                    self.ea[s] = ah
                    self.ec[s] = 0
            s += 1

    # A contact or front guard ends a drive in cell (x, y): close its edge d
    # on the map so the planner cannot send the mouse back into it. Board:
    # not an edge the map knows is open (it was driven or sensed open; the
    # v46 crash closed one and the room became unreachable), and a bump
    # with no wall seen close ahead was a side post, not a wall across.
    def mark_hit(self, x, y, d, front):
        sv = self.solver
        if sv is None or not sv.inside(x, y):
            return
        if POS_MODEL:
            b = 1 << d
            i = y * sv.cols + x
            k = 4 * i + d
            n = self.hits.get(k, 0) + 1
            self.hits[k] = n
            if n >= 2:
                # hit twice at the same place: whatever it is, the mouse
                # cannot get through there - close it and go round (v50
                # retried a third time, slipped and lost its position)
                log("hit", n, "times at", x, y, "side", d, "- closed, going round")
            elif (sv.known[i] & b) and not (sv.wall[i] & b):
                log("front guard at a known opening - map kept")
                return
            elif not front and not (self.aC < 6 and self.mC < FRONT_NEAR + 30.0):
                log("side hit - map kept")
                return
        sv.set_wall(x, y, d, 1)

    # Cell index containing along-position a on axis d, in the corridor the
    # mouse is in (-1 = outside the map).
    def cell_at(self, d, a):
        sv = self.solver
        if d & 1:
            x = self.X
            y = DY[d] * a
        else:
            x = DX[d] * a
            y = self.Y
        if x < 0.0 or y < 0.0:
            return -1
        x = int(x / CELL)
        y = int(y / CELL)
        if not sv.inside(x, y):
            return -1
        return y * sv.cols + x

    # Side s (0 = L, 1 = R) changed class at sensor along-position ae: old =
    # 1 wall -> open, 2 open -> wall. Only used if the map agrees: the cell
    # behind that boundary has the old class on that side, the cell ahead
    # the new one (if the cell ahead is not mapped yet, the fix is applied
    # now and undone if the cell's walls, sensed at the stop, disagree).
    def edge_event(self, s, old, ae):
        sv = self.solver
        if sv is None:
            return
        d = self.d
        bnd = CELL * float(round(ae / CELL))
        e = bnd + (self.ep if old == 1 else -self.ep) - ae
        # accept what the current uncertainty allows: ~25 mm once lined up on a
        # wall, up to ~40-60 mm while unsure (a start placed a few cm off -
        # v63's fixed 25 mm blocked exactly that correction)
        sa = self.sa()
        if fabs(e) > min(60.0, 2.0 * _sqrt(sa * sa + 144.0)):
            return
        sd = (d + 1) & 3 if s == 0 else (d + 3) & 3
        bit = 1 << sd
        cb = self.cell_at(d, bnd - CELL / 2.0)
        ca = self.cell_at(d, bnd + CELL / 2.0)
        if cb < 0 or ca < 0 or not (sv.known[cb] & bit):
            return
        if (1 if (sv.wall[cb] & bit) else 0) != (1 if old == 1 else 0):
            return
        want = 0 if old == 1 else 1     # the cell ahead must have the new class
        prov = 0
        if sv.known[ca] & bit:
            if (1 if (sv.wall[ca] & bit) else 0) != want:
                return
        else:
            prov = 1
        c = self.abs_fix(e, 12.0)       # edges scatter 10-20 mm (v62 log): count for less
        if prov:
            if len(self.epend) < 4:
                self.epend.append((ca, bit, want, d, c, ae - bnd))
        else:
            self.edge_stat(old, ae - bnd)
        log("edge", "L" if s == 0 else "R", "wo" if old == 1 else "ow", "e", int(e), "fix", int(c),
            "new" if prov else "map", "p", int(self.ep))

    # Learn the edge offset from confirmed edges (x = sensor position minus
    # boundary): a wall end is seen p past the boundary and a wall start p
    # before it, while a position error shifts both the same way - so half
    # the difference of the two averages is p.
    def edge_stat(self, old, x):
        if old == 1:
            self.exw = x if self.enw == 0 else self.exw + 0.25 * (x - self.exw)
            self.enw += 1
        else:
            self.exo = x if self.eno == 0 else self.exo + 0.25 * (x - self.exo)
            self.eno += 1
        if self.enw >= 2 and self.eno >= 2:
            self.ep = clamp(0.5 * (self.exw - self.exo), 12.0, 22.0)   # v62 drifted to 24

    # After a new cell's walls are recorded: undo edge fixes it contradicts.
    def edge_confirm(self):
        sv = self.solver
        for (ca, bit, want, dd, c, x) in self.epend:
            if not (sv.known[ca] & bit):
                continue
            if (1 if (sv.wall[ca] & bit) else 0) != want:
                self.X -= DX[dd] * c
                self.Y -= DY[dd] * c
                log("edge undo", int(c))
            else:
                self.edge_stat(1 if want == 0 else 2, x)
        self.epend = []

    # ---- in-place turn by a heading change (deg, map frame) --------------
    def turn_by(self, delta):
        if POS_MODEL:
            # board: the axle stays put in every spin. Their odometry creep is
            # not real: after 180s the estimate sat 7-10 mm left (v49/v51), and
            # v71b: the left encoder under-counts running backwards in left
            # spins, so 90s got a fake ~20 mm move too (latfix corr 19)
            x = self.X
            y = self.Y
            self.turn_core(delta)
            self.X = x
            self.Y = y
            return
        self.turn_core(delta)

    def turn_core(self, delta):
        if TURN_BB:
            self.turn_bb(delta)
            return
        self.iw = 0.0
        target = self.th + delta
        n = 0
        good = 0
        ovs = 0.0
        t0 = self.now()
        lim = int(3000 / DT_MS) + int(fabs(delta) * 4.0 / DT_MS)
        self.wturn = 0.0
        while n < lim:
            err = target - self.th
            if delta * err < 0.0 and fabs(err) > ovs:
                ovs = fabs(err)
            # where the heading will end up if the command dropped now
            # (the motors keep turning for about TURN_LEAD seconds)
            ep = err - self.rate * TURN_LEAD
            if fabs(ep) < TURN_TOL and fabs(err) < TURN_TOL + 1.0 and fabs(self.rate) < 40.0:
                good += 1
                if good >= 2:
                    break
            else:
                good = 0
            if TURN_KLIN > 0.0:
                # board: decelerate on the predicted error, finish on a gentle
                # linear approach, ramp the rate up instead of kicking
                mag = fabs(ep)
                w = _sqrt(2.0 * ALPHA_TURN * mag)
                lin = TURN_KLIN * mag
                if lin < w:
                    w = lin
                if w > W_TURN:
                    w = W_TURN
                if w < TURN_WMIN and fabs(ep) >= TURN_TOL:
                    w = TURN_WMIN
                if ep < 0.0:
                    w = -w
                up = ALPHA_TURN * self.dt
                if w > 0.0:
                    cap = (self.wturn if self.wturn > 0.0 else 0.0) + up
                    if w > cap:
                        w = cap
                elif w < 0.0:
                    cap = (self.wturn if self.wturn < 0.0 else 0.0) - up
                    if w < cap:
                        w = cap
                self.wturn = w
            else:
                mag = fabs(err)
                wr = _sqrt(2.0 * ALPHA_TURN * mag)
                if wr > W_TURN:
                    wr = W_TURN
                w = wr + 6.0 * mag
                if w > W_TURN:
                    w = W_TURN
                if err < 0.0:
                    w = -w
            self.step(0.0, w)
            n += 1
        self.settle()
        err = target - self.th
        if delta * err < 0.0 and fabs(err) > ovs:
            ovs = fabs(err)
        if DEBUG:
            log("turn", int(delta), "err", int(10 * err) / 10.0, "ovs", int(10 * ovs) / 10.0,
                "ms", self.now() - t0)
        self.p22 += 0.0005      # yaw slide when the wheels stop
        self.p11 += 25.0

    # v69 board: stopped on a new cell, the still side medians (80 readings)
    # are the best sideways measurement there is. Set the sideways position
    # from them. And if the walls had to move the estimate since the last
    # still fix on this straight, the gyro heading was off by that much per
    # mm driven: correct it, standing still, a bounded step at a time (v67:
    # the gyro said 3-8 deg right while the walls showed it drifting left,
    # and the 4 deg steering limit could not win).
    def stop_lat(self):
        if not HW_STOP_LAT:
            return
        okl = self.sLm and self.sLn >= 15 and fabs(self.sLm - NOM_L) < 25.0
        okr = self.sRm and self.sRn >= 15 and fabs(self.sRm - NOM_R) < 25.0
        if okl and okr and fabs(self.sLm + self.sRm - NOM_L - NOM_R) > 12.0:
            # v71j: the two walls do not add up to a corridor (both demos at
            # 1 2: L+R 121 and 145 against ~104) - the beams saw wall ends or
            # posts. Trust nothing here (they "fixed" the heading 3-4 deg wrong)
            log("stop lat skipped - side readings", self.sLm, self.sRm)
            return
        d = self.d
        psi = wrap180(self.th - 90.0 * d) * D2R
        hc = SIDE_X * _sin(psi)
        if okl and okr:
            z = 0.5 * ((NOM_L - self.sLm) + (self.sRm - NOM_R)) - hc
            m2 = 9.0
        elif okl:
            z = NOM_L - self.sLm - hc
            m2 = 36.0
        elif okr:
            z = self.sRm - NOM_R - hc
            m2 = 36.0
        else:
            return
        e = z - (self.lateral(d) - self.cell_lat(self.cx, self.cy, d))
        p = self.p11
        g = p / (p + m2)
        dl = g * e
        self.X -= DY[d] * dl
        self.Y += DX[d] * dl
        self.p11 = p * m2 / (p + m2)
        a = self.along(d)
        c = 0.0
        if okl and okr and self.sl_d == d and 120.0 < fabs(a - self.sl_a) < 1100.0:
            c = clamp(0.5 * (self.kfl + e) / fabs(a - self.sl_a), -0.044, 0.044)    # v71j: at most 2.5 deg per stop
            self.th += c / D2R
        if fabs(z) > 5.0:
            self.boost_to = a + 150.0
        self.sl_d = d
        self.sl_a = a
        self.kfl = 0.0
        log("stop lat", int(z), "corr", int(dl), "hdg fix", int(10 * c / D2R) / 10.0)

    # After a turn: any along error from before the turn is now a SIDEWAYS
    # error in the new corridor. Standing still, take the median side
    # readings and set the lateral position from them, so the drive steers
    # back to the centre from the first millimetre (noisy readings while
    # moving found it too late, and it clipped the left posts).
    def lat_fix(self, steps):
        if not HW_TURN_LATFIX or IS_SIM:
            return
        if steps > 0:
            sl = []
            sr = []
            n = 0
            while n < steps:
                self.step(0.0, 0.0)
                if self.dL < SIDE_WALL:
                    sl.append(self.dL)
                if self.dR < SIDE_WALL:
                    sr.append(self.dR)
                n += 1
            self.side_medians(sl, sr)
        # a side median only counts if it is plausible for a wall next to a
        # roughly centred mouse (one noisy side once read 84 mm -> fake 40 mm)
        okl = self.sLm and fabs(self.sLm - NOM_L) < 25.0
        okr = self.sRm and fabs(self.sRm - NOM_R) < 25.0
        if okl and okr and fabs(self.sLm + self.sRm - NOM_L - NOM_R) > 12.0:
            if fabs(self.sLm - NOM_L) <= fabs(self.sRm - NOM_R):     # v71j: not a clean corridor: the nearer-to-normal side only
                okr = 0
            else:
                okl = 0
        if POS_MODEL:
            # weighted by the noise: a median of n side readings (+-24 mm
            # each) is good to 30/sqrt(n) mm; one wall alone also carries the
            # corridor-width uncertainty. The turn already set p11 from the
            # along uncertainty before it.
            vl = 900.0 / (self.sLn if self.sLn > 0 else 1) + 9.0
            vr = 900.0 / (self.sRn if self.sRn > 0 else 1) + 9.0
            # v66: the side sensors sit SIDE_X ahead of the axle, so a heading
            # error after the turn moves their readings (v65 U-turn: 5 deg left
            # read as 8 mm left, the axle was ~3 mm left)
            psi = wrap180(self.th - 90.0 * self.d) * D2R
            hc = SIDE_X * _sin(psi)
            if okl and okr:
                z = clamp(0.5 * ((NOM_L - self.sLm) + (self.sRm - NOM_R)) - hc, -25.0, 25.0)
                m2 = 0.25 * (vl + vr)
            elif okl:
                z = clamp(NOM_L - self.sLm - hc, -25.0, 25.0)     # v66 clipped at 15 when it was ~22 off
                m2 = vl + 25.0
            elif okr:
                z = clamp(self.sRm - NOM_R - hc, -25.0, 25.0)
                m2 = vr + 25.0
            else:
                return
            k = self.piv_k
            if HW_PIVOT_LEARN and self.front_turn and self.turn_side != 0 and k >= 0:
                # after a 90 lined up on a wall, the sideways offset now is how
                # far the spin ENDED from the cell centre along the old axis
                # (+ = towards the old front wall), whether from the front
                # reading or the spin itself. Learn where to line up for each
                # spin direction (v65: 3-17 mm short after every left turn).
                tw = clamp(-z if self.turn_side > 0 else z, -10.0, 10.0)
                r = 0.35 if (okl and okr) else 0.2      # one wall: also carries the side-reading uncertainty
                self.piv[k] = clamp(self.piv[k] + r * tw, -HW_PIVOT_MAX, HW_PIVOT_MAX)
                self.piv[1 - k] = clamp(self.piv[1 - k] + 0.4 * r * tw, -HW_PIVOT_MAX, HW_PIVOT_MAX)  # a front-reading offset is shared
            self.piv_k = -1
            self.front_turn = 0
            self.turn_side = 0
            d = self.d
            p = self.p11
            g = p / (p + m2)
            e = z - (self.lateral(d) - self.cell_lat(self.cx, self.cy, d))
            dl = g * e
            self.X -= DY[d] * dl
            self.Y += DX[d] * dl
            self.p11 = p * m2 / (p + m2)
            self.boost_to = self.along(d) + 150.0       # get back to the centre line quickly
            self.sl_d = d
            self.sl_a = self.along(d)
            self.kfl = 0.0
            log("latfix", int(z), "mm corr", int(dl), "g", int(100 * g), "hdg", int(psi / D2R), "side L/R", self.sLm, self.sRm,
                "piv ccw/cw", int(self.piv[0]), int(self.piv[1]))
            return

    # Turn on the spot to heading nd without swinging the nose into a wall.
    # Spinning, the two FRONT corners swing out (~86 mm) only on the side
    # the mouse turns towards (and ahead / behind); the other side only meets
    # the short rear corners (~61 mm). So: 90s go towards an opening as
    # usual; a 180 turns towards the open side; a 90 that would face a wall
    # first faces the open opposite side and steps along it.
    def safe_turn(self, nd):
        sv = self.solver
        d = self.d
        if nd == d or SPIN_SHIFT <= 0.0 or sv is None or not sv.inside(self.cx, self.cy):
            self.turn_to(nd)
            return
        w = sv.wall[self.cy * sv.cols + self.cx]
        if nd == ((d + 2) & 3):
            self.turn_180(w)
        elif (w & (1 << nd)) and not (w & (1 << ((nd + 2) & 3))):
            o = (nd + 2) & 3
            self.turn_to(o)
            self.move_to(o, self.cell_along(self.cx, self.cy, o) + SPIN_SHIFT)
            self.turn_180(w)
            log("safe turn via", o)
        else:
            self.turn_to(nd)

    # 180 on the spot, turning towards the open side (w = this cell's walls).
    def turn_180(self, w):
        d = self.d
        lt = (d + 1) & 3
        rt = (d + 3) & 3
        if (w & (1 << ((d + 2) & 3))) and not (w & (1 << d)):
            # the back is swept too: step off a wall behind first
            self.move_to(d, self.cell_along(self.cx, self.cy, d) + SPIN_SHIFT)
        elif (w & (1 << d)) and not (w & (1 << ((d + 2) & 3))):
            # dead end: back ~15 mm off the end wall so the nose clears it
            self.move_to(d, self.cell_along(self.cx, self.cy, d) - 15.0)
        if (w & (1 << rt)) and not (w & (1 << lt)):
            sg = 1.0                    # CCW: swings over the open left
        elif (w & (1 << lt)) and not (w & (1 << rt)):
            sg = -1.0
        else:
            if (w & (1 << lt)) and (w & (1 << rt)):
                self.front_check(15)    # fresh side readings to pick the roomier side
            sg = 1.0
            if self.sLm and self.sRm and (self.sLm - NOM_L) < (self.sRm - NOM_R):
                sg = -1.0               # both walls: away from the nearer one
        nd = (d + 2) & 3
        delta = 90.0 * nd - self.th
        while sg > 0.0 and delta <= 0.0:
            delta += 360.0
        while sg < 0.0 and delta >= 0.0:
            delta -= 360.0
        while delta > 270.0:
            delta -= 360.0
        while delta < -270.0:
            delta += 360.0
        self.turn_by(delta)
        self.d = nd
        if POS_MODEL:
            self.turn_side = 0          # a 180 tells nothing about the pivot along the old axis
            self.swap_axes(delta)
            return
        self.p12 = 0.0
        if self.p11 < 100.0:
            self.p11 = 100.0


    def turn_to(self, nd):
        self.rB = [0.0, 1e9, 2e9]       # new axis: old wall-position readings no longer apply
        self.boost_to = -1e9
        delta = wrap180(90.0 * nd - self.th)
        if fabs(delta) < 3.0:
            self.d = nd
            return
        self.turn_by(delta)
        self.d = nd
        if POS_MODEL:
            self.turn_side = 0
            if fabs(fabs(wrap180(delta)) - 90.0) < 30.0:
                self.turn_side = 1 if delta > 0.0 else -1
            self.swap_axes(delta)
            return
        self.p12 = 0.0
        p = self.p11
        self.p11 = 100.0 if p < 100.0 else p

    # ---- straight driving ----------------------------------------------
    # One corridor-tracking step along axis d at commanded speed vc: lateral
    # offset -> heading demand -> heading loop -> gyro rate loop, then the
    # Kalman update from the side walls and the front-wall distance fix.
    def track(self, d, latc, vc):
        if self.enc_chk:
            self.first_drive_check()
        lat = self.lateral(d) - latc
        psi = wrap180(self.th - 90.0 * d) * D2R
        if POS_MODEL:
            # heading held >12 deg off for 0.3 s while driving: a corner is
            # caught on a post (v46 crash: 18 deg, wheels slipping, encoders
            # still counting) - treat it as a bump
            if vc > 30.0 and fabs(psi) > 0.21:
                self.pin += 1
                if self.pin > 30:
                    self.pin = 0
                    self.bump = 1
                    self.sa0 = 40.0     # the wheels slipped: along position unknown
                    self.odo = 0.0
                    log("PINNED - heading stuck at", int(psi / D2R))
            else:
                self.pin = 0
            # wheels and gyro disagree on the turn rate for 0.2 s: a wheel is
            # held or slipping (v47: corner on the room's centre post - the
            # encoders counted a cell, the mouse moved 75 mm). Caught early,
            # before the position is far off.
            ew = (self.vrw - self.vlw) / (2.0 * HALF_AXLE * D2R)
            if vc > 30.0 and fabs(ew - self.rate_p) > 45.0:
                self.slip += 1
                if self.slip > 20:
                    self.slip = 0
                    self.bump = 1
                    self.sa0 = 40.0
                    self.odo = 0.0
                    log("SLIP - wheels", int(ew), "gyro", int(self.rate_p), "deg/s  L", int(self.vlw), "R",
                        int(self.vrw), "mm/s")
                    if fabs(self.vlw) < 15.0 and fabs(self.vrw) > 80.0:
                        log("LEFT ENCODER stopped counting while driving (cable / battery?)")
                    elif fabs(self.vrw) < 15.0 and fabs(self.vlw) > 80.0:
                        log("RIGHT ENCODER stopped counting while driving (cable / battery?)")
            else:
                self.slip = 0
        pm = PSI_MAX
        if POS_MODEL and self.along(d) < self.boost_to:
            pm = HW_PSI_TURN            # just after a turn + latfix: steer back to centre faster
        pref = clamp(-KLAT * lat, -pm, pm)
        w = clamp(KPSI * (pref - psi) / D2R, -120.0, 120.0)
        # SIDE REFLEX: the closer a side wall is inside SIDE_GUARD, the harder
        # the mouse turns away from it (median of the last three valid
        # readings, so one stray short echo does not kick the heading)
        if SIDE_GUARD > 0.0:
            gr = SIDE_GUARD
            if POS_MODEL and SIDE < 0:
                gr = HW_SIDE_GUARD_L
            if self.aR < 6 and self.mR < gr:
                w += clamp(5.0 * (gr - self.mR), 0.0, 120.0)
            gl = SIDE_GUARD
            if POS_MODEL and SIDE > 0:
                gl = HW_SIDE_GUARD_L        # physical left (mirrored frame: map R is the left)
            if self.aL < 6 and self.mL < gl:
                w -= clamp(5.0 * (gl - self.mL), 0.0, 120.0)
        self.wlast = w
        self.step(vc, w)
        self.kf(latc)
        self.front_fix()
        if EDGE_FIX:
            self.edge_track()

    # Trapezoidal profile: accelerate at ACCEL, cap at vmax, and never be
    # faster than what still allows reaching v_end at the end of `rem` mm.
    def profile(self, rem, v_end, vmax):
        if rem < 0.0:
            rem = 0.0
        vt = _sqrt(v_end * v_end + 2.0 * ACCEL * rem)
        if vt > vmax:
            vt = vmax
        vc = self.vc
        if vt > vc:
            vc += ACCEL * self.dt
            if vc > vt:
                vc = vt
        else:
            vc = vt
        if v_end == 0.0 and vc < STOP_CREEP and rem > 3.0:
            vc = STOP_CREEP
        self.vc = vc
        return vc

    # Exploration drive from the centre of (cx, cy) along self.d. Each cell
    # it approaches is classified DECIDE_AHEAD mm before its centre; if the
    # solver wants to carry straight on, the goal is extended by a cell and
    # the mouse never stops. Ends stopped on a cell centre.
    def explore_drive(self, solver, vmax):
        d = self.d
        x0 = self.cx
        y0 = self.cy
        a0 = self.cell_along(x0, y0, d)
        latc = self.cell_lat(x0, y0, d)
        self.epend = []
        goal = 1
        k = 1                       # cell being approached (relative index)
        decided = 0
        vl = 0
        nl = 0
        vr = 0
        nr = 0
        fs = 0
        fo = 0
        tl = 0
        guard = 0
        swl = []                    # board: this cell's wall readings (side-centre learning)
        swr = []
        self.vc = 0.0
        if KEEP_IW:
            self.iw = self.iw_fwd
        steps = 0
        while steps < 4000:
            rel = self.along(d) - a0
            s = rel + SIDE_X - CELL * k
            if -70.0 <= s <= 55.0:
                tl += 1
                # 8190 = no reading (dropout), not "open": only valid
                # readings vote, short = wall, long = open
                if self.dL < 8190:
                    nl += 1
                    if self.dL < SIDE_WALL:
                        vl += 1
                        if POS_MODEL and len(swl) < 30:
                            swl.append(self.dL)
                if self.dR < 8190:
                    nr += 1
                    if self.dR < SIDE_WALL:
                        vr += 1
                        if POS_MODEL and len(swr) < 30:
                            swr.append(self.dR)
                if self.dC < 8190:
                    if fabs(self.along(d) + FRONT_OFF + self.dC - (a0 + CELL * k + CELL / 2.0)) < 60.0:
                        fs += 1
                    else:
                        fo += 1
            if STOP_EACH and SKIP_SEEN and not decided and rel >= CELL * k - DECIDE_AHEAD:
                # stop-each mode: a cell already mapped needs no stop - if the
                # planner carries straight on through it, keep driving (every
                # restart's wheel slip would cost along-track accuracy)
                decided = 1
                x1 = x0 + DX[d] * k
                y1 = y0 + DY[d] * k
                if solver.inside(x1, y1):
                    i1 = y1 * solver.cols + x1
                    rm = solver.room
                    if solver.visited[i1] and not (rm is not None and i1 in rm) \
                            and solver.is_open(i1, d, 0) and solver.next_dir(x1, y1, d) == d:
                        log("pass", x1, y1)
                        goal = k + 1
                        k += 1
                        decided = 0
            if not decided and not STOP_EACH and rel >= CELL * k - DECIDE_AHEAD:
                # wall = short readings outnumber long ones AND appear in at
                # least a quarter of all samples (stray short echoes do not)
                wl = 1 if (2 * vl > nl and vl >= 2 and 4 * vl >= tl) else 0
                wr = 1 if (2 * vr > nr and vr >= 2 and 4 * vr >= tl) else 0
                wf = 1 if (fs > fo and fs >= 2 and 4 * fs >= tl) else 0
                self.vote = (vl, nl, vr, nr)
                if POS_MODEL and wl and wr:
                    self.nom_learn(swl, swr)
                x1 = x0 + DX[d] * k
                y1 = y0 + DY[d] * k
                i1 = y1 * solver.cols + x1
                if POS_MODEL and solver.inside(x1, y1) and solver.visited[i1]:
                    # board, continuous: a cell already mapped keeps its map (a
                    # quick vote on the move must never add a wall to it)
                    rm = solver.check_room()
                    nd = -2 if (rm is not None and i1 in rm) else solver.next_dir(x1, y1, d)
                    wf = 0 if solver.is_open(i1, d, 0) else 1
                    if DEBUG:
                        log("pass", x1, y1, "next", nd)
                else:
                    wl, wr = self.side_check(wl, wr, wf)
                    nd = record_cell(solver, x1, y1, d, wl, wf, wr)
                    if EDGE_FIX:
                        self.edge_confirm()
                    if DEBUG and POS_MODEL:
                        log("cell", x1, y1, "d", d, "walls L F R", wl, wf, wr, "votes L", vl, "/", nl, "F", fs, "/",
                            fs + fo, "R", vr, "/", nr, "next", nd, "th", int(self.th))
                    elif DEBUG:
                        log("cell", x1, y1, "d", d, "ToF L/C/R", self.dL, self.dC, self.dR,
                            "votes L", vl, "/", nl, "R", vr, "/", nr, "F", fs, "/", fs + fo, "-> wall L F R", wl, wf, wr, "next", nd,
                            "th", int(self.th))
                decided = 1
                if nd == d and wf == 0:
                    goal = k + 1
                    k += 1
                    decided = 0
                    vl = 0
                    nl = 0
                    vr = 0
                    nr = 0
                    fs = 0
                    fo = 0
                    tl = 0
                    swl = []
                    swr = []
            rem = CELL * goal - rel + STOP_EXTRA
            self.a_goal = a0 + CELL * goal
            if rem <= self.v * (TAU * 0.5 + (0.025 if BRAKE_HOLD else 0.0)) + 1.0:
                break
            if 0 < self.dC < 15:
                guard += 1
            else:
                guard = 0
            if self.bump and POS_MODEL:
                self.contact()              # board: stops the run (HW_ON_CONTACT = 0: drives on)
            if self.bump:
                self.back_off()
                goal = int((self.along(d) - a0 + CELL / 2.0) / CELL)
                if goal < 0:
                    goal = 0
                self.mark_hit(x0 + DX[d] * goal, y0 + DY[d] * goal, d, 0)
                break
            if self.front_block():
                if rem > 70.0:
                    guard = 3
                else:
                    break               # wall right where this cell's front edge is: arrived
            if guard >= 3:
                # unexpected wall right ahead: stop in the cell we are in and
                # put the wall on the map so the planner cannot send us back
                goal = int((rel + CELL / 2.0) / CELL)
                self.mark_hit(x0 + DX[d] * goal, y0 + DY[d] * goal, d, 1)
                log("FRONT GUARD")
                break
            if fabs(wrap180(self.th - 90.0 * d)) > 35.0:
                log("HEADING LOST - stop and straighten")
                goal = int((rel + CELL / 2.0) / CELL)
                if goal < 0:
                    goal = 0
                self.settle()
                self.turn_to(d)
                break
            self.track(d, latc, self.profile(rem, 0.0, vmax))
            steps += 1
        if KEEP_IW:
            self.iw_fwd = self.iw
        self.stop_hold(d)
        self.vc = 0.0
        self.cx = x0 + DX[d] * goal
        self.cy = y0 + DY[d] * goal

    # Straight piece of a planned path: run along axis d (corridor line latc)
    # until the along-position reaches a_end, arriving at v_end. v_end = 0
    # means stop there.
    def straight(self, d, latc, a_end, v_end, vmax):
        self.d = d
        self.epend = []
        if KEEP_IW:
            self.iw = self.iw_fwd
        self.a_goal = a_end if v_end == 0.0 else 1e9
        n = 0
        while n < 3000:
            rem = a_end - self.along(d) + (STOP_EXTRA if v_end == 0.0 else 0.0)
            if v_end > 0.0:
                if rem <= 0.0:
                    break
            elif rem <= self.v * (TAU * 0.5 + (0.025 if BRAKE_HOLD else 0.0)) + 1.0:
                break
            if self.bump and POS_MODEL:
                self.contact()
            if 0 < self.dC < 15 or self.bump or self.front_block():
                hit = self.bump
                if self.bump:
                    self.back_off()
                if rem > 70.0 or v_end > 0.0:
                    # a wall where the map says open: stop in this cell, put
                    # the wall on the map, and let the caller replan
                    self.lost = 1
                    log("FRONT GUARD rem", int(rem))
                    self.mark_hit(int(self.X / CELL), int(self.Y / CELL), d, 0 if hit else 1)
                break
            if fabs(wrap180(self.th - 90.0 * d)) > 35.0:
                log("HEADING LOST - stop and straighten")
                self.lost = 1
                break
            self.track(d, latc, self.profile(rem, v_end, vmax))
            if TEST_STRAIGHT and n % 8 == 0:
                # telemetry for the straight-line calibration run
                print("along", int(self.along(d)), "lat", int(self.lateral(d) - latc),
                      "head", int(10 * (self.th - 90.0 * d)) / 10.0, "w_cmd", int(self.wlast),
                      "rate", int(self.rate), "v", int(self.v), "ToF L/R", self.dL, self.dR,
                      "pwm L/R", self.pl_last, self.pr_last, "trim", int(10 * self.iw) / 10.0)
            n += 1
        if KEEP_IW:
            self.iw_fwd = self.iw

    # Smooth 90 degree turn through cell (ex, ey): quarter circle of radius
    # CELL/2 from the entry-edge midpoint to the exit-edge midpoint, centred
    # on the inner corner post. Feed-forward w = v/R plus feedback on the
    # radius error and on the heading error to the circle tangent.
    def arc(self, din, dout, ex, ey, v):
        sg = 1.0 if dout == ((din + 1) & 3) else -1.0
        r0 = CELL / 2.0
        ox = ex * CELL + r0 - r0 * DX[din] + r0 * DX[dout]
        oy = ey * CELL + r0 - r0 * DY[din] + r0 * DY[dout]
        wff = sg * v / r0 / D2R
        th_end = self.th + wrap180(90.0 * dout - self.th)
        n = 0
        while n < 400:
            rx = self.X - ox
            ry = self.Y - oy
            r = _sqrt(rx * rx + ry * ry)
            if r < 1.0:
                r = 1.0
            # done once the radius points along the entry heading (exit edge
            # reached) or the heading has come round to the new axis
            if rx * DX[din] + ry * DY[din] >= 0.995 * r:
                break
            if sg * (self.th - th_end) > -1.0:
                break
            if sg > 0.0:
                tx = -ry / r
                ty = rx / r
            else:
                tx = ry / r
                ty = -rx / r
            a = self.th * D2R
            eh = _cos(a) * ty - _sin(a) * tx
            w = wff + sg * ARC_KR * (r - r0) + ARC_KH * eh / D2R
            self.vc = v
            self.step(v, w)
            n += 1
        self.d = dout
        self.p11 += 16.0
        self.p12 = 0.0
        self.p22 += 0.0001

    # Execute a planned list of move headings: one in-place turn to the first
    # heading, then straights merged into single profiles joined by arcs.
    # Returns 1 when the path was completed, 0 when a front guard stopped it
    # (the mouse is then stopped in cell (cx, cy) and the caller replans).
    def follow(self, moves, vmax, vturn, rev_last=0, one=0):
        if not moves:
            return 1
        nmv = len(moves)
        f0 = moves[0]
        if f0 != self.d:
            self.align_front(f0)
        if f0 != self.d:
            self.safe_turn(f0)
            self.lat_fix(30 if POS_MODEL else 15)
        self.vc = 0.0
        self.lost = 0
        x = self.cx
        y = self.cy
        i = 0
        while i < nmv:
            d = moves[i]
            j = i
            while j < nmv and moves[j] == d:
                j += 1
                if one:
                    break               # v70 return: one cell at a time, sensing standing still on each
            ex = x + DX[d] * (j - i)
            ey = y + DY[d] * (j - i)
            latc = self.cell_lat(x, y, d)
            ce = self.cell_along(ex, ey, d)
            nf = moves[j] if j < nmv else 0
            if ARCS and j < nmv and moves[j] != ((d + 2) & 3):
                self.straight(d, latc, ce - CELL / 2.0, vturn, vmax)
                if self.lost:
                    break
                self.arc(d, moves[j], ex, ey, vturn)
            else:
                self.straight(d, latc, ce, 0.0, vmax)
                self.stop_hold(d)
                self.vc = 0.0
                if self.lost:
                    break
                if j < nmv:
                    self.cx = ex
                    self.cy = ey
                    if one and nf == d:
                        self.still_walls(40, 0)     # side and front medians standing still
                        self.stop_fix()
                        self.stop_lat()
                    else:
                        self.align_front(nf)
                        self.safe_turn(nf)
                        self.lat_fix(30 if POS_MODEL else 15)
            x = ex
            y = ey
            i = j
        if self.lost:
            self.settle()
            # straighten before the caller replans, else every retry starts
            # with the same bad heading and is lost again straight away
            self.turn_to(self.d)
            self.vc = 0.0
            self.cx = int(self.X / CELL)
            self.cy = int(self.Y / CELL)
            if POS_MODEL:
                self.lat_fix(20)
            return 0
        self.cx = x
        self.cy = y
        return 1

    # Maze turned out to be on the other side of the start heading: mirror the
    # map frame. Only the straight start corridor (row 0) has been driven so
    # far, and it has walls on both sides, so the map itself is unchanged.
    def mirror(self):
        global SIDE
        SIDE = -SIDE
        self.th = -self.th
        self.Y = CELL - self.Y
        self.p12 = -self.p12
        self.dL, self.dR = self.dR, self.dL
        self.hL, self.hR = self.hR, self.hL
        self.aL, self.aR = self.aR, self.aL
        self.rL, self.rR = self.rR, self.rL
        self.mL, self.mR = self.mR, self.mL
        global NOM_L, NOM_R
        t = NOM_L
        NOM_L = NOM_R
        NOM_R = t
        self.edge_reset()
        self.epend = []
        log("maze is on the", "LEFT" if SIDE > 0 else "RIGHT", "of the start heading")

    # First side opening on the start corridor tells which side the maze is
    # on (the other side of that corridor is the arena's outer wall).
    # Mirroring flips the whole map, so on the board it needs a clear vote:
    # one misread right wall must not make every real left opening "outside".
    def side_check(self, wl, wr, wf=0):
        global SIDE_AUTO
        if SIDE_AUTO and (self.d != 0 or self.cy != 0):
            SIDE_AUTO = 0               # left the start corridor: side is settled
        if SIDE_AUTO and (wl == 0 or wr == 0):
            if wl == 1 and wr == 0:
                vl, nl, vr, nr = self.vote
                # (a weak vote is still taken when right is the only way on)
                if FRONT_STOP > 0.0 and not wf and not (nr >= 3 and 3 * vr <= nr and vl >= 3 and 4 * vl >= 3 * nl):
                    log("side: weak right opening", vl, nl, vr, nr, "- not mirroring yet")
                    return wl, wr
                SIDE_AUTO = 0
                self.mirror()
                return wr, wl
            SIDE_AUTO = 0
            log("maze is on the LEFT of the start heading")
        return wl, wr

    # Start cell: sit still for 0.6 s and vote on each wall from the valid
    # ToF readings (dropouts ignored). Also sets the centred side distance.
    def start_walls(self):
        return self.still_walls(60, 1)

    # Stand still at a cell centre and vote on each wall from valid ToF
    # readings (dropouts ignored). calib=1 also measures each side sensor's
    # centred reading (start cell only).
    def still_walls(self, steps, calib):
        global SIDE_NOM, NOM_L, NOM_R
        vl = nl = vr = nr = fs = fo = 0
        sl = []
        sr = []
        sc = []
        gs = 0.0
        gn = 0
        pel = self.rwl
        per = self.rwr
        n = 0
        while n < steps:
            self.step(0.0, 0.0)
            if n >= 6 and self.rwl == pel and self.rwr == per:     # v71i: raw readings (the left count is computed from the gyro now)
                gs += self.rate_p       # wheels dead still: the gyro should read zero
                gn += 1
            pel = self.rwl
            per = self.rwr
            if self.dL < 8190:
                nl += 1
                if self.dL < SIDE_WALL:
                    vl += 1
                    sl.append(self.dL)
            if self.dR < 8190:
                nr += 1
                if self.dR < SIDE_WALL:
                    vr += 1
                    sr.append(self.dR)
            if self.dC < 8190:
                sc.append(self.dC)
                if self.dC + FRONT_OFF < CELL / 2.0 + 60.0:
                    fs += 1
                else:
                    fo += 1
            n += 1
        wl = 1 if (2 * vl > nl and vl >= 2) else 0
        wr = 1 if (2 * vr > nr and vr >= 2) else 0
        wf = 1 if (fs > fo and fs >= 2) else 0
        self.vote = (vl, nl, vr, nr)
        self.fmed = 8190
        if len(sc) >= 3:
            sc.sort()
            self.fmed = sc[len(sc) // 2]
        self.fmed_ok = 1
        self.fvals = sc
        self.vfront = (fs, fs + fo)
        self.side_medians(list(sl), list(sr))
        if not calib:
            self.zupt_add(gs, gn)
        if NOM_FIXED:
            pass                        # measured constants: never re-learned
        elif calib and wl and wr:
            # each sensor's own centred reading: VL53L0X units differ by 10-20
            # mm, and a shared value makes one-wall centring pull to one side
            # median, so stray short echoes and noise do not bias it
            sl.sort()
            sr.sort()
            # mean of the two sides: independent of where in the cell the
            # mouse was placed (an off-centre start must not become "centre")
            SIDE_NOM = clamp(0.5 * (sl[len(sl) // 2] + sr[len(sr) // 2]), 15.0, 95.0)
            NOM_L = SIDE_NOM + 0.5 * SIDE_OFFSET
            NOM_R = SIDE_NOM - 0.5 * SIDE_OFFSET
            self.nom_n = 1
        elif POS_MODEL and wl and wr:
            self.nom_learn(sl, sr)
        return wl, wf, wr

    # Board: walls on both sides re-measure the centred side reading - the
    # mean of the two medians does not depend on where across the corridor
    # the mouse is. Standing still (still_walls) and on the move (continuous
    # exploration: v54/v55 kept the start cell's value all run, 57 in the
    # v55 log against ~51 normally, and every one-wall correction was off).
    def nom_learn(self, sl, sr):
        global SIDE_NOM, NOM_L, NOM_R
        if NOM_FIXED or len(sl) < 8 or len(sr) < 8 or fabs(wrap180(self.th - 90.0 * self.d)) > 10.0:   # the L+R sum hardly changes with heading (v65: 5-7 deg off, never learned)
            return
        sl.sort()
        sr.sort()
        m = 0.5 * (sl[len(sl) // 2] + sr[len(sr) // 2])
        if fabs(m - SIDE_NOM) < 6.0:      # v71j: was 20 - a bad pair at 1 2 (L 66 R 79) moved it 52 -> 61
            SIDE_NOM = m if self.nom_n == 0 else SIDE_NOM + 0.3 * (m - SIDE_NOM)
            self.nom_n += 1
            NOM_L = SIDE_NOM + 0.5 * SIDE_OFFSET * SIDE     # SIDE < 0: map L/R swapped
            NOM_R = SIDE_NOM - 0.5 * SIDE_OFFSET * SIDE
            if self.nom_n == 1 or fabs(SIDE_NOM - self.nom_log) > 2.0:
                self.nom_log = SIDE_NOM
                log("side nom", int(10 * SIDE_NOM) / 10.0)

def explore_more(m):
    global EXPLORE_ALL
    if EXPLORE_ALL and (m.now() - m.t0) / 1000.0 >= HW_EXPLORE_TMAX:
        EXPLORE_ALL = 0                 # out of time: head for the room now
        log("explore time up - going to the room")
    return EXPLORE_ALL


def record_cell(solver, x, y, d, wl, wf, wr):
    if solver.inside(x, y):
        solver.set_wall(x, y, (d + 1) & 3, wl)
        solver.set_wall(x, y, (d + 3) & 3, wr)
        solver.set_wall(x, y, d, wf)
        i = y * solver.cols + x
        b = (d + 2) & 3
        if not (solver.known[i] & (1 << b)):
            solver.set_wall(x, y, b, 0)
        solver.visited[i] = 1
    room = solver.check_room()
    if room is not None and (y * solver.cols + x) in room:
        return -2
    return solver.next_dir(x, y, d)


# On the board: wait for SW1 so the mouse can be placed, then 1 s to let go.
def wait_for_start():
    if IS_SIM == 1 or not HW_WAIT_BUTTON:
        return
    try:
        while uct_mouse.get_button() == 0:
            uct_mouse.delay_ms(50)
        while uct_mouse.get_button() != 0:     # wait for the finger to come off SW1
            uct_mouse.delay_ms(20)
    except Exception:
        return
    uct_mouse.delay_ms(1500)                    # hands clear before the gyro is calibrated




def mission():
    # run the mission; whatever happens (finish, crash, Python error), save
    # the log lines to runlog.txt on the board afterwards
    try:
        _mission()
    except ContactStop:
        pass
    except Exception as e:
        try:
            log("battery at the end", int(100 * uct_mouse.get_vbatt()) / 100.0, "V")
        except Exception:
            pass
        # board: an error used to leave the mouse "frozen" with no clue why
        # (untethered, nobody sees the traceback): show it and log it
        try:
            uct_mouse.set_motors(0, 0)
        except Exception:
            pass
        try:
            log("ERROR", type(e).__name__, str(e)[:60])
        except Exception:
            pass
        try:
            msg = str(e) if str(e) else type(e).__name__
            uct_mouse.display_text(1, "ERR " + msg[:12])
        except Exception:
            pass
        raise
    finally:
        try:
            uct_mouse.set_motors(0, 0)
        except Exception:
            pass
        save_log()


def _mission():
    global SIDE_NOM, LOGBUF, SAVE_OK
    if not uct_mouse.init():
        return
    print("S4 BUILD", BUILD)
    if not on_simulator():
        LOGBUF = ["S4 BUILD " + BUILD]
    try:
        uct_mouse.display_text(1, "S4 " + BUILD[:3])
    except Exception:
        pass
    select_profile()
    uct_mouse.set_polarity(MOT_POL_L, MOT_POL_R)
    try:
        uct_mouse.set_encoder_polarity(1, 1)
    except Exception:
        pass
    rows = MAZE_ROWS
    cols = MAZE_COLS
    if rows <= 0 or cols <= 0:
        try:
            rc = uct_mouse.get_maze_dimensions()
            rows = int(rc[0])
            cols = int(rc[1])
        except Exception:
            rows = 4
            cols = 6
        if IS_SIM == 0:
            # board: the API cannot know which way the mouse was placed, so use
            # a square grid big enough either way (unused cells stay walled off)
            if cols > rows:
                rows = cols
            cols = rows
    solver = MazeSolver(rows, cols)
    m = Mouse()
    m.solver = solver
    if not IS_SIM:
        try:
            import gc
            gc.collect()
            log("mem free", gc.mem_free())      # board RAM left after loading (v56: ~40 KB program)
        except Exception:
            pass
    wait_for_start()
    if LOGBUF is not None:
        # empty the old log only now: the board runs this on every power-up,
        # including plugging in USB to copy the log (v52 wiped it that way)
        try:
            open("runlog.txt", "w").close()
        except Exception:
            pass
    m.calibrate()
    if not IS_SIM:
        try:
            vb = uct_mouse.get_vbatt()
            log("battery", int(100 * vb) / 100.0, "V", "- LOW: charge before testing" if vb < 3.7 else "")
        except Exception:
            pass
    m.enc_chk = 1 if (POS_MODEL and not IS_SIM) else 0     # encoders checked on the first drive (no start nudge)
    if SYSID:
        m.sysid()
    log("maze", rows, "x", cols, "side nom", SIDE_NOM, "start ToF L/C/R", m.dL, m.dC, m.dR,
        "slot counts", m.nv, "gyro gain", GYRO_GAIN)

    if HW_TEST_TURN > 0 and not IS_SIM:
        # calibration: spin turns on the spot; each prints a "turn" line
        # (err = final heading error, ovs = overshoot, tries = 1 is ideal)
        k = 0
        while k < 2 * HW_TEST_TURN:
            m.turn_by(90.0 if k < HW_TEST_TURN else -90.0)
            m.hold(30)
            k += 1
        print("TURN TEST done: heading", m.th, "(should be close to 0)")
        return
    if TEST_STRAIGHT > 0:
        # calibration: drive straight, print heading every cell, stop
        m.follow([0] * TEST_STRAIGHT, V_EXPLORE, V_TURN)
        m.hold(50)
        print("STRAIGHT TEST done: heading", m.th, "deg  lateral est", m.Y - CELL / 2.0, "mm")
        print("Measure how far it ended up left/right of where it should be.")
        return
    # ---------------- Phase 1: explore ----------------
    wl, wf, wr = m.start_walls()
    wl, wr = m.side_check(wl, wr, wf)
    log("start walls L F R", wl, wf, wr, "centred L/R", NOM_L, NOM_R)
    if POS_MODEL and not IS_SIM:
        # placed a few mm off-centre? the still side readings just taken say
        # by how much: set the sideways position now, not over the first cells
        m.p11 = 100.0
        m.lat_fix(0)
        m.sa0 = 15.0                    # along: only as good as the placement until a wall or edge fixes it
    nd = record_cell(solver, 0, 0, 0, wl, wf, wr)
    moves_left = 6 * solver.n
    while nd >= 0 and moves_left > 0:
        moves_left -= 1
        if solver.room is not None and EXPLORE_ALL and not explore_more(m):
            nd = solver.next_dir(m.cx, m.cy, m.d)   # re-plan towards the room
            if nd < 0:
                break
        if nd != m.d:
            m.align_front(nd)
            m.safe_turn(nd)
            fc = m.front_check(FRONT_CHECK) if FRONT_CHECK > 0 else 0
            m.lat_fix(0 if FRONT_CHECK > 0 else 15)     # 0 = reuse the side readings front_check just took
            if fc:
                # facing a wall the map called open (misread or bad turn):
                # never drive into it - mark it and choose again
                solver.set_wall(m.cx, m.cy, m.d, 1)
                nd = solver.next_dir(m.cx, m.cy, m.d)
                log("FRONT CHECK: wall ahead after turn, next", nd)
                continue
        m.explore_drive(solver, V_EXPLORE)
        if STOP_EACH and SKIP_SEEN and solver.visited[m.cy * solver.cols + m.cx]:
            # mapped already: only a quick look ahead to fix the position
            m.front_median(30)
            if m.between():
                m.creep_to_wall()
            log("seen", m.cx, m.cy, "d", m.d, "front", m.fmed, "fix", m.stop_fix(), "th", int(m.th),
                "sa", int(m.sa()))
            m.kfh = 0.0
        elif STOP_EACH:
            # stopped on the new cell's centre: sense standing still, then plan
            wl, wf, wr = m.still_walls(SENSE_STEPS, 0)
            if m.between():
                m.creep_to_wall()
                wl, wf, wr = m.still_walls(SENSE_STEPS, 0)      # sense again where it really is
            wl, wr = m.side_check(wl, wr, wf)
            record_cell(solver, m.cx, m.cy, m.d, wl, wf, wr)
            vl, nl, vr, nr = m.vote
            if wl and 100 * vl < 85 * nl:
                solver.weak.append((m.cx, m.cy, (m.d + 1) & 3))
            if wr and 100 * vr < 85 * nr:
                solver.weak.append((m.cx, m.cy, (m.d + 3) & 3))
            if EDGE_FIX:
                m.edge_confirm()
            fx = m.stop_fix()
            m.stop_lat()
            log("cell", m.cx, m.cy, "d", m.d, "walls L F R", wl, wf, wr, "votes L", m.vote[0], "/", m.vote[1],
                "F", m.vfront[0], "/", m.vfront[1], "R", m.vote[2], "/", m.vote[3], "front", m.fmed,
                "fix", fx, "th", int(m.th), "sa", int(m.sa()), "side", m.sLm, m.sRm)
            m.kfh = 0.0
        i = m.cy * solver.cols + m.cx
        room = solver.check_room()
        if room is not None and i in room and not explore_more(m):
            nd = -2
        else:
            nd = solver.next_dir(m.cx, m.cy, m.d)
            if nd < 0 and solver.room is None and solver.unweak():
                log("no room yet - re-checking doubtful walls")
                nd = solver.next_dir(m.cx, m.cy, m.d)
        log("at", m.cx, m.cy, "d", m.d, "next", nd, "t", (m.now() - m.t0) / 1000.0)
        if m.efl + m.efr != m.ef_log:
            # steps whose encoder count went the wrong way (undone with the gyro)
            m.ef_log = m.efl + m.efr
            log("enc wrong-way steps L", m.efl, "R", m.efr)
    if solver.room is not None and (m.cy * solver.cols + m.cx) not in solver.room:
        # explored the whole maze: now drive to the room for the pirouette
        log("maze mapped - going to the room")
        k = 0
        while k < 4 and (m.cy * solver.cols + m.cx) not in solver.room:
            moves = solver.plan(m.cx, m.cy, m.d, solver.room)
            if not moves:
                break
            m.follow(moves, V_EXPLORE, V_TURN)
            k += 1
    if solver.room is None:
        rm = solver.best_room()
        if rm is not None:
            # no room passed the strict test (a rim wall misread or unseen):
            # use the most room-like open 2x2 block and drive into it
            log("no room confirmed - using best 2x2 block", rm)
            k = 0
            while k < 4 and (m.cy * solver.cols + m.cx) not in rm:
                moves = solver.plan(m.cx, m.cy, m.d, rm)
                if not moves:
                    break
                m.follow(moves, V_EXPLORE, V_TURN)
                k += 1
            if (m.cy * solver.cols + m.cx) in rm:
                solver.room = rm
    if solver.room is None:
        log("no room found")
    else:
        # ---------------- Phase 2: pirouette (clockwise) ----------------
        log("PIROUETTE at", m.cx, m.cy)
        s = m.spin_spot(solver)
        m.turn_by(-360.0 * SIDE)
        if s >= 0:
            m.move_to(s, m.cell_along(m.cx, m.cy, s))     # back to the centre line
    # ---------------- Phase 3: return to start ----------------
    k = 0
    while k < 4:
        moves = solver.plan(m.cx, m.cy, m.d, [0])
        log("return", moves)
        if not moves or m.follow(moves, V_RETURN, V_TURN, 1, HW_RETURN_STOPS if not IS_SIM else 0):
            break
        k += 1                          # a front guard stopped it: replan from here
    m.align_front(0)
    m.safe_turn(0)                      # face the start heading (safe 180: back off, roomier side)
    # ---------------- Phase 4: pause, sprint ----------------
    save_log()                          # standing at the start: write now, not during the pause
    SAVE_OK = 0                         # the sprint never pauses to write (finally writes it)
    m.zupt(int(3000 / DT_MS))
    if solver.room is not None:
        k = 0
        while k < 4:
            moves = solver.plan(m.cx, m.cy, m.d, solver.room)
            log("sprint", moves)
            if not moves or m.follow(moves, V_SPRINT, V_TURN):
                break
            k += 1
    m.hold(30)
    uct_mouse.set_motors(0, 0)
    log("done t", (m.now() - m.t0) / 1000.0)


try:
    __name__
except NameError:
    __name__ = "__main__"

if __name__ == "__main__":
    try:
        mission()
    except Exception as e:
        try:
            uct_mouse.set_motors(0, 0)
        except Exception:
            pass
        raise e