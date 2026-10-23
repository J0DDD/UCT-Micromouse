# =========================================================================
# UCT Micromouse - Task 2 configuration
# =========================================================================
# All tuning constants live here so that main.py, motion.py and maze.py stay
# free of magic numbers. Values that differ between the physical mouse and
# the simulator are selected once via ON_HARDWARE.
# =========================================================================

import sys

ON_HARDWARE = sys.platform in ('pyboard', 'stm32')

# ---------------------------------------------------------------------------
# Maze geometry
# ---------------------------------------------------------------------------
CELL_M = 0.20               # cell pitch (m)
DEFAULT_ROWS = 4            # fallback if the API cannot report the arena size
DEFAULT_COLS = 6

# ---------------------------------------------------------------------------
# Chassis
# ---------------------------------------------------------------------------
if ON_HARDWARE:
    TICKS_PER_M = 5670.0    # calibrate.ticks(): 5658 and 5678 ticks pushed over 1.000 m (2026-10-22)
    HALF_TRACK  = 0.0565    # effective; calibrate.spin() gave 0.0594 and 0.0549 (pivots scrub)
    GYRO_TRUST  = 0.90      # calibrate.spin(): the gyro matched the real rotation in both
                            # runs, encoder angles were off by -7.5% / +13% (wheel scrub)
else:
    TICKS_PER_M = 4400.0    # simulator: 470 ticks/rev, 17 mm wheel radius
    HALF_TRACK  = 0.054
    GYRO_TRUST  = 0.95      # sim gyro is clean; encoders are corrupted by slip

# Motor / encoder direction (overridden by polarity.txt "ml,mr,el,er" if present)
if ON_HARDWARE:
    MOTOR_POLARITY = (1, 1)     # (-1, -1) from Task 1 drove this board backwards
    ENCODER_POLARITY = (1, 1)   # check with calibrate.sensors(): push forward -> counts rise
else:
    MOTOR_POLARITY = (1, 1)
    ENCODER_POLARITY = (1, 1)
REVERSE_TRIP_S = 0.3        # stop if a wheel turns against its command this long
# Gyro-vs-encoder wheel-slip correction of distance. Off on the robot: until
# HALF_TRACK is calibrated, ordinary steering looks like slip (removed ~5%).
SLIP_CORRECT = not ON_HARDWARE

# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------
if ON_HARDWARE:
    LOOP_MS = 20            # MicroPython needs ~17 ms per control step (calibrate.timing())
else:
    LOOP_MS = 10            # the simulator steps physics in 10 ms increments
DT = LOOP_MS / 1000.0

# Wheel velocity loop: pwm = feedforward(v) + KP * err + integral
if ON_HARDWARE:
    FF_DEADBAND = 40.0      # PWM needed to break stiction (initial guess, integrator adapts)
else:
    FF_DEADBAND = 28.0
FF_GAIN    = 180.0          # PWM per m/s above the deadband (sim: 72 PWM / 0.4 m/s)
FF_TABLE_L = None           # optional measured (speed m/s, PWM) curves, see the hardware block
FF_TABLE_R = None
WHEEL_KP   = 120.0
WHEEL_KI   = 900.0
WHEEL_I_MAX = 35.0          # integral clamp (PWM)
VEL_FILTER = 0.5            # IIR weight on the previous wheel speed estimate
MOTOR_TAU  = 0.06           # used to predict coasting distance when cutting power

# ---------------------------------------------------------------------------
# Straight-line motion
# ---------------------------------------------------------------------------
EXPLORE_SPEED = 0.25        # m/s
SPRINT_SPEED  = 0.36        # m/s: ~97% of the sim motors' top speed. Running flat out
                            # gained nothing measurable and removed steering headroom.
ACCEL         = 1.2         # m/s^2
DECEL         = 1.0         # m/s^2
CREEP_SPEED   = 0.05        # minimum approach speed so we always arrive
START_SOFT_MS = 200         # gentle acceleration window when pulling away from rest
START_ACCEL   = 0.3         # m/s^2 during that window

# Steering: cascaded lateral -> heading -> yaw rate
K_LATERAL     = 8.0         # desired heading offset (rad) per metre of lateral error
MAX_HEAD_OFS  = 0.20        # clamp on the desired heading offset (rad)
K_HEADING     = 10.0        # yaw rate (rad/s) per rad of heading error
MAX_STEER_W   = 2.5         # rad/s
K_HEADING_FIX = 15.0        # wall-based heading-estimate correction (rad / m lateral / m travelled)
LAT_FILTER    = 0.6         # IIR weight on the previous lateral error
POSE_LAT_GAIN = 0.2         # per-sample pull of the position estimate onto the wall-measured line

# Smooth (curved) 90 degree turns used where the path is already known
TURN_RADIUS   = 0.10        # m: quarter circle from cell boundary to cell boundary
ARC_SPEED     = 0.23        # m/s: outer wheel runs at 1.54x this (sim limit ~0.37 m/s)
SPRINT_ACCEL  = 2.0         # m/s^2
SPRINT_DECEL  = 1.8         # m/s^2
RETURN_SPEED  = 0.32        # m/s for known-path segments of the return run

# ---------------------------------------------------------------------------
# Turning
# ---------------------------------------------------------------------------
TURN_W_MAX    = 5.0         # rad/s peak pivot rate
TURN_ALPHA    = 40.0        # rad/s^2 pivot acceleration
TURN_DECEL    = 25.0        # rad/s^2 planned braking
TURN_LAG      = 0.06        # s, drivetrain lag allowed for when braking
TURN_W_MIN    = 0.6         # rad/s creep used to close the last degrees
TURN_TOL      = 0.025       # rad (~1.4 deg) acceptance window
TURN_RATE_TOL = 0.20        # rad/s, must have nearly stopped rotating
TURN_TIMEOUT_MS = 3000

# ---------------------------------------------------------------------------
# Time-of-flight sensing (metres)
# ---------------------------------------------------------------------------
TOF_INVALID   = 8000        # mm readings at/above this are out of range
TOF_MAX_M     = 2.0
TOF_MEDIAN_N  = 5           # median over the last N valid readings per sensor
TOF_HOLD_S    = 0.12        # a dropout shorter than this keeps the last value
                            # (the robot's firmware drops many close-range readings)
if ON_HARDWARE:
    SIDE_REF_DEFAULT  = 0.065   # side reading when centred (recalibrated in the start cell)
    FRONT_STOP_DEFAULT = 0.065  # front reading to stop at: 3-6 cm (avg 4.6) at 0.060 -> ~5 cm
    FRONT_STOP_MIN = 0.035      # never accept a calibrated front reference outside this
    FRONT_STOP_MAX = 0.080
    # per-sensor fixed error (true - reading) from calibrate.tof() vs a ruler, (L, C, R) in m
    TOF_OFFSET = (0.0, 0.0, 0.0)  # per-sensor offsets were not stable between sessions;
                                  # the start-cell calibration handles a common offset
    FRONT_LOOKAHEAD_CELLS = 1   # wide hardware beam: only trust one cell ahead
else:
    SIDE_REF_DEFAULT  = 0.060
    FRONT_STOP_DEFAULT = 0.035
    FRONT_STOP_MIN = 0.015
    FRONT_STOP_MAX = 0.080
    TOF_OFFSET = (0.0, 0.0, 0.0)
    FRONT_LOOKAHEAD_CELLS = 2
SIDE_WALL_MARGIN  = 0.050   # side wall present if reading < side_ref + margin
FRONT_WALL_MARGIN = 0.100   # front wall present if reading < front_stop + margin
FRONT_TRUST_RANGE = 0.30    # use the front ToF for stopping when closer than this
FRONT_FILTER_GAIN = 0.25    # blend of each new front sample into the odometry-predicted distance
FRONT_POSE_RANGE = 0.50     # front readings to a mapped wall closer than this correct position
FRONT_POSE_GAIN = 0.10      # per-sample pull of the along-track position toward that reading
FRONT_POSE_WINDOW = 0.06    # ignore readings implying a jump bigger than this (wrong wall)
# Re-localisation at every stop from all three ToFs against mapped walls
if ON_HARDWARE:
    LOCALISE_RANGE = 0.30   # wide hardware beams: only trust walls about a cell away
else:
    LOCALISE_RANGE = 0.50
LOCALISE_WINDOW = 0.06      # reject fixes implying a bigger jump (m)
LOCALISE_GAIN = 0.8
if ON_HARDWARE:
    SIDE_SENSOR_X = 0.040   # side ToF distance ahead of the axle (measure on the robot!)
else:
    SIDE_SENSOR_X = 0.050   # simulator sensor geometry
EDGE_DEBOUNCE = 2           # samples a wall-presence change must persist
EDGE_WINDOW   = 0.035       # only accept edge fixes this close to the expected spot (m)
if ON_HARDWARE:
    EDGE_FIX_GAIN = 0.0     # off until verified: the robot's noisy ToFs (dropout hold +
                            # median) can detect a wall end ~3 cm late
else:
    EDGE_FIX_GAIN = 0.8     # fraction of the edge-derived error applied
ODO_LEARN_GAIN = 0.3        # how fast along-track fixes adapt the odometry scale
ODO_LEARN_MIN_DIST = 0.15   # only learn from fixes at least this far apart (m)
ODO_SCALE_MIN = 0.85        # limits on the learned scale
ODO_SCALE_MAX = 1.15

# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------
GYRO_CAL_SAMPLES = 100      # stationary samples (1 s in the sim, 2 s on the robot)
GYRO_CAL_MAX_SPREAD = 8.0   # dps: a wider spread means the mouse moved during calibration
# Mapping on the move (axle position relative to the centre of the cell being entered)
MAP_ON_THE_MOVE = not ON_HARDWARE   # robot: stop in every new cell to map it (wide beams)
STILL_WEIGHT = 2            # map votes of a stationary observation (moving: 1)
# Start cell N wall / E opening: permanent on the robot (rules hold there); one
# vote in the sim, whose generator sometimes removes that wall
START_WALL_PRIOR = 100 if ON_HARDWARE else 1
SAMPLE_FROM = -0.12         # side sensors are then just inside the cell...
SAMPLE_TO = -0.065          # ...and well clear of the far posts
DECIDE_BEFORE_CENTRE = 0.06 # decide go-on / stop this far before the centre
SPRINT_PAUSE_MS  = 3000
TURN_COST_CELLS  = 0.4       # sprint planner: a smooth 90 deg turn costs ~0.4 cells of time
VERBOSE = False             # the grader buffers stdout; keep printing minimal
if not ON_HARDWARE:
    try:
        import os
        VERBOSE = os.environ.get("MAZE_VERBOSE") == "1"
    except Exception:
        pass

# ===========================================================================
# Physical robot tuning (overrides the simulator-tuned values above)
# ===========================================================================
if ON_HARDWARE:
    # Measured speed-vs-PWM per wheel (calibrate.motors(), 2026-10-22). The
    # curves are far from linear, so the speed loops' feedforward looks the
    # PWM up here instead of using FF_DEADBAND + FF_GAIN * v.
    FF_TABLE_L = ((0.0, 17), (0.052, 20), (0.141, 30), (0.193, 40), (0.231, 50),
                  (0.259, 60), (0.273, 70), (0.298, 80), (0.308, 90), (0.328, 100))
    FF_TABLE_R = ((0.0, 27), (0.035, 30), (0.094, 40), (0.204, 50), (0.244, 60),
                  (0.264, 70), (0.276, 80), (0.290, 90), (0.305, 100))
    # The slower wheel tops out at ~0.30 m/s, so every speed is scaled down.
    EXPLORE_SPEED = 0.18
    SPRINT_SPEED  = 0.25
    RETURN_SPEED  = 0.22
    ARC_SPEED     = 0.16    # outer wheel 1.54x -> ~0.25 m/s, leaves steering headroom
    ACCEL         = 0.8
    DECEL         = 0.7
    SPRINT_ACCEL  = 1.0
    SPRINT_DECEL  = 0.9
    # Gentler pivots: wheel speeds stay well inside the motors' range, braking
    # allows for the slower drivetrain and 20 ms loop, and the final creep is
    # fast enough to stay above the deadband instead of hunting.
    TURN_W_MAX    = 3.0
    TURN_ALPHA    = 15.0
    TURN_DECEL    = 10.0
    TURN_LAG      = 0.10
    TURN_W_MIN    = 1.0
    TURN_TOL      = 0.035
    TURN_RATE_TOL = 0.35
    # Side-wall centring: the robot's side ToFs still jitter ~10-15 mm after
    # filtering, so smooth the lateral error more and steer on it more gently.
    LAT_FILTER    = 0.85    # ~6 samples (~120 ms) of smoothing instead of ~2
    K_LATERAL     = 4.0     # half the sim gain: 10 mm off-centre -> 2.3 deg heading offset
    MAX_HEAD_OFS  = 0.10    # cap the correction angle at ~6 deg (sim: ~11 deg)
    K_HEADING_FIX = 0.0     # off: the gyro is accurate (spin test 0.2%), and misread
                            # side walls were steering the heading estimate wrong
    # Stop-time position fixes: only from walls of the cell the mouse is in,
    # and moving the estimate less per fix (wide beams, noisy readings).
    LOCALISE_RANGE  = 0.12
    LOCALISE_WINDOW = 0.04
    LOCALISE_GAIN   = 0.5
    VERBOSE       = True    # print each move over USB while testing tethered
