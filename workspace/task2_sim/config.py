# =========================================================================
# UCT Micromouse - Task 2 configuration (SIMULATOR version)
# =========================================================================
# All tuning constants live here so that main.py, motion.py and maze.py stay
# free of magic numbers. This folder is tuned for tools/physics_sim.py and the
# Gradescope autograder; the physical robot has its own copy of the code in
# workspace/task2_robot with its own measured values.
# =========================================================================

ON_HARDWARE = False

# ---------------------------------------------------------------------------
# Maze geometry
# ---------------------------------------------------------------------------
CELL_M = 0.20               # cell pitch (m)
DEFAULT_ROWS = 4            # fallback if the API cannot report the arena size
DEFAULT_COLS = 6

# ---------------------------------------------------------------------------
# Chassis (tools/simulation_config.json)
# ---------------------------------------------------------------------------
TICKS_PER_M = 4400.0        # 470 ticks/rev, 17 mm wheel radius
HALF_TRACK  = 0.054
GYRO_TRUST  = 0.95          # sim gyro is clean; encoders are corrupted by slip

MOTOR_POLARITY = (1, 1)     # overridden by polarity.txt "ml,mr,el,er" if present
ENCODER_POLARITY = (1, 1)
REVERSE_TRIP_S = 0.3        # stop if a wheel turns against its command this long
SLIP_CORRECT = True         # gyro-vs-encoder wheel-slip correction of distance

# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------
LOOP_MS = 10                # the simulator steps physics in 10 ms increments
DT = LOOP_MS / 1000.0

# Wheel velocity loop: pwm = feedforward(v) + KP * err + integral
FF_DEADBAND = 28.0          # PWM needed to break stiction
FF_GAIN    = 180.0          # PWM per m/s above the deadband (72 PWM / 0.4 m/s)
FF_TABLE_L = None           # (unused in the simulator: the linear model fits)
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
SIDE_REF_DEFAULT  = 0.060   # side reading when centred (recalibrated in the start cell)
FRONT_STOP_DEFAULT = 0.035  # front reading at the cell centre with a wall ahead
FRONT_STOP_MIN = 0.015      # only accept a calibrated front reference inside this range
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
LOCALISE_RANGE = 0.50
LOCALISE_WINDOW = 0.06      # reject fixes implying a bigger jump (m)
LOCALISE_GAIN = 0.8
SIDE_SENSOR_X = 0.050       # side ToFs sit this far ahead of the axle
EDGE_DEBOUNCE = 2           # samples a wall-presence change must persist
EDGE_WINDOW   = 0.035       # only accept edge fixes this close to the expected spot (m)
EDGE_FIX_GAIN = 0.8         # fraction of the edge-derived error applied
ODO_LEARN_GAIN = 0.3        # how fast along-track fixes adapt the odometry scale
ODO_LEARN_MIN_DIST = 0.15   # only learn from fixes at least this far apart (m)
ODO_SCALE_MIN = 0.85        # limits on the learned scale
ODO_SCALE_MAX = 1.15

# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------
GYRO_CAL_SAMPLES = 100      # 1 s of stationary samples
GYRO_CAL_MAX_SPREAD = 8.0   # dps: a wider spread means the mouse moved during calibration
MAP_ON_THE_MOVE = True      # map new cells while driving through them
STILL_WEIGHT = 2            # map votes of a stationary observation (moving: 1)
START_WALL_PRIOR = 1        # one vote: the sim's generator sometimes removes the start N wall
SAMPLE_FROM = -0.12         # (axle relative to the centre of the cell being entered)
SAMPLE_TO = -0.065          # side sensors are then inside the cell, clear of the far posts
DECIDE_BEFORE_CENTRE = 0.06 # decide go-on / stop this far before the centre
SPRINT_PAUSE_MS  = 3000
TURN_COST_CELLS  = 0.4      # sprint planner: a smooth 90 deg turn costs ~0.4 cells of time
VERBOSE = False             # the grader buffers stdout; keep printing minimal
try:
    import os
    VERBOSE = os.environ.get("MAZE_VERBOSE") == "1"
except Exception:
    pass
