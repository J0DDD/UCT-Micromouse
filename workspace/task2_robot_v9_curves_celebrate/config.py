# =========================================================================
# UCT Micromouse - Task 2 configuration (PHYSICAL ROBOT version)
# =========================================================================
# All tuning constants live here so that main.py, motion.py and maze.py stay
# free of magic numbers. Every value is for the physical mouse; values marked
# "calibrate.xxx()" come from the routines in calibrate.py (run them from the
# REPL and copy the suggestions here). The simulator version of the code
# lives in workspace/task2_sim with its own config.
# =========================================================================

ON_HARDWARE = True

# ---------------------------------------------------------------------------
# Maze geometry
# ---------------------------------------------------------------------------
CELL_M = 0.20               # cell pitch (m)
DEFAULT_ROWS = 4            # fallback if the API cannot report the arena size
DEFAULT_COLS = 6

# ---------------------------------------------------------------------------
# Chassis
# ---------------------------------------------------------------------------
TICKS_PER_M = 5670.0        # calibrate.ticks(): 5658 and 5678 ticks pushed over 1.000 m
HALF_TRACK  = 0.0550        # effective; calibrate.spin() 2026-10-23: 0.0548 (cw) / 0.0551 (ccw)
GYRO_SCALE  = 0.992         # calibrate.turns(12) 2026-10-23, tethered: 1.000 -> ~22 deg short,
                            # 0.978 -> ~40 deg too far (over 1080 deg); straight-line zero ~0.992.
                            # Untethered check at 0.992: within ~10 deg over 1080 both ways.
GYRO_TRUST  = 0.90          # calibrate.spin(): the gyro matched the real rotation,
                            # encoder angles were off by -7.5% / +13% (wheel scrub)

MOTOR_POLARITY = (1, 1)     # (-1, -1) from Task 1 drove this board backwards
ENCODER_POLARITY = (1, 1)   # calibrate.sensors(): push forward -> both counts rise
REVERSE_TRIP_S = 0.3        # stop if a wheel turns against its command this long
SLIP_CORRECT = False        # gyro-vs-encoder slip correction: ordinary steering looked like slip

# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------
LOOP_MS = 20                # MicroPython needs ~17 ms per control step (calibrate.timing())
DT = LOOP_MS / 1000.0

# Wheel velocity loop: pwm = feedforward(v) + KP * err + integral.
# The motors' speed-vs-PWM curves are far from linear, so the feedforward is
# looked up in measured (speed m/s, PWM) tables, separately per wheel and per
# direction - a wheel can need much more PWM to start backwards (pivots).
FF_DEADBAND = 40.0          # (only used if a table is None)
FF_GAIN    = 180.0
# calibrate.motors(), 2026-10-23 (the right motor barely moves below ~35-40 PWM)
FF_TABLE_L = ((0.0, 15), (0.043, 20), (0.131, 30), (0.184, 40), (0.221, 50), (0.252, 60),
              (0.277, 70), (0.289, 80), (0.300, 90), (0.313, 100))
FF_TABLE_R = ((0.0, 25), (0.024, 30), (0.066, 40), (0.198, 50), (0.236, 60), (0.252, 70),
              (0.267, 80), (0.285, 90), (0.299, 100))
# calibrate.motors(reverse=True), 2026-10-23
FF_TABLE_L_REV = ((0.0, 15), (0.032, 20), (0.115, 30), (0.182, 40), (0.230, 50), (0.255, 60),
                  (0.272, 70), (0.282, 80), (0.297, 90), (0.303, 100))
FF_TABLE_R_REV = ((0.0, 25), (0.025, 30), (0.122, 40), (0.217, 50), (0.244, 60), (0.261, 70),
                  (0.273, 80), (0.286, 90), (0.298, 100))
WHEEL_KP   = 120.0
WHEEL_KI   = 900.0
WHEEL_I_MAX = 35.0          # integral clamp (PWM)
# The proportional gain is scheduled on the measured curve's local slope so the
# loop gain (PWM change x speed per PWM) is WHEEL_LOOP_GAIN everywhere; the
# integral gain keeps the WHEEL_KI / WHEEL_KP ratio.
WHEEL_LOOP_GAIN = 0.6
WHEEL_KP_MIN = 30.0
WHEEL_KP_MAX = 200.0
VEL_FILTER = 0.5            # IIR weight on the previous wheel speed estimate
MOTOR_TAU  = 0.06           # drivetrain lag (used when planning braking)
COAST_TIME = 0.075          # s: after cutting power the mouse rolls ~speed * COAST_TIME
                            # (calibrate.straight(1) x3 on open floor: 0.071-0.077)
                            # (calibrate.straight() in the open prints the overshoot)

# ---------------------------------------------------------------------------
# Straight-line motion (the slower wheel tops out at ~0.30 m/s)
# ---------------------------------------------------------------------------
EXPLORE_SPEED = 0.12        # m/s: slow and steady while mapping
# Return / sprint top speed by straight length: (1 cell, 2 cells, 3+ cells).
# The slower wheel tops out at ~0.30 m/s and steering needs some headroom
# above the forward speed, so keep the last entries below ~0.28.
RETURN_SPEEDS = (0.18, 0.20, 0.22)  # was 0.16 for every straight
SPRINT_SPEEDS = (0.22, 0.25, 0.27)  # was 0.25 for every straight
# Curved corners: a quarter circle of TURN_RADIUS through the corner cell
# instead of stop + pivot. The outer wheel runs (1 + HALF_TRACK / TURN_RADIUS)
# = ~1.55x ARC_SPEED and the slower wheel tops out at ~0.30 m/s; the inner
# one runs ~0.45x, and the right motor barely moves below ~0.07 m/s.
SPRINT_CURVES = True        # False = stop and pivot at every sprint corner (as before)
RETURN_CURVES = False       # same for the return run (try once the sprint is happy)
TURN_RADIUS   = 0.10        # m: half a cell, the path passes the corner post at 10 cm
ARC_SPEED     = 0.16        # m/s along the arc (outer wheel ~0.25, inner ~0.07)
ACCEL         = 0.6         # m/s^2
DECEL         = 0.5         # m/s^2
SPRINT_ACCEL  = 0.8
SPRINT_DECEL  = 0.45        # was 0.7: braked late and hard, then stopped abruptly at walls
CREEP_SPEED   = 0.04        # minimum approach speed so we always arrive
START_SOFT_MS = 200         # gentle acceleration window when pulling away from rest
START_ACCEL   = 0.3         # m/s^2 during that window

# Steering: cascaded lateral -> heading -> yaw rate. The side ToFs jitter
# ~10-15 mm even after filtering, so the lateral error is smoothed more and
# steered on gently.
K_LATERAL     = 3.0         # desired heading offset (rad) per metre of lateral error
                            # (3.5 on the smoothed wall reading weaved in sprints; 2.5 was slow)
# Steer on the position estimate (gyro + encoders, pulled onto the walls every
# period) rather than on the smoothed wall reading. False = the old behaviour.
STEER_ON_POSE = True
POSE_GAIN_TWO = 0.15        # per-period pull of the lateral position onto two walls
POSE_GAIN_ONE = 0.08        # ...onto one wall (noisier: a single sensor)
POSE_GAIN_FAR = 0.03        # ...onto walls a cell away (+-25 mm jitter)
K_LATERAL_ONE_WALL = 0.7    # centring strength factor with only one wall (was 0.5)
K_LATERAL_FAR = 0.4         # ...and with only walls one cell further out (~ref + 0.2 m)
# Parallel check: fit the side-wall offset over a stretch of travel; its slope
# is the true angle to the walls, compared with what the heading estimate says.
WALL_SLOPE_LEN = 0.12       # m of continuous wall (same walls, no posts) per check
WALL_SLOPE_MIN_N = 15       # readings needed in that stretch
WALL_SLOPE_GAIN = 0.4       # correct the heading estimate by this fraction of the error (0 = off)
WALL_SLOPE_MAX_ERR = 0.12   # rad (~7 deg): ignore a check claiming more (bad readings)
FAR_WALL_TOL = 0.09         # a side reading within this of ref + 0.2 m counts as the wall one
                            # cell over (covers +0.11..+0.29, picking up where a near wall ends)
MAX_HEAD_OFS  = 0.10        # cap on the correction angle (~6 deg)
K_HEADING     = 10.0        # yaw rate (rad/s) per rad of heading error
MAX_STEER_W   = 2.5         # rad/s
K_HEADING_FIX = 0.0         # off: the gyro is accurate, misread side walls steered it wrong
LAT_FILTER    = 0.85        # ~6 samples (~120 ms) of smoothing with one wall
LAT_FILTER_TWO = 0.6        # less with two walls (averaged, cleaner, less lag)
LAT_FILTER_FAR = 0.92       # much more for walls a cell away (+-25 mm jitter at that range)
POSE_LAT_GAIN = 0.2         # per-sample pull of the position estimate onto the wall-measured line

# ---------------------------------------------------------------------------
# Turning (always on the spot on the robot)
# ---------------------------------------------------------------------------
TURN_W_MAX    = 2.5         # rad/s peak pivot rate (was 3.0)
TURN_ALPHA    = 12.0        # rad/s^2 pivot acceleration (was 15)
TURN_DECEL    = 8.0         # rad/s^2 planned braking (was 10)
TURN_LAG      = 0.14        # s, lag allowed for when braking. Measured over 12 turns: 0.10 ~10 deg,
                            # 0.14 ~5 deg (best), 0.18 ~15 deg - the visible overshoot looks like
                            # gyro reading latency, which braking earlier cannot fix
TURN_W_MIN    = 0.8         # rad/s creep for the last degrees (was 1.0: less coast past the target;
                            # the right motor still moves at ~30 PWM in reverse)
TURN_TOL      = 0.035       # rad (~2 deg) acceptance window
TURN_RATE_TOL = 0.35        # rad/s, must have nearly stopped rotating
TURN_TIMEOUT_MS = 3000
FACE_TOL = 0.021            # rad (~1.2 deg): re-align before a drive if further off than this
# Turns end with the active brake instead of coasting past the target
TURN_BRAKE_LEAD = 0.06      # s: brake when the angle left <= turn rate x this (0 = off).
                            # Swings past the target -> raise; stops short and creeps -> lower
TURN_BRAKE_PWM = 25         # reverse push per wheel while braking a turn
# Active braking at the end of every drive (stops the freer left wheel rolling
# on longer than the right and turning the mouse at each stop)
BRAKE_PWM = 25              # reverse push on a wheel still turning (0 = off, coast as before)
BRAKE_STOP_V = 0.015        # m/s: a wheel slower than this counts as stopped
BRAKE_MAX_MS = 300
BRAKED_SETTLE_MS = 60       # wait after a braked stop / turn (was 150: the brake has
                            # already stopped the wheels; raise if stop readings get noisy)
PIVOT_HOLD_GAIN = 3.0       # (m/s per m) drive the centre back if it rolls during a pivot; 0 = off
PIVOT_HOLD_VMAX = 0.04      # m/s cap on that correction

# ---------------------------------------------------------------------------
# Time-of-flight sensing (metres)
# ---------------------------------------------------------------------------
TOF_INVALID   = 8000        # mm readings at/above this are out of range
TOF_MAX_M     = 2.0
TOF_MEDIAN_N  = 5           # median over the last N valid readings per sensor
TOF_HOLD_S    = 0.12        # a dropout shorter than this keeps the last value
                            # (the firmware drops many close-range readings)
SIDE_REF_DEFAULT  = 0.065   # side reading when centred (recalibrated in the start cell)
FRONT_STOP_DEFAULT = 0.065  # front reading to stop at: ~5 cm nose gap (wall-stop tests)
FRONT_CAL_ROUNDS = 2        # start-cell front calibration: face N then S this many times
FRONT_STOP_MIN = 0.035      # only accept a calibrated front reference inside this range
FRONT_STOP_MAX = 0.080
FRONT_STOP_EXTRA = 0.010    # stop this much further from a wall ahead than the cell centre
# Squaring up on the front wall at corners (return / sprint) before turning
SQUARE_MAX_ERR = 0.06       # only if the front reading is within this of the stopping reading
SQUARE_TOL = 0.005          # m: good enough
SQUARE_SPEED = 0.05         # m/s creep forwards / backwards
SQUARE_TIMEOUT_MS = 1500
TOF_OFFSET = (0.0, 0.0, 0.0)  # per-sensor offsets were not stable between sessions
FRONT_LOOKAHEAD_CELLS = 1   # wide beam: only trust one cell ahead
SIDE_WALL_MARGIN  = 0.050   # side wall present if reading < side_ref + margin
# Steering: cells are exactly 0.2 m, so any side reading up to half a cell
# beyond the centred reading is a wall IN THIS CELL (the next cell's wall
# would be ~0.2 m further) - even if the mouse has drifted far from it.
STEER_WALL_ON  = 0.09       # a side wall counts below side_ref + this...
STEER_WALL_OFF = 0.11       # ...and stops counting above side_ref + this (hysteresis)
CLOSE_SPEED = 0.08          # m/s cap only while inside the minimum side clearance
                            # (an open side alone no longer slows it down)
OPEN_SIDE_LEAN = 0.0        # m: off - aiming towards the single wall flipped sides when the
                            # wall switched from left to right between cells
# Minimum clearance from the closest wall/post (on top of the centring)
# Posts stick out ~2 cm from the wall face: a short burst of closer side
# readings. Steering ignores such bursts (cannot dodge them in time anyway).
POST_DROP = 0.012           # m below the running wall level counts as a possible post
POST_MAX_LEN = 0.06         # m of travel: a closer reading lasting longer is a real wall
POST_RISE = 0.03            # m above the wall level: the wall fell away, restart tracking
POST_BASE_GAIN = 0.2        # how quickly the running wall level follows the readings
CLEAR_MIN_READING = 0.050   # m: a side reading below this (sensor to wall/post) triggers
                            # the clearance rule. Centred readings are ~0.05-0.07 m.
MAX_HEAD_OFS_CLOSE = 0.16   # rad (~9 deg) correction angle allowed when too close
FRONT_WALL_MARGIN = 0.100   # front wall present if reading < front_stop + margin
FRONT_TRUST_RANGE = 0.30    # use the front ToF for stopping when closer than this
FRONT_FILTER_GAIN = 0.25    # blend of each new front sample into the odometry-predicted distance
FRONT_TOF_LAG = 0.06        # s: age of a front reading (sensor + median filter); while
                            # moving the wall is speed x this closer than it reads
STOP_LEARN_GAIN = 0.4       # each wall stop shifts the stopping point by this part of its miss
STOP_BIAS_MAX = 0.025       # m: limit on that learned shift
FRONT_JUMP_CONFIRM = 3      # readings a jump in the front distance must persist to be believed
FRONT_MAP_WINDOW = 0.05     # with a mapped wall ahead, ignore front readings this far off it
FRONT_EMERGENCY = 0.02      # stop anyway if the front reads this much under front_stop
FRONT_POSE_RANGE = 0.25     # front readings to a mapped wall closer than this correct position
                            # (longer readings came out ~14 cm short: the cone hits posts/floor)
FRONT_POSE_GAIN = 0.10      # per-sample pull of the along-track position toward that reading
FRONT_POSE_WINDOW = 0.06    # ignore readings implying a jump bigger than this (wrong wall)
# Stop-time position fixes: only from walls of the cell the mouse is in
LOCALISE_RANGE  = 0.12
LOCALISE_WINDOW = 0.04
LOCALISE_GAIN   = 0.5
SIDE_SENSOR_X = 0.040       # side ToF distance ahead of the axle (calibrate.edge())
EDGE_DEBOUNCE = 2
EDGE_WINDOW   = 0.035
EDGE_FIX_GAIN = 0.0         # off: filtered readings can see a wall end ~3 cm late
ODO_LEARN_GAIN = 0.3        # how fast front-wall stops adapt the odometry scale
ODO_LEARN_MIN_DIST = 0.15
ODO_SCALE_MIN = 0.85
ODO_SCALE_MAX = 1.15

# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------
GYRO_CAL_SAMPLES = 100      # 2 s of stationary samples
GYRO_CAL_MAX_SPREAD = 8.0   # dps: a wider spread means the mouse moved during calibration
GYRO_BIAS_TRACK = 0.2       # each stationary stop moves the gyro bias this fraction towards
                            # the reading at rest (cancels slow drift over a long run)
MAP_ON_THE_MOVE = False     # stop in every new cell and map it while stationary (wide beams)
STILL_WEIGHT = 2            # map votes of a stationary observation
START_WALL_PRIOR = 100      # the start cell's N wall / E opening are fixed by the rules
SAMPLE_FROM = -0.12         # (only used when mapping on the move)
SAMPLE_TO = -0.065
DECIDE_BEFORE_CENTRE = 0.06
SPRINT_PAUSE_MS  = 3000
TURN_COST_CELLS  = 1.5      # planner: a pivot (stop, turn, restart) costs ~1.5 cells of time
VERBOSE = True              # print each move over USB while testing tethered
# End-of-run celebration after a successful sprint (celebrate.py)
CELEBRATE = True
CELEBRATE_DELAY_MS = 2000   # pause after the sprint stops before the twerk starts
CELEBRATE_BEATS = 16        # wiggles (each one swings the rear one way)
CELEBRATE_BEAT_MS = 160     # length of one wiggle
CELEBRATE_FRAME_MS = 700    # how long each OLED animation frame stays up
TWERK_V = 0.05              # m/s back / forth bob during a wiggle
TWERK_W = 2.0               # rad/s rear swing during a wiggle
