# =========================================================================
# UCT Micromouse - Motion control and state estimation
# =========================================================================
# Layers (inner to outer):
#   1. Wheel velocity loops: per-wheel feedforward (deadband + slope) + PI on
#      encoder speed, with anti-windup. Absorbs deadband, motor imbalance and
#      battery sag, so the layers above can think in m/s and rad/s.
#   2. State estimate:
#        heading  - gyro/encoder complementary filter (Task 1), slowly
#                   corrected by side walls;
#        position - (px, py) in maze metres, integrated from distance along
#                   the heading (gyro-detected wheel slip removed), corrected
#                   sideways by the side walls, along-track by wall edges
#                   (posts) and by front-wall stops.
#   3. Path tracking: straights and quarter-circle turns are followed as
#      geometry (distance to a line / to a circle) rather than timed
#      manoeuvres, so errors do not accumulate through chains of turns.
#   4. Primitives: drive to a point, smooth 90 degree arc, pivot to an
#      absolute heading, and the 360 degree pirouette.
# =========================================================================

import math
import uct_mouse
from config import *

HALF_PI = math.pi / 2.0
TWO_PI = 2.0 * math.pi
DIR_UX = (0.0, 1.0, 0.0, -1.0)      # unit vectors of maze directions N, E, S, W
DIR_UY = (1.0, 0.0, -1.0, 0.0)


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def wrap_pi(a):
    while a > math.pi:
        a -= TWO_PI
    while a < -math.pi:
        a += TWO_PI
    return a


def heading_of(direction):
    """Maze direction (0=N, 1=E, 2=S, 3=W) -> world angle (rad, 0 = East, CCW +)."""
    return (1 - direction) * HALF_PI


def cell_centre(x, y):
    return (x + 0.5) * CELL_M, (y + 0.5) * CELL_M


def setup_polarity():
    """Apply motor/encoder directions: polarity.txt ("ml,mr,el,er") if it
    exists on the robot, otherwise MOTOR_POLARITY / ENCODER_POLARITY."""
    m = MOTOR_POLARITY
    e = ENCODER_POLARITY
    try:
        with open("polarity.txt", "r") as f:
            vals = f.read().strip().split(",")
        m = (int(vals[0]), int(vals[1]))
        if len(vals) >= 4:
            e = (int(vals[2]), int(vals[3]))
    except Exception:
        pass
    uct_mouse.set_polarity(m[0], m[1])
    uct_mouse.set_encoder_polarity(e[0], e[1])


class ToFFilter:
    """Per-sensor clean-up: hold the last value through short dropouts, then
    take the median of the last TOF_MEDIAN_N valid readings."""

    def __init__(self):
        self.buf = []
        self.age = 0.0
        self.value = TOF_MAX_M

    def update(self, mm, dt, offset=0.0):
        if 0 < mm < TOF_INVALID:
            self.buf.append(mm / 1000.0 + offset)
            if len(self.buf) > TOF_MEDIAN_N:
                self.buf.pop(0)
            self.age = 0.0
            srt = sorted(self.buf)
            self.value = srt[len(srt) // 2]
        else:
            self.age += dt
            if self.age > TOF_HOLD_S:
                self.buf = []
                self.value = TOF_MAX_M
        return self.value


class EdgeTracker:
    """Detects side-wall edges (posts) while driving along a corridor.

    The side sensors sit SIDE_SENSOR_X ahead of the axle, so a wall appearing
    or disappearing beside them happens when the axle is SIDE_SENSOR_X short
    of a cell boundary. Boundaries lie on multiples of CELL_M along the travel
    axis. A state change must persist for EDGE_DEBOUNCE samples. update()
    returns the along-track position correction (m) when an edge is confirmed.
    """

    def __init__(self, left, right, along):
        self.state = [left, right]
        self.count = [0, 0]
        self.last_along = along
        self.last_step = 0.0

    def update(self, left, right, along):
        fix = 0.0
        now = (left, right)
        for i in range(2):
            if now[i] != self.state[i]:
                self.count[i] += 1
                if self.count[i] >= EDGE_DEBOUNCE:
                    self.state[i] = now[i]
                    self.count[i] = 0
                    # the change began EDGE_DEBOUNCE - 1 samples ago
                    f = self._fix(along - (EDGE_DEBOUNCE - 1) * self.last_step)
                    if f != 0.0:
                        fix = f
            else:
                self.count[i] = 0
        self.last_step = max(0.0, along - self.last_along)
        self.last_along = along
        return fix

    def _fix(self, at):
        k = int((at + SIDE_SENSOR_X) / CELL_M + 0.5)
        err = k * CELL_M - SIDE_SENSOR_X - at
        if abs(err) < EDGE_WINDOW:
            return err
        return 0.0


class Robot:
    def __init__(self):
        self.ticks_per_m = TICKS_PER_M
        self.half_track = HALF_TRACK
        self.gyro_trust = GYRO_TRUST
        self.gyro_bias = 0.0
        self.side_ref = SIDE_REF_DEFAULT
        self.front_stop = FRONT_STOP_DEFAULT

        self.heading = 0.0          # continuous (unwrapped) world heading, rad
        self.px = 0.5 * CELL_M      # position in maze metres (start cell centre)
        self.py = 0.5 * CELL_M
        self.gyro_rate = 0.0        # rad/s
        self.gyro_total = 0.0       # integrated gyro alone, rad (calibrate.spin)
        self.vl = 0.0               # filtered wheel speeds, m/s
        self.vr = 0.0
        self.il = 0.0               # wheel-loop integrators, PWM
        self.ir = 0.0
        self.lat = 0.0              # filtered wall-measured lateral error, m (+ = left)
        self.lat_valid = False
        self.slip_correct = False   # only meaningful while driving straight
        self.odo_scale = 1.0        # learned true-metres per encoder-metre (slip, wheel size)
        self.odo_since_fix = 0.0    # odometry distance since the last exact along-track fix
        self.corr_since_fix = 0.0   # corrections applied since then
        self.tof_l = TOF_MAX_M
        self.tof_c = TOF_MAX_M
        self.tof_r = TOF_MAX_M
        self.filt_l = ToFFilter()
        self.filt_c = ToFFilter()
        self.filt_r = ToFFilter()
        self.rev_l = 0.0            # how long each wheel has turned against its command
        self.rev_r = 0.0

        enc = uct_mouse.get_encoders()
        self.enc_l = enc[0]
        self.enc_r = enc[1]
        self.last_ms = uct_mouse.get_ticks_ms()
        self.dt = DT                # measured length of the last control period, s

    # ------------------------------------------------------------------
    # Sensing and state estimation
    # ------------------------------------------------------------------
    def _tof_m(self, mm):
        if mm <= 0 or mm >= TOF_INVALID:
            return TOF_MAX_M
        return mm / 1000.0

    def _read_sensors(self):
        # On the robot delay_ms() is a plain wait and the Python code itself
        # takes time, so integrate with the period that actually elapsed.
        now = uct_mouse.get_ticks_ms()
        self.dt = clamp((now - self.last_ms) / 1000.0, 0.002, 0.1)
        self.last_ms = now
        enc = uct_mouse.get_encoders()
        dl = (enc[0] - self.enc_l) / self.ticks_per_m
        dr = (enc[1] - self.enc_r) / self.ticks_per_m
        self.enc_l = enc[0]
        self.enc_r = enc[1]

        rate = math.radians(uct_mouse.get_gyro() - self.gyro_bias)
        self.gyro_rate = rate
        d_gyro = rate * self.dt
        self.gyro_total += d_gyro           # pure gyro angle, for calibration
        d_enc = (dr - dl) / (2.0 * self.half_track)
        d_head = self.gyro_trust * d_gyro + (1.0 - self.gyro_trust) * d_enc

        ds = (dl + dr) / 2.0
        if SLIP_CORRECT and self.slip_correct and ds > 0.0:
            # A wheel that slips turns the chassis without the encoders seeing
            # it; the yaw the gyro sees beyond the encoder estimate is distance
            # that slipping wheel did not actually cover.
            lost = abs(d_gyro - d_enc) * self.half_track
            ds = max(0.0, ds - lost)
        ds *= self.odo_scale
        if ds > 0.0:
            self.odo_since_fix += ds
        mid = self.heading + 0.5 * d_head
        self.px += ds * math.cos(mid)
        self.py += ds * math.sin(mid)
        self.heading += d_head

        self.vl = VEL_FILTER * self.vl + (1.0 - VEL_FILTER) * dl / self.dt
        self.vr = VEL_FILTER * self.vr + (1.0 - VEL_FILTER) * dr / self.dt

        t = uct_mouse.get_tof()
        self.tof_l = self.filt_l.update(t[0], self.dt, TOF_OFFSET[0])
        self.tof_c = self.filt_c.update(t[2], self.dt, TOF_OFFSET[1])
        self.tof_r = self.filt_r.update(t[4], self.dt, TOF_OFFSET[2])

    def along_track_fix(self, err, ux, uy, gain, exact=False):
        """Apply an along-track correction `err` (m, + = we are further than
        the estimate said). Exact fixes (stopping against a front wall) also
        refine the odometry scale: since the previous exact fix the mouse
        really covered odometry + all corrections applied meanwhile."""
        self.px += ux * err * gain
        self.py += uy * err * gain
        self.corr_since_fix += err * gain
        if not exact:
            return
        d = self.odo_since_fix
        if d > ODO_LEARN_MIN_DIST:
            ratio = clamp((d + self.corr_since_fix) / d, 0.9, 1.1)
            self.odo_scale = clamp(self.odo_scale * (1.0 + ODO_LEARN_GAIN * (ratio - 1.0)),
                                   ODO_SCALE_MIN, ODO_SCALE_MAX)
        self.odo_since_fix = 0.0
        self.corr_since_fix = 0.0

    def localise_axis(self, direction, wall_along, reading, sensor_ofs):
        """Stationary position fix along maze `direction` from a ToF reading to
        a mapped wall face at `wall_along` on that axis. `sensor_ofs` is the
        reading the sensor gives at a cell centre with the wall at that cell's
        boundary (front_stop for the front sensor, side_ref for the sides)."""
        ux = DIR_UX[direction]
        uy = DIR_UY[direction]
        expected = wall_along - 0.5 * CELL_M - (reading - sensor_ofs)
        err = expected - (self.px * ux + self.py * uy)
        if abs(err) < LOCALISE_WINDOW:
            self.px += ux * err * LOCALISE_GAIN
            self.py += uy * err * LOCALISE_GAIN

    def set_position(self, x, y):
        self.px = x
        self.py = y

    def left_wall(self):
        return self.tof_l < self.side_ref + SIDE_WALL_MARGIN

    def right_wall(self):
        return self.tof_r < self.side_ref + SIDE_WALL_MARGIN

    def _wall_lateral(self):
        """Lateral offset of the side-sensor point from the corridor centre."""
        lw = self.left_wall()
        rw = self.right_wall()
        if lw and rw:
            return (self.tof_r - self.tof_l) / 2.0, True
        if lw:
            return self.side_ref - self.tof_l, True
        if rw:
            return self.tof_r - self.side_ref, True
        return 0.0, False

    # ------------------------------------------------------------------
    # Wheel velocity loops
    # ------------------------------------------------------------------
    def _feedforward(self, speed, table):
        """PWM expected to give `speed` (m/s, >= 0): linear model, or linear
        interpolation in a measured (speed, PWM) table when one is given."""
        if table is None:
            return FF_DEADBAND + FF_GAIN * speed
        if speed >= table[-1][0]:
            return float(table[-1][1])
        for i in range(1, len(table)):
            if speed <= table[i][0]:
                s0, p0 = table[i - 1]
                s1, p1 = table[i]
                return p0 + (p1 - p0) * (speed - s0) / (s1 - s0)
        return float(table[-1][1])

    def _wheel_pwm(self, ref, meas, integ, table=None):
        if abs(ref) < 0.002:
            return 0.0, 0.0
        err = ref - meas
        ff = self._feedforward(abs(ref), table)
        if ref < 0.0:
            ff = -ff
        trial = clamp(integ + WHEEL_KI * err * self.dt, -WHEEL_I_MAX, WHEEL_I_MAX)
        # anti-windup: stop integrating while the output is saturated in the
        # direction the error pushes (otherwise braking afterwards is delayed)
        raw = ff + WHEEL_KP * err + trial
        if (raw > 100.0 and err > 0.0) or (raw < -100.0 and err < 0.0):
            trial = integ
        pwm = clamp(ff + WHEEL_KP * err + trial, -100.0, 100.0)
        return pwm, trial

    def tick(self, v, w):
        """One control period: command forward speed v (m/s) and yaw rate w (rad/s)."""
        ref_l = v - w * self.half_track
        ref_r = v + w * self.half_track
        pl, self.il = self._wheel_pwm(ref_l, self.vl, self.il, FF_TABLE_L)
        pr, self.ir = self._wheel_pwm(ref_r, self.vr, self.ir, FF_TABLE_R)
        self._check_direction(ref_l, ref_r)
        uct_mouse.set_motors(int(pl), int(pr))
        if ON_HARDWARE:
            # hold a steady LOOP_MS period: only wait for what is left of it
            wait = LOOP_MS - (uct_mouse.get_ticks_ms() - self.last_ms)
            if wait > 0:
                uct_mouse.delay_ms(wait)
        else:
            uct_mouse.delay_ms(LOOP_MS)     # the simulator advances one step per 10 ms
        self._read_sensors()

    def _check_direction(self, ref_l, ref_r):
        """Safety: a wheel that keeps turning against its command means the
        motor or encoder polarity is wrong - the speed loop would then run
        away at full power, so stop and say so."""
        if ref_l * self.vl < 0.0 and abs(self.vl) > 0.05:
            self.rev_l += self.dt
        else:
            self.rev_l = 0.0
        if ref_r * self.vr < 0.0 and abs(self.vr) > 0.05:
            self.rev_r += self.dt
        else:
            self.rev_r = 0.0
        if self.rev_l > REVERSE_TRIP_S or self.rev_r > REVERSE_TRIP_S:
            uct_mouse.set_motors(0, 0)
            raise RuntimeError("%s wheel turns against its command: check MOTOR_POLARITY / "
                               "ENCODER_POLARITY in config.py" % ("left" if self.rev_l > REVERSE_TRIP_S else "right"))

    def stop(self, settle_ms=150):
        """Cut the motors and wait for the chassis to come to rest."""
        self.il = 0.0
        self.ir = 0.0
        self.slip_correct = False
        uct_mouse.set_motors(0, 0)
        waited = 0
        while waited < settle_ms:
            uct_mouse.delay_ms(LOOP_MS)
            self._read_sensors()
            waited += LOOP_MS

    def sample_still(self, n=5):
        """Average n stationary ToF samples (more robust wall classification)."""
        sl = 0.0
        sc = 0.0
        sr = 0.0
        for _ in range(n):
            uct_mouse.delay_ms(LOOP_MS)
            self._read_sensors()
            sl += self.tof_l
            sc += self.tof_c
            sr += self.tof_r
        self.tof_l = sl / n
        self.tof_c = sc / n
        self.tof_r = sr / n

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------
    def calibrate_gyro(self, samples=GYRO_CAL_SAMPLES):
        """Average the gyro at rest to get its bias. If the readings spread
        more than GYRO_CAL_MAX_SPREAD the mouse was moved (e.g. still being
        placed), which would bake that motion into every later heading - so
        try again, up to three times."""
        uct_mouse.set_motors(0, 0)
        for attempt in range(3):
            for _ in range(10):
                uct_mouse.delay_ms(LOOP_MS)
            total = 0.0
            lo = 1e9
            hi = -1e9
            for _ in range(samples):
                uct_mouse.delay_ms(LOOP_MS)
                g = uct_mouse.get_gyro()
                total += g
                lo = min(lo, g)
                hi = max(hi, g)
            self.gyro_bias = total / samples
            if hi - lo < GYRO_CAL_MAX_SPREAD:
                break
            print("gyro moved during calibration (spread %.1f dps) - keep the mouse still" % (hi - lo))
        enc = uct_mouse.get_encoders()
        self.enc_l = enc[0]
        self.enc_r = enc[1]
        self.last_ms = uct_mouse.get_ticks_ms()

    def calibrate_in_start_cell(self):
        """The start cell has walls left and right and the mouse is placed at
        its centre, so the side readings give the centred reference. If there
        is also a wall straight ahead in this cell, its reading becomes the
        front stopping reference; otherwise FRONT_STOP_DEFAULT is kept."""
        self.sample_still(10)
        if self.tof_l < 0.15 and self.tof_r < 0.15:
            self.side_ref = (self.tof_l + self.tof_r) / 2.0
        # Only a wall in this same cell gives a trustworthy front reference:
        # extrapolating from a wall a cell further away inherits the sensor's
        # range error (on the robot ~3 cm, which made it drive into walls).
        if FRONT_STOP_MIN < self.tof_c < FRONT_STOP_MAX:
            self.front_stop = self.tof_c

    # ------------------------------------------------------------------
    # Steering
    # ------------------------------------------------------------------
    def _steer(self, path_heading, lateral_from_pose, v):
        """Yaw-rate command to follow a path whose tangent is `path_heading`.

        Lateral error comes from the side walls when present (and those
        readings also correct the heading estimate and the position), else
        from the position estimate.
        """
        meas, valid = self._wall_lateral()
        if valid:
            if self.lat_valid:
                self.lat = LAT_FILTER * self.lat + (1.0 - LAT_FILTER) * meas
            else:
                self.lat = meas
            # Persistent lateral error while aligned with walls means the
            # heading estimate is off (e.g. slip at the end of a pivot);
            # nudge it so the mouse settles back onto the centre line.
            self.heading += K_HEADING_FIX * self.lat * v * self.dt
            lateral = self.lat
        else:
            lateral = lateral_from_pose
        self.lat_valid = valid
        head_ofs = clamp(-K_LATERAL * lateral, -MAX_HEAD_OFS, MAX_HEAD_OFS)
        err = path_heading + head_ofs - self.heading
        return clamp(K_HEADING * err, -MAX_STEER_W, MAX_STEER_W)

    # ------------------------------------------------------------------
    # Primitives
    # ------------------------------------------------------------------
    def nearest_heading(self, direction):
        """Absolute target angle for a maze direction, nearest to the current heading."""
        return self.heading + wrap_pi(heading_of(direction) - self.heading)

    def drive_to(self, ex, ey, direction, v_max, v_end=0.0, wall_offset=0.0,
                 accel_max=ACCEL, decel=DECEL, wall_along=None, on_tick=None):
        """Drive along the corridor centre line in maze `direction` until the
        axle reaches the point (ex, ey) on that line, finishing at `v_end`
        (0 = stop at the point).

        wall_offset: where a wall ahead would sit relative to the end point,
        as extra front-ToF distance beyond `front_stop` (0 when stopping at a
        cell centre, CELL/2 when the drive ends at the entry of a turn cell).
        If such a wall is seen, the filtered front ToF takes over the final
        approach. Side walls correct the lateral position, wall edges the
        along-track position.

        wall_along: position along the travel axis of the first wall ahead
        according to the map (None if unknown). Whenever that wall is within
        FRONT_POSE_RANGE, the front ToF pins the along-track position, and a
        stop in front of it gives an exact fix that also calibrates odometry.

        on_tick: optional callback run every control period; it may return a
        new (ex, ey, wall_along) to extend the drive on the fly (used to keep
        going through cells while exploring instead of stopping in each).
        """
        ux = DIR_UX[direction]
        uy = DIR_UY[direction]
        path_heading = self.nearest_heading(direction)
        v = max(0.0, (self.vl + self.vr) / 2.0)
        # Pulling away from rest is when a wheel is most likely to slip; keep
        # the speed low for the first moments so a slip yaws the chassis less.
        soft_ticks = START_SOFT_MS // LOOP_MS if v < 0.03 else 0
        self.slip_correct = True
        self.lat_valid = False
        edges = EdgeTracker(self.left_wall(), self.right_wall(), self.px * ux + self.py * uy)
        front = self.tof_c
        front_ref_x = self.px
        front_ref_y = self.py
        guard = 0
        while guard < 3000:
            guard += 1
            # A side wall starting/ending marks a cell boundary: re-anchor along-track
            fix = edges.update(self.left_wall(), self.right_wall(), self.px * ux + self.py * uy)
            if fix != 0.0:
                self.along_track_fix(fix, ux, uy, EDGE_FIX_GAIN)
            if on_tick is not None:
                ext = on_tick()
                if ext is not None:
                    ex = ext[0]
                    ey = ext[1]
                    wall_along = ext[2]
            accel = accel_max
            if guard <= soft_ticks:
                accel = START_ACCEL
            dx = ex - self.px
            dy = ey - self.py
            remaining = dx * ux + dy * uy
            lateral_pose = dx * uy - dy * ux        # + when the mouse is left of the line
            # front distance: odometry prediction corrected by each new sample
            moved = (self.px - front_ref_x) * ux + (self.py - front_ref_y) * uy
            front_ref_x = self.px
            front_ref_y = self.py
            front -= moved
            if self.tof_c < TOF_MAX_M:
                if front >= TOF_MAX_M or abs(self.tof_c - front) > 0.05:
                    front = self.tof_c          # (re)acquire a wall
                else:
                    front += FRONT_FILTER_GAIN * (self.tof_c - front)
            else:
                front = TOF_MAX_M
            if wall_along is not None and front < FRONT_POSE_RANGE:
                # the mapped wall ahead tells us where we are along the corridor
                err = self._along_from_front(front, wall_along) - (self.px * ux + self.py * uy)
                if abs(err) < FRONT_POSE_WINDOW:
                    self.along_track_fix(err, ux, uy, FRONT_POSE_GAIN)
                    dx = ex - self.px
                    dy = ey - self.py
                    remaining = dx * ux + dy * uy
            if front < FRONT_TRUST_RANGE + wall_offset:
                by_tof = front - self.front_stop - wall_offset
                # wall lines up with our end point, or is unexpectedly closer
                if abs(by_tof - remaining) < 0.06 or by_tof < remaining:
                    remaining = by_tof
                if front - self.front_stop < 0.0:
                    remaining = min(remaining, 0.0)     # never drive into a wall
            coast = v * MOTOR_TAU if v_end == 0.0 else 0.0
            if remaining <= coast:
                break
            v_allow = math.sqrt(v_end * v_end + 2.0 * decel * max(0.0, remaining - coast))
            v = min(v + accel * self.dt, v_max, max(v_allow, CREEP_SPEED))
            w = self._steer(path_heading, lateral_pose, v)
            if self.lat_valid:
                # walls tell us where the centre line really is
                ofs = self.lat - lateral_pose
                self.px -= uy * ofs * POSE_LAT_GAIN
                self.py += ux * ofs * POSE_LAT_GAIN
            # never let steering drive a wheel backwards while going straight
            w_lim = 0.8 * v / self.half_track
            w = clamp(w, -w_lim, w_lim)
            self.tick(v, w)
        if v_end == 0.0:
            self.stop()
            if wall_along is not None and self.tof_c < FRONT_POSE_RANGE:
                # at rest in front of a mapped wall: exact along-track fix
                self.sample_still(3)
                err = self._along_from_front(self.tof_c, wall_along) - (self.px * ux + self.py * uy)
                if abs(err) < FRONT_POSE_WINDOW:
                    self.along_track_fix(err, ux, uy, 1.0, True)
        self.slip_correct = False

    def _along_from_front(self, front, wall_along):
        """Axle position along the travel axis implied by a front reading of
        `front` to a wall face at `wall_along` (a reading of front_stop means
        the axle is half a cell short of the wall)."""
        return wall_along - 0.5 * CELL_M - (front - self.front_stop)

    def arc(self, cx, cy, d_in, d_out, v):
        """Smooth 90 degree turn through the cell centred at (cx, cy), entering
        in maze direction d_in and leaving in d_out: a quarter circle of
        TURN_RADIUS around the corner post, from the entry boundary to the
        exit boundary.

        Tracked geometrically: the lateral error is the distance from that
        circle, the path heading is the circle's tangent at the mouse, and the
        arc ends when the mouse crosses the exit boundary - so an error in one
        turn is corrected rather than passed on to the next.
        """
        uix = DIR_UX[d_in]
        uiy = DIR_UY[d_in]
        uox = DIR_UX[d_out]
        uoy = DIR_UY[d_out]
        sign = 1.0 if uix * uoy - uiy * uox > 0.0 else -1.0     # +1 = left (CCW)
        ox = cx - TURN_RADIUS * uix + TURN_RADIUS * uox          # circle centre (the post)
        oy = cy - TURN_RADIUS * uiy + TURN_RADIUS * uoy
        exit_x = cx + TURN_RADIUS * uox
        exit_y = cy + TURN_RADIUS * uoy
        self.lat_valid = False
        guard = 0
        while guard < 400:
            guard += 1
            if (self.px - exit_x) * uox + (self.py - exit_y) * uoy >= 0.0:
                break
            rx = self.px - ox
            ry = self.py - oy
            r = math.sqrt(rx * rx + ry * ry)
            tangent = math.atan2(ry, rx) + sign * HALF_PI
            tangent = self.heading + wrap_pi(tangent - self.heading)
            lateral = -sign * (r - TURN_RADIUS)     # + when left of the path
            head_ofs = clamp(-K_LATERAL * lateral, -MAX_HEAD_OFS, MAX_HEAD_OFS)
            v_meas = max(0.05, (self.vl + self.vr) / 2.0)
            w = sign * v_meas / TURN_RADIUS + K_HEADING * (tangent + head_ofs - self.heading)
            self.tick(v, w)

    def turn_to(self, target):
        """Pivot on the spot to an absolute heading (rad).

        Speed profile: accelerate at TURN_ALPHA, then brake so that the
        remaining angle *after the drivetrain lag* (rate * TURN_LAG) is just
        enough to stop at TURN_DECEL. Braking is never rate-limited, so the
        mouse does not overshoot and hunt; a small residual error is closed
        at TURN_W_MIN instead of a full-speed reversal.
        """
        w = 0.0
        elapsed = 0
        while elapsed < TURN_TIMEOUT_MS:
            err = target - self.heading
            rate = self.gyro_rate
            if abs(err) < TURN_TOL and abs(rate) < TURN_RATE_TOL:
                break
            if abs(err) < TURN_TOL:
                w_mag = 0.0
            else:
                brake = max(0.0, abs(err) - abs(rate) * TURN_LAG)
                w_mag = min(TURN_W_MAX, math.sqrt(2.0 * TURN_DECEL * brake))
                w_mag = max(w_mag, TURN_W_MIN)
            w_goal = w_mag if err > 0.0 else -w_mag
            if abs(w_goal) > abs(w):
                step = TURN_ALPHA * self.dt
                w = clamp(w_goal, w - step, w + step)
            else:
                w = w_goal
            self.tick(0.0, w)
            elapsed += LOOP_MS
        self.stop()

    def face(self, direction):
        target = self.nearest_heading(direction)
        if abs(target - self.heading) > TURN_TOL:
            self.turn_to(target)

    def pirouette(self):
        """360 degree clockwise spin on the spot, ending on the same heading."""
        self.turn_to(self.heading - TWO_PI)
