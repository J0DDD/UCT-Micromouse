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
#   3. Path tracking: straights are followed as geometry (distance to the
#      corridor line) rather than timed manoeuvres.
#   4. Primitives: drive to a point, pivot to an absolute heading, and the
#      360 degree pirouette. (The robot turns on the spot at every corner.)
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


class WallSlope:
    """Parallel check against the side walls while driving.

    If the mouse is angled to the corridor by theta, its offset from the
    centre line changes by theta per metre travelled. A least-squares line
    through the wall-measured offset over WALL_SLOPE_LEN of travel (same
    walls throughout, no posts) gives the true mean angle; the heading
    estimate's own mean angle over the same stretch is compared with it.
    add() returns (estimate - measured) angle error in rad once a window is
    complete, else None."""

    def __init__(self):
        self.reset(-1)

    def reset(self, key):
        self.key = key
        self.n = 0
        self.sx = 0.0
        self.sxx = 0.0
        self.sxl = 0.0
        self.sl = 0.0
        self.spsi = 0.0
        self.s0 = None
        self.psi0 = 0.0
        self.psi1 = 0.0

    def add(self, key, s, lat, psi):
        if key != self.key:
            self.reset(key)                 # walls changed: start a new stretch
        if self.s0 is None:
            self.s0 = s
            self.psi0 = psi
        x = s - self.s0
        self.n += 1
        self.sx += x
        self.sxx += x * x
        self.sxl += x * lat
        self.sl += lat
        self.spsi += psi
        self.psi1 = psi
        if x < WALL_SLOPE_LEN or self.n < WALL_SLOPE_MIN_N:
            return None
        den = self.n * self.sxx - self.sx * self.sx
        if den <= 0.0:
            self.reset(key)
            return None
        slope = (self.n * self.sxl - self.sx * self.sl) / den
        # the side sensors sit SIDE_SENSOR_X ahead of the axle, so their offset
        # also changes by that lever arm times the change of angle
        slope -= SIDE_SENSOR_X * (self.psi1 - self.psi0) / x
        err = self.spsi / self.n - slope
        self.reset(key)
        return err


class PostRejector:
    """Side reading for STEERING with wall posts removed.

    A post sticks out ~2 cm from the wall face, so passing one shows as a
    short burst of closer readings. Steering cannot dodge it in time anyway
    (the burst is over in a few cm), and reacting just makes the mouse
    swerve, so a drop of more than POST_DROP below the running wall level
    that lasts less than POST_MAX_LEN of travel is ignored (the wall level is
    reported instead). A closeness that persists longer is a real wall."""

    def __init__(self):
        self.base = None
        self.burst = 0.0

    def reset(self):
        self.base = None
        self.burst = 0.0

    def update(self, x, ds):
        if self.base is None or x > self.base + POST_RISE:
            self.base = x                   # (re)start, or the wall fell away
            self.burst = 0.0
            return x
        if x < self.base - POST_DROP:
            self.burst += ds
            if self.burst < POST_MAX_LEN:
                return self.base            # a post going past: ignore it
            self.base = x                   # sustained: a real, closer wall
            self.burst = 0.0
            return x
        self.burst = 0.0
        self.base += POST_BASE_GAIN * (x - self.base)
        return x


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
        self.ticks_per_m_l = TICKS_PER_M * TICKS_TRIM_L     # each wheel's own scale
        self.ticks_per_m_r = TICKS_PER_M * TICKS_TRIM_R
        self.half_track = HALF_TRACK
        self.gyro_trust = GYRO_TRUST
        self.gyro_bias = 0.0
        self.side_ref = SIDE_REF_DEFAULT    # mean of the two side references (reporting)
        self.side_ref_l = SIDE_REF_DEFAULT  # per-sensor references (calibrate_in_start_cell)
        self.side_ref_r = SIDE_REF_DEFAULT
        self.too_close = False              # a side sensor is inside the minimum clearance
        self.slope = WallSlope()            # parallel check against the side walls
        self.last_slope_fix = 0.0           # deg, last heading correction it made (log)
        self.post_l = PostRejector()        # side readings for steering, posts removed
        self.post_r = PostRejector()
        self.steer_l = TOF_MAX_M
        self.steer_r = TOF_MAX_M
        self.steer_fwd = 0.0
        self.lat_far = False                # lateral measured from walls one cell further out
        self.steer_lw = False               # wall states with hysteresis, for steering only
        self.steer_rw = False
        self.fwd = 0.0                      # forward odometer (m), to hold position in pivots
        self.front_stop = FRONT_STOP_DEFAULT
        self.stop_bias = 0.0                # m, learned: extra distance to keep at wall stops
        self.last_stop_err = None           # m, how far the last wall stop missed (log)

        self.heading = 0.0          # continuous (unwrapped) world heading, rad
        self.px = 0.5 * CELL_M      # position in maze metres (start cell centre)
        self.py = 0.5 * CELL_M
        self.gyro_rate = 0.0        # rad/s
        self.gyro_total = 0.0       # integrated gyro alone, rad (calibrate.spin)
        self.cut_speed = 0.0        # speed when the last drive cut the motors (calibration)
        self.vl = 0.0               # filtered wheel speeds, m/s
        self.vr = 0.0
        self.il = 0.0               # wheel-loop integrators, PWM
        self.ir = 0.0
        self.vbatt = FF_TABLE_VBATT # filtered battery voltage (battery compensation)
        v = uct_mouse.get_vbatt()
        if VBATT_MIN < v < VBATT_MAX:
            self.vbatt = v
        self.lat = 0.0              # filtered wall-measured lateral error, m (+ = left)
        self.lat_valid = False
        self.lat_delta = 0.0        # m, wall correction of the lateral position this period
        self.slip_correct = False   # only meaningful while driving straight
        self.odo_scale = 1.0        # learned true-metres per encoder-metre (slip, wheel size)
        self.pos_unc = 0.0          # m, how far off the position may be (grows with driving
                                    # and curves, shrinks with wall fixes)
        self.arcs_since_fix = 0     # curves driven since the position was last pinned
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
        dl = (enc[0] - self.enc_l) / self.ticks_per_m_l
        dr = (enc[1] - self.enc_r) / self.ticks_per_m_r
        self.enc_l = enc[0]
        self.enc_r = enc[1]

        # GYRO_SCALE corrects the gyro's scale (from calibrate.turns()): if every
        # turn comes out a few % short or long, this makes 90 deg really 90 deg
        rate = math.radians(uct_mouse.get_gyro() - self.gyro_bias) * GYRO_SCALE
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
        self.pos_unc = min(POS_UNC_MAX, self.pos_unc + POS_UNC_PER_M * abs(ds))
        self.fwd += ds
        if ds > 0.0:
            self.odo_since_fix += ds
        mid = self.heading + 0.5 * d_head
        self.px += ds * math.cos(mid)
        self.py += ds * math.sin(mid)
        self.heading += d_head

        if BATTERY_COMP:
            v = uct_mouse.get_vbatt()
            if VBATT_MIN < v < VBATT_MAX:   # ignore glitched readings
                self.vbatt += VBATT_FILTER * (v - self.vbatt)

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
        self._pinned(1.0 if exact else gain)
        if not exact:
            return
        d = self.odo_since_fix
        if d > ODO_LEARN_MIN_DIST:
            ratio = clamp((d + self.corr_since_fix) / d, 0.9, 1.1)
            self.odo_scale = clamp(self.odo_scale * (1.0 + ODO_LEARN_GAIN * (ratio - 1.0)),
                                   ODO_SCALE_MIN, ODO_SCALE_MAX)
        self.odo_since_fix = 0.0
        self.corr_since_fix = 0.0

    def _pinned(self, gain):
        """A wall fix of strength `gain` (1 = exact): the position is that
        much more certain again."""
        self.pos_unc *= 1.0 - gain
        if self.pos_unc < POS_UNC_FIXED:
            self.arcs_since_fix = 0

    def localise_axis(self, direction, wall_along, reading, sensor_ofs):
        """Stationary position fix along maze `direction` from a ToF reading to
        a mapped wall face at `wall_along` on that axis. `sensor_ofs` is the
        reading the sensor gives at a cell centre with the wall at that cell's
        boundary (front_stop for the front sensor, side_ref for the sides)."""
        ux = DIR_UX[direction]
        uy = DIR_UY[direction]
        expected = wall_along - 0.5 * CELL_M - (reading - sensor_ofs)
        err = expected - (self.px * ux + self.py * uy)
        if abs(err) < LOCALISE_WINDOW + self.pos_unc:
            self.px += ux * err * LOCALISE_GAIN
            self.py += uy * err * LOCALISE_GAIN
            self._pinned(LOCALISE_GAIN)

    def set_position(self, x, y):
        self.px = x
        self.py = y

    def left_wall(self):
        return self.tof_l < self.side_ref_l + SIDE_WALL_MARGIN

    def right_wall(self):
        return self.tof_r < self.side_ref_r + SIDE_WALL_MARGIN

    def _update_steer_walls(self):
        """Wall presence for steering, with hysteresis: a wall must read
        clearly close to appear and clearly far to disappear, so an open
        side whose wide beam catches posts does not flip the lateral
        estimate back and forth."""
        ds = max(0.0, self.fwd - self.steer_fwd)
        self.steer_fwd = self.fwd
        self.steer_l = self.post_l.update(self.tof_l, ds)
        self.steer_r = self.post_r.update(self.tof_r, ds)
        if self.steer_lw:
            self.steer_lw = self.steer_l < self.side_ref_l + STEER_WALL_OFF
        else:
            self.steer_lw = self.steer_l < self.side_ref_l + STEER_WALL_ON
        if self.steer_rw:
            self.steer_rw = self.steer_r < self.side_ref_r + STEER_WALL_OFF
        else:
            self.steer_rw = self.steer_r < self.side_ref_r + STEER_WALL_ON

    def _wall_lateral(self):
        """Lateral offset of the side-sensor point from the corridor centre,
        each side measured against its own calibrated reference."""
        self._update_steer_walls()
        lw = self.steer_lw
        rw = self.steer_rw
        self.lat_far = False
        if lw and rw:
            return ((self.steer_r - self.side_ref_r) - (self.steer_l - self.side_ref_l)) / 2.0, True
        if lw:
            return self.side_ref_l - self.steer_l, True
        if rw:
            return self.steer_r - self.side_ref_r, True
        # No wall in this cell on either side: cells are exactly CELL_M, so a
        # wall one cell further out reads ~ref + CELL_M - use that (noisier,
        # so it is smoothed more and trusted less, see _steer).
        far_l = self.steer_l - (self.side_ref_l + CELL_M)     # + = further from it than centred
        far_r = self.steer_r - (self.side_ref_r + CELL_M)
        ok_l = abs(far_l) < FAR_WALL_TOL
        ok_r = abs(far_r) < FAR_WALL_TOL
        if ok_l or ok_r:
            self.lat_far = True
            if ok_l and ok_r:
                return (far_r - far_l) / 2.0, True
            if ok_l:
                return -far_l, True
            return far_r, True
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

    def _curve_slope(self, speed, table):
        """Local slope of a measured motor curve: m/s gained per PWM step at
        `speed`. Steep where the motor is touchy (the right motor just above
        its deadband), shallow near top speed."""
        for i in range(1, len(table)):
            if speed <= table[i][0] or i == len(table) - 1:
                s0, p0 = table[i - 1]
                s1, p1 = table[i]
                return max(1e-4, (s1 - s0) / (p1 - p0))
        return 1e-4

    def _wheel_pwm(self, ref, meas, integ, table=None):
        if abs(ref) < 0.002:
            return 0.0, 0.0
        err = ref - meas
        ff = self._feedforward(abs(ref), table)
        if ref < 0.0:
            ff = -ff
        kp = WHEEL_KP
        ki = WHEEL_KI
        if table is not None:
            # Gain scheduling: aim for the same loop gain everywhere on the
            # motor's (very non-linear) curve, so a touchy region does not
            # make the wheel over-correct and the mouse weave.
            kp = clamp(WHEEL_LOOP_GAIN / self._curve_slope(abs(ref), table), WHEEL_KP_MIN, WHEEL_KP_MAX)
            ki = kp * (WHEEL_KI / WHEEL_KP)
        trial = clamp(integ + ki * err * self.dt, -WHEEL_I_MAX, WHEEL_I_MAX)
        # anti-windup: stop integrating while the output is saturated in the
        # direction the error pushes (otherwise braking afterwards is delayed)
        raw = ff + kp * err + trial
        if (raw > 100.0 and err > 0.0) or (raw < -100.0 and err < 0.0):
            trial = integ
        pwm = clamp(ff + kp * err + trial, -100.0, 100.0)
        return pwm, trial

    def tick(self, v, w):
        """One control period: command forward speed v (m/s) and yaw rate w (rad/s)."""
        ref_l = v - w * self.half_track
        ref_r = v + w * self.half_track
        # each wheel and each direction has its own measured motor curve
        pl, self.il = self._wheel_pwm(ref_l, self.vl, self.il,
                                      FF_TABLE_L if ref_l >= 0.0 else FF_TABLE_L_REV)
        pr, self.ir = self._wheel_pwm(ref_r, self.vr, self.ir,
                                      FF_TABLE_R if ref_r >= 0.0 else FF_TABLE_R_REV)
        self._check_direction(ref_l, ref_r)
        if BATTERY_COMP:
            # The motor tables were measured at FF_TABLE_VBATT: with a fuller
            # battery the same PWM spins the wheels faster (and slower when
            # low), so scale the output to give the speed the tables promise.
            s = clamp(FF_TABLE_VBATT / self.vbatt, BATTERY_COMP_MIN, BATTERY_COMP_MAX)
            pl = clamp(pl * s, -100.0, 100.0)
            pr = clamp(pr * s, -100.0, 100.0)
        uct_mouse.set_motors(int(pl), int(pr))
        # delay_ms() is a plain wait on the robot and the Python code itself
        # takes time: hold a steady LOOP_MS period by waiting only for the rest
        wait = LOOP_MS - (uct_mouse.get_ticks_ms() - self.last_ms)
        if wait > 0:
            uct_mouse.delay_ms(wait)
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

    def sample_still(self, n=5, track_bias=None):
        """Average n stationary ToF samples (more robust wall classification).

        Also re-estimates the gyro bias: the mouse is standing still, so the
        gyro should read zero. Over a 100+ s run even 0.1-0.2 deg/s of left-
        over bias turns into 10-20 deg of heading error (mouse no longer
        parallel to the walls after a turn), so every stop nudges the bias
        towards what the gyro reads at rest - only if the encoders confirm
        nothing moved."""
        sl = 0.0
        sc = 0.0
        sr = 0.0
        sg = 0.0
        e0 = uct_mouse.get_encoders()
        for _ in range(n):
            uct_mouse.delay_ms(LOOP_MS)
            self._read_sensors()
            sl += self.tof_l
            sc += self.tof_c
            sr += self.tof_r
            sg += uct_mouse.get_gyro()
        self.tof_l = sl / n
        self.tof_c = sc / n
        self.tof_r = sr / n
        e1 = uct_mouse.get_encoders()
        if track_bias is None:
            track_bias = n >= 5
        if track_bias and abs(e1[0] - e0[0]) <= 1 and abs(e1[1] - e0[1]) <= 1:
            self.gyro_bias += GYRO_BIAS_TRACK * (sg / n - self.gyro_bias)

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
        """Measure the sensor references in the start cell, which the rules
        wall in on the North, South and West sides.

        Side reference: the mean of the left and right readings - exact even
        if the mouse is placed a little off-centre, because one side reads
        short by exactly what the other reads long.
        Front stopping reference: the same trick front-on. Turn to face the
        North wall, then the South wall (FRONT_CAL_ROUNDS times each) and
        average: placement error cancels between the two directions and the
        repeats average out the sensor noise. Returns a short report string.
        """
        self.sample_still(10)
        side_l = self.tof_l
        side_r = self.tof_r
        sides_ok = side_l < 0.15 and side_r < 0.15
        if sides_ok:
            self.side_ref = (side_l + side_r) / 2.0
            self.side_ref_l = self.side_ref
            self.side_ref_r = self.side_ref
        h0 = self.heading
        north = []
        south = []
        for _ in range(FRONT_CAL_ROUNDS):
            self.turn_to(h0 + HALF_PI)
            self.sample_still(10)
            if self.tof_c < FRONT_STOP_MAX + 0.04:
                north.append(self.tof_c)
            self.turn_to(h0 - HALF_PI)
            self.sample_still(10)
            if self.tof_c < FRONT_STOP_MAX + 0.04:
                south.append(self.tof_c)
        self.turn_to(h0)
        self.sample_still(3)
        if north and south:
            n_mean = sum(north) / len(north)
            s_mean = sum(south) / len(south)
            stop = (n_mean + s_mean) / 2.0
            if FRONT_STOP_MIN < stop < FRONT_STOP_MAX:
                self.front_stop = stop
                north_ofs = (s_mean - n_mean) / 2.0
                if sides_ok:
                    # How far North of centre the mouse sits: facing North read
                    # short by it, facing South long. Facing East the left
                    # sensor saw the North wall (short by it), the right the
                    # South wall (long by it) - so each sensor's own centred
                    # reference follows.
                    self.side_ref_l = side_l + north_ofs
                    self.side_ref_r = side_r - north_ofs
                return "front cal: N %s S %s -> %.3f | side refs L %.3f R %.3f (placed %+.0f mm N of centre)" % (
                    ",".join(["%.3f" % v for v in north]), ",".join(["%.3f" % v for v in south]), stop,
                    self.side_ref_l, self.side_ref_r, 1000.0 * north_ofs)
            return "front cal: average %.3f out of range - kept default %.3f" % (stop, self.front_stop)
        return "front cal: no North/South wall in the start cell - kept default %.3f" % self.front_stop

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
                # two walls give a cleaner (averaged) measurement: smooth it
                # less; walls a cell away are noisier: smooth them more
                f = LAT_FILTER_TWO if (self.steer_lw and self.steer_rw) else LAT_FILTER
                if self.lat_far:
                    f = LAT_FILTER_FAR
                self.lat = f * self.lat + (1.0 - f) * meas
            else:
                self.lat = meas
            # Persistent lateral error while aligned with walls means the
            # heading estimate is off (e.g. slip at the end of a pivot);
            # nudge it so the mouse settles back onto the centre line.
            self.heading += K_HEADING_FIX * self.lat * v * self.dt
            # parallel check: only walls of this cell (far walls are too noisy)
            if not self.lat_far:
                key = (2 if self.steer_lw else 0) + (1 if self.steer_rw else 0)
                e = self.slope.add(key, self.fwd, meas, self.heading - path_heading)
                if e is not None and abs(e) < WALL_SLOPE_MAX_ERR:
                    self.heading -= WALL_SLOPE_GAIN * e
                    self.last_slope_fix = -WALL_SLOPE_GAIN * e * 57.29578
            else:
                self.slope.reset(-1)
            lateral = self.lat
            if STEER_ON_POSE:
                # Steer on the position estimate instead: the gyro and
                # encoders move it every period without delay, and each wall
                # reading pulls it part of the way to what the walls say. A
                # smoothed wall reading lags (and lags more the faster the
                # mouse goes), and jumps when the walls beside it change -
                # both made the sprint weave. Walls that are less reliable
                # pull less instead of being steered on more weakly.
                g = POSE_GAIN_TWO if (self.steer_lw and self.steer_rw) else POSE_GAIN_ONE
                if self.lat_far:
                    g = POSE_GAIN_FAR
                # the side sensors sit SIDE_SENSOR_X ahead of the axle: when
                # angled, they are that much more to the side than the axle
                axle = meas - SIDE_SENSOR_X * math.sin(self.heading - path_heading)
                self.lat_delta = g * (axle - lateral_from_pose)
                lateral = lateral_from_pose + self.lat_delta
            # With only one wall, aim slightly towards it, away from the posts
            # of the open side (steering target only - self.lat, which also
            # corrects the position estimate, stays the true offset).
            if self.steer_lw and not self.steer_rw:
                lateral -= OPEN_SIDE_LEAN
            elif self.steer_rw and not self.steer_lw:
                lateral += OPEN_SIDE_LEAN
        else:
            lateral = lateral_from_pose
        if not (valid and STEER_ON_POSE):
            self.lat_delta = 0.0
        self.lat_valid = valid
        # Minimum clearance: whatever the centring is doing, if either side
        # sensor reads less than CLEAR_MIN_READING (a wall, or a post on an
        # open side) that measurement takes over and steers away harder.
        lim = MAX_HEAD_OFS
        k_lat = K_LATERAL
        if valid and not STEER_ON_POSE:
            if not (self.steer_lw and self.steer_rw):
                k_lat = K_LATERAL * K_LATERAL_ONE_WALL  # one wall: a weaker estimate, trust it less
            if self.lat_far:
                k_lat = K_LATERAL * K_LATERAL_FAR       # walls a cell away: weaker still
        near_l = CLEAR_MIN_READING - self.steer_l      # + = left sensor inside the minimum
        near_r = CLEAR_MIN_READING - self.steer_r
        close_l = near_l > 0.0
        close_r = near_r > 0.0
        self.too_close = close_l or close_r
        if close_l and close_r:
            # squeezed on both sides: steer for the middle of the two readings
            lateral = (self.steer_r - self.steer_l) / 2.0
            k_lat = K_LATERAL
            lim = MAX_HEAD_OFS_CLOSE
        elif close_l:
            lateral = max(lateral, near_l)               # push right, at least by the intrusion
            k_lat = K_LATERAL
            lim = MAX_HEAD_OFS_CLOSE
        elif close_r:
            lateral = min(lateral, -near_r)              # push left
            k_lat = K_LATERAL
            lim = MAX_HEAD_OFS_CLOSE
        head_ofs = clamp(-k_lat * lateral, -lim, lim)
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
        self.post_l.reset()
        self.post_r.reset()
        self.slope.reset(-1)
        self.steer_fwd = self.fwd
        edges = EdgeTracker(self.left_wall(), self.right_wall(), self.px * ux + self.py * uy)
        front = self.tof_c
        front_ref_x = self.px
        front_ref_y = self.py
        jump = 0                    # consecutive readings disagreeing with the filtered front
        tof_stop = False            # the stop was placed by the front ToF
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
                # the reading is FRONT_TOF_LAG old (sensor time + median
                # filter): the wall is already that much closer by now
                tof_now = self.tof_c - max(0.0, (self.vl + self.vr) / 2.0) * FRONT_TOF_LAG
                if front >= TOF_MAX_M:
                    front = tof_now             # acquire a wall
                    jump = 0
                elif abs(tof_now - front) > 0.05:
                    # a jump only counts once it persists: single stray short
                    # readings (cone catching a post) used to end drives early
                    jump += 1
                    if jump >= FRONT_JUMP_CONFIRM:
                        front = tof_now
                        jump = 0
                else:
                    jump = 0
                    front += FRONT_FILTER_GAIN * (tof_now - front)
            else:
                front = TOF_MAX_M
            if wall_along is not None and front < FRONT_POSE_RANGE:
                # the mapped wall ahead tells us where we are along the corridor
                err = self._along_from_front(front, wall_along) - (self.px * ux + self.py * uy)
                # the less sure the position (after curves), the bigger the
                # mismatch that is still believed to be the mapped wall
                if abs(err) < FRONT_POSE_WINDOW + self.pos_unc:
                    self.along_track_fix(err, ux, uy, FRONT_POSE_GAIN)
                    dx = ex - self.px
                    dy = ey - self.py
                    remaining = dx * ux + dy * uy
            if front < FRONT_TRUST_RANGE + wall_offset:
                # stop FRONT_STOP_EXTRA short of the centre for more clearance
                by_tof = front - (self.front_stop + FRONT_STOP_EXTRA + self.stop_bias) - wall_offset
                usable = True
                if wall_along is not None:
                    # the map says where the wall ahead is: a reading that
                    # disagrees with it is a stray, not the wall
                    expected = wall_along - (self.px * ux + self.py * uy) - 0.5 * CELL_M + self.front_stop
                    usable = abs(front - expected) < FRONT_MAP_WINDOW + self.pos_unc
                # wall lines up with our end point, or is unexpectedly closer
                if usable and (abs(by_tof - remaining) < 0.06 or by_tof < remaining):
                    remaining = by_tof
                    tof_stop = True
                if front < self.front_stop - FRONT_EMERGENCY:
                    remaining = min(remaining, 0.0)     # never drive into a wall
            # after the motors are cut the mouse rolls on ~speed * COAST_TIME
            coast = max(0.0, (self.vl + self.vr) / 2.0) * COAST_TIME if v_end == 0.0 else 0.0
            if remaining <= coast:
                self.cut_speed = (self.vl + self.vr) / 2.0      # for calibrate.straight()
                break
            v_allow = math.sqrt(v_end * v_end + 2.0 * decel * max(0.0, remaining - coast))
            v = min(v + accel * self.dt, v_max, max(v_allow, CREEP_SPEED))
            if self.too_close:
                # inside the minimum clearance of a wall/post: slow down while
                # the clearance rule steers away
                v = min(v, CLOSE_SPEED)
            w = self._steer(path_heading, lateral_pose, v)
            if STEER_ON_POSE:
                # apply the wall correction _steer made to the lateral position
                self.px -= uy * self.lat_delta
                self.py += ux * self.lat_delta
            elif self.lat_valid and not self.lat_far:
                # walls tell us where the centre line really is (only walls
                # in this cell - the far ones are too noisy for the estimate)
                ofs = self.lat - lateral_pose
                self.px -= uy * ofs * POSE_LAT_GAIN
                self.py += ux * ofs * POSE_LAT_GAIN
            # never let steering drive a wheel backwards while going straight
            w_lim = 0.8 * v / self.half_track
            w = clamp(w, -w_lim, w_lim)
            self.tick(v, w)
        if v_end == 0.0:
            self.brake()
            self.stop(BRAKED_SETTLE_MS)     # the brake has already stopped the wheels
            self.last_stop_err = None
            near = wall_along is not None and self.tof_c < FRONT_POSE_RANGE
            if (tof_stop and wall_offset == 0.0) or near:
                self.sample_still(3)        # one sample serves both uses below
            if tof_stop and wall_offset == 0.0:
                # Learn from every wall stop: if the mouse keeps ending up too
                # close (or too far), shift where it stops by part of the miss
                err = self.tof_c - (self.front_stop + FRONT_STOP_EXTRA)   # - = too close
                if abs(err) < SQUARE_MAX_ERR:
                    self.last_stop_err = err
                    self.stop_bias = clamp(self.stop_bias - STOP_LEARN_GAIN * err,
                                           -STOP_BIAS_MAX, STOP_BIAS_MAX)
            if wall_along is not None and self.tof_c < FRONT_POSE_RANGE:
                # at rest in front of a mapped wall: exact along-track fix
                err = self._along_from_front(self.tof_c, wall_along) - (self.px * ux + self.py * uy)
                if abs(err) < FRONT_POSE_WINDOW + self.pos_unc:
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
        turn is corrected rather than passed on to the next (the straight
        after it re-centres on the walls).
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
        # a curve is driven on odometry and gyro alone (no walls to check
        # against), and the wheels scrub: the position is less certain after it
        self.pos_unc = min(POS_UNC_MAX, self.pos_unc + ARC_POS_UNC)
        self.arcs_since_fix += 1

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
        fwd0 = self.fwd
        while elapsed < TURN_TIMEOUT_MS:
            # hold the centre on the spot: a sticky wheel that lags makes the
            # mouse roll while it pivots, so drive that roll back to zero
            v_hold = clamp(-PIVOT_HOLD_GAIN * (self.fwd - fwd0), -PIVOT_HOLD_VMAX, PIVOT_HOLD_VMAX)
            err = target - self.heading
            rate = self.gyro_rate
            if abs(err) < TURN_TOL and abs(rate) < TURN_RATE_TOL:
                break
            # End the turn with the active brake, not by coasting: once the
            # angle left is what the mouse covers while stopping, stop both
            # wheels dead. Timed on the encoders' turn rate, which has no
            # delay (the gyro reading lags slightly, which made turns brake
            # late and swing past). If it stops short it creeps on and brakes
            # again, so it never has to come back from past the target.
            enc_rate = (self.vr - self.vl) / (2.0 * self.half_track)
            if (TURN_BRAKE_LEAD > 0.0 and err * enc_rate > 0.0
                    and abs(err) <= abs(enc_rate) * TURN_BRAKE_LEAD):
                self.brake(TURN_BRAKE_PWM)
                w = 0.0
                elapsed += LOOP_MS
                continue
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
            self.tick(v_hold, w)
            elapsed += LOOP_MS
        self.stop(BRAKED_SETTLE_MS)

    def square_up(self):
        """At rest facing a wall: creep forwards / backwards until the front
        reading equals the stopping reading, so the mouse is exactly where a
        turn should start. Corrects a stop that came in short or long (the
        fast runs stop less precisely), which a turn would otherwise convert
        into a sideways error for the next straight. Returns the error
        (m, + = was too far from the wall) or None if there is no wall close
        enough ahead to use."""
        target = self.front_stop + FRONT_STOP_EXTRA
        self.sample_still(3)
        err = self.tof_c - target
        if abs(err) > SQUARE_MAX_ERR:
            return None                     # no wall close ahead (or a bad reading)
        start_err = err
        hold = round(self.heading / HALF_PI) * HALF_PI      # keep the current axis
        elapsed = 0
        while elapsed < SQUARE_TIMEOUT_MS and abs(err) > SQUARE_TOL:
            v = SQUARE_SPEED if err > 0.0 else -SQUARE_SPEED
            w = clamp(K_HEADING * (hold - self.heading), -1.0, 1.0)
            self.tick(v, w)
            elapsed += LOOP_MS
            err = self.tof_c - target
        self.stop()
        return start_err

    def brake(self, pwm=None):
        """Active braking at the end of a drive. Simply cutting the power lets
        each wheel roll to a stop on its own, and the left motor (much less
        friction) rolls on longer than the right, leaving the mouse slightly
        turned after every stop. Instead, each wheel that is still turning
        gets a short reverse push until it has stopped - both wheels stop
        within a fraction of a second, independently of their friction."""
        if pwm is None:
            pwm = BRAKE_PWM
        if pwm <= 0:
            return
        elapsed = 0
        while elapsed < BRAKE_MAX_MS:
            pl = 0
            pr = 0
            if self.vl > BRAKE_STOP_V:
                pl = -pwm
            elif self.vl < -BRAKE_STOP_V:
                pl = pwm
            if self.vr > BRAKE_STOP_V:
                pr = -pwm
            elif self.vr < -BRAKE_STOP_V:
                pr = pwm
            if pl == 0 and pr == 0:
                break
            uct_mouse.set_motors(pl, pr)
            wait = LOOP_MS - (uct_mouse.get_ticks_ms() - self.last_ms)
            if wait > 0:
                uct_mouse.delay_ms(wait)
            self._read_sensors()
            elapsed += LOOP_MS
        uct_mouse.set_motors(0, 0)

    def face(self, direction):
        """Turn to a maze direction - or, if already facing it, re-align when
        the heading is more than FACE_TOL off (e.g. a small yaw left by the
        last stop), so the next drive starts straight."""
        target = self.nearest_heading(direction)
        if abs(target - self.heading) > FACE_TOL:
            self.turn_to(target)

    def pirouette(self):
        """360 degree clockwise spin on the spot, ending on the same heading."""
        self.turn_to(self.heading - TWO_PI)
