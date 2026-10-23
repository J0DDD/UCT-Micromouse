# =========================================================================
# UCT Micromouse - Hardware bring-up and calibration routines
# =========================================================================
# Run these from the MicroPython REPL with the USB cable attached, in this
# order, and copy the suggested values into config.py (rebuild and upload with
# build_robot.py --port COM10 after each change):
#
#   import calibrate
#   calibrate.sensors()      # 1. polarity + ToF/gyro sanity, side_ref/front_stop
#   calibrate.timing()       # 2. control-loop and planner timing on the MCU
#   calibrate.ticks()        # 3. TICKS_PER_M   (push it exactly 1 m by hand)
#   calibrate.straight(5)    #    check: drives 1 m closed-loop
#   calibrate.spin(3)        # 4. HALF_TRACK    (check it ends facing the same way)
#   calibrate.motors()       # 5. motor tables, forwards (needs ~1.5 m clear floor ahead)
#   calibrate.motors(reverse=True)   #    ... and backwards (clear floor BEHIND)
#   calibrate.spin(3, clockwise=True)  #  right turns use the right wheel in reverse
#   calibrate.edge()         # 6. SIDE_SENSOR_X (in the maze, see docstring)
#
# (Each routine can also be smoke-tested against tools/physics_sim.py.)
# =========================================================================

import math
import uct_mouse
from config import *
from motion import Robot, ToFFilter, cell_centre, clamp, setup_polarity, TWO_PI

_robot = None


def _setup():
    """Initialise once per session; return a fresh, freshly calibrated Robot."""
    global _robot
    if _robot is None:
        uct_mouse.init()
        setup_polarity()
    uct_mouse.set_motors(0, 0)
    print("hands off - calibrating in 1.5 s")
    uct_mouse.delay_ms(1500)
    _robot = Robot()                 # fresh state: no stale ToF/odometry from a previous test
    r = _robot
    r.calibrate_gyro()
    r.heading = 0.0
    r.set_position(0.5 * CELL_M, 0.5 * CELL_M)
    r.sample_still(5)                # current sensor readings before anything moves
    return r


def _encoders_m(r):
    e = uct_mouse.get_encoders()
    return e[0] / r.ticks_per_m, e[1] / r.ticks_per_m


# -------------------------------------------------------------------------
# 1. Sensors and polarity
# -------------------------------------------------------------------------
def sensors(seconds=30, period_ms=250):
    """Print live ToF / gyro / encoder / battery readings.

    Checks to do while it runs:
      * push the mouse forward by hand: BOTH encoder counts must increase
        (otherwise fix the encoder polarity in polarity.txt);
      * rotate it anticlockwise (left) by hand: gyro must read POSITIVE;
      * hand in front of each ToF: L = index 0, C = index 2, R = index 4.
    Then place it exactly centred in a cell with walls left, right and ahead:
    the printed averages are SIDE_REF_DEFAULT and FRONT_STOP_DEFAULT.
    """
    r = _setup()
    n = int(seconds * 1000 / period_ms)
    sl = 0.0
    sc = 0.0
    sr = 0.0
    k = 0
    for i in range(n):
        uct_mouse.delay_ms(period_ms)
        t = uct_mouse.get_tof()
        e = uct_mouse.get_encoders()
        g = uct_mouse.get_gyro() - r.gyro_bias
        print("ToF L=%4d FL=%4d C=%4d FR=%4d R=%4d | gyro %+7.2f dps | enc %6d %6d | %.2f V" % (
            t[0], t[1], t[2], t[3], t[4], g, e[0], e[1], uct_mouse.get_vbatt()))
        if i >= n - 8:          # average the last 2 s
            sl += t[0]
            sc += t[2]
            sr += t[4]
            k += 1
    print("")
    print("Averages over the last %d samples: L=%.1f C=%.1f R=%.1f mm" % (k, sl / k, sc / k, sr / k))
    print("If the mouse was centred in a closed cell:")
    print("  SIDE_REF_DEFAULT   = %.3f   (mean of L and R)" % ((sl + sr) / (2000.0 * k)))
    print("  FRONT_STOP_DEFAULT = %.3f" % (sc / (1000.0 * k)))
    print("Gyro bias at rest: %+.3f dps" % r.gyro_bias)


def tof(samples=50, period_ms=20):
    """ToF diagnosis: hold the mouse still with each sensor facing a wall at a
    known distance (measured from the sensor face with a ruler) and compare.

    For the left (L), centre (C) and right (R) sensors prints, over `samples`
    readings: mean and spread of the filtered distance (get_tof), how often it
    dropped out (8190 = signal below the firmware's threshold), the raw
    distance and the return signal strength in kcps.
      * constant offset vs the ruler -> harmless, the code calibrates it out
      * large spread / frequent dropouts -> a problem worth fixing
      * low signal (< ~200 kcps) -> dark/angled surface or a dirty sensor
    """
    uct_mouse.init()
    uct_mouse.set_motors(0, 0)
    idx = (0, 2, 4)
    names = ("L", "C", "R")
    vals = [[], [], []]
    raws = [[], [], []]
    sigs = [[], [], []]
    filt = [[], [], []]
    filters = [ToFFilter(), ToFFilter(), ToFFilter()]
    drops = [0, 0, 0]
    for _ in range(samples):
        uct_mouse.delay_ms(period_ms)
        f = uct_mouse.get_tof()
        d = uct_mouse.get_tof_detailed()
        for k in range(3):
            i = idx[k]
            if f[i] >= TOF_INVALID or f[i] <= 0:
                drops[k] += 1
            else:
                vals[k].append(f[i])
            raws[k].append(d[i][0])
            sigs[k].append(d[i][1])
            filt[k].append(filters[k].update(f[i], period_ms / 1000.0, TOF_OFFSET[k]) * 1000.0)
    for k in range(3):
        v = vals[k]
        rv = raws[k]
        sg = sigs[k]
        if v:
            mean = sum(v) / len(v)
            sd = math.sqrt(sum((x - mean) ** 2 for x in v) / len(v))
            line = "mean %6.1f mm  spread(sd) %5.1f  min %4d  max %4d" % (mean, sd, min(v), max(v))
        else:
            line = "no valid readings"
        print("%s: %s | dropouts %d/%d | raw mean %6.1f | signal %5.0f kcps (min %d)" % (
            names[k], line, drops[k], samples, sum(rv) / len(rv), sum(sg) / len(sg), min(sg)))
        fv = filt[k][5:]                 # what the robot's filter (motion.ToFFilter) outputs
        fm = sum(fv) / len(fv)
        fsd = math.sqrt(sum((x - fm) ** 2 for x in fv) / len(fv))
        print("   filtered + offset-corrected: mean %6.1f mm  spread(sd) %5.1f  min %4d  max %4d" % (fm, fsd, min(fv), max(fv)))


# -------------------------------------------------------------------------
# 2. Timing
# -------------------------------------------------------------------------
def timing(ticks=200):
    """Measure the real control period and how long the planner takes on the
    MCU (floodfill and the sprint planner on an empty 4x6 map)."""
    r = _setup()
    t0 = uct_mouse.get_ticks_ms()
    worst = 0
    prev = t0
    for _ in range(ticks):
        r.tick(0.0, 0.0)
        now = uct_mouse.get_ticks_ms()
        worst = max(worst, now - prev)
        prev = now
    period = (uct_mouse.get_ticks_ms() - t0) / float(ticks)
    import maze as mz
    m = mz.MazeMap(DEFAULT_ROWS, DEFAULT_COLS)
    goals = m.exploration_goals()
    t1 = uct_mouse.get_ticks_ms()
    for _ in range(10):
        m.next_direction(0, 0, mz.EAST, goals, True)
    t2 = uct_mouse.get_ticks_ms()
    for i in range(m.n):
        m.visited[i] = 1
    m.fastest_path((0, 0), mz.EAST, [(DEFAULT_COLS - 1, DEFAULT_ROWS - 1)], TURN_COST_CELLS)
    t3 = uct_mouse.get_ticks_ms()
    print("control period: mean %.1f ms, worst %d ms (target %d ms)" % (period, worst, LOOP_MS))
    print("floodfill step: %.1f ms   sprint planner: %d ms" % ((t2 - t1) / 10.0, t3 - t2))
    if period > LOOP_MS * 1.3:
        print("WARNING: the loop overruns its period; consider LOOP_MS = %d" % (int(period) + 2))


# -------------------------------------------------------------------------
# 3. Distance scale
# -------------------------------------------------------------------------
def straight(cells=5, speed=EXPLORE_SPEED):
    """Drive `cells` cells straight ahead (closed loop, no walls needed).

    Mark the floor under the axle before and after, measure the real distance
    in metres and compute:  TICKS_PER_M_new = TICKS_PER_M * odometer / real
    (the odometer and current TICKS_PER_M are printed).
    """
    r = _setup()
    l0, r0 = _encoders_m(r)
    ex = r.px + cells * CELL_M
    peak = [0.0, 0.0, 0]                 # max measured speed, sum of cruise speeds, samples

    def watch():
        v = (r.vl + r.vr) / 2.0
        peak[0] = max(peak[0], v)
        if r.px > r.px - 1.0 and v > 0.5 * speed:
            peak[1] += v
            peak[2] += 1
        return None

    r.drive_to(ex, r.py, 1, speed, on_tick=watch)
    l1, r1 = _encoders_m(r)
    odo = ((l1 - l0) + (r1 - r0)) / 2.0
    print("odometer: %.4f m (left %.4f, right %.4f), commanded %.3f m" % (
        odo, l1 - l0, r1 - r0, cells * CELL_M))
    r.sample_still(5)
    print("front reading at the stop: %.3f m (stops at %.3f when a wall ends the run)" % (
        r.tof_c, r.front_stop))
    print("speed: commanded max %.3f m/s, measured peak %.3f, measured cruise %.3f" % (
        speed, peak[0], peak[1] / peak[2] if peak[2] else 0.0))
    over = odo - cells * CELL_M
    print("overshoot past the target: %+.1f mm (motors cut at %.3f m/s)" % (over * 1000.0, r.cut_speed))
    if r.cut_speed > 0.01:
        print("  with NO wall ahead (open floor), COAST_TIME = %.3f would cancel it (now %.3f)" % (
            max(0.0, COAST_TIME + over / r.cut_speed), COAST_TIME))
    print("TICKS_PER_M is %.1f -> new value = %.1f * %.4f / <measured metres>" % (
        r.ticks_per_m, r.ticks_per_m, odo))
    return odo


# -------------------------------------------------------------------------
# 4. Track width
# -------------------------------------------------------------------------
def spin(turns=3, clockwise=False):
    """Pivot `turns` full turns anticlockwise, comparing the rotation the
    encoders imply with the gyro's alone.

    HALF_TRACK_new = HALF_TRACK * encoder_angle / gyro_angle (printed).
    Also say how far from its starting direction it ends: if the gyro angle
    is right (~360 x turns) but the mouse is visibly off, the gyro scale is wrong.
    """
    r = _setup()
    l0, r0 = _encoders_m(r)
    g0 = r.gyro_total
    h0 = r.heading
    sign = -1.0 if clockwise else 1.0
    for k in range(1, turns * 4 + 1):           # quarter turns, as in the maze
        r.turn_to(h0 + sign * k * TWO_PI / 4.0)
    l1, r1 = _encoders_m(r)
    enc_angle = ((r1 - r0) - (l1 - l0)) / (2.0 * r.half_track)
    gyro_angle = r.gyro_total - g0
    ratio = enc_angle / gyro_angle if gyro_angle != 0.0 else 1.0
    print("%s: gyro alone: %.1f deg   encoders: %.1f deg   estimate: %.1f deg (target %.0f)" % (
        "clockwise" if clockwise else "anticlockwise", math.degrees(gyro_angle),
        math.degrees(enc_angle), math.degrees(r.heading - h0), sign * 360.0 * turns))
    print("HALF_TRACK is %.4f -> suggested %.4f" % (r.half_track, r.half_track * ratio))
    return ratio


def ticks(seconds=20):
    """Ticks per metre without the controller: push the mouse BY HAND exactly
    1.000 m straight along a ruler or tape (start and end marks under the
    axle) within `seconds`. Prints the counts; TICKS_PER_M = mean count."""
    uct_mouse.init()
    setup_polarity()
    uct_mouse.set_motors(0, 0)
    e0 = uct_mouse.get_encoders()
    print("push the mouse 1.000 m forward now (%d s)..." % seconds)
    for i in range(seconds):
        uct_mouse.delay_ms(1000)
        e = uct_mouse.get_encoders()
        print("%2d s: left %6d  right %6d" % (i + 1, e[0] - e0[0], e[1] - e0[1]))
    e = uct_mouse.get_encoders()
    dl = e[0] - e0[0]
    dr = e[1] - e0[1]
    print("TICKS_PER_M = %.0f   (left %d, right %d; if it was not exactly 1 m, divide by the distance)" % (
        (dl + dr) / 2.0, dl, dr))


# -------------------------------------------------------------------------
# 5. Motor characteristic
# -------------------------------------------------------------------------
def motors(levels=(20, 30, 40, 50, 60, 70, 80, 90, 100), hold_ms=600, reverse=False):
    """Open-loop PWM sweep (both wheels at the same PWM) on open floor.

    Reports each wheel's steady speed per PWM level and prints the measured
    curves as tables ready to paste into config.py (FF_TABLE_L/R, or
    FF_TABLE_L_REV/R_REV with reverse=True). Needs about 1.5 m of clear floor
    ahead (or BEHIND with reverse=True) - it does not steer.
    """
    r = _setup()
    sgn = -1 if reverse else 1
    rows = []
    for p in levels:
        uct_mouse.set_motors(sgn * p, sgn * p)
        t_end = uct_mouse.get_ticks_ms() + hold_ms
        settle = uct_mouse.get_ticks_ms() + hold_ms // 2
        started = False
        while uct_mouse.get_ticks_ms() < t_end:
            uct_mouse.delay_ms(LOOP_MS)
            if not started and uct_mouse.get_ticks_ms() >= settle:
                l0, r0 = _encoders_m(r)
                ts = uct_mouse.get_ticks_ms()
                started = True
        l1, r1 = _encoders_m(r)
        dt = (uct_mouse.get_ticks_ms() - ts) / 1000.0
        rows.append((p, abs(l1 - l0) / dt, abs(r1 - r0) / dt))
        print("PWM %4d: left %.3f m/s  right %.3f m/s" % (sgn * p, rows[-1][1], rows[-1][2]))
    uct_mouse.set_motors(0, 0)
    suffix = "_REV" if reverse else ""
    for side in (1, 2):
        name = "L" if side == 1 else "R"
        moving = [row for row in rows if row[side] > 0.02]
        if not moving:
            print("%s wheel barely moved - check motor polarity / battery" % name)
            continue
        # where it starts: halfway between the last still level and the first moving one
        first = moving[0][0]
        still = [row[0] for row in rows if row[0] < first]
        start = (still[-1] + first) / 2.0 if still else first - 5
        pts = ["(0.0, %d)" % int(start + 0.5)]
        best = 0.0
        for row in moving:
            if row[side] > best + 0.003:            # keep the table strictly increasing
                pts.append("(%.3f, %d)" % (row[side], row[0]))
                best = row[side]
        print("FF_TABLE_%s%s = (%s)" % (name, suffix, ", ".join(pts)))
    print("Paste these into config.py; set SPRINT_SPEED to ~0.85 x the slower top speed.")
    return rows


# -------------------------------------------------------------------------
# 6. Side sensor position (wall edges)
# -------------------------------------------------------------------------
def edge(cells=2, speed=0.15):
    """Find how far ahead of the axle the side ToFs look (SIDE_SENSOR_X).

    Place the mouse centred in a cell, facing along a corridor where a side
    wall starts or ends at a cell boundary within the next `cells` cells
    (e.g. leave out one wall segment). It drives slowly and logs where each
    side reading jumps; since boundaries are at 0.1 m + k * 0.2 m from the
    start, SIDE_SENSOR_X = boundary - axle position at the jump.
    """
    r = _setup()
    r.sample_still(5)                # fresh readings before taking the initial wall state
    x0 = r.px
    state = [r.left_wall(), r.right_wall()]
    events = []

    def watch():
        lw = r.left_wall()
        rw = r.right_wall()
        if lw != state[0] or rw != state[1]:
            if r.px - x0 > 0.01:
                events.append(r.px - x0)
            state[0] = lw
            state[1] = rw
        return None

    r.drive_to(x0 + cells * CELL_M, r.py, 1, speed, on_tick=watch)
    if not events:
        print("no wall edge seen - check the setup")
        return None
    for a in events:
        k = int((a - 0.5 * CELL_M) / CELL_M + 0.5)
        boundary = 0.5 * CELL_M + k * CELL_M
        print("edge at axle %.3f m -> SIDE_SENSOR_X = %.3f" % (a, boundary - a))
    return events
