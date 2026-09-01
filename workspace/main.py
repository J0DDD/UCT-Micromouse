# =========================================================================
# UCT Micromouse - Milestone 1: Run a 1 m × 1 m Square (Reference)
# =========================================================================
# ALGORITHM:
#   Drive a 1.0 m × 1.0 m square (4 sides × 4 right-angle turns) using
#   closed-loop feedback on BOTH phases:
#
#   Straight phase  — Fused encoder + gyro control.
#       Encoder balance (P-controller) compensates for motor speed asymmetry.
#       Gyro heading integration corrects residual drift that encoder-only
#       control cannot eliminate (steady-state P-error + wheel slip).
#       Both corrections are summed into each motor PWM command.
#
#   Turn phase      — Gyroscope integration.
#       The on-board IMU gyro (°/s) is integrated over time. Motors run
#       opposite directions until the accumulated heading change reaches 90°.
#       This is robust to motor-speed asymmetry because the gyro directly
#       measures angular rate—it does not care which wheel is faster.
#
# GRADING NOTE (from test_suite.py):
#   The autograder applies 8% motor imbalance + 8% wheel slip.
#   Open-loop timing (e.g. delay_ms(1500)) will accumulate large heading
#   errors and score poorly. The gyro turn + encoder straight implemented
#   here directly compensates for both perturbations.
#
# TUNING:
#   1. Run milestone 1 with VERBOSE = True and observe the printed values.
#   2. Adjust TICKS_PER_M until one straight phase covers exactly 1.0 m.
#   3. If turns are under/over-shooting, adjust GYRO_GAIN (starts at 1.0).
# =========================================================================

import uct_mouse

# ---------------------------------------------------------------------------
# Tuning constants — calibrate these for your specific mouse
# ---------------------------------------------------------------------------
SIDE_LENGTH_M   = 1.00          # metres per straight
TICKS_PER_M     = 5730          # encoder ticks per metre — matches simulator config (tpr=1170, R=0.0325m)
SIDE_TICKS      = int(SIDE_LENGTH_M * TICKS_PER_M)
# Speed & Drive Tuning
FWD_SPEED       = 35.0          # forward target speed (0 ... 100 range, mapped above deadband)
TURN_PWM        = 70            # in-place turning PWM (each wheel)
TURN_PWM_SLOW   = 64            # reduced turn PWM as we approach target (must still be above dead_bands)

# Controller Gains
KP_BALANCE      = 0.5           # P-gain: corrects left/right encoder imbalance during straight
KH_HEADING      = 1.5           # P-gain: corrects accumulated heading drift via gyro integration

# Physical Hardware Calibration Constants (Simulator Dead-bands)
LEFT_DEADBAND   = 58.0          # Left motor dead-band threshold
RIGHT_DEADBAND  = 62.0          # Right motor dead-band threshold

# Slew-Rate Limits (Acceleration Control)
MAX_SLEW_PER_STEP = 2.0         # Max change in command per 10ms step (equivalent to 200 units/sec)

GYRO_TARGET_DEG = 90.0          # target turn angle in degrees
GYRO_GAIN       = 0.91          # scale factor (calibrated for physical ground rotation)
GYRO_SLOP_DEG   = 4.0           # slow-down threshold: reduce power in final X degrees of turn

VERBOSE         = True          # print debug info during run


# ---------------------------------------------------------------------------
# Helpers to read sensors cleanly
# ---------------------------------------------------------------------------

GYRO_BIAS = 0.0

def _sensors():
    """Returns (lenc, renc, gyro_dps) from the current shadow state, subtracting gyro bias."""
    lenc, renc = uct_mouse.get_encoders()
    gyro = uct_mouse.get_gyro()
    return lenc, renc, gyro - GYRO_BIAS


def calibrate_gyro():
    """
    Calibrates the gyroscope Z-axis bias while the mouse is stationary.
    Collects 100 samples over 1.0 second (10ms intervals) to calculate average bias.
    """
    global GYRO_BIAS
    print("  [Calibrating Gyro] Please keep the mouse still...")
    
    uct_mouse.set_motors(0, 0)
    
    # Sensor warm-up: let socket connection and telemetry stream stabilize (200ms)
    for _ in range(20):
        uct_mouse.delay_ms(10)
        
    GYRO_BIAS = 0.0  # Zero out bias during calibration run
    samples = []
    for _ in range(100):  # 100 samples at 10ms = 1.0 second
        uct_mouse.delay_ms(10)
        samples.append(uct_mouse.get_gyro())
        
    GYRO_BIAS = sum(samples) / len(samples)
    print(f"  [Calibrating Gyro] Complete. Estimated bias: {GYRO_BIAS:.4f} dps")


# ---------------------------------------------------------------------------
# Slew-Rate and Dead-band Helpers
# ---------------------------------------------------------------------------

current_l = 0.0
current_r = 0.0

def apply_deadband(cmd_l: float, cmd_r: float) -> tuple:
    """Applies deadband offset feed-forward to raw controller motor outputs."""
    # Left motor mapping
    if cmd_l > 0:
        pwm_l = LEFT_DEADBAND + cmd_l * (100.0 - LEFT_DEADBAND) / 100.0
    elif cmd_l < 0:
        pwm_l = -LEFT_DEADBAND + cmd_l * (100.0 - LEFT_DEADBAND) / 100.0
    else:
        pwm_l = 0.0

    # Right motor mapping
    if cmd_r > 0:
        pwm_r = RIGHT_DEADBAND + cmd_r * (100.0 - RIGHT_DEADBAND) / 100.0
    elif cmd_r < 0:
        pwm_r = -RIGHT_DEADBAND + cmd_r * (100.0 - RIGHT_DEADBAND) / 100.0
    else:
        pwm_r = 0.0

    return int(pwm_l), int(pwm_r)


def set_motors_ramped(target_l: float, target_r: float, slew_rate: float = MAX_SLEW_PER_STEP):
    """Slew-rate limits the motor inputs to prevent wheel slip on rapid acceleration."""
    global current_l, current_r
    
    diff_l = target_l - current_l
    diff_r = target_r - current_r
    
    current_l += max(-slew_rate, min(slew_rate, diff_l))
    current_r += max(-slew_rate, min(slew_rate, diff_r))
    
    pwm_l, pwm_r = apply_deadband(current_l, current_r)
    
    # Clamp to safe physical ranges
    pwm_l = max(-95, min(95, pwm_l))
    pwm_r = max(-95, min(95, pwm_r))
    
    uct_mouse.set_motors(pwm_l, pwm_r)


def stop_motors():
    """Forces an immediate hard stop and resets current ramp state."""
    global current_l, current_r
    current_l = 0.0
    current_r = 0.0
    uct_mouse.set_motors(0, 0)


# ---------------------------------------------------------------------------
# Movement primitive: drive one straight side
# ---------------------------------------------------------------------------

def drive_straight(side_num: int):
    """
    Drive forward exactly SIDE_LENGTH_M metres using fused encoder + gyro control.

    Two correction terms are summed into each motor PWM every physics step:

      1. Encoder balance (KP_BALANCE):
         Measures the tick-count difference between wheels and applies a
         proportional correction. Compensates for motor gain asymmetry but
         has a non-zero steady-state error under pure P-control.

      2. Gyro heading (KH_HEADING):
         Integrates the gyro (°/s) over time to measure accumulated heading
         drift. Positive drift (curving left / CCW) increases right-motor
         PWM and decreases left-motor PWM to steer back. This catches what
         the encoder balance misses: steady-state P-error and wheel slip.

    The loop exits when the average of both encoder deltas reaches SIDE_TICKS.
    """
    lenc0, renc0, _ = _sensors()
    target = SIDE_TICKS
    dt_s   = 0.010          # 100Hz control loop step
    heading_drift = 0.0     # accumulated heading error in degrees (+ = drifting CCW/left)

    if VERBOSE:
        print(f"  [Side {side_num}] Driving {SIDE_LENGTH_M} m  (target {target} ticks)...")

    while True:
        lenc, renc, gyro_dps = _sensors()
        dl = lenc - lenc0
        dr = renc - renc0
        avg = (dl + dr) / 2.0

        if avg >= target:
            break

        # Integrate heading drift (CCW = positive gyro = curving left)
        heading_drift += gyro_dps * dt_s

        # Term 1: encoder balance — keeps arc lengths equal
        cross_error = dl - dr
        enc_correction = KP_BALANCE * cross_error

        # Term 2: gyro heading — drives heading drift back to zero
        #   drift > 0 → curving left → increase right, decrease left
        heading_correction = KH_HEADING * heading_drift

        l_speed = FWD_SPEED - enc_correction + heading_correction
        r_speed = FWD_SPEED + enc_correction - heading_correction

        # Clamp speed commands to safe controller bounds
        l_speed = max(0.0, min(100.0, l_speed))
        r_speed = max(0.0, min(100.0, r_speed))

        # Use a gentler slew rate (1.0) on the very first side to prevent starting wheel-spin, 
        # and normal slew rate (2.0) on subsequent sides.
        slew = 1.0 if side_num == 1 else MAX_SLEW_PER_STEP
        set_motors_ramped(l_speed, r_speed, slew)
        uct_mouse.delay_ms(10)  # Exchange commands and step the simulator physics (10ms rate)

    # Hard stop
    stop_motors()
    lenc_f, renc_f, _ = _sensors()
    if VERBOSE:
        dl_f = lenc_f - lenc0
        dr_f = renc_f - renc0
        print(f"  [Side {side_num}] Done. Ticks L={dl_f}  R={dr_f}  "
              f"imbalance={abs(dl_f-dr_f)} ticks  heading_drift={heading_drift:.1f}°")

    uct_mouse.delay_ms(120)   # coast and settle before turning


# ---------------------------------------------------------------------------
# Movement primitive: turn 90° clockwise using gyro integration
# ---------------------------------------------------------------------------

def turn_left_90(corner_num: int):
    """
    Rotate 90° counter-clockwise in place using gyro integration.

    Strategy:
      - Right wheel drives forward, left wheel drives backward (CCW spin).
      - gyro_dps is positive CCW (right-hand rule Z-axis), so we integrate
        positively: accumulated += gyro_dps × dt
      - Slow-crawl in final GYRO_SLOP_DEG degrees to prevent overshoot.

    This traces: East → North → West → South → East, which fits the
    simulator's starting orientation (theta=0, facing East) inside the arena.
    """
    if VERBOSE:
        print(f"  [Corner {corner_num}] Turning 90° CCW...")

    accumulated = 0.0
    dt_s = 0.010   # 10ms step size

    while accumulated < GYRO_TARGET_DEG * GYRO_GAIN:
        _, _, gyro_dps = _sensors()

        # gyro_dps is positive CCW — integrate directly for a CCW (left) turn.
        step_deg = gyro_dps * dt_s
        accumulated += step_deg

        remaining = GYRO_TARGET_DEG * GYRO_GAIN - accumulated

        if remaining <= GYRO_SLOP_DEG:
            uct_mouse.set_motors(-TURN_PWM_SLOW, TURN_PWM_SLOW)
        else:
            uct_mouse.set_motors(-TURN_PWM, TURN_PWM)
        uct_mouse.delay_ms(10)  # Exchange commands and step the simulator physics (10ms rate)

    stop_motors()
    if VERBOSE:
        print(f"  [Corner {corner_num}] Turn complete. Accumulated {accumulated:.1f}°")

    uct_mouse.delay_ms(150)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def run_square():
    if not uct_mouse.init():
        print("Initialization failed.")
        return

    # Load per-chassis polarity calibration
    try:
        with open("polarity.txt", "r") as f:
            lines = f.read().strip().split(",")
            uct_mouse.set_polarity(int(lines[0]), int(lines[1]))
            if len(lines) >= 4:
                uct_mouse.set_encoder_polarity(int(lines[2]), int(lines[3]))
    except Exception:
        uct_mouse.set_polarity(-1, -1)
        uct_mouse.set_encoder_polarity(1, 1)

    print("=== Milestone 1: Run a 1 m × 1 m Square ===")
    print(f"  Encoder target : {SIDE_TICKS} ticks/side  ({TICKS_PER_M} ticks/m)")
    print(f"  Turn target    : {GYRO_TARGET_DEG}° (gain={GYRO_GAIN})")
    print()

    # Perform gyro calibration before movement begins
    calibrate_gyro()
    print()

    # On physical hardware, wait for user button SW1 (PE6) press before starting
    import sys
    if sys.platform in ('pyboard', 'stm32'):
        print("Press SW1 (User button) on the board to start the run...")
        while uct_mouse.get_button() == 0:
            uct_mouse.delay_ms(50)
        print("Starting in 1 second...")
        uct_mouse.delay_ms(1000)

    for i in range(4):
        drive_straight(i + 1)
        turn_left_90(i + 1)

    # Final stop
    uct_mouse.set_motors(0, 0)
    uct_mouse.delay_ms(3000)  # Stop for at least 3 seconds to trigger autograder evaluation completion
    print()
    print("=== Milestone 1 Complete! ===")


if __name__ == "__main__":
    run_square()
