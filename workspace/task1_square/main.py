# =========================================================================
# UCT Micromouse - Milestone 1: Run a Square (1m x 1m)
# =========================================================================
# ASSIGNMENT DESCRIPTION:
# Implement a control loop to drive the mouse in a 1 meter by 1 meter square,
# turning 90 degrees at each corner, and returning to the start position.
# 
# KEY CONTROLS:
# - uct_mouse.set_motors(left_pwm, right_pwm) -> Set speed (-100 to 100)
# - uct_mouse.get_encoders() -> Returns (left_ticks, right_ticks)
# - uct_mouse.get_tof()      -> Returns (left_mm, center_mm, right_mm)
# - uct_mouse.delay_ms(ms)   -> Suspends execution and updates sensors
#
# =========================================================================

import uct_mouse
import math

# ---------------------------------------------------------------------------
# Tuning constants — To be calibrated for scenario
# ---------------------------------------------------------------------------

# Shape Tuning
SIDE_LENGTH_M   = 1.00 # metres per straight
SIDES_TO_TRAVEL = 4    # number of sides to travel
GYRO_TARGET_DEG = 90.0 # target turn angle in degrees

# Encoder Variables
TICKS_PER_M     = 5730          # encoder ticks per metre — matches simulator config (tpr=1170, R=0.0325m)
TICK_DIST_M     = 1 / TICKS_PER_M
SIDE_TICKS      = int(SIDE_LENGTH_M * TICKS_PER_M)

# Angle Calculation variables
WHEEL_BASE      = 10.5 / 100    # distance between wheels in m
# GYRO_TRUST                    # the trust in the gyro is set when deciding hardware vs software

# Speed & Drive Tuning
FWD_SPEED       = 85.0          # forward target speed (0 ... 100 range)
MIN_SPEED       = 50.0          # minimum speed that motor needs to turn at
MAX_SPEED       = 100.0         # maximum speed that motors can turn at
MIN_TURN_SPEED  = 50            # minimum speed for turning
TURN_HEADROOM   = 15            # Turn speed headroom

# Timing constant
LOOP_DT_MS = 10
LOOP_DT_S = LOOP_DT_MS / 1000

# Feedback Variables
KP_HEAD = 5
KI_HEAD = 0
KD_HEAD = 0
KB_HEAD = 0 # 1 / KI # Kb is a tuning gain and is typically 1/Ki 

KP_TURN_ANGLE   = 1.25 # Error will be between 0-90 for majority of the time hence this should be high
KI_TURN_ANGLE   = 0.5
KD_TURN_ANGLE   = 0.25
KB_TURN_ANGLE   = 0.02
TURN_TOLERANCE  = 1.00 # tolerance allowed in final angular position after turning
GYRO_TOLERANCE  = 5.0  # speed at which gyro needs to fall under in order to exit loop

KP_TURN_VEL = 0.01
KI_TURN_VEL = 0

# Variables to quickly set if want to do sides or just turn
TURN = True
SIDES = True

# ---------------------------------------------------------------------------
# Sensor Reading Helper Functions - Handles Pre-Filter Sensor Readings
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
    
    safe_set_motors(0, 0)
    
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
    print("---------------------------------")
    print()

# ---------------------------------------------------------------------------
# Sensor Reading Conversions - Updating the current state of the micromouse
# ---------------------------------------------------------------------------
def update_distance(current_dist_m, lenc, renc, lenc_0, renc_0):
    """Calculates physical distance travelled and angle from raw sensors."""
    # Subtract the encoder values at the end of the previous state (turning or driving straight)
    lenc -= lenc_0
    renc -= renc_0

    # DEBUGGING
    # print(f"lenc: {lenc}, renc: {renc}")
    # DEBUGGING

    # Convert encoders to distance
    avg_ticks = (lenc + renc) / 2.0
    current_dist_m = avg_ticks * TICK_DIST_M
    
    # DEBUGGING
    #print(f"Current distance: {current_dist_m} m")
    # DEBUGGING

    return current_dist_m

def update_heading(current_angle_deg, gyro_dps, dt_s):
    """ Calculate the current heading from the raw sensors"""
    # Convert gyro reading to angle by integrating
    current_angle_deg += gyro_dps * dt_s

    # DEBUGGING
    #print(f"Current Angle: {current_heading_deg} degrees")
    # DEBUGGING

    return current_angle_deg

def update_turn_angle(current_angle_deg, lenc, renc, prev_lenc, prev_renc, gyro_dps, dt_s):
    """ Calculate the current angular position/heading using a complementary filter which uses the gyro reading and encoder readings"""
    # Convert readings to the change in the angle since last update
    d_gyro = gyro_dps * dt_s
    d_enc  = math.degrees(((renc - prev_renc) - (lenc - prev_lenc)) / (WHEEL_BASE * TICKS_PER_M))

    d_fused = (d_gyro * GYRO_TRUST) + ((1 - GYRO_TRUST) * d_enc)

    current_angle_deg += d_fused

    # DEBUGGING
    # print(f"Current Angle: {current_angle_deg:.3f}, Gyro DPS: {gyro_dps:.3f}, Gyro: {d_gyro:.3f}, Enc: {d_enc:.3f}, Change: {d_fused:.3f}")
    # DEBUGGING

    return current_angle_deg

def update_velocity(lenc, renc, prev_lenc, prev_renc, dt_s):
    """  Returns velocity of each wheel"""
    left_vel = (lenc - prev_lenc) / dt_s
    right_vel = (renc - prev_renc) / dt_s

    return left_vel, right_vel

# ---------------------------------------------------------------------------
# General Helper Functions
# ---------------------------------------------------------------------------
def clamp(value, min_out, max_out):
    """ Only returns the input value if it is within bounds. Otherwise returns closest bound"""
    return max(min_out, min(value, max_out))

# ---------------------------------------------------------------------------
# Controller Logic - PID logic for distance and heading
# --------------------------------------------------------------------------- 
def calc_heading_pid(target_deg, current_deg, gyro_dps, dt_s, I):
    error = target_deg - current_deg

    # Calculate the PID correction values
    P = KP_HEAD * error
    D = -KD_HEAD * gyro_dps

    correction_raw = P + I + D

    # Back calculation anti-windup pulls the integration term towards a feasible value (so that it does not grow infinitely)
    correction = clamp(correction_raw, -(FWD_SPEED - MIN_SPEED) , (MAX_SPEED - FWD_SPEED)) # Clamps correction 
    I +=  (KI_HEAD * error * dt_s) + (KB_HEAD * (correction - correction_raw)) 

    steering_correction = P + I + D
    steering_correction = clamp(steering_correction, -(FWD_SPEED - MIN_SPEED), ( MAX_SPEED - FWD_SPEED))

    return steering_correction, I

def calc_angle_pid(target_deg, current_deg, prev_error, dt_s, I):
    """ Calculate the additional speed needed to be added to the motors when turning. Start out high and go low. Restricted in speed by derivative term"""
    error = target_deg - current_deg

    # Calculate the PID correction values
    P = KP_TURN_ANGLE * abs(error)
    D = KD_TURN_ANGLE * (error - prev_error) / dt_s

    correction_raw = P + I + D
    additional_speed = clamp(correction_raw, 0, MAX_SPEED)
    I += (KI_TURN_ANGLE * error * dt_s) + (KB_TURN_ANGLE * (additional_speed - correction_raw))
 
    return additional_speed, error, I

def calc_wheel_balance_pid(left_vel, right_vel, dt_s, I):
    """ Calculate correction values for the angle for turning"""
    l_mag = abs(left_vel)
    r_mag = abs(right_vel)
    error = l_mag - r_mag # If positive, left is spinning faster

    # Calculate the PID correction values
    P = KP_TURN_VEL * error
    I += KI_TURN_VEL * error * dt_s

    balance_correction = P + I

    return balance_correction, I

# ---------------------------------------------------------------------------
# Combine Error Correction - Correct speed and angle
# ---------------------------------------------------------------------------
def apply_drive_correction(steering_correction):
    l_speed = FWD_SPEED - steering_correction
    r_speed = FWD_SPEED + steering_correction
    
    # Clamp to max physical limits (50 to 100)
    l_pwm = clamp(l_speed, MIN_SPEED, MAX_SPEED)
    r_pwm = clamp(r_speed, MIN_SPEED, MAX_SPEED)
    
    safe_set_motors(l_pwm, r_pwm)

def apply_turn_balance(additional_speed, balance_correction, error):
    base_magnitude = clamp((MIN_TURN_SPEED + additional_speed), (MIN_TURN_SPEED + TURN_HEADROOM), (MAX_SPEED - TURN_HEADROOM))

    l_magnitude = clamp(( base_magnitude - balance_correction), MIN_TURN_SPEED, MAX_SPEED)
    r_magnitude = clamp((base_magnitude + balance_correction), MIN_TURN_SPEED, MAX_SPEED)

    if (error < 0):
        l_pwm = int(l_magnitude)
        r_pwm = -int(r_magnitude)
    else:
        l_pwm = -int(l_magnitude)
        r_pwm = int(r_magnitude)
    
    safe_set_motors(l_pwm, r_pwm)

# ---------------------------------------------------------------------------
# Movement primitive: drive one straight side
# ---------------------------------------------------------------------------

def drive_straight(distance_m):
    """
    TODO: Implement closed-loop straight line control.
    Use encoder counts to measure distance, and gyroscope Z-axis angular rate
    (gyro) to correct heading drift.
    """
    print(f"Driving straight for {distance_m}m...")
    # Student code here
    target_ticks = int(distance_m * TICKS_PER_M)

    lenc_0, renc_0, _ = _sensors()
    
    current_dist    = 0.0
    current_heading = 0.0
    I_heading       = 0.00   # Integral term for heading
    dt_s            = 0.010  # 10ms control loop step
    
    while current_dist < target_ticks * TICK_DIST_M:
        # 1. Get sensor readings
        lenc, renc, gyro_dps = _sensors()

        # 1. Update distance and heading
        current_dist = update_distance(current_dist, lenc, renc, lenc_0, renc_0)
        current_heading = update_heading(current_heading, gyro_dps, dt_s)
        
        # 2. Calculate correction
        steering_correction, I_heading = calc_heading_pid(0.0, current_heading, gyro_dps, dt_s, I_heading)

        # 3. Mix outputs and send to motors
        apply_drive_correction(steering_correction)
        
        # 4. Pace loop timing (critical for physical hardware)
        uct_mouse.delay_ms(10)
        
    # Stop motors after reaching distance
    safe_set_motors(0, 0)

# ---------------------------------------------------------------------------
# Movement primitive: turn 90°
# ---------------------------------------------------------------------------

def turn_desired_angle(desired_angle, tolerance_deg=1.0):
    """
    TODO: Implement closed-loop turning control.
    Use the gyroscope Z-axis angular rate (gyro) to integrate heading angle
    and turn exactly 90 degrees counter-clockwise.
    """
    print("Turning 90 degrees left...")
    # Student code here
    
    current_angle = 0
    prev_lenc, prev_renc, _ = _sensors()
    I_angle = 0         # Integral term for turning
    I_wheel_balance = 0 # Integral term for the error in the wheel speeds
    dt_s = LOOP_DT_S

    error = desired_angle - current_angle
    prev_error = error

    while (abs(error) > tolerance_deg) or (abs(gyro_dps) > GYRO_TOLERANCE):
            # 1. Get sensor readings
            lenc, renc, gyro_dps = _sensors()

            # 2. Update the angle
            current_angle = update_turn_angle(current_angle, lenc, renc, prev_lenc, prev_renc, gyro_dps, dt_s)                     

            # 3. Calculate desired "speed" (PWM)
            additonal_speed, error, I_angle = calc_angle_pid(desired_angle, current_angle, prev_error, dt_s, I_angle)

            # 4. Measure wheel "velocities" (Rate of change of the encoder values)
            left_vel, right_vel = update_velocity(lenc, renc, prev_lenc, prev_renc, dt_s)

            # 5. Calculate  the correction needed to balance the wheel speeds
            balance_correction, I_wheel_balance = calc_wheel_balance_pid(left_vel, right_vel, dt_s, I_wheel_balance)

            # 6. Set motor speeds
            apply_turn_balance(additonal_speed, balance_correction, error)

            # 7. Set the current encoder counts as the previous to be used in the next loop iteration
            # Must be set before delay_ms()
            prev_lenc, prev_renc = lenc, renc
            prev_error = error
            
            # 8. Pace loop timing (critical for physical hardware)
            uct_mouse.delay_ms(10)

        # Stop motors after reaching distance

    safe_set_motors(0, 0)

# ---------------------------------------------------------------------------
# Movement Input - Setting Motor Speed
# ---------------------------------------------------------------------------
def safe_set_motors(l, r):
    """Always sends integer PWM values to the hardware, regardless of
    whether the caller passed floats."""
    uct_mouse.set_motors(int(l), int(r))

# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def run_square():
    if not uct_mouse.init():
        print("Initialization failed.")
        return

    # Load polarity calibration if it exists
    try:
        with open("polarity.txt", "r") as f:
            lines = f.read().strip().split(",")
            uct_mouse.set_polarity(int(lines[0]), int(lines[1]))
            if len(lines) >= 4:
                uct_mouse.set_encoder_polarity(int(lines[2]), int(lines[3]))
    except Exception:
        uct_mouse.set_polarity(-1, -1)
        uct_mouse.set_encoder_polarity(1,1)

    print("--- Milestone 1: Run a Square ---")
    print(f"  Encoder target : {SIDE_TICKS} ticks/side  ({1.00 / TICK_DIST_M} ticks/m)")
    print(f"  Turn target    : {GYRO_TARGET_DEG}°")
    print("---------------------------------")
    print()
    
    # Calibrate gyroscope before movement starts
    calibrate_gyro()

    # On physical hardware, wait for user button SW1 (PE6) press before starting
    import sys
    global ON_HARDWARE
    global GYRO_TRUST
    ON_HARDWARE = sys.platform in ('pyboard', 'stm32')
    if ON_HARDWARE:
        # The gyro is quite noisy on the micromouse hence the trust level decreases
        GYRO_TRUST = 0.25

        print("Press SW1 (User button) on the board to start the run...")
        while uct_mouse.get_button() == 0:
            uct_mouse.delay_ms(50)
        print("Starting in 1 second...")
        uct_mouse.delay_ms(1000)
    else:
        # The gyro is very accurate on the simulator therefore the trust level increases
        GYRO_TRUST = 0.90

    for side in range(SIDES_TO_TRAVEL):
        if (SIDES):
            # 1. State current side
            print(f"Side: {side}")
            # 2. Drive forward 1 meter
            safe_set_motors(FWD_SPEED, FWD_SPEED)
            drive_straight(SIDE_LENGTH_M)
            
            # 3. Settle briefly
            safe_set_motors(0, 0)
            uct_mouse.delay_ms(200)
        
        if (TURN):
            # 4. Turn 90 degrees left
            turn_desired_angle(GYRO_TARGET_DEG, TURN_TOLERANCE)
            
            # 5. Settle briefly
            safe_set_motors(0, 0)
            uct_mouse.delay_ms(500)
        if ON_HARDWARE:
            calibrate_gyro()

    # Final stop
    if not ON_HARDWARE:
        drive_straight(0.005)
    uct_mouse.delay_ms(2800)  # Stop for at least 3 seconds to trigger autograder evaluation completion
    print()
    print("=== Milestone 1 Complete! ===")

if __name__ == "__main__":
    run_square()
