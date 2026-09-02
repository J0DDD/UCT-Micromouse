# =========================================================================
# UCT Micromouse - Milestone 1: Run a Square (1m x 1m) (Framework)
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
# GRADING:
# - The autograder applies 8% motor imbalance and 8% wheel slip.
# - Open-loop timing alone will accumulate errors. Use encoder and gyro
#   feedback to compensate.
# =========================================================================

import uct_mouse
import math

# ---------------------------------------------------------------------------
# Tuning constants — To be calibrated for scenario
# ---------------------------------------------------------------------------

# Shape Tuning
SIDE_LENGTH_M   = 1.00 # metres per straight
GYRO_TARGET_DEG = 90.0 # target turn angle in degrees
TURN_TOLERANCE  = 1.00 # tolerance allowed in final angular position after turning
SIDES_TO_TRAVEL = 4    # number of sides to travel

# Encoder Variables
TICKS_PER_M     = 5730          # encoder ticks per metre — matches simulator config (tpr=1170, R=0.0325m)
TICK_DIST_M     = 1 / TICKS_PER_M
SIDE_TICKS      = int(SIDE_LENGTH_M * TICKS_PER_M)

# Speed & Drive Tuning
FWD_SPEED       = 90.0      # forward target speed (0 ... 100 range)
MIN_SPEED       = 50.0      # minimum speed that motor needs to turn at
MAX_SPEED       = 100.0     # maximum speed that motors can turn at
MIN_TURN_SPEED  = 60        # minimum speed for turning
TURN_HEADROOM   = 15        # Turn speed headroom 

# Controller Gains (PID variables)
KP_DIST = 1 
KI_DIST = 1

KP_HEAD = 20
KI_HEAD = 1
KD_HEAD = 0.5
KB_HEAD = 0.1 # 1 / KI # Kb is a tuning gain and is typically 1/Ki 

KP_TURN_ANGLE = 0.6
KI_TURN_ANGLE = 0
KD_TURN_ANGLE = 0.05
KB_TURN_ANGLE = 0 # 1 / KI # Kb is a tuning gain and is typically 1/Ki

KP_TURN_VEL = 0.02

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
# Sensor Readings Conversion - Updating the current state of the micromouse
# ---------------------------------------------------------------------------
def update_distance(current_dist_m, lenc_0, renc_0):
    """Calculates physical position and angle from raw sensors."""
    lenc, renc, _ = _sensors()

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

def update_angle(current_angle_deg, dt_s):
    """ Calculate the current angular position/heading from the raw sensors"""
    _, _, gyro_dps = _sensors()

    # Convert gyro reading to angle by integrating
    current_angle_deg += gyro_dps * dt_s

    # Could potentially calculate in place turning angle from [(right distance) - (left distance)] / wheel separation 

    # DEBUGGING
    #print(f"Current Angle: {current_heading_deg} degrees")
    # DEBUGGING

    return current_angle_deg, gyro_dps

def update_velocity(prev_lenc, prev_renc, dt_s):
    """  Returns velocity of each wheel as wheel as current encoder count"""
    lenc, renc, _ = _sensors()

    left_vel = (lenc - prev_lenc) / dt_s
    right_vel = (renc - prev_renc) / dt_s

    return left_vel, right_vel, lenc, renc


# ---------------------------------------------------------------------------
# Controller Logic - PID logic for distance and heading
# ---------------------------------------------------------------------------
"""
def calc_distance_pi(target_m, current_m, dt_s, err_sum):
    error = target_m - current_m
    err_sum += error * dt_s
    base_speed = (KP_DIST * error) + (KI_DIST * err_sum)
    return base_speed, err_sum
"""
    
def calc_heading_pid(target_deg, current_deg, gyro_dps, dt_s, I):
    error = target_deg - current_deg

    # Calculate the PID correction values
    P = KP_HEAD * error
    D = -KD_HEAD * gyro_dps

    correction_raw = P + I + D

    # Back calculation anti-windup pulls the integration term towards a feasible value (so that it does not grow infinitely)
    correction = max( -(FWD_SPEED - MIN_SPEED) , min( correction_raw, MAX_SPEED - FWD_SPEED)) # Clamps correction 
    I +=  (KI_HEAD * error * dt_s) + (KB_HEAD * (correction - correction_raw)) 

    steering_correction = P + I + D
    steering_correction = max( -(FWD_SPEED - MIN_SPEED), min(steering_correction, MAX_SPEED - FWD_SPEED))

    # DEBUGGING
    # print(f"Error: {error}, P: {P}, I: {I}, D: {D}")
    # DEBUGGING

    return steering_correction, I

def calc_angle_pid(target_deg, current_deg, gyro_dps, dt_s, I):
    """ Calculate correction values for the angle for turning. Outputs desired speed for each wheel """
    error = target_deg - current_deg

    # Calculate the PID correction values
    P = KP_TURN_ANGLE * error
    D = -KD_TURN_ANGLE * gyro_dps

    correction_raw = P + I + D

    # Back calculation anti-windup pulls the integration term towards a feasible value (so that it does not grow infinitely)
    correction = max(-MAX_SPEED, min( correction_raw, MAX_SPEED)) # Clamps correction to be within -100 and +100 which allows for the error correction to be negative (i.e. overshoot occured)
    I +=  (KI_TURN_ANGLE * error * dt_s) + (KB_TURN_ANGLE * (correction - correction_raw)) 

    desired_turn_speed = P + I + D
    desired_turn_speed = max(-MAX_SPEED, min(desired_turn_speed, MAX_SPEED))

    # DEBUGGING
    # print(f"Error: {error}, P: {P}, I: {I}, D: {D}")
    # DEBUGGING

    return desired_turn_speed, I

def calc_wheel_balance_pid(left_vel, right_vel):
    """ Calculate correction values for the angle for turning"""
    l_mag = abs(left_vel)
    r_mag = abs(right_vel)
    error = l_mag - r_mag # If positive, left is spinning faster

    # Calculate the PID correction values
    P = KP_TURN_VEL * error

    balance_correction = P
    
    # DEBUGGING
    # print(f"Error: {error}, P: {P}")
    # DEBUGGING

    return balance_correction

# ---------------------------------------------------------------------------
# Combine Error Correction - Correct speed and angle
# ---------------------------------------------------------------------------
def apply_drive_correction(steering_correction):
    l_speed = FWD_SPEED - steering_correction
    r_speed = FWD_SPEED + steering_correction
    
    # Clamp to max physical limits (50 to 100)
    l_pwm = max(MIN_SPEED, min(MAX_SPEED, int(l_speed)))
    r_pwm = max(MIN_SPEED, min(MAX_SPEED, int(r_speed)))
    
    safe_set_motors(l_pwm, r_pwm)

def apply_turn_balance(desired_turn_speed, balance_correction):
    base_magnitude = abs(desired_turn_speed)
    base_magnitude = max(MIN_TURN_SPEED + TURN_HEADROOM, min(MAX_SPEED - TURN_HEADROOM, base_magnitude))

    l_magnitude = max(MIN_TURN_SPEED, min(MAX_SPEED, base_magnitude - balance_correction))
    r_magnitude = max(MIN_TURN_SPEED, min(MAX_SPEED, base_magnitude + balance_correction))

    if (desired_turn_speed < 0):
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
    # Snapshot starting encoder values
    target_ticks = int(SIDE_TICKS)

    lenc_0, renc_0, _ = _sensors()
    
    current_dist = 0.0
    current_heading = 0.0
    I_heading = 0.00 # Integral term for heading
    dt_s = 0.010  # 10ms control loop step
    
    while current_dist < target_ticks * TICK_DIST_M:
        # 1. Update distance and heading
        current_dist = update_distance(current_dist, lenc_0, renc_0)
        current_heading, gyro_dps = update_angle(current_heading, dt_s)
        
        # 2. Calculate PID distance and PD heading corrections
        steering_correction, I_heading = calc_heading_pid(0.0, current_heading, gyro_dps, dt_s, I_heading)

        # DEBUGGING
        # print(f"Current Distance: {current_dist}, Current Heading: {current_heading}, Steering Correction: {steering_correction}")
        # DEBUGGING

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
    I_angle = 0       # Integral term for turning
    last_time = _now_ms() - 50

    error = desired_angle - current_angle

    while (abs(error) > tolerance_deg):
            # 1. Calculate time difference between loop iterations
            if ON_HARDWARE:
                """now = _now_ms()
                dt_s = _elapsed_s(last_time, now)
                last_time = now"""
                dt_s = 0.09
            else:
                # Simulator's internal clock will advance by exactly this value
                dt_s = 0.010

            # 2. Update the angle, yaw rate and the error
            current_angle, gyro_dps = update_angle(current_angle, dt_s)
            error = desired_angle - current_angle                       

            # 3. Calculate desired "speed" (PWM)
            desired_turn_speed, I_angle = calc_angle_pid(desired_angle, current_angle, gyro_dps, dt_s, I_angle)

            # 4. Measure wheel "velocities" (Rate of change of the encoder values)
            left_vel, right_vel, prev_lenc, prev_renc = update_velocity(prev_lenc, prev_renc, dt_s)

            # 5. Calculate  the correction needed to balance the wheel speeds
            balance_correction = calc_wheel_balance_pid(left_vel, right_vel)

            # 6. Set motor speeds
            apply_turn_balance(desired_turn_speed, balance_correction)
            
            # 7. Pace loop timing (critical for physical hardware)
            uct_mouse.delay_ms(10)

            # DEBUGGING
            # print(f"Current Angle: {current_angle}, gyro_dps: {gyro_dps}, dt_s: {dt_s}")
            # DEBUGGING
            
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
    ON_HARDWARE = sys.platform in ('pyboard', 'stm32')
    if ON_HARDWARE:
        print("Press SW1 (User button) on the board to start the run...")
        while uct_mouse.get_button() == 0:
            uct_mouse.delay_ms(50)
        print("Starting in 1 second...")
        uct_mouse.delay_ms(1000)

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
            uct_mouse.delay_ms(200)

    # Final stop
    uct_mouse.delay_ms(2800)  # Stop for at least 3 seconds to trigger autograder evaluation completion
    print()
    print("=== Milestone 1 Complete! ===")

# ---------------------------------------------------------------------------
# Improved Time Difference Tracking - Specific to system
# ---------------------------------------------------------------------------
try:
    import utime
    def _now_ms():
        return utime.ticks_ms()
    def _elapsed_s(start_ms, end_ms):
        return utime.ticks_diff(end_ms, start_ms) / 1000.0
except ImportError:
    import time
    def _now_ms():
        return time.monotonic() * 1000.0
    def _elapsed_s(start_ms, end_ms):
        return (end_ms - start_ms) / 1000.0

if __name__ == "__main__":
    run_square()
