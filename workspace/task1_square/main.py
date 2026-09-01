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
SIDES_TO_TRAVEL = 4  # number of sides to travel

# Encoder Variables
# TICK_DIST_M = (2.0 * math.pi * 0.031) / 8.0 # Distance in metres that mouse travels per encoder tick. Determined by (2*pi*r) / encoder resolution i.e. wheel radius is 31mm and there are 8 ticks per revolution 
# SIDE_TICKS = int(SIDE_LENGTH_M * (1 / TICK_DIST_M)) # number of encoder ticks per side travelled
TICKS_PER_M     = 5730          # encoder ticks per metre — matches simulator config (tpr=1170, R=0.0325m)
TICK_DIST_M = 1 / TICKS_PER_M
SIDE_TICKS      = int(SIDE_LENGTH_M * TICKS_PER_M)

# Speed & Drive Tuning
FWD_SPEED       = 85.0      # forward target speed (0 ... 100 range)
MIN_SPEED       = 50.0      # minimum speed that motor needs to turn at
MAX_SPEED       = 100.0     # maximum speed that motors can turn at
TURN_SPEED      = 70.0      # in-place turning PWM (each wheel)

# Controller Gains (PID variables)
KP_DIST = 1 
KI_DIST = 1

KP_HEAD = 20
KI_HEAD = 1
KD_HEAD = 0.5
KB_HEAD = 0.1 # 1 / KI_HEAD # Kb is a tuning gain and is typically 1/Ki 

# Kalman Filter Constants

VERBOSE         = True          # print debug info during run

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
# Sensor Reading Improvement - Kalman Filter
# ---------------------------------------------------------------------------
"""
    TODO: Implement Kalman Filtering techniques to improve sensor readings
    """


# ---------------------------------------------------------------------------
# Sensor Readings Conversion - Updating the current state of the micromouse
# ---------------------------------------------------------------------------
def update_state(current_dist_m, current_heading_deg, dt_s, lenc_0, renc_0):
    """Calculates physical position and angle from raw sensors."""
    lenc, renc, gyro_dps = _sensors()

    # Subtract the encoder values at the end of the previous state (turning or driving straight)
    lenc -= lenc_0
    renc -= renc_0

    # DEBUGGING
    print(f"lenc: {lenc}, renc: {renc}")
    # DEBUGGING

    # Convert encoders to distance
    avg_ticks = (lenc + renc) / 2.0
    current_dist_m = avg_ticks * TICK_DIST_M
    
    # Convert gyro to angle by integrating
    current_heading_deg += gyro_dps * dt_s

    # DEBUGGING
    #print(f"Current distance: {current_dist_m} m, Current Angle: {current_heading_deg} degrees")
    # DEBUGGING

    return current_dist_m, current_heading_deg, gyro_dps

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

# ---------------------------------------------------------------------------
# Combine Error Correction - Correct speed and angle
# ---------------------------------------------------------------------------
def apply_motor_correction(steering_correction):
    l_speed = FWD_SPEED - steering_correction
    r_speed = FWD_SPEED + steering_correction
    
    # Clamp to max physical limits (50 to 100)
    l_pwm = max(MIN_SPEED, min(MAX_SPEED, int(l_speed)))
    r_pwm = max(MIN_SPEED, min(MAX_SPEED, int(r_speed)))
    
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
        # 1. Update state and sensors
        current_dist, current_heading, gyro_dps = update_state(current_dist, current_heading, dt_s, lenc_0, renc_0)
        
        # 2. Calculate PID distance and PD heading corrections
        steering_correction, I_heading = calc_heading_pid(0.0, current_heading, gyro_dps, dt_s, I_heading)

        # DEBUGGING
        # print(f"Current Distance: {current_dist}, Current Heading: {current_heading}, Steering Correction: {steering_correction}")
        # DEBUGGING

        # 3. Mix outputs and send to motors
        apply_motor_correction(steering_correction)
                
        # 4. Pace loop timing (critical for physical hardware)
        uct_mouse.delay_ms(10)
        
    # Stop motors after reaching distance
    safe_set_motors(0, 0)

# ---------------------------------------------------------------------------
# Movement primitive: turn 90°
# ---------------------------------------------------------------------------

def turn_left_90():
    """
    TODO: Implement closed-loop turning control.
    Use the gyroscope Z-axis angular rate (gyro) to integrate heading angle
    and turn exactly 90 degrees counter-clockwise.
    """
    print("Turning 90 degrees left...")
    # Student code here
    safe_set_motors(-TURN_SPEED, TURN_SPEED)
    uct_mouse.delay_ms(925)
    safe_set_motors(0, 0)
    pass

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
    if sys.platform in ('pyboard', 'stm32'):
        print("Press SW1 (User button) on the board to start the run...")
        while uct_mouse.get_button() == 0:
            uct_mouse.delay_ms(50)
        print("Starting in 1 second...")
        uct_mouse.delay_ms(1000)

    # Initiliase the previous recorded encoder values as 0
    prev_lenc = 0
    prev_renc = 0

    for side in range(SIDES_TO_TRAVEL):
        # 1. State current side
        print(f"Side: {side}")
        # 2. Drive forward 1 meter
        safe_set_motors(FWD_SPEED, FWD_SPEED)
        drive_straight(SIDE_LENGTH_M)
        
        # 3. Settle briefly
        safe_set_motors(0, 0)
        uct_mouse.delay_ms(200)
        
        # 4. Turn 90 degrees left
        turn_left_90()
        
        # 5. Settle briefly
        safe_set_motors(0, 0)
        uct_mouse.delay_ms(200)

    # Final stop
    uct_mouse.delay_ms(2800)  # Stop for at least 3 seconds to trigger autograder evaluation completion
    print()
    print("=== Milestone 1 Complete! ===")

if __name__ == "__main__":
    run_square()
