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
TICK_DIST_M = (2.0 * math.pi * 0.031) / 8.0 # Distance in metres that mouse travels per encoder tick. Determined by (2*pi*r) / encoder resolution i.e. wheel radius is 31mm and there are 8 ticks per revolution 
SIDE_TICKS = int(SIDE_LENGTH_M * (1 / TICK_DIST_M)) # number of encoder ticks per side travelled

# Speed & Drive Tuning
FWD_SPEED       = 70.0          # forward target speed (0 ... 100 range)
TURN_SPEED        = 70.0            # in-place turning PWM (each wheel)

# Controller Gains (PID variables)
KP_DIST = 3 
KI_DIST = 2.5

KP_HEAD = 1
KD_HEAD = -1

# Kalman Filter Constants

# Physical Hardware Calibration (Minimum PWM threshold to get the wheel to actually start spinning, i.e. overcome static friction, etc)
LEFT_DEADBAND   = 40          # Left motor dead-band threshold
RIGHT_DEADBAND  = 40          # Right motor dead-band threshold

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
def update_state(current_dist_m, current_heading_deg, dt_s):
    """Calculates physical position and angle from raw sensors."""
    lenc, renc, gyro_dps = _sensors()
    
    # Convert encoders to distance
    avg_ticks = (lenc + renc) / 2.0
    current_dist_m = avg_ticks * TICK_DIST_M
    
    # Convert gyro to angle
    current_heading_deg += gyro_dps * dt_s
    
    return current_dist_m, current_heading_deg, gyro_dps

# ---------------------------------------------------------------------------
# Controller Logic - PID logic for distance and heading (angle the MM is heading)
# ---------------------------------------------------------------------------
def calc_distance_pi(target_m, current_m, dt_s, err_sum):
    error = target_m - current_m
    err_sum += error * dt_s
    base_speed = (KP_DIST * error) + (KI_DIST * err_sum)
    return base_speed, err_sum

def calc_heading_pd(target_deg, current_deg, gyro_dps):
    error = target_deg - current_deg
    # The gyro directly provides the derivative of heading
    steering_correction = (KP_HEAD * error) - (KD_HEAD * gyro_dps)
    return steering_correction

# ---------------------------------------------------------------------------
# Combine Error Correction - Correct speed and angle
# ---------------------------------------------------------------------------
def apply_motor_mixer(base_speed, steering_correction):
    l_speed = base_speed + steering_correction
    r_speed = base_speed - steering_correction
    
    # Clamp to max physical limits (-100 to 100)
    l_pwm = max(-100, min(100, int(l_speed)))
    r_pwm = max(-100, min(100, int(r_speed)))
    
    uct_mouse.set_motors(l_pwm, r_pwm)

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
    lenc0, renc0, _ = _sensors()
    target_ticks = int(SIDE_TICKS)
    
    current_dist = 0.0
    current_heading = 0.0
    err_sum = 0.0
    dt_s = 0.010  # 10ms control loop step
    
    while current_dist < target_ticks * TICK_DIST_M:
        # 1. Update state and sensors
        current_dist, current_heading, gyro_dps = update_state(current_dist, current_heading, dt_s)
        
        # 2. Calculate PI distance and PD heading corrections
        base_speed, err_sum = calc_distance_pi(distance_m, current_dist, dt_s, err_sum)
        steering_correction = calc_heading_pd(0.0, current_heading, gyro_dps)
        
        # 3. Mix outputs and send to motors
        apply_motor_mixer(base_speed, steering_correction)
                
        # 4. Pace loop timing (critical for physical hardware)
        uct_mouse.delay_ms(10)
        
    # Stop motors after reaching distance
    uct_mouse.set_motors(0, 0)

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
    uct_mouse.set_motors(-TURN_SPEED, TURN_SPEED)
    uct_mouse.delay_ms(600)
    uct_mouse.set_motors(0, 0)
    pass

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
        uct_mouse.set_encoder_polarity(-1,-1)

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

    for side in range(SIDES_TO_TRAVEL):
        # 1. State current side
        print(f"Side: {side}")
        # 2. Drive forward 1 meter
        uct_mouse.set_motors(LEFT_DEADBAND, RIGHT_DEADBAND)
        drive_straight(SIDE_LENGTH_M)
        
        # 3. Settle briefly
        uct_mouse.set_motors(0, 0)
        uct_mouse.delay_ms(200)
        
        # 4. Turn 90 degrees left
        turn_left_90()
        
        # 5. Settle briefly
        uct_mouse.set_motors(0, 0)
        uct_mouse.delay_ms(200)

    # Final stop
    uct_mouse.delay_ms(2800)  # Stop for at least 3 seconds to trigger autograder evaluation completion
    print()
    print("=== Milestone 1 Complete! ===")

if __name__ == "__main__":
    run_square()
