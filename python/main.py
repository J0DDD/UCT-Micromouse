import uct_mouse

print("Initializing Micromouse API...")
uct_mouse.init()

# Ensure motors and LEDs are stopped initially
uct_mouse.set_motors(0, 0)
uct_mouse.set_led(0, 0)
uct_mouse.set_led(1, 0)
uct_mouse.set_led(2, 0)

print("Running Milestone 0 LED range test. Press SW1 (User Button) to start wall-following.")

is_wall_following = 0
side = 0
target_dist = 0

while True:
    # Read TOF sensors
    tof = uct_mouse.get_tof()
    tof_l = tof[0]
    tof_c = tof[2]
    tof_r = tof[4]
    
    # Always run LED Range Test
    val0 = 0
    if tof_l < 200:
        val0 = 1
    uct_mouse.set_led(0, val0)

    val1 = 0
    if tof_c < 200:
        val1 = 1
    uct_mouse.set_led(1, val1)

    val2 = 0
    if tof_r < 200:
        val2 = 1
    uct_mouse.set_led(2, val2)
    
    # Check if button is pressed (SW1 returns 1 when pressed)
    if is_wall_following == 0:
        btn_pressed = uct_mouse.get_button()
        if btn_pressed == 1:
            print("SW1 Pressed! Initiating wall-following...")
            # Decide side to follow based on closer wall
            if tof_l < tof_r:
                side = 1
                target_dist = tof_l
                print("Following LEFT wall")
            else:
                side = 2
                target_dist = tof_r
                print("Following RIGHT wall")
            is_wall_following = 1
            # Debounce delay
            uct_mouse.delay_ms(200)

    if is_wall_following == 1:
        # Check for collision front wall
        if tof_c < 150:
            print("Front obstacle detected! Stopping wall-following.")
            uct_mouse.set_motors(0, 0)
            is_wall_following = 0
            # Debounce delay to prevent immediate re-trigger
            uct_mouse.delay_ms(500)
            
        # Drive straight, correcting distance to side wall
        if is_wall_following == 1:
            if side == 1:
                error = target_dist - tof_l
                corr = error * 0.4
                l_pwm = int(85 + corr)
                r_pwm = int(85 - corr)
            else:
                error = target_dist - tof_r
                corr = error * 0.4
                l_pwm = int(85 - corr)
                r_pwm = int(85 + corr)
                
            # Clamp PWM values to safe limits
            if l_pwm < 75:
                l_pwm = 75
            if l_pwm > 100:
                l_pwm = 100
            if r_pwm < 75:
                r_pwm = 75
            if r_pwm > 100:
                r_pwm = 100
            
            uct_mouse.set_motors(l_pwm, r_pwm)

    # Call delay_ms to keep C kernel tick active and allow background VCP commands to process (50Hz loop)
    uct_mouse.delay_ms(20)
