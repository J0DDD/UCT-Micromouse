#include "py/runtime.h"
#include "py/mphal.h"
#include "micromouse_kernel.h"
#include "ZD25WQ80C.h"

extern volatile bool mouse_initialized;
extern void initMicroMouse(void);

static mp_obj_t mpy_uct_mouse_init(void) {
    // Disable I2C interrupts in the NVIC to prevent conflicts with polling-mode C-Kernel reads
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);

    // 1. Enable DMA clocks before accessing registers to prevent HardFault, then disable active channels
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel6->CCR &= ~DMA_CCR_EN;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    DMA2_Channel1->CCR &= ~DMA_CCR_EN;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;
    DMA2_Channel3->CCR &= ~DMA_CCR_EN;
    DMA2_Channel4->CCR &= ~DMA_CCR_EN;
    DMA2_Channel5->CCR &= ~DMA_CCR_EN;
    DMA2_Channel6->CCR &= ~DMA_CCR_EN;
    DMA2_Channel7->CCR &= ~DMA_CCR_EN;

    // Force de-initialization state first to pause background tick I2C reads
    mouse_initialized = false;

    // Re-initialize I2C1 and I2C2 to ensure GPIO alternate functions are correct
    // after MicroPython boot pin configurations have finished.
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c2;
    hi2c1.State = HAL_I2C_STATE_RESET;
    hi2c2.State = HAL_I2C_STATE_RESET;
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_DeInit(&hi2c2);
    
    extern void MX_I2C1_Init(void);
    extern void MX_I2C2_Init(void);
    MX_I2C1_Init();
    MX_I2C2_Init();

    // Enable GPIO clocks for motors, encoders, LEDs
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // Explicitly configure PD7 (MOTOR_EN) as a Push-Pull output, initial state LOW
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    // Re-initialize TIM3 (Motor PWM)
    extern TIM_HandleTypeDef htim3;
    HAL_TIM_PWM_DeInit(&htim3);
    extern void MX_TIM3_Init(void);
    MX_TIM3_Init();

    // Re-assert PC6, PC7, PC8, PC9 as AF2 (TIM3 PWM)
    GPIO_InitTypeDef GPIO_InitStruct_TIM3 = {0};
    GPIO_InitStruct_TIM3.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct_TIM3.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_TIM3.Pull = GPIO_NOPULL;
    GPIO_InitStruct_TIM3.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_TIM3.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_TIM3);

    // Start all 4 TIM3 PWM channels
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // Re-initialize TIM4 (Encoders)
    extern TIM_HandleTypeDef htim4;
    HAL_TIM_IC_DeInit(&htim4);
    extern void MX_TIM4_Init(void);
    MX_TIM4_Init();

    // Re-assert PD12, PD13, PD14, PD15 as AF2 (TIM4 IC)
    GPIO_InitTypeDef GPIO_InitStruct_TIM4 = {0};
    GPIO_InitStruct_TIM4.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct_TIM4.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_TIM4.Pull = GPIO_PULLUP;
    GPIO_InitStruct_TIM4.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_TIM4.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct_TIM4);

    // Enable TIM4 interrupts in NVIC and start input capture
    HAL_NVIC_SetPriority(TIM4_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
    HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_4);

    // Enable PB3 master LED gate
    GPIO_InitTypeDef GPIO_InitStruct_Led = {0};
    GPIO_InitStruct_Led.Pin = GPIO_PIN_3;
    GPIO_InitStruct_Led.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_Led.Pull = GPIO_NOPULL;
    GPIO_InitStruct_Led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    // Initialize PC13 (LED0)
    GPIO_InitStruct_Led.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    // Initialize PA4 (LED1) and PA5 (LED2) on 2026 boards
    GPIO_InitStruct_Led.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

    // Release and configure PC14 (LED1) and PC15 (LED2) on legacy 2025 boards
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    PWR->CR1 &= ~PWR_CR1_DBP;

    GPIO_InitStruct_Led.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
    
    initMicroMouse();
    mouse_initialized = true;

    return mp_obj_new_int(1);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_init_obj, mpy_uct_mouse_init);

// 2. uct_mouse.set_motors(left_pwm, right_pwm)
static mp_obj_t mpy_uct_mouse_set_motors(mp_obj_t left, mp_obj_t right) {
    int l = mp_obj_get_int(left);
    int r = mp_obj_get_int(right);
    kernel_set_pwm(l, r);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mpy_uct_mouse_set_motors_obj, mpy_uct_mouse_set_motors);

// 3. uct_mouse.get_tof() -> tuple (left, front_left, center, front_right, right)
static mp_obj_t mpy_uct_mouse_get_tof(void) {
    const KernelState_t* state = kernel_get_state();
    mp_obj_t tuple[5] = {
        mp_obj_new_int(state->tof_l),
        mp_obj_new_int(state->tof_al),
        mp_obj_new_int(state->tof_c),
        mp_obj_new_int(state->tof_ar),
        mp_obj_new_int(state->tof_r)
    };
    return mp_obj_new_tuple(5, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_tof_obj, mpy_uct_mouse_get_tof);

// 3b. uct_mouse.get_tof_raw() -> tuple (left, front_left, center, front_right, right) raw un-thresholded mm
static mp_obj_t mpy_uct_mouse_get_tof_raw(void) {
    const KernelState_t* state = kernel_get_state();
    mp_obj_t tuple[5] = {
        mp_obj_new_int(state->tof_raw_l),
        mp_obj_new_int(state->tof_raw_al),
        mp_obj_new_int(state->tof_raw_c),
        mp_obj_new_int(state->tof_raw_ar),
        mp_obj_new_int(state->tof_raw_r)
    };
    return mp_obj_new_tuple(5, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_tof_raw_obj, mpy_uct_mouse_get_tof_raw);

// 3c. uct_mouse.get_tof_signals() -> tuple (left, front_left, center, front_right, right) signal rate in kcps
static mp_obj_t mpy_uct_mouse_get_tof_signals(void) {
    const KernelState_t* state = kernel_get_state();
    mp_obj_t tuple[5] = {
        mp_obj_new_int(state->tof_sig_l),
        mp_obj_new_int(state->tof_sig_al),
        mp_obj_new_int(state->tof_sig_c),
        mp_obj_new_int(state->tof_sig_ar),
        mp_obj_new_int(state->tof_sig_r)
    };
    return mp_obj_new_tuple(5, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_tof_signals_obj, mpy_uct_mouse_get_tof_signals);

// 3d. uct_mouse.get_tof_detailed() -> tuple of 5 tuples ((dist_mm, sig_kcps), ...)
static mp_obj_t mpy_uct_mouse_get_tof_detailed(void) {
    const KernelState_t* state = kernel_get_state();
    mp_obj_t l[2]  = { mp_obj_new_int(state->tof_raw_l),  mp_obj_new_int(state->tof_sig_l) };
    mp_obj_t al[2] = { mp_obj_new_int(state->tof_raw_al), mp_obj_new_int(state->tof_sig_al) };
    mp_obj_t c[2]  = { mp_obj_new_int(state->tof_raw_c),  mp_obj_new_int(state->tof_sig_c) };
    mp_obj_t ar[2] = { mp_obj_new_int(state->tof_raw_ar), mp_obj_new_int(state->tof_sig_ar) };
    mp_obj_t r[2]  = { mp_obj_new_int(state->tof_raw_r),  mp_obj_new_int(state->tof_sig_r) };
    
    mp_obj_t sensors[5] = {
        mp_obj_new_tuple(2, l),
        mp_obj_new_tuple(2, al),
        mp_obj_new_tuple(2, c),
        mp_obj_new_tuple(2, ar),
        mp_obj_new_tuple(2, r)
    };
    return mp_obj_new_tuple(5, sensors);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_tof_detailed_obj, mpy_uct_mouse_get_tof_detailed);

// 4. uct_mouse.get_encoders() -> tuple (left, right)
static mp_obj_t mpy_uct_mouse_get_encoders(void) {
    extern void kernel_snapshot_state(void);
    kernel_snapshot_state();
    const KernelState_t* state = kernel_get_state();
    mp_obj_t tuple[2] = {
        mp_obj_new_int(state->lenc),
        mp_obj_new_int(state->renc)
    };
    return mp_obj_new_tuple(2, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_encoders_obj, mpy_uct_mouse_get_encoders);

// 4b. uct_mouse.get_gyro() -> float
static mp_obj_t mpy_uct_mouse_get_gyro(void) {
    extern void kernel_snapshot_state(void);
    kernel_snapshot_state();
    const KernelState_t* state = kernel_get_state();
    return mp_obj_new_float(state->gyro);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_gyro_obj, mpy_uct_mouse_get_gyro);

// 4c. uct_mouse.get_imu_type() -> int
static mp_obj_t mpy_uct_mouse_get_imu_type(void) {
    extern int getIMUType(void);
    return mp_obj_new_int(getIMUType());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_imu_type_obj, mpy_uct_mouse_get_imu_type);

// 4d. uct_mouse.scan_i2c(bus) -> list
static mp_obj_t mpy_uct_mouse_scan_i2c(mp_obj_t bus_obj) {
    int bus = mp_obj_get_int(bus_obj);
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c2;
    I2C_HandleTypeDef *hi2c = (bus == 1) ? &hi2c1 : &hi2c2;
    extern uint8_t I2C_Scan(I2C_HandleTypeDef *hi2c, uint8_t *foundAddresses, uint8_t maxAddresses);
    
    uint8_t addrs[16];
    uint8_t count = I2C_Scan(hi2c, addrs, 16);
    mp_obj_t list = mp_obj_new_list(0, NULL);
    for (uint8_t i = 0; i < count; i++) {
        mp_obj_list_append(list, mp_obj_new_int(addrs[i]));
    }
    return list;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mpy_uct_mouse_scan_i2c_obj, mpy_uct_mouse_scan_i2c);

// 4e. uct_mouse.read_i2c(bus, addr, reg, len) -> list of bytes
static mp_obj_t mpy_uct_mouse_read_i2c(size_t n_args, const mp_obj_t *args) {
    int bus = mp_obj_get_int(args[0]);
    int addr = mp_obj_get_int(args[1]);
    int reg = mp_obj_get_int(args[2]);
    int len = (n_args > 3) ? mp_obj_get_int(args[3]) : 1;
    
    extern I2C_HandleTypeDef hi2c1;
    extern I2C_HandleTypeDef hi2c2;
    I2C_HandleTypeDef *hi2c = (bus == 1) ? &hi2c1 : &hi2c2;
    
    if (hi2c->Instance != NULL) {
        hi2c->Instance->ICR = 0x3F38;
    }
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    
    uint8_t buf[32] = {0};
    if (len > 32) len = 32;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), (uint16_t)reg, 1, buf, (uint16_t)len, 10);
    
    mp_obj_t list = mp_obj_new_list(0, NULL);
    if (status == HAL_OK) {
        for (int i = 0; i < len; i++) {
            mp_obj_list_append(list, mp_obj_new_int(buf[i]));
        }
    }
    return list;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mpy_uct_mouse_read_i2c_obj, 3, 4, mpy_uct_mouse_read_i2c);

// 5. uct_mouse.get_vbatt() -> float
static mp_obj_t mpy_uct_mouse_get_vbatt(void) {
    const KernelState_t* state = kernel_get_state();
    return mp_obj_new_float(state->v_batt);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_vbatt_obj, mpy_uct_mouse_get_vbatt);

// 6. uct_mouse.delay_ms(ms)
static mp_obj_t mpy_uct_mouse_delay_ms(mp_obj_t ms_obj) {
    int ms = mp_obj_get_int(ms_obj);
    mp_hal_delay_ms(ms);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mpy_uct_mouse_delay_ms_obj, mpy_uct_mouse_delay_ms);

// 7. uct_mouse.set_polarity(left, right)
static mp_obj_t mpy_uct_mouse_set_polarity(mp_obj_t left, mp_obj_t right) {
    int l = mp_obj_get_int(left);
    int r = mp_obj_get_int(right);
    kernel_set_polarity((int16_t)l, (int16_t)r);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mpy_uct_mouse_set_polarity_obj, mpy_uct_mouse_set_polarity);

// 7_2. uct_mouse.set_encoder_polarity(left, right)
static mp_obj_t mpy_uct_mouse_set_encoder_polarity(mp_obj_t left, mp_obj_t right) {
    int l = mp_obj_get_int(left);
    int r = mp_obj_get_int(right);
    kernel_set_encoder_polarity((int16_t)l, (int16_t)r);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mpy_uct_mouse_set_encoder_polarity_obj, mpy_uct_mouse_set_encoder_polarity);

// 7b. uct_mouse.get_line_sensors() -> tuple (fl, fr, sl, sr)
static mp_obj_t mpy_uct_mouse_get_line_sensors(void) {
    const KernelState_t* state = kernel_get_state();
    mp_obj_t tuple[4] = {
        mp_obj_new_int(state->ir_fl),
        mp_obj_new_int(state->ir_fr),
        mp_obj_new_int(state->ir_sl),
        mp_obj_new_int(state->ir_sr)
    };
    return mp_obj_new_tuple(4, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_line_sensors_obj, mpy_uct_mouse_get_line_sensors);

// 7c. uct_mouse.dump_logs()
#include "serial_interface.h"
#include "py/mpprint.h"
static void mpy_stdout_print(const uint8_t *buf, uint32_t len) {
    mp_printf(&mp_plat_print, "%.*s", (int)len, (const char *)buf);
}

// 7c. uct_mouse.dump_logs()
static mp_obj_t mpy_uct_mouse_dump_logs(void) {
    #include "kernel_logger.h"
    kernel_logger_dump_custom(mpy_stdout_print);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_dump_logs_obj, mpy_uct_mouse_dump_logs);

// 7d. uct_mouse.get_telemetry() -> tuple (ax, ay, az, gx, gy, gz, lenc, renc, current, battery_pct)
static mp_obj_t mpy_uct_mouse_get_telemetry(void) {
    extern float IMU_Accel[3];
    extern float IMU_Gyro[3];
    extern int16_t Current;
    extern int8_t batteryLife;
    const KernelState_t* state = kernel_get_state();
    
    mp_obj_t tuple[10] = {
        mp_obj_new_float(IMU_Accel[0]),
        mp_obj_new_float(IMU_Accel[1]),
        mp_obj_new_float(IMU_Accel[2]),
        mp_obj_new_float(IMU_Gyro[0]),
        mp_obj_new_float(IMU_Gyro[1]),
        mp_obj_new_float(IMU_Gyro[2]),
        mp_obj_new_int(state->lenc),
        mp_obj_new_int(state->renc),
        mp_obj_new_float((float)Current),
        mp_obj_new_int((int)batteryLife)
    };
    return mp_obj_new_tuple(10, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_telemetry_obj, mpy_uct_mouse_get_telemetry);

// 7e. uct_mouse.log_custom(json_str)
static mp_obj_t mpy_uct_mouse_log_custom(mp_obj_t str_obj) {
    const char *str = mp_obj_str_get_str(str_obj);
    extern void kernel_logger_write_custom(const char* json_str);
    kernel_logger_write_custom(str);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mpy_uct_mouse_log_custom_obj, mpy_uct_mouse_log_custom);

// 7f. uct_mouse.get_ticks_ms() -> int
static mp_obj_t mpy_uct_mouse_get_ticks_ms(void) {
    return mp_obj_new_int(HAL_GetTick());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_ticks_ms_obj, mpy_uct_mouse_get_ticks_ms);

// 7g. uct_mouse.set_led(led_idx, state)
#include "LEDs.h"
static mp_obj_t mpy_uct_mouse_set_led(mp_obj_t led_idx_obj, mp_obj_t state_obj) {
    int led_idx = mp_obj_get_int(led_idx_obj);
    int state = mp_obj_get_int(state_obj);
    
    // PB3 (CTRL_LEDS) must be set high to enable LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (led_idx == 0) {
        LED0.state = state ? 1 : 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, pin_state);
    } else if (led_idx == 1) {
        LED1.state = state ? 1 : 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, pin_state);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, pin_state);
    } else if (led_idx == 2) {
        LED2.state = state ? 1 : 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, pin_state);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, pin_state);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mpy_uct_mouse_set_led_obj, mpy_uct_mouse_set_led);

// 7h. uct_mouse.get_button() -> int
static mp_obj_t mpy_uct_mouse_get_button(void) {
    // SW1 is on PE6, active low
    int pressed = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET) ? 1 : 0;
    return mp_obj_new_int(pressed);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_button_obj, mpy_uct_mouse_get_button);

// 7i. uct_mouse.reboot_dfu() -> none
static mp_obj_t mpy_uct_mouse_reboot_dfu(void) {
    extern void jump_to_bootloader(void);
    jump_to_bootloader();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_reboot_dfu_obj, mpy_uct_mouse_reboot_dfu);

// 7j. uct_mouse.erase_flash() -> none
static mp_obj_t mpy_uct_mouse_erase_flash(void) {
    extern ZD25WQ80C_t flash;
    if (!flash.initialized) {
        initZD25WQ80C();
    }
    ZD25WQ80C_ChipErase();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_erase_flash_obj, mpy_uct_mouse_erase_flash);

// 7k. uct_mouse.display_text(row, text) -> none
static mp_obj_t mpy_uct_mouse_display_text(mp_obj_t row_obj, mp_obj_t text_obj) {
    int row = mp_obj_get_int(row_obj);
    const char *text = "";
    if (text_obj != mp_const_none) {
        text = mp_obj_str_get_str(text_obj);
    }
    extern void kernel_set_oled_line(int line, const char* text);
    kernel_set_oled_line(row, text);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mpy_uct_mouse_display_text_obj, mpy_uct_mouse_display_text);

// 7l. uct_mouse.clear_display() -> none
static mp_obj_t mpy_uct_mouse_clear_display(void) {
    extern void kernel_set_oled_line(int line, const char* text);
    kernel_set_oled_line(-1, "");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_clear_display_obj, mpy_uct_mouse_clear_display);

// Define module globals table
static const mp_rom_map_elem_t uct_mouse_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_uct_mouse) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&mpy_uct_mouse_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_reboot_dfu),  MP_ROM_PTR(&mpy_uct_mouse_reboot_dfu_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_motors),  MP_ROM_PTR(&mpy_uct_mouse_set_motors_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tof),     MP_ROM_PTR(&mpy_uct_mouse_get_tof_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tof_raw), MP_ROM_PTR(&mpy_uct_mouse_get_tof_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tof_signals), MP_ROM_PTR(&mpy_uct_mouse_get_tof_signals_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tof_detailed), MP_ROM_PTR(&mpy_uct_mouse_get_tof_detailed_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_encoders),MP_ROM_PTR(&mpy_uct_mouse_get_encoders_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gyro),    MP_ROM_PTR(&mpy_uct_mouse_get_gyro_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_imu_type),MP_ROM_PTR(&mpy_uct_mouse_get_imu_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan_i2c),    MP_ROM_PTR(&mpy_uct_mouse_scan_i2c_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_i2c),    MP_ROM_PTR(&mpy_uct_mouse_read_i2c_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_vbatt),   MP_ROM_PTR(&mpy_uct_mouse_get_vbatt_obj) },
    { MP_ROM_QSTR(MP_QSTR_delay_ms),    MP_ROM_PTR(&mpy_uct_mouse_delay_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_polarity),MP_ROM_PTR(&mpy_uct_mouse_set_polarity_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_encoder_polarity),MP_ROM_PTR(&mpy_uct_mouse_set_encoder_polarity_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_line_sensors), MP_ROM_PTR(&mpy_uct_mouse_get_line_sensors_obj) },
    { MP_ROM_QSTR(MP_QSTR_dump_logs),    MP_ROM_PTR(&mpy_uct_mouse_dump_logs_obj) },
    { MP_ROM_QSTR(MP_QSTR_erase_flash),  MP_ROM_PTR(&mpy_uct_mouse_erase_flash_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_telemetry), MP_ROM_PTR(&mpy_uct_mouse_get_telemetry_obj) },
    { MP_ROM_QSTR(MP_QSTR_log_custom),   MP_ROM_PTR(&mpy_uct_mouse_log_custom_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ticks_ms), MP_ROM_PTR(&mpy_uct_mouse_get_ticks_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_ticks_ms),     MP_ROM_PTR(&mpy_uct_mouse_get_ticks_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_led),      MP_ROM_PTR(&mpy_uct_mouse_set_led_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_button),   MP_ROM_PTR(&mpy_uct_mouse_get_button_obj) },
    { MP_ROM_QSTR(MP_QSTR_display_text), MP_ROM_PTR(&mpy_uct_mouse_display_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_display_text), MP_ROM_PTR(&mpy_uct_mouse_display_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_display),MP_ROM_PTR(&mpy_uct_mouse_clear_display_obj) },
};
static MP_DEFINE_CONST_DICT(uct_mouse_module_globals, uct_mouse_module_globals_table);

// Register built-in module
const mp_obj_module_t uct_mouse_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&uct_mouse_module_globals,
};
MP_REGISTER_MODULE(MP_QSTR_uct_mouse, uct_mouse_module);

// Undefine preprocessor overrides so we can declare the actual hardware interrupt vectors in this file.
#undef TIM4_IRQHandler
#undef EXTI0_IRQHandler
#undef EXTI1_IRQHandler
#undef EXTI2_IRQHandler
#undef EXTI3_IRQHandler
#undef EXTI4_IRQHandler
#undef EXTI9_5_IRQHandler
#undef EXTI15_10_IRQHandler

#include "stm32l4xx_hal.h"



void EXTI0_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF0;
    extern void __real_EXTI0_IRQHandler(void);
    __real_EXTI0_IRQHandler();
}

void EXTI1_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF1;
    extern void __real_EXTI1_IRQHandler(void);
    __real_EXTI1_IRQHandler();
}

void EXTI2_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF2;
    extern void __real_EXTI2_IRQHandler(void);
    __real_EXTI2_IRQHandler();
}

void EXTI3_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF3;
    extern void __real_EXTI3_IRQHandler(void);
    __real_EXTI3_IRQHandler();
}

void EXTI4_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF4;
    extern void __real_EXTI4_IRQHandler(void);
    __real_EXTI4_IRQHandler();
}

void EXTI9_5_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF5 | EXTI_PR1_PIF6 | EXTI_PR1_PIF7 | EXTI_PR1_PIF8 | EXTI_PR1_PIF9;
    extern void __real_EXTI9_5_IRQHandler(void);
    __real_EXTI9_5_IRQHandler();
}

void EXTI15_10_IRQHandler(void) {
    EXTI->PR1 = EXTI_PR1_PIF10 | EXTI_PR1_PIF11 | EXTI_PR1_PIF12 | EXTI_PR1_PIF13 | EXTI_PR1_PIF14 | EXTI_PR1_PIF15;
    extern void __real_EXTI15_10_IRQHandler(void);
    __real_EXTI15_10_IRQHandler();
}
