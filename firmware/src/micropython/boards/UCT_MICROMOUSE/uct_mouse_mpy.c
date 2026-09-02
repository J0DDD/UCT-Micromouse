#include "py/runtime.h"
#include "py/mphal.h"
#include "micromouse_kernel.h"
#include "ZD25WQ80C.h"

extern volatile bool mouse_initialized;
extern void initMicroMouse(void);

static mp_obj_t mpy_uct_mouse_init(void) {
    if (mouse_initialized) {
        return mp_obj_new_int(1);
    }

    // Disable I2C interrupts in the NVIC to prevent conflicts with polling-mode C-Kernel reads
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);

    // 1. Force disable all DMA channels to prevent background memory corruption
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

    // Re-initialize TIM3 (Motor PWM) and TIM4 (Encoders) alternate functions
    extern TIM_HandleTypeDef htim3;
    extern TIM_HandleTypeDef htim4;
    HAL_TIM_PWM_DeInit(&htim3);
    HAL_TIM_IC_DeInit(&htim4);
    
    extern void MX_TIM3_Init(void);
    extern void MX_TIM4_Init(void);
    MX_TIM3_Init();
    MX_TIM4_Init();

    // Enable GPIOD and GPIOC clocks to ensure motor enable and PWM control are active
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // Explicitly re-initialize PD7 (MOTOR_EN) as a Push-Pull output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Enable PB3 master LED gate
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct_Led = {0};
    GPIO_InitStruct_Led.Pin = GPIO_PIN_3;
    GPIO_InitStruct_Led.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_Led.Pull = GPIO_NOPULL;
    GPIO_InitStruct_Led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    // Initialize PC13 (LED0)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct_Led.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    // Initialize PA4 (LED1) and PA5 (LED2)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct_Led.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Led);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    
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

// 4. uct_mouse.get_encoders() -> tuple (left, right)
static mp_obj_t mpy_uct_mouse_get_encoders(void) {
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
    const KernelState_t* state = kernel_get_state();
    return mp_obj_new_float(state->gyro);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mpy_uct_mouse_get_gyro_obj, mpy_uct_mouse_get_gyro);

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
static void mpy_stdout_print(const uint8_t *buf, uint32_t len) {
    mp_hal_stdout_tx_strn((const char *)buf, len);
}

// 7c. uct_mouse.dump_logs()
static mp_obj_t mpy_uct_mouse_dump_logs(void) {
    #include "kernel_logger.h"
    kernel_logger_init();
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
static mp_obj_t mpy_uct_mouse_set_led(mp_obj_t led_idx_obj, mp_obj_t state_obj) {
    int led_idx = mp_obj_get_int(led_idx_obj);
    int state = mp_obj_get_int(state_obj);
    
    // PB3 (CTRL_LEDS) must be set high to enable LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (led_idx == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, pin_state);
    } else if (led_idx == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, pin_state);
    } else if (led_idx == 2) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, pin_state);
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

// Define module globals table
static const mp_rom_map_elem_t uct_mouse_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_uct_mouse) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&mpy_uct_mouse_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_reboot_dfu),  MP_ROM_PTR(&mpy_uct_mouse_reboot_dfu_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_motors),  MP_ROM_PTR(&mpy_uct_mouse_set_motors_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tof),     MP_ROM_PTR(&mpy_uct_mouse_get_tof_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_encoders),MP_ROM_PTR(&mpy_uct_mouse_get_encoders_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_gyro),    MP_ROM_PTR(&mpy_uct_mouse_get_gyro_obj) },
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
    { MP_ROM_QSTR(MP_QSTR_set_led),      MP_ROM_PTR(&mpy_uct_mouse_set_led_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_button),   MP_ROM_PTR(&mpy_uct_mouse_get_button_obj) },
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

// Actual hardware interrupt vector definitions

void TIM4_IRQHandler(void) {
    // Call our custom HAL encoder tick handler
    extern TIM_HandleTypeDef htim4;
    HAL_TIM_IRQHandler(&htim4);
}

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
