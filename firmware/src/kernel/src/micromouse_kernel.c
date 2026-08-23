#include "micromouse_kernel.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h> // For offsetof()

// Hardware proxy implementation
#include "stm32l4xx_hal.h"
// Include Jesse's hardware drivers from the MicroMouseTemplate submodule
#include "Motors.h"

// Suppress macro redefinition warning from IMU.h
#ifdef I2C_TIMEOUT
#undef I2C_TIMEOUT
#endif
#include "IMU.h"

#include "Buttons.h"
#include "VL53L0X.h"

#include "INA219.h"
#include "ADCs.h"
#include "SSD1306.h"
// Expose Jesse's underlying global hardware variables
extern float IMU_Gyro[3];
extern float IMU_Gyro_DPS[3];
extern float IMU_Accel[3];
extern int16_t Vbattery;
extern int16_t Current;

// 2026 Generation Hardware Hooks:
// Encoders are not yet implemented in the base Simulink hardware template.
// These will eventually be updated by EXTI/Timer interrupts as pulses arrive.
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;

static KernelState_t current_state;
static KernelState_t shadow_state;
static volatile uint32_t watchdog_timer_ms = 0;
static uint32_t stream_rate_hz = 25; // Lower default to prevent 115200 baud bandwidth saturation
static bool force_sync = false;
static bool use_delta_enc = false;
static char kernel_error_msg[64] = "";

// --- Simulink Display Override Buffers ---
static char g_oled_header[20] = "";
static char g_oled_line1[20] = "";
static char g_oled_line2[20] = "";
static char g_oled_line3[20] = "";
static char g_oled_line4[20] = "";
#if defined(COMPILING_FOR_MICROPYTHON)
static char kernel_title[20] = "   MicroPython    ";
#elif defined(COMPILING_FOR_PIKASCRIPT)
static char kernel_title[20] = "   PikaScript     ";
#else
static char kernel_title[20] = "   Simulink       ";
#endif

// Hardware-specific wiring polarities (1 = normal, -1 = reversed)
// Standard differential drive usually requires one to be -1. Adjust as needed per physical board.
static volatile int16_t polarity_l = -1;
static volatile int16_t polarity_r = -1;
static volatile int16_t enc_polarity_l = 1;
static volatile int16_t enc_polarity_r = 1;

// -------------------------------------------------------------
// Telemetry Configuration Table
// -------------------------------------------------------------
typedef enum { VAR_UINT8, VAR_UINT16, VAR_INT32, VAR_FLOAT_3DP, VAR_FLOAT_2DP } VarType_t;

typedef struct {
    const char* name;
    size_t offset;
    VarType_t type;
    bool allow_delta;
} TelemetryField_t;

#define FIELD(n, t, d) {#n, offsetof(KernelState_t, n), t, d}

static const TelemetryField_t telemetry_table[] = {
    FIELD(tof_l,  VAR_UINT16,    false), FIELD(tof_al, VAR_UINT16,    false),
    FIELD(tof_c,  VAR_UINT16,    false), FIELD(tof_ar, VAR_UINT16,    false),
    FIELD(tof_r,  VAR_UINT16,    false),
    FIELD(ir_fl,  VAR_UINT16,    false), FIELD(ir_fr,  VAR_UINT16,    false),
    FIELD(ir_sl,  VAR_UINT16,    false), FIELD(ir_sr,  VAR_UINT16,    false),
    FIELD(lenc,   VAR_INT32,     true),  FIELD(renc,   VAR_INT32,     true),
    FIELD(gyro,   VAR_FLOAT_3DP, false), FIELD(v_batt, VAR_FLOAT_2DP, false),
    FIELD(btn1,   VAR_UINT8,     false), FIELD(btn2,   VAR_UINT8,     false),
    FIELD(ax,      VAR_FLOAT_2DP, false), FIELD(ay,      VAR_FLOAT_2DP, false),
    FIELD(az,      VAR_FLOAT_2DP, false), FIELD(gx,      VAR_FLOAT_2DP, false),
    FIELD(gy,      VAR_FLOAT_2DP, false), FIELD(gz,      VAR_FLOAT_2DP, false),
    FIELD(current, VAR_FLOAT_2DP, false),
    FIELD(bdcr,    VAR_INT32,     false)
};

static const int num_telemetry_fields = sizeof(telemetry_table) / sizeof(telemetry_table[0]);

void kernel_init(void) {
    memset(&current_state, 0, sizeof(KernelState_t));
    memset(&shadow_state, 0, sizeof(KernelState_t));
    watchdog_timer_ms = 0;

}

void kernel_set_pwm(int16_t left_pwm, int16_t right_pwm) {
    // Apply hardware-specific wiring polarities
    int16_t actual_l = left_pwm * polarity_l;
    int16_t actual_r = right_pwm * polarity_r;

    printf("ACTUATE: L=%d (act=%d), R=%d (act=%d)\r\n", left_pwm, actual_l, right_pwm, actual_r);

    current_state.left_pwm = left_pwm;   // Report the original INTENT back to telemetry
    current_state.right_pwm = right_pwm; 
    
    MOTOR_L.magnitude = actual_l;
    MOTOR_R.magnitude = actual_r;
    watchdog_timer_ms = 0; // Feed watchdog on standalone internal actuation

    // Convert from percentage (0-100) to timer duty cycle (0-999)
    int16_t abs_l = (actual_l < 0) ? -actual_l : actual_l;
    int16_t abs_r = (actual_r < 0) ? -actual_r : actual_r;
    uint32_t duty_l = ((uint32_t)abs_l * 1000) / 100;
    uint32_t duty_r = ((uint32_t)abs_r * 1000) / 100;
    if (duty_l > 999) duty_l = 999;
    if (duty_r > 999) duty_r = 999;

    // Bypass Jesse's int8_t abs() logic which causes signed integer casting faults 
    // on the Left Motor. Apply the hardware timer mapping natively here:
    if (actual_l >= 0) {
        TIM3->CCR3 = duty_l;
        TIM3->CCR4 = 0;
    } else {
        TIM3->CCR3 = 0;
        TIM3->CCR4 = duty_l; 
    }

    if (actual_r >= 0) {
        TIM3->CCR1 = duty_r;
        TIM3->CCR2 = 0;
    } else {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = duty_r;
    }

    // Only enable the motor driver if there's actual movement requested.
    // Otherwise, explicitly disable it for safety and to prevent any boot-up glitches.
    if (left_pwm != 0 || right_pwm != 0) {
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
    }
}

void jump_to_bootloader(void) {
    // Stop motors for safety
    kernel_set_pwm(0, 0);
    
    // Disable NVIC interrupts
    __disable_irq();
    
    // Disable SysTick
    SysTick->CTRL = 0;
    
    // Clear any pending interrupts in NVIC
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    
    // Set up the jump address for STM32L4 (System Memory begins at 0x1FFF0000)
    uint32_t jump_addr = *(__IO uint32_t *)(0x1FFF0004);
    void (*SysMemBootJump)(void) = (void (*)(void))jump_addr;
    
    // Set Main Stack Pointer (MSP) to bootloader's stack pointer
    __set_MSP(*(__IO uint32_t *)(0x1FFF0000));
    
    // Jump!
    SysMemBootJump();
    
    while (1);
}

const KernelState_t* kernel_get_state(void) {
    return &current_state;
}

void kernel_parse_downlink(const char* rx_string) {
    // Feed the watchdog on any communication
    watchdog_timer_ms = 0;

    if (rx_string[0] != '{') {
        snprintf(kernel_error_msg, sizeof(kernel_error_msg), "missing opening brace");
        return;
    }

    if (strstr(rx_string, "\"a\":") != NULL) {
        int pwm_l, pwm_r;
        // Use standard %d to avoid newlib-nano %hd stripping bugs, and safely check return code
        if (sscanf(rx_string, "{\"a\":[%d,%d]}", &pwm_l, &pwm_r) == 2) {
            kernel_set_pwm((int16_t)pwm_l, (int16_t)pwm_r);
        } else {
            snprintf(kernel_error_msg, sizeof(kernel_error_msg), "actuator parse failed");
        }
    } 
    else if (strstr(rx_string, "\"c\":") != NULL) {
        int rate;
        if (strstr(rx_string, "\"rate\":") != NULL) {
            if (sscanf(rx_string, "{\"c\":{\"rate\":%d}}", &rate) == 1) {
                stream_rate_hz = (uint32_t)rate;
            } else {
                snprintf(kernel_error_msg, sizeof(kernel_error_msg), "rate parse failed");
            }
        }
        if (strstr(rx_string, "\"sync\":1") != NULL) force_sync = true;
        if (strstr(rx_string, "\"enc_mode\":\"d\"") != NULL) use_delta_enc = true;
        if (strstr(rx_string, "\"enc_mode\":\"a\"") != NULL) use_delta_enc = false;
        if (strstr(rx_string, "\"dump\":1") != NULL) {
            extern void kernel_logger_dump(void);
            kernel_logger_dump();
        }
        if (strstr(rx_string, "\"erase\":1") != NULL) {
            extern HAL_StatusTypeDef ZD25WQ80C_ChipErase(void);
            kernel_set_pwm(0, 0);
            ZD25WQ80C_ChipErase();
        }
        if (strstr(rx_string, "\"bootloader\":1") != NULL) {
            jump_to_bootloader();
        }
    }
    else if (strstr(rx_string, "\"p\":") != NULL) {
        // Polling ping, handled implicitly
    }
    else {
        snprintf(kernel_error_msg, sizeof(kernel_error_msg), "unknown command");
    }
}

uint32_t kernel_get_stream_rate_hz(void) {
    return stream_rate_hz;
}

void kernel_snapshot_state(void) {
    current_state.tof_l  = TOF_sb_left_result.Distance;
    current_state.tof_al = TOF_sb_front_left_result.Distance;
    current_state.tof_c  = TOF_sb_front_result.Distance;
    current_state.tof_ar = TOF_sb_front_right_result.Distance;
    current_state.tof_r  = TOF_sb_right_result.Distance;
    current_state.ir_fl = 0; // TODO: Connect to Jesse's IR drivers
    current_state.ir_fr = 0; 
    current_state.ir_sl = 0;
    current_state.ir_sr = 0;
    current_state.lenc = leftEncoderCount * enc_polarity_l;
    current_state.renc = rightEncoderCount * enc_polarity_r;
    current_state.gyro   = IMU_Gyro_DPS[2] * 2.0f;    // Yaw axis in degrees per second (DPS) with hardware scaling correction
    current_state.v_batt = (float)Vbattery / 1000.0f; 
    current_state.btn1 = SW1.state;
    current_state.btn2 = SW2.state;
    current_state.ax = IMU_Accel[0];
    current_state.ay = IMU_Accel[1];
    current_state.az = IMU_Accel[2];
    current_state.gx = IMU_Gyro[0];
    current_state.gy = IMU_Gyro[1];
    current_state.gz = IMU_Gyro[2];
    current_state.current = (float)Current;
    current_state.bdcr = RCC->BDCR;
}

int kernel_generate_uplink(char* tx_buffer, int max_len) {
    // 1. Snapshot the physical hardware state into our current_state struct before transmitting
    kernel_snapshot_state();

    // 2. Generate the JSON string (Absolute or Delta-Shadow based on configuration)
    int written = snprintf(tx_buffer, max_len, "{");
    bool first = true;
    
    // If there is a pending error message, broadcast it immediately
    if (kernel_error_msg[0] != '\0') {
        written += snprintf(tx_buffer + written, max_len - written, "\"err\":\"%s\"", kernel_error_msg);
        kernel_error_msg[0] = '\0';
        first = false;
    }

    if (force_sync) {
        if (!first) { written += snprintf(tx_buffer + written, max_len - written, ","); }
        written += snprintf(tx_buffer + written, max_len - written, "\"sync\":1");
        first = false;
    }

    for (int i = 0; i < num_telemetry_fields; i++) {
        const TelemetryField_t* f = &telemetry_table[i];
        void* curr_ptr = (uint8_t*)&current_state + f->offset;
        void* shadow_ptr = (uint8_t*)&shadow_state + f->offset;

        // Calculate data size for memory comparison
        size_t sz = (f->type == VAR_UINT8) ? 1 : ((f->type == VAR_UINT16) ? 2 : 4);

        // Sparse Encoding Check: Skip unchanged fields unless forcing an absolute baseline
        if (!force_sync && (memcmp(curr_ptr, shadow_ptr, sz) == 0)) {
            continue; 
        }

        if (!first) { written += snprintf(tx_buffer + written, max_len - written, ","); }
        first = false;

        bool do_delta = (use_delta_enc && f->allow_delta && !force_sync);
        const char* pfx = do_delta ? "+" : "";

        switch(f->type) {
            case VAR_UINT8:  written += snprintf(tx_buffer+written, max_len-written, "\"%s%s\":%u", pfx, f->name, *(uint8_t*)curr_ptr); break;
            case VAR_UINT16: written += snprintf(tx_buffer+written, max_len-written, "\"%s%s\":%u", pfx, f->name, *(uint16_t*)curr_ptr); break;
            case VAR_INT32:  {
                int32_t val = do_delta ? (*(int32_t*)curr_ptr - *(int32_t*)shadow_ptr) : *(int32_t*)curr_ptr;
                written += snprintf(tx_buffer+written, max_len-written, "\"%s%s\":%ld", pfx, f->name, val);
                break; }
            case VAR_FLOAT_3DP:
            case VAR_FLOAT_2DP: {
                float val = *(float*)curr_ptr;
                int i_part = (int)val, frac_mult = (f->type == VAR_FLOAT_3DP) ? 1000 : 100;
                int f_part = (int)(val * frac_mult) % frac_mult;
                if (f_part < 0) f_part = -f_part;
                const char* fmt = (f->type == VAR_FLOAT_3DP) ? "\"%s\":%s%d.%03d" : "\"%s\":%s%d.%02d";
                written += snprintf(tx_buffer+written, max_len-written, fmt, f->name, (val < 0 && i_part == 0) ? "-" : "", i_part, f_part);
                break; }
        }
    }

    written += snprintf(tx_buffer + written, max_len - written, "}\r\n");
        
    shadow_state = current_state;
    force_sync = false;
    
    return written;
}

void kernel_watchdog_tick(void) {
    watchdog_timer_ms++;
    
    /* Temporarily disabled for diagnostic testing
    if (watchdog_timer_ms == 1001) {
        HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
        
        current_state.left_pwm = 0;
        current_state.right_pwm = 0;
        
        MOTOR_L.magnitude = 0;
        MOTOR_R.magnitude = 0;
        
        TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
        TIM3->CCR3 = 0;
        TIM3->CCR4 = 0;
    }
    */
}

void kernel_set_title(const char* title) {
    if (title) strncpy(kernel_title, title, sizeof(kernel_title) - 1);
    kernel_title[sizeof(kernel_title) - 1] = '\0';
}

void kernel_set_oled_header(const char* text) {
    if (text) strncpy(g_oled_header, text, sizeof(g_oled_header) - 1);
    g_oled_header[sizeof(g_oled_header) - 1] = '\0';
}
void kernel_set_oled_line1(const char* text) {
    if (text) strncpy(g_oled_line1, text, sizeof(g_oled_line1) - 1);
    g_oled_line1[sizeof(g_oled_line1) - 1] = '\0';
}
void kernel_set_oled_line2(const char* text) {
    if (text) strncpy(g_oled_line2, text, sizeof(g_oled_line2) - 1);
    g_oled_line2[sizeof(g_oled_line2) - 1] = '\0';
}
void kernel_set_oled_line3(const char* text) {
    if (text) strncpy(g_oled_line3, text, sizeof(g_oled_line3) - 1);
    g_oled_line3[sizeof(g_oled_line3) - 1] = '\0';
}
void kernel_set_oled_line4(const char* text) {
    if (text) strncpy(g_oled_line4, text, sizeof(g_oled_line4) - 1);
    g_oled_line4[sizeof(g_oled_line4) - 1] = '\0';
}

void kernel_update_display(void) {
    // Limit I2C display updates to 10Hz to eliminate scanline tearing while maintaining responsiveness
    static uint32_t last_display_time = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_display_time < 100) return; 
    last_display_time = now;

    // Check if Simulink is providing display data. If so, use it.
    if (g_oled_header[0] != '\0' || g_oled_line1[0] != '\0' || 
        g_oled_line2[0] != '\0' || g_oled_line3[0] != '\0' || 
        g_oled_line4[0] != '\0') {
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts(g_oled_header, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 16);
        SSD1306_Puts(g_oled_line1, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 28);
        SSD1306_Puts(g_oled_line2, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        SSD1306_Puts(g_oled_line3, &Font_7x10, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 52);
        SSD1306_Puts(g_oled_line4, &Font_7x10, SSD1306_COLOR_WHITE);
    } else {
        char buf[32];
        
        // 1. The Yellow Zone (Top 16 Pixels)
        SSD1306_GotoXY(0, 0);
        SSD1306_Puts(kernel_title, &Font_7x10, SSD1306_COLOR_WHITE);

        // 2. The Blue Zone (Starts at y=16)
        // We pad with spaces ("%-4d", "   ") to overwrite old characters without needing to Fill(BLACK)
        SSD1306_GotoXY(0, 16);
        snprintf(buf, sizeof(buf), "CMD: %-4d  %-4d   ", current_state.left_pwm, current_state.right_pwm);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_GotoXY(0, 28);
        uint32_t val_first = (current_state.tof_l < 8190) ? current_state.tof_l : current_state.tof_al;
        uint32_t val_mid   = current_state.tof_c;
        uint32_t val_third = (current_state.tof_r < 8190) ? current_state.tof_r : current_state.tof_ar;
        snprintf(buf, sizeof(buf), "TOF:%4lu %4lu %4lu", (unsigned long)val_first, (unsigned long)val_mid, (unsigned long)val_third);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_GotoXY(0, 40);
        int vbatt_int = (int)current_state.v_batt;
        int vbatt_frac = (int)(current_state.v_batt * 100.0f) % 100;
        float bat_val = current_state.v_batt;
        int bat_pct = (int)((bat_val - 3.2f) / (4.2f - 3.2f) * 100.0f);
        if (bat_pct > 100) bat_pct = 100;
        if (bat_pct < 0) bat_pct = 0;
        snprintf(buf, sizeof(buf), "BAT:%d.%02dV %d%% %-3dmA", vbatt_int, vbatt_frac < 0 ? -vbatt_frac : vbatt_frac, bat_pct, Current);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        SSD1306_GotoXY(0, 52);
        if (watchdog_timer_ms > 1000) {
            SSD1306_Puts("WDG: CUTOFF       ", &Font_7x10, SSD1306_COLOR_WHITE);
        } else {
            snprintf(buf, sizeof(buf), "WDG: %-4lu ms      ", watchdog_timer_ms);
            SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
        }
    }
    
    // Auto-recover I2C2 bus if it gets stuck in BUSY or ERROR state (due to motor noise)
    extern I2C_HandleTypeDef hi2c2;
    static uint32_t i2c2_busy_ticks = 0;
    if (hi2c2.State == HAL_I2C_STATE_BUSY) {
        i2c2_busy_ticks++;
    } else {
        i2c2_busy_ticks = 0;
    }

    if (hi2c2.ErrorCode != HAL_I2C_ERROR_NONE || i2c2_busy_ticks > 20) {
        i2c2_busy_ticks = 0;
        extern void restartI2C(I2C_HandleTypeDef *hi2c);
        restartI2C(&hi2c2);
        
        extern uint8_t SSD1306_Init(void);
        SSD1306_Init();
    }

    SSD1306_UpdateScreen();
}

void kernel_set_polarity(int16_t left, int16_t right) {
    polarity_l = left;
    polarity_r = right;
}

void kernel_set_encoder_polarity(int16_t left, int16_t right) {
    enc_polarity_l = left;
    enc_polarity_r = right;
}