#ifndef SIMULINK_WRAPPER_H
#define SIMULINK_WRAPPER_H

#include <stdint.h>

// --------------------------------------------------------------------------
// Simulink Hardware Abstraction Layer
// Targeted by Simulink Coder output blocks during "Standalone" deployment
// --------------------------------------------------------------------------

void simulink_ext_set_motors(int16_t left, int16_t right);
void simulink_ext_get_tof(uint16_t *left, uint16_t *center, uint16_t *right);
void simulink_ext_get_tof_raw(uint16_t *left, uint16_t *center, uint16_t *right);
void simulink_ext_get_tof_signals(uint16_t *left, uint16_t *center, uint16_t *right);
void simulink_ext_get_encoders(int32_t *left, int32_t *right);
float simulink_ext_get_gyro(void);
void simulink_ext_get_gyro_xyz(float *x, float *y, float *z);
void simulink_ext_get_accel_xyz(float *x, float *y, float *z);
float simulink_ext_get_imu_temp(void);
void simulink_ext_get_switches(uint8_t *sw1, uint8_t *sw2);
float simulink_ext_get_vbatt(void);
void simulink_ext_get_line_sensors(uint16_t *fl, uint16_t *fr, uint16_t *sl, uint16_t *sr);
void simulink_ext_get_pwr_meter(float *voltage, float *current, float *power, float *v_shunt, float *capacity);
void simulink_ext_set_leds(uint8_t led0, uint8_t led1, uint8_t led2);
void simulink_ext_set_oled_header(const char *text);
void simulink_ext_set_oled_line1(const char *text);
void simulink_ext_set_oled_line2(const char *text);
void simulink_ext_set_oled_line3(const char *text);
void simulink_ext_set_oled_line4(const char *text);

// Extensible Application-Level Logging (No firmware recompilation needed)
void simulink_ext_log_str(const char *key, const char *value);
void simulink_ext_log_num(const char *key, double value);

void simulink_ext_cleanup(void);

#endif // SIMULINK_WRAPPER_H