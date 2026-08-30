#include "simulink_wrapper.h"

// --------------------------------------------------------------------------
// Simulink Hardware Abstraction Layer
// Targeted by Simulink Coder output blocks during "Standalone" deployment
// --------------------------------------------------------------------------

#ifdef __arm__
// --- PHYSICAL HARDWARE MODE (Compiling for STM32) ---
#include "micromouse_kernel.h"
#include "LEDs.h"

extern float IMU_Gyro[3];
extern float IMU_Accel[3];
extern float IMU_Temp;
extern int16_t Vbattery;
extern int16_t Current;

void simulink_ext_set_motors(int16_t left, int16_t right) {
    kernel_set_pwm(left, right);
}

void simulink_ext_set_leds(uint8_t led0, uint8_t led1, uint8_t led2) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // PB3 (gating CTRL_LEDS)
    
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, led0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// To actually display these, you will eventually want to store them in a global char array
// and update kernel_update_display() in micromouse_kernel.c to print them!
void simulink_ext_set_oled_header(const char *text) { kernel_set_oled_header(text); }
void simulink_ext_set_oled_line1(const char *text) { kernel_set_oled_line1(text); }
void simulink_ext_set_oled_line2(const char *text) { kernel_set_oled_line2(text); }
void simulink_ext_set_oled_line3(const char *text) { kernel_set_oled_line3(text); }
void simulink_ext_set_oled_line4(const char *text) { kernel_set_oled_line4(text); }

// Generic key-value telemetry for application logic.
// These can be configured to forward data to the JSON telemetry stream,
// print directly over UART for debugging, or safely absorb the data.
void simulink_ext_log_str(const char *key, const char *value) {
    (void)key;
    (void)value;
}
void simulink_ext_log_num(const char *key, double value) {
    (void)key;
    (void)value;
}

void simulink_ext_get_tof(uint16_t *left, uint16_t *center, uint16_t *right) {
    const KernelState_t* state = kernel_get_state();
    *left   = state->tof_l;
    *center = state->tof_c;
    *right  = state->tof_r;
}
void simulink_ext_get_encoders(int32_t *left, int32_t *right) {
    const KernelState_t* state = kernel_get_state();
    *left  = state->lenc;
    *right = state->renc;
}
float simulink_ext_get_gyro(void) {
    return kernel_get_state()->gyro;
}
void simulink_ext_get_gyro_xyz(float *x, float *y, float *z) {
    *x = IMU_Gyro[0];
    *y = IMU_Gyro[1];
    *z = IMU_Gyro[2];
}
void simulink_ext_get_accel_xyz(float *x, float *y, float *z) {
    *x = IMU_Accel[0];
    *y = IMU_Accel[1];
    *z = IMU_Accel[2];
}
float simulink_ext_get_imu_temp(void) {
    return IMU_Temp;
}
void simulink_ext_get_switches(uint8_t *sw1, uint8_t *sw2) {
    const KernelState_t* state = kernel_get_state();
    *sw1 = state->btn1;
    *sw2 = state->btn2;
}
float simulink_ext_get_vbatt(void) {
    return kernel_get_state()->v_batt;
}

void simulink_ext_get_line_sensors(uint16_t *fl, uint16_t *fr, uint16_t *sl, uint16_t *sr) {
    const KernelState_t* state = kernel_get_state();
    *fl = state->ir_fl;
    *fr = state->ir_fr;
    *sl = state->ir_sl;
    *sr = state->ir_sr;
}

void simulink_ext_get_pwr_meter(float *voltage, float *current, float *power, float *v_shunt, float *capacity) {
    *voltage = kernel_get_state()->v_batt;
    *current  = 0.0f; // Placeholder: Can be mapped to INA219 Current later
    *power    = 0.0f; // Placeholder: Can be mapped to INA219 Power later
    *v_shunt  = 0.0f; // Placeholder: Can be mapped to INA219 Shunt Voltage
    *capacity = 0.0f; // Placeholder: Estimated mAh or SoC
}

void simulink_ext_cleanup(void) {
    // No-op on physical hardware
}

#else
// --- STANDALONE PC TCP CLIENT MODE (macOS, Linux, and Windows) ---
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#ifdef _MSC_VER
#pragma comment(lib, "ws2_32.lib")
#endif
#define close closesocket
#define socket_errno WSAGetLastError()
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#ifndef MODEL_NAME
#include <dlfcn.h>
#endif
#define socket_errno errno
#endif

static void diag_log(const char *fmt, ...) {
    FILE *f = fopen("/Users/nicolls/proj/eee3097s/2026/UCT-Micromouse/c_client_diag.log", "a");
    if (!f) return;
    
    // Get timestamp
#ifdef _WIN32
    SYSTEMTIME st;
    GetLocalTime(&st);
    fprintf(f, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_info = localtime(&tv.tv_sec);
    char time_buf[30];
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm_info);
    fprintf(f, "[%s.%03d] ", time_buf, (int)(tv.tv_usec / 1000));
#endif

    va_list args;
    va_start(args, fmt);
    vfprintf(f, fmt, args);
    va_end(args);
    
    fclose(f);
}

static void stop_simulink_simulation(void) {
#ifndef MODEL_NAME
    diag_log("[stop_simulink_simulation] Attempting to stop Simulink simulation...\n");
#ifdef _WIN32
    // On Windows, resolve from the loaded MEX or MATLAB module
    HMODULE hMex = GetModuleHandle("libmex.dll");
    if (!hMex) hMex = GetModuleHandle(NULL);
    if (hMex) {
        typedef int (__cdecl *mexEvalString_t)(const char *);
        mexEvalString_t pMexEvalString = (mexEvalString_t)GetProcAddress(hMex, "mexEvalString");
        if (pMexEvalString) {
            diag_log("[stop_simulink_simulation] Calling mexEvalString (Windows)\n");
            pMexEvalString("set_param(bdroot, 'SimulationCommand', 'stop');");
            return;
        }
    }
#else
    // On macOS and Linux, resolve via dlsym
    void *handle = RTLD_DEFAULT;
    typedef int (*mexEvalString_t)(const char *);
    mexEvalString_t pMexEvalString = (mexEvalString_t)dlsym(handle, "mexEvalString");
    if (pMexEvalString) {
        diag_log("[stop_simulink_simulation] Calling mexEvalString (POSIX)\n");
        pMexEvalString("set_param(bdroot, 'SimulationCommand', 'stop');");
        return;
    } else {
        diag_log("[stop_simulink_simulation] Failed to find mexEvalString symbol.\n");
    }
#endif
#else
    (void)0; // Standalone client mode, no-op
#endif
}

// We expose sim_fd globally so the main loop can monitor connection status
int sim_fd = -1;
static int packets_received = 0;
static int connection_failed = 0;

static uint16_t sim_tof_l = 0, sim_tof_c = 0, sim_tof_r = 0;
static uint16_t sim_ir_fl = 0, sim_ir_fr = 0, sim_ir_sl = 0, sim_ir_sr = 0;
static int32_t sim_lenc = 0, sim_renc = 0;
static float sim_gyro = 0.0f;
static float sim_vbatt = 6.0f;

static void sim_init_socket(void) {
    if (sim_fd != -1) return;
    if (connection_failed) return;

    diag_log("[sim_init_socket] Start connection attempt. packets_received=%d\n", packets_received);
#ifdef MATLAB_MEX_FILE
    diag_log("[sim_init_socket] Compiled WITH MATLAB_MEX_FILE defined.\n");
#else
    diag_log("[sim_init_socket] Compiled WITHOUT MATLAB_MEX_FILE defined.\n");
#endif
#ifdef SL_INTERNAL
    diag_log("[sim_init_socket] Compiled WITH SL_INTERNAL defined.\n");
#endif

    // If we already received packets but lost connection, do NOT try to reconnect.
    if (packets_received > 0) {
        diag_log("[sim_init_socket] Connection was previously active but lost. Skipping reconnect.\n");
        return;
    }

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
        diag_log("[sim_init_socket] Winsock initialization failed.\n");
        return;
    }
#endif

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8000);

#ifdef _WIN32
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
        diag_log("[sim_init_socket] Invalid loopback address.\n");
        return;
    }
#else
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        diag_log("[sim_init_socket] Invalid loopback address.\n");
        return;
    }
#endif

    // Retry up to 30 times (3.0 seconds max) for initial connection
    int retries = 30;
    while (1) {
        sim_fd = (int)socket(AF_INET, SOCK_STREAM, 0);
        if (sim_fd < 0) {
            diag_log("[sim_init_socket] Socket creation error, errno=%d\n", socket_errno);
            return;
        }
#ifdef __APPLE__
        int nosigpipe = 1;
        setsockopt(sim_fd, SOL_SOCKET, SO_NOSIGPIPE, (void *)&nosigpipe, sizeof(nosigpipe));
#endif

        diag_log("[sim_init_socket] Connecting, retry=%d...\n", 30 - retries + 1);
        if (connect(sim_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) >= 0) {
            // Connected successfully!
            diag_log("[sim_init_socket] Connected successfully! sim_fd=%d\n", sim_fd);
            break;
        }

        close(sim_fd);
        sim_fd = -1;

        retries--;
        if (retries <= 0) {
            diag_log("[sim_init_socket] Connection failed after 30 retries.\n");
            connection_failed = 1;
            return;
        }

#ifdef _WIN32
        Sleep(100);
#else
        usleep(100000); // 100ms
#endif
    }

    // Set non-blocking mode
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(sim_fd, FIONBIO, &mode);
#else
    int flags = fcntl(sim_fd, F_GETFL, 0);
    fcntl(sim_fd, F_SETFL, flags | O_NONBLOCK);
#endif

    diag_log("[sim_init_socket] Set socket to non-blocking mode.\n");
}

static void sim_poll_socket(void) {
    if (sim_fd == -1) {
        if (packets_received > 0) return;
        sim_init_socket();
    }
    if (sim_fd == -1) return;

    char temp[512];
    int total_bytes = 0;
    int eagain_count = 0;
    
    // Lock-step read loop: poll socket character by character until we hit newline
    while (total_bytes < sizeof(temp) - 1) {
        int n = recv(sim_fd, temp + total_bytes, 1, 0);
        if (n > 0) {
            if (temp[total_bytes] == '\n' || temp[total_bytes] == '\r') {
                temp[total_bytes] = '\0';
                if (total_bytes > 0) {
                    char *p; int val; double dval;
                    if ((p = strstr(temp, "\"tof_l\":")))  { sscanf(p, "\"tof_l\":%d", &val); sim_tof_l = (uint16_t)val; }
                    if ((p = strstr(temp, "\"tof_c\":")))  { sscanf(p, "\"tof_c\":%d", &val); sim_tof_c = (uint16_t)val; }
                    if ((p = strstr(temp, "\"tof_r\":")))  { sscanf(p, "\"tof_r\":%d", &val); sim_tof_r = (uint16_t)val; }
                    if ((p = strstr(temp, "\"ir_fl\":")))  { sscanf(p, "\"ir_fl\":%d", &val); sim_ir_fl = (uint16_t)val; }
                    if ((p = strstr(temp, "\"ir_fr\":")))  { sscanf(p, "\"ir_fr\":%d", &val); sim_ir_fr = (uint16_t)val; }
                    if ((p = strstr(temp, "\"ir_sl\":")))  { sscanf(p, "\"ir_sl\":%d", &val); sim_ir_sl = (uint16_t)val; }
                    if ((p = strstr(temp, "\"ir_sr\":")))  { sscanf(p, "\"ir_sr\":%d", &val); sim_ir_sr = (uint16_t)val; }
                    if ((p = strstr(temp, "\"lenc\":")))   { sscanf(p, "\"lenc\":%d", &val); sim_lenc = (int32_t)val; }
                    else if ((p = strstr(temp, "\"+lenc\":"))) { sscanf(p, "\"+lenc\":%d", &val); sim_lenc += (int32_t)val; }
                    
                    if ((p = strstr(temp, "\"renc\":")))   { sscanf(p, "\"renc\":%d", &val); sim_renc = (int32_t)val; }
                    else if ((p = strstr(temp, "\"+renc\":"))) { sscanf(p, "\"+renc\":%d", &val); sim_renc += (int32_t)val; }
                    if ((p = strstr(temp, "\"gyro\":")))   { sscanf(p, "\"gyro\":%lf", &dval); sim_gyro = (float)dval; }
                    if ((p = strstr(temp, "\"v_batt\":"))) { sscanf(p, "\"v_batt\":%lf", &dval); sim_vbatt = (float)dval; }
                    
                    packets_received++;
                    if (packets_received % 100 == 1) {
                        diag_log("[sim_poll_socket] Telemetry parsed: pkts=%d, tof_c=%d, gyro=%.2f\n", 
                                 packets_received, sim_tof_c, sim_gyro);
                    }
                }
                break;
            }
            total_bytes++;
        } else if (n < 0) {
#ifdef _WIN32
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK) {
                eagain_count++;
                if (eagain_count > 500) { // Timeout after 500ms
                    diag_log("[sim_poll_socket] WSAEWOULDBLOCK timeout (>500ms). Closing socket.\n");
                    close(sim_fd);
                    sim_fd = -1;
                    break;
                }
                Sleep(1);
                continue;
            }
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                eagain_count++;
                if (eagain_count > 500) { // Timeout after 500ms
                    diag_log("[sim_poll_socket] EAGAIN timeout (>500ms). Closing socket.\n");
                    close(sim_fd);
                    sim_fd = -1;
                    break;
                }
                usleep(1000);
                continue;
            }
#endif
            diag_log("[sim_poll_socket] Connection lost or read error, errno=%d. Closing socket.\n", socket_errno);
            close(sim_fd);
            sim_fd = -1;
            stop_simulink_simulation();
            break;
        } else {
            // Server closed connection
            diag_log("[sim_poll_socket] Simulator server closed connection (EOF). Closing socket.\n");
            close(sim_fd);
            sim_fd = -1;
            stop_simulink_simulation();
            break;
        }
    }
}

void simulink_ext_set_motors(int16_t left, int16_t right) {
    if (sim_fd == -1) {
        if (packets_received > 0 || connection_failed) return;
        sim_init_socket();
    }
    if (sim_fd != -1) {
        char tx[64];
        int len = snprintf(tx, sizeof(tx), "{\"a\":[%d,%d]}\r\n", left, right);
#ifdef __linux__
        if (send(sim_fd, tx, len, MSG_NOSIGNAL) < 0) {
#else
        if (send(sim_fd, tx, len, 0) < 0) {
#endif
            diag_log("[set_motors] Send failed, errno=%d. Closing socket.\n", socket_errno);
            close(sim_fd);
            sim_fd = -1;
            stop_simulink_simulation();
            return;
        }
        sim_poll_socket(); // Block and wait for telemetry update (lock-step)
    }
}

void simulink_ext_get_tof(uint16_t *left, uint16_t *center, uint16_t *right) { 
    *left = sim_tof_l; *center = sim_tof_c; *right = sim_tof_r; 
}

void simulink_ext_get_encoders(int32_t *left, int32_t *right) { 
    *left = sim_lenc; *right = sim_renc; 
}

float simulink_ext_get_gyro(void) { 
    return sim_gyro; 
}

void simulink_ext_get_gyro_xyz(float *x, float *y, float *z) { 
    *x = 0.0f; *y = 0.0f; *z = sim_gyro; 
}

void simulink_ext_get_accel_xyz(float *x, float *y, float *z) { 
    *x = 0.0f; *y = 0.0f; *z = 0.0f; 
}

float simulink_ext_get_imu_temp(void) { return 25.0f; }
void simulink_ext_get_switches(uint8_t *sw1, uint8_t *sw2) { *sw1 = 0; *sw2 = 0; }
float simulink_ext_get_vbatt(void) { return sim_vbatt; }
void simulink_ext_get_line_sensors(uint16_t *fl, uint16_t *fr, uint16_t *sl, uint16_t *sr) { 
    *fl = sim_ir_fl; *fr = sim_ir_fr; *sl = sim_ir_sl; *sr = sim_ir_sr; 
}
void simulink_ext_get_pwr_meter(float *voltage, float *current, float *power, float *v_shunt, float *capacity) { 
    *voltage = sim_vbatt; *current = 0.0f; *power = 0.0f; *v_shunt = 0.0f; *capacity = 0.0f; 
}

void simulink_ext_set_leds(uint8_t led0, uint8_t led1, uint8_t led2) { (void)led0; (void)led1; (void)led2; }
void simulink_ext_set_oled_header(const char *text) { (void)text; }
void simulink_ext_set_oled_line1(const char *text) { (void)text; }
void simulink_ext_set_oled_line2(const char *text) { (void)text; }
void simulink_ext_set_oled_line3(const char *text) { (void)text; }
void simulink_ext_set_oled_line4(const char *text) { (void)text; }
void simulink_ext_log_str(const char *key, const char *value) { (void)key; (void)value; }
void simulink_ext_log_num(const char *key, double value) { (void)key; (void)value; }

void simulink_ext_cleanup(void) {
    diag_log("[simulink_ext_cleanup] Cleaning up socket. sim_fd=%d\n", sim_fd);
    if (sim_fd != -1) {
        close(sim_fd);
        sim_fd = -1;
    }
#ifdef _WIN32
    WSACleanup();
#endif
}
#endif