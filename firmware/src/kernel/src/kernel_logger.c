#include "micromouse_kernel.h"
#include "ZD25WQ80C.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

// External Flash Partition Definitions (New boards: 1 MB external flash)
#define EXT_LOGGER_PARTITION_START  0x00000U
#define EXT_LOGGER_PARTITION_MAX    0xFFFFFUL   // 1024 KB limit
#define EXT_LOG_SECTOR_SIZE         4096U

// Internal Flash Partition Definitions (Legacy boards: 32 KB internal partition, Sectors 240-255)
#define INT_LOGGER_PARTITION_START  0x08078000U
#define INT_LOGGER_PARTITION_MAX    0x0807FFFFU // 32 KB limit
#define INT_FLASH_PAGE_SIZE         2048U

#define LOG_PAGE_SIZE               256U

#include "serial_interface.h"
extern ZD25WQ80C_t flash;
void kernel_logger_init(void);
static void backup_write_log_addr(uint32_t addr);
static void backup_invalidate_log(void);
static uint32_t backup_read_log_addr(void);

static uint8_t log_page_buf[LOG_PAGE_SIZE] __attribute__((aligned(8)));
static uint16_t log_page_idx = 0;
static uint32_t log_write_addr = 0;
static uint32_t log_partition_start = 0;
static uint32_t log_partition_max = 0;
static bool has_ext_flash = false;
static bool logging_active = false;
static bool header_written = false;
static uint32_t log_start_time = 0;
static uint32_t last_log_time = 0;

// Track last-written shadow state for sparse log comparisons
static int16_t last_left_pwm = 0;
static int16_t last_right_pwm = 0;
static uint16_t last_tof_l = 0;
static uint16_t last_tof_al = 0;
static uint16_t last_tof_c = 0;
static uint16_t last_tof_ar = 0;
static uint16_t last_tof_r = 0;
static int32_t last_lenc = 0;
static int32_t last_renc = 0;
static float last_gyro = 0.0f;
static float last_v_batt = 0.0f;
static float last_ax = 0.0f;
static float last_ay = 0.0f;
static float last_az = 0.0f;

// Unique Device ID (UID) starting address on STM32L4
#define STM32L4_UID_ADDR 0x1FFF7590U

static uint32_t compute_code_hash(void) {
    uint32_t hash = 2166136261U;
    
    // 1. Hash PikaScript bytecode region (Page 240 / 0x08078000, 8KB)
    const uint8_t *pika_ptr = (const uint8_t*)0x08078000;
    for (int i = 0; i < 8192; i++) {
        hash ^= pika_ptr[i];
        hash *= 16777619U;
    }
    
    // 2. Hash first block of internal FAT filesystem (0x08060000)
    const uint8_t *fs_ptr = (const uint8_t*)0x08060000;
    for (int i = 0; i < 512; i++) {
        hash ^= fs_ptr[i];
        hash *= 16777619U;
    }
    
    return hash;
}

// Ensures current sector/page is erased before writing
static void ensure_sector_erased(uint32_t addr) {
    if (has_ext_flash) {
        if (addr % EXT_LOG_SECTOR_SIZE == 0) {
            ZD25WQ80C_SectorErase(addr);
        }
    } else {
        if (addr % INT_FLASH_PAGE_SIZE == 0) {
            FLASH_EraseInitTypeDef EraseInitStruct;
            uint32_t PageError;
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.Banks = FLASH_BANK_2;
            EraseInitStruct.Page = (addr - 0x08040000U) / INT_FLASH_PAGE_SIZE;
            EraseInitStruct.NbPages = 1;
            
            HAL_FLASH_Unlock();
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
            HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
            HAL_FLASH_Lock();
        }
    }
}

// Flush page buffer to flash (external SPI or internal flash)
static void flush_log_page(void) {
    if (log_page_idx > 0) {
        if (log_write_addr + LOG_PAGE_SIZE <= log_partition_max) {
            ensure_sector_erased(log_write_addr);
            // Pad with space characters to fill the 256-byte page boundary
            while (log_page_idx < LOG_PAGE_SIZE) {
                log_page_buf[log_page_idx++] = ' ';
            }
            
            if (has_ext_flash) {
                ZD25WQ80C_PageProgram(log_write_addr, log_page_buf, LOG_PAGE_SIZE);
            } else {
                HAL_FLASH_Unlock();
                __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
                for (uint32_t i = 0; i < LOG_PAGE_SIZE; i += 8) {
                    uint64_t data64 = *(uint64_t*)(log_page_buf + i);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, log_write_addr + i, data64);
                }
                HAL_FLASH_Lock();
            }
            
            log_write_addr += LOG_PAGE_SIZE;
            // Persist pointer to Backup Domain register
            backup_write_log_addr(log_write_addr);
        }
        log_page_idx = 0;
    }
}

// Append formatted JSON string directly to page buffer
static void append_to_log(const char *str) {
    size_t len = strlen(str);
    for (size_t i = 0; i < len; i++) {
        log_page_buf[log_page_idx++] = str[i];
        if (log_page_idx >= LOG_PAGE_SIZE) {
            flush_log_page();
        }
    }
}

void kernel_logger_write_custom(const char *json_str) {
    if (!logging_active) {
        logging_active = true;
        kernel_logger_init();
    }
    append_to_log(json_str);
    append_to_log("\n");
}

static void backup_write_log_addr(uint32_t addr) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    RTC->BKP4R = addr;
    RTC->BKP5R = 0x12345678U;
}

static void backup_invalidate_log(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    RTC->BKP5R = 0;
}

static uint32_t backup_read_log_addr(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    if (RTC->BKP5R == 0x12345678U) {
        uint32_t addr = RTC->BKP4R;
        if (addr >= log_partition_start && addr <= log_partition_max) {
            return addr;
        }
    }
    
    // Backup register was wiped. Recover address dynamically by scanning page-by-page.
    uint32_t scan_addr = log_partition_start;
    uint8_t first_byte = 0;
    
    // Check if the partition starts with valid JSON '{'
    if (has_ext_flash) {
        if (ZD25WQ80C_Read(scan_addr, &first_byte, 1) != HAL_OK || first_byte != '{') {
            return log_partition_start;
        }
    } else {
        first_byte = *(const uint8_t*)scan_addr;
        if (first_byte != '{') {
            return log_partition_start;
        }
    }
    
    while (scan_addr < log_partition_max) {
        if (has_ext_flash) {
            if (ZD25WQ80C_Read(scan_addr, &first_byte, 1) != HAL_OK || first_byte == 0xFF) {
                break;
            }
        } else {
            first_byte = *(const uint8_t*)scan_addr;
            if (first_byte == 0xFF) {
                break;
            }
        }
        scan_addr += LOG_PAGE_SIZE;
    }
    return scan_addr;
}

void kernel_logger_init(void) {
    // Probe external SPI flash
    has_ext_flash = initZD25WQ80C();
    if (has_ext_flash) {
        log_partition_start = EXT_LOGGER_PARTITION_START;
        log_partition_max = EXT_LOGGER_PARTITION_MAX;
    } else {
        log_partition_start = INT_LOGGER_PARTITION_START;
        log_partition_max = INT_LOGGER_PARTITION_MAX;
    }
    
    // Read recovered pointer from STM32 Backup Domain registers
    log_write_addr = backup_read_log_addr();
    log_page_idx = 0;
    header_written = (log_write_addr > log_partition_start);
    logging_active = false;
}

void kernel_logger_tick(void) {
    // 1. Check for automatic logging trigger: starts when motors first actuate
    const KernelState_t* s = kernel_get_state();
    if (!logging_active) {
        if (s->left_pwm != 0 || s->right_pwm != 0) {
            logging_active = true;
            // Force reset log partition pointers to start fresh on a new run
            if (log_partition_start == 0 && log_partition_max == 0) {
                kernel_logger_init();
            }
            log_write_addr = log_partition_start;
            log_page_idx = 0;
            header_written = false;
            backup_write_log_addr(log_write_addr);
        } else {
            return; // Stay idle
        }
    }

    // Check flash space limit
    if (log_write_addr >= log_partition_max) {
        return;
    }

    // 2. Write verification headers on first tick
    if (!header_written) {
        uint32_t *uid = (uint32_t*)STM32L4_UID_ADDR;
        uint32_t code_hash = compute_code_hash();
        char header_buf[160];
        const char *board_str = has_ext_flash ? "2026" : "2025";
        extern int getIMUType(void);
        int imu_t = getIMUType();
        const char *imu_str = (imu_t == 2) ? "LSM6DS3" : 
                              (imu_t == 1) ? "ICM42605" : "UNKNOWN";
        snprintf(header_buf, sizeof(header_buf), 
                 "{\"log_header\":1,\"board\":\"%s\",\"imu\":\"%s\",\"uid\":\"%08X%08X%08X\",\"hash\":%lu,\"ext\":%d}\n", 
                 board_str, imu_str,
                 (unsigned int)uid[0], (unsigned int)uid[1], (unsigned int)uid[2], 
                 (unsigned long)code_hash, has_ext_flash ? 1 : 0);
        append_to_log(header_buf);
        header_written = true;
        log_start_time = HAL_GetTick();
        last_log_time = log_start_time;

        // Initialize shadow states
        last_left_pwm = s->left_pwm; last_right_pwm = s->right_pwm;
        last_tof_l = s->tof_l; last_tof_al = s->tof_al; last_tof_c = s->tof_c;
        last_tof_ar = s->tof_ar; last_tof_r = s->tof_r;
        last_lenc = s->lenc; last_renc = s->renc;
        last_gyro = s->gyro; last_v_batt = s->v_batt;
        
        extern float IMU_Accel[3];
        last_ax = IMU_Accel[0]; last_ay = IMU_Accel[1]; last_az = IMU_Accel[2];
    }

    // 3. Perform sparse compression check
    uint32_t current_time = HAL_GetTick();
    uint32_t dt = current_time - last_log_time;
    last_log_time = current_time;

    char record_buf[256];
    int written = snprintf(record_buf, sizeof(record_buf), "{\"+t\":%lu", (unsigned long)dt);

    // Check and log variables
    if (s->left_pwm != last_left_pwm) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"l\":%d", s->left_pwm);
        last_left_pwm = s->left_pwm;
    }
    if (s->right_pwm != last_right_pwm) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"r\":%d", s->right_pwm);
        last_right_pwm = s->right_pwm;
    }
    if (s->tof_l != last_tof_l) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"tl\":%u", s->tof_l);
        last_tof_l = s->tof_l;
    }
    if (s->tof_al != last_tof_al) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"tal\":%u", s->tof_al);
        last_tof_al = s->tof_al;
    }
    if (s->tof_c != last_tof_c) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"tc\":%u", s->tof_c);
        last_tof_c = s->tof_c;
    }
    if (s->tof_ar != last_tof_ar) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"tar\":%u", s->tof_ar);
        last_tof_ar = s->tof_ar;
    }
    if (s->tof_r != last_tof_r) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"tr\":%u", s->tof_r);
        last_tof_r = s->tof_r;
    }
    if (s->lenc != last_lenc) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"le\":%ld", (long)s->lenc);
        last_lenc = s->lenc;
    }
    if (s->renc != last_renc) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"re\":%ld", (long)s->renc);
        last_renc = s->renc;
    }
    if (abs((int)(s->gyro * 100.0f) - (int)(last_gyro * 100.0f)) > 1) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"g\":%.2f", (double)s->gyro);
        last_gyro = s->gyro;
    }
    if (abs((int)(s->v_batt * 100.0f) - (int)(last_v_batt * 100.0f)) > 1) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"v\":%.2f", (double)s->v_batt);
        last_v_batt = s->v_batt;
    }

    extern float IMU_Accel[3];
    if (abs((int)(IMU_Accel[0] * 10.0f) - (int)(last_ax * 10.0f)) > 1) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"ax\":%.2f", (double)IMU_Accel[0]);
        last_ax = IMU_Accel[0];
    }
    if (abs((int)(IMU_Accel[1] * 10.0f) - (int)(last_ay * 10.0f)) > 1) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"ay\":%.2f", (double)IMU_Accel[1]);
        last_ay = IMU_Accel[1];
    }
    if (abs((int)(IMU_Accel[2] * 10.0f) - (int)(last_az * 10.0f)) > 1) {
        written += snprintf(record_buf + written, sizeof(record_buf) - written, ",\"az\":%.2f", (double)IMU_Accel[2]);
        last_az = IMU_Accel[2];
    }

    written += snprintf(record_buf + written, sizeof(record_buf) - written, "}\n");
    append_to_log(record_buf);
}

void kernel_logger_dump_custom(void (*print_fn)(const uint8_t *buf, uint32_t len)) {
    kernel_set_pwm(0, 0);
    
    const char *start_msg = "\r\n--- START LOG DUMP ---\r\n";
    print_fn((const uint8_t *)start_msg, (uint32_t)strlen(start_msg));
    
    flush_log_page();
    
    static uint8_t dump_buf[LOG_PAGE_SIZE];
    uint32_t current_addr = log_partition_start;
    
    while (current_addr < log_write_addr && (current_addr + LOG_PAGE_SIZE) <= log_partition_max) {
        if (has_ext_flash) {
            if (ZD25WQ80C_Read(current_addr, dump_buf, LOG_PAGE_SIZE) == HAL_OK) {
                if (dump_buf[0] == 0xFF) {
                    break;
                }
                print_fn(dump_buf, LOG_PAGE_SIZE);
            }
        } else {
            memcpy(dump_buf, (const void*)current_addr, LOG_PAGE_SIZE);
            if (dump_buf[0] == 0xFF) {
                break;
            }
            print_fn(dump_buf, LOG_PAGE_SIZE);
        }
        current_addr += LOG_PAGE_SIZE;
    }
    
    const char *end_msg = "\r\n--- END LOG DUMP ---\r\n";
    print_fn((const uint8_t *)end_msg, (uint32_t)strlen(end_msg));
    
    backup_invalidate_log();
}

static void default_uart_print(const uint8_t *buf, uint32_t len) {
    UART_HandleTypeDef* huart = serial_interface_get_huart();
    if (huart != NULL) {
        HAL_UART_Transmit(huart, (uint8_t *)buf, len, 1000);
    }
}

void kernel_logger_dump(void) {
    __disable_irq();
    kernel_logger_dump_custom(default_uart_print);
    __enable_irq();
}
