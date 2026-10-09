#include "py/mphal.h"
#include "py/obj.h"
#include "py/stream.h"
#include "extmod/misc.h"
#include "usb.h"
#include "uart.h"
#include "main.h"
#include "serial_interface.h"
#include "dma.h"
#include "micromouse_kernel.h"
#include "SSD1306.h"

#if MICROPY_HW_TINYUSB_STACK
#include "shared/tinyusb/mp_usbd_cdc.h"
#endif

// Extern hardware handles and init functions from the template
extern UART_HandleTypeDef huart1;
extern void initMicroMouse(void);
extern void MX_DMA_Init(void);
extern void MX_GPIO_Init(void);
extern void MX_ADC1_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_TIM3_Init(void);
extern void MX_TIM4_Init(void);
extern void MX_TIM5_Init(void);
extern void MX_TIM7_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_NVIC_Init(void);
extern void MX_SPI2_Init(void);

// Extern background routines from the C-Kernel
extern void refreshADCs(void);
extern void refreshSWValues(void);
extern void refreshTOFValues(void);
extern void refreshIMUValues(void);
extern void refreshINA219Values(void);
extern void kernel_update_display(void);
extern void serial_interface_tick(void);
extern void kernel_watchdog_tick(void);

// Global flag to track if physical hardware has been initialized
volatile bool mouse_initialized = false;

// Dummy board startup hook called before clocks are configured
void board_startup(void) {
}

void uart_print(const char *str) {
    if (USART1 != NULL && (RCC->APB2ENR & RCC_APB2ENR_USART1EN)) {
        for (const char *p = str; *p; p++) {
            while (!(USART1->ISR & USART_ISR_TXE));
            USART1->TDR = (uint8_t)*p;
        }
    }
}

// Define the strong SystemClock_Config to override MicroPython's default weak one in system_stm32.c.
// This sets up the clock tree (80MHz SysClk, 48MHz USB FS, ADC, I2C) using the 8 MHz HSE crystal on PH0/PH1.
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        while (1);
    }

    HAL_PWR_EnableBkUpAccess();

    // 1. Configure System Clock using 8 MHz HSE crystal on PH0/PH1 -> 80 MHz SysClk
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Backup internal oscillator
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;  // 8 MHz / 1 = 8 MHz VCO input
    RCC_OscInitStruct.PLL.PLLN = 20; // 8 MHz * 20 = 160 MHz VCO
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // 160 MHz / 2 = 80 MHz SysClk
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        // Fallback to HSI if HSE crystal is not present
        RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = 1;
        RCC_OscInitStruct.PLL.PLLN = 10;
        RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
        HAL_RCC_OscConfig(&RCC_OscInitStruct);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        while (1);
    }

    // 2. Configure USB (48 MHz), ADC (48 MHz), and I2C clocks using PLLSAI1
    // Configure PLLSAI1: 8 MHz HSE / 1 * 12 = 96 MHz VCOSAI1
    // PLLSAI1Q: 96 MHz / 2 = 48 MHz for USB OTG FS
    // PLLSAI1R: 96 MHz / 2 = 48 MHz for ADC
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC
                                             | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    
    PeriphClkInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1N = 12;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2; // 48 MHz for USB OTG FS
    PeriphClkInitStruct.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2; // 48 MHz for ADC
    PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK | RCC_PLLSAI1_ADC1CLK;
    
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        // Fallback for PLLSAI1 on HSI
        PeriphClkInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
        PeriphClkInitStruct.PLLSAI1.PLLSAI1M = 1;
        PeriphClkInitStruct.PLLSAI1.PLLSAI1N = 12;
        PeriphClkInitStruct.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4; // 192 / 4 = 48 MHz
        PeriphClkInitStruct.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    }

    // Enable VDDUSB power supply for STM32L4 USB Full-Speed PHY
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddUSB();

    // Configure SysTick for 1ms time base (required for HAL_GetTick and MicroPython mp_hal_delay_ms)
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_PRIORITYGROUP_4, TICK_INT_PRIORITY, 0));
}

void Error_Handler(void) {
    uart_print("\n!!! Error_Handler Called !!!\n");
    while (1) {
        // Flash LED1 (pin_C13) to indicate crash
        mp_hal_pin_high(pin_C13);
        for (volatile int i = 0; i < 500000; i++);
        mp_hal_pin_low(pin_C13);
        for (volatile int i = 0; i < 500000; i++);
    }
}

// Early board initialization hook called after system clock is fully configured (80 MHz)
void board_early_init(void) {
    // 1. Core peripheral DMA & GPIO init
    MX_DMA_Init();
    MX_GPIO_Init();
    
    // 2. Initialize USART1 and configure baudrate first so we can output logs immediately
    MX_USART1_UART_Init();
    __HAL_UART_DISABLE(&huart1);
    USART1->BRR = 694; // Exact 115200 baud at 80.000 MHz (from 8 MHz HSE crystal)
    __HAL_UART_ENABLE(&huart1);

    // Set C-Kernel logger UART reference
    extern void serial_interface_set_huart(UART_HandleTypeDef *huart);
    serial_interface_set_huart(&huart1);

    extern void kernel_logger_init(void);
    kernel_logger_init();

    // UART output - active immediately!
    uart_print("\n--- Boot Log Start ---\n");
    extern int pyb_hard_fault_debug;
    pyb_hard_fault_debug = 1;

    // 3. Initialize NVIC
    MX_NVIC_Init();

    // Allow SWD debugging during low-power WFI / sleep modes to prevent ST-Link lockup
    if (DBGMCU != NULL) {
        DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;
    }

    // Explicitly configure VDDUSB for USB OTG Full Speed PHY
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddUSB();

    // Configure PB3 (CTRL_LEDS) as GPIO Output Push-Pull and write it HIGH to enable the LED master gate
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct_LedGate = {0};
    GPIO_InitStruct_LedGate.Pin = GPIO_PIN_3;
    GPIO_InitStruct_LedGate.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_LedGate.Pull = GPIO_NOPULL;
    GPIO_InitStruct_LedGate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_LedGate);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

    // Configure PC13 (LED0), PA4 (LED1), PA5 (LED2) as Outputs and set them HIGH to turn all three onboard LEDs ON at boot
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct_LEDs = {0};
    GPIO_InitStruct_LEDs.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_LEDs.Pull = GPIO_NOPULL;
    GPIO_InitStruct_LEDs.Speed = GPIO_SPEED_FREQ_LOW;

    // LED0 (PC13)
    GPIO_InitStruct_LEDs.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_LEDs);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    // LED1 (PA4 on 2026 boards) and LED2 (PA5 on 2026 boards)
    GPIO_InitStruct_LEDs.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_LEDs);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);

    // Release and configure PC14 and PC15 (LED1 and LED2 on legacy 2025 boards)
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    PWR->CR1 &= ~PWR_CR1_DBP;

    GPIO_InitStruct_LEDs.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_LEDs);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

    initMicroMouse();
    mouse_initialized = true;

    uart_print("Boot sequence completed successfully.\n");
}

// Background tick function hook called inside MicroPython VM execution and delay loops
void kernel_background_tick(void) {
    // Never execute blocking I2C sensor or display transactions from ISR context (e.g. USB interrupts)
    if (__get_IPSR() != 0) {
        return;
    }

    static bool in_tick = false;
    if (in_tick) {
        return;
    }
    in_tick = true;

    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick >= 10) { // 100 Hz
        last_tick = now;

        if (mouse_initialized) {
            refreshADCs();
            refreshSWValues();
            refreshTOFValues();
            refreshIMUValues();
            refreshINA219Values();
            
            // Snapshot physical state to the C-Kernel state structure
            extern void kernel_snapshot_state(void);
            kernel_snapshot_state();

            // Run C-Kernel telemetry logger at 25 Hz (every 40ms / 4 ticks)
            static uint32_t logger_tick_count = 0;
            if (++logger_tick_count >= 4) {
                logger_tick_count = 0;
                extern void kernel_logger_tick(void);
                kernel_logger_tick();
            }

            // Rate-limit OLED display updates to 10 Hz (every 100ms)
            static uint32_t last_display_update = 0;
            if (now - last_display_update >= 100) {
                last_display_update = now;
                kernel_update_display();
            }

            serial_interface_tick();
            kernel_watchdog_tick();
        }
    }
    in_tick = false;
}

#undef TIM4_IRQHandler
extern TIM_HandleTypeDef htim4;
void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim4);
}

#include "extmod/vfs_fat.h"
#include "factoryreset.h"

static const char fresh_boot_py[] =
    "# boot.py -- run on boot to configure USB and filesystem\r\n"
    "import os, pyb\r\n"
    "try:\r\n"
    "    for f in os.listdir('/flash'):\r\n"
    "        if f.startswith('._') or f in ('.DS_Store', '.Trashes'):\r\n"
    "            try: os.remove('/flash/' + f)\r\n"
    "            except Exception: pass\r\n"
    "except Exception:\r\n"
    "    pass\r\n"
    "pyb.main('main.py')\r\n"
;

static const char fresh_main_py[] =
    "# main.py -- UCT Micromouse Default Telemetry Streamer\r\n"
    "import uct_mouse\r\n"
    "\r\n"
    "# Initialize hardware (wakes OLED screen and sensor peripherals)\r\n"
    "uct_mouse.init()\r\n"
    "uct_mouse.set_motors(0, 0)\r\n"
    "\r\n"
    "print('--- UCT Micromouse Online ---')\r\n"
    "print('Streaming live telemetry. Replace main.py with your code!')\r\n"
    "\r\n"
    "while True:\r\n"
    "    tof = uct_mouse.get_tof()\r\n"
    "    enc = uct_mouse.get_encoders()\r\n"
    "    vbatt = uct_mouse.get_vbatt()\r\n"
    "    gyro = uct_mouse.get_gyro()\r\n"
    "    print('VBatt: %.2fV | Gyro: %+.2f dps | Enc: (%d, %d) | ToF: %s' % (vbatt, gyro, enc[0], enc[1], str(tof)))\r\n"
    "    uct_mouse.delay_ms(250)\r\n"
;

static const char fresh_readme_txt[] =
    "This is the UCT Micromouse (STM32L476VE).\r\n"
    "\r\n"
    "You can get started right away by writing your Python code in 'main.py'.\r\n"
    "\r\n"
    "For online docs and resources, please visit:\r\n"
    "https://uct-micromouse.github.io/\r\n"
;

static const char fresh_no_index[] = "";

typedef struct _factory_file_t {
    const char *name;
    size_t len;
    const char *data;
} factory_file_t;

static const factory_file_t factory_files[] = {
    {"boot.py", sizeof(fresh_boot_py) - 1, fresh_boot_py},
    {"main.py", sizeof(fresh_main_py) - 1, fresh_main_py},
    {"README.txt", sizeof(fresh_readme_txt) - 1, fresh_readme_txt},
    {".metadata_never_index", 0, fresh_no_index},
};

void factory_reset_make_files(FATFS *fatfs) {
    char ram_buf[1024];
    for (size_t i = 0; i < sizeof(factory_files) / sizeof(factory_files[0]); ++i) {
        const factory_file_t *f = &factory_files[i];
        FIL fp;
        FRESULT res = f_open(fatfs, &fp, f->name, FA_WRITE | FA_CREATE_ALWAYS);
        if (res == FR_OK) {
            UINT n;
            size_t copy_len = f->len < sizeof(ram_buf) ? f->len : sizeof(ram_buf);
            memcpy(ram_buf, f->data, copy_len);
            f_write(&fp, ram_buf, copy_len, &n);
            f_close(&fp);
        }
    }
}

// If the flash partition is blank/unformatted, format it as a valid FAT filesystem with default files
int factory_reset_create_filesystem(void) {
    uart_print("MPY: Initializing fresh FAT filesystem on external SPI flash...\n");
    
    fs_user_mount_t vfs;
    vfs.blockdev.flags = 0;
    pyb_flash_init_vfs(&vfs);
    uint8_t working_buf[512];
    FRESULT res = f_mkfs(&vfs.fatfs, FM_FAT, 0, working_buf, sizeof(working_buf));
    if (res != FR_OK) {
        uart_print("MPY: Failed to create flash filesystem!\n");
        return -19; // -ENODEV
    }

    // Set volume label
    f_setlabel(&vfs.fatfs, MICROPY_HW_FLASH_FS_LABEL);

    // Populate the filesystem with factory default files
    factory_reset_make_files(&vfs.fatfs);

    uart_print("MPY: Flash filesystem successfully created and populated.\n");
    return 0; // success
}


