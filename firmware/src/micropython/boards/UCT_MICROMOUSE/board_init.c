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
// This sets up the clock tree (80MHz) and peripheral dividers (I2C1, I2C2, SAI1, ADC, USB)
// as required by Jesse's C-Kernel.
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        // Panic block
        while (1);
    }

    HAL_PWR_EnableBkUpAccess();

    // 1. Configure System Clock using HSI + PLL (80 MHz)
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
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

    // 2. Configure USB, ADC, and I2C clocks.
    // USB uses PLLSAI1-Q (16MHz / 1 * 12 / 4 = 48 MHz)
    // ADC uses PLLSAI1-R (16MHz / 1 * 12 / 2 = 96 MHz)
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC
                                             | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    
    PeriphClkInitStruct.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1N = 12;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK | RCC_PLLSAI1_ADC1CLK;
    
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        while (1);
    }
}

// Define strong mp_hal_stdout_tx_strn to override the default weak one in mphalport.c.
// This allows redirecting output to USART1 (pins on physical board) for logging.
mp_uint_t mp_hal_stdout_tx_strn(const char *str, size_t len) {
    mp_uint_t ret = len;
    bool did_write = false;

    // Direct register-level VCP UART output redirect to USART1 for boot logging and fault reporting
    if (USART1 != NULL && (RCC->APB2ENR & RCC_APB2ENR_USART1EN)) {
        for (size_t i = 0; i < len; i++) {
            while (!(USART1->ISR & USART_ISR_TXE));
            USART1->TDR = (uint8_t)str[i];
        }
        did_write = true;
    }

    if (MP_STATE_PORT(pyb_stdio_uart) != NULL) {
        uart_tx_strn(MP_STATE_PORT(pyb_stdio_uart), str, len);
        did_write = true;
    }
    #if MICROPY_HW_USB_CDC && MICROPY_HW_TINYUSB_STACK
    mp_uint_t cdc_res = mp_usbd_cdc_tx_strn(str, len);
    if (cdc_res > 0) {
        did_write = true;
        ret = MIN(cdc_res, ret);
    }
    #endif
    int dupterm_res = mp_os_dupterm_tx_strn(str, len);
    if (dupterm_res >= 0) {
        did_write = true;
        ret = MIN((mp_uint_t)dupterm_res, ret);
    }

    return did_write ? ret : 0;
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
    USART1->BRR = 694; 
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

    // 3. Initialize NVIC and other peripheral controllers
    MX_NVIC_Init();
    
    uart_print("Initializing ADC...\n");
    MX_ADC1_Init();
    // Disable the ADC DMA interrupt in NVIC. The DMA hardware circular transfer
    // will continue updating values in the buffer, but it won't interrupt the CPU
    // (avoiding IRQ loop conflicts with MicroPython's dma.c)
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Channel3_IRQn);
    
    uart_print("Initializing I2C1...\n");
    MX_I2C1_Init();
    
    uart_print("Initializing I2C2...\n");
    MX_I2C2_Init();
    
    uart_print("Initializing Timers...\n");
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM7_Init();
    
    uart_print("Initializing SPI2 (External Flash)...\n");
    MX_SPI2_Init();

    // Initialize OLED display early on boot to show welcome feedback
    uart_print("Initializing Boot OLED Display...\n");
    SSD1306_Init();
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(4, 2);
    SSD1306_Puts("UCT Mouse", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(4, 24);
    SSD1306_Puts("REPL/VCP Ready", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(4, 40);
    SSD1306_Puts("Status: Idle", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();

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

    // LED1 (PA4) and LED2 (PA5)
    GPIO_InitStruct_LEDs.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_LEDs);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);

    uart_print("Boot sequence completed successfully.\n");
}

// Background tick function hook called inside MicroPython VM execution and delay loops
void kernel_background_tick(void) {
    extern volatile bool ext_flash_busy;
    if (ext_flash_busy) {
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
        
        // Check deferred flash flush
        extern void bdev_check_flush(void);
        bdev_check_flush();

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

#include "extmod/vfs_fat.h"
#include "factoryreset.h"

static const char fresh_boot_py[] =
    "# boot.py -- run on boot to configure USB and filesystem\r\n"
    "# Put app code in main.py\r\n"
    "\r\n"
    "import machine\r\n"
    "import pyb\r\n"
    "#pyb.main('main.py') # main script to run after this one\r\n"
    "#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device\r\n"
;

static const char fresh_main_py[] =
    "# main.py -- put your code here!\r\n"
;

static const char fresh_readme_txt[] =
    "This is the UCT Micromouse (STM32L476RG).\r\n"
    "\r\n"
    "You can get started right away by writing your Python code in 'main.py'.\r\n"
    "\r\n"
    "For online docs and resources, please visit:\r\n"
    "https://uct-micromouse.github.io/\r\n"
;

typedef struct _factory_file_t {
    const char *name;
    size_t len;
    const char *data;
} factory_file_t;

static const factory_file_t factory_files[] = {
    {"boot.py", sizeof(fresh_boot_py) - 1, fresh_boot_py},
    {"main.py", sizeof(fresh_main_py) - 1, fresh_main_py},
    {"README.txt", sizeof(fresh_readme_txt) - 1, fresh_readme_txt},
};

void factory_reset_make_files(FATFS *fatfs) {
    for (size_t i = 0; i < sizeof(factory_files) / sizeof(factory_files[0]); ++i) {
        const factory_file_t *f = &factory_files[i];
        FIL fp;
        FRESULT res = f_open(fatfs, &fp, f->name, FA_WRITE | FA_CREATE_ALWAYS);
        if (res == FR_OK) {
            UINT n;
            f_write(&fp, f->data, f->len, &n);
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


