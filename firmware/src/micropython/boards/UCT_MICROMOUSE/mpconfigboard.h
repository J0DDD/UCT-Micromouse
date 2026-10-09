#define MICROPY_HW_BOARD_NAME       "UCT-MICROMOUSE"
#define MICROPY_HW_MCU_NAME         "STM32L476VE"
#define MICROPY_HW_FLASH_FS_LABEL   "UCT_MMOUSE"


#define MICROPY_HW_HAS_SWITCH       (0)
#define MICROPY_HW_HAS_FLASH        (1)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_USB       (1)
#define MICROPY_HW_ENABLE_DAC       (0)

// HSE clock configuration (8 MHz quartz crystal on PH0/PH1 -> 80 MHz SysClk)
#define MICROPY_HW_CLK_USE_HSE      (1)
#define MICROPY_HW_CLK_PLLM         (1)
#define MICROPY_HW_CLK_PLLN         (20)
#define MICROPY_HW_CLK_PLLP         (RCC_PLLP_DIV7)
#define MICROPY_HW_CLK_PLLQ         (RCC_PLLQ_DIV2)
#define MICROPY_HW_CLK_PLLR         (RCC_PLLR_DIV2)

// PLLSAI1 configuration (8 MHz HSE * 12 / 2 = 48 MHz for USB and ADC)
#define MICROPY_HW_CLK_PLLSAIN      (12)
#define MICROPY_HW_CLK_PLLSAIP      (RCC_PLLP_DIV7)
#define MICROPY_HW_CLK_PLLSAIQ      (RCC_PLLQ_DIV2)
#define MICROPY_HW_CLK_PLLSAIR      (RCC_PLLR_DIV2)

#define MICROPY_HW_FLASH_LATENCY    FLASH_LATENCY_4

// The board does not use LSE (PC14/PC15 are routed to LEDs)
#define MICROPY_HW_RTC_USE_LSE      (0)

// USART1 config (Pins match the physical board's serial connection)
#define MICROPY_HW_UART1_TX         (pin_B6)
#define MICROPY_HW_UART1_RX         (pin_B7)

// REPL is duplicated / routed over USART1 (ST-Link VCP) as well as USB VCP
#define MICROPY_HW_UART_REPL        PYB_UART_1
#define MICROPY_HW_UART_REPL_BAUD   115200
#define MICROPY_HW_ENABLE_UART_DEBUG (1)

// I2C buses (I2C1 for TOF/IMU, I2C2 for OLED/Sensors)
#define MICROPY_HW_I2C1_SCL         (pin_B8)
#define MICROPY_HW_I2C1_SDA         (pin_B9)
#define MICROPY_HW_I2C2_SCL         (pin_B10)
#define MICROPY_HW_I2C2_SDA         (pin_B11)

// SPI buses (SPI2 for External NOR Flash: PB12=CS, PB13=SCK, PB14=MISO, PB15=MOSI)
#define MICROPY_HW_SPI2_SCK         (pin_B13)
#define MICROPY_HW_SPI2_MISO        (pin_B14)
#define MICROPY_HW_SPI2_MOSI        (pin_B15)

// USRSW disabled from MicroPython core to prevent false factory reset mode during battery boot.
// Student user button (SW1 on PE6) is handled directly via uct_mouse.get_button().
#define MICROPY_HW_HAS_SWITCH       (0)

// LEDs (PC13, PC14, PC15)
#define MICROPY_HW_LED1             (pin_C13)
#define MICROPY_HW_LED2             (pin_A4)
#define MICROPY_HW_LED3             (pin_A5)
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_high(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_low(pin))

// USB config
#define MICROPY_HW_USB_FS           (1)
#define MICROPY_HW_USB_MSC          (1)
#define MICROPY_HW_FLASH_MOUNT_AT_BOOT (1)

// Board startup and loop hooks to run the background C-Kernel task
void board_startup(void);
void board_early_init(void);
void kernel_background_tick(void);
void board_start_soft_reset(void);

#define MICROPY_BOARD_STARTUP       board_startup
#define MICROPY_BOARD_EARLY_INIT    board_early_init
#define MICROPY_VM_HOOK_LOOP        kernel_background_tick();
#define MICROPY_BOARD_START_SOFT_RESET(state) board_start_soft_reset()

// Expose the custom uct_mouse module as a built-in module
extern const struct _mp_obj_module_t uct_mouse_module;
#define MICROPY_PORT_BUILTIN_MODULES \
    { MP_ROM_QSTR(MP_QSTR_uct_mouse), MP_ROM_PTR(&uct_mouse_module) },

// Enable internal flash storage for the MicroPython FAT filesystem (64KB at 0x08060000)
#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE (1)

// Preprocessor overrides to rename core MicroPython interrupt handlers.
// This allows us to define the actual hardware vectors in our custom board code.
#define TIM4_IRQHandler             __real_TIM4_IRQHandler
#define EXTI0_IRQHandler            __real_EXTI0_IRQHandler
#define EXTI1_IRQHandler            __real_EXTI1_IRQHandler
#define EXTI2_IRQHandler            __real_EXTI2_IRQHandler
#define EXTI3_IRQHandler            __real_EXTI3_IRQHandler
#define EXTI4_IRQHandler            __real_EXTI4_IRQHandler
#define EXTI9_5_IRQHandler          __real_EXTI9_5_IRQHandler
#define EXTI15_10_IRQHandler        __real_EXTI15_10_IRQHandler



