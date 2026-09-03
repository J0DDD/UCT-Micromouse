MCU_SERIES = l4
CMSIS_MCU = STM32L476xx
AF_FILE = boards/stm32l476_af.csv
LD_FILES = boards/stm32l476xe.ld boards/common_basic.ld
OPENOCD_CONFIG = boards/openocd_stm32l4.cfg

MICROPY_HW_ENABLE_ISR_UART_FLASH_FUNCS_IN_RAM = 1

INC += -I$(BOARD_DIR)
INC += -I$(BOARD_DIR)/src_link/kernel/inc
INC += -I$(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Inc
INC += -I$(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/USB_DEVICE/App
INC += -I$(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/USB_DEVICE/Target

CFLAGS_EXTRA += -Dmain=jesse_legacy_main -DCOMPILING_FOR_MICROPYTHON -Wno-float-conversion -Wno-builtin-declaration-mismatch -Wno-discarded-qualifiers -Wno-unused-variable -Wno-implicit-function-declaration -Wno-parentheses

SRC_C += $(BOARD_DIR)/src_link/kernel/src/micromouse_kernel.c
SRC_C += $(BOARD_DIR)/src_link/kernel/src/serial_interface.c
SRC_C += $(BOARD_DIR)/src_link/kernel/src/kernel_logger.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/IMU.c
SRC_C += $(BOARD_DIR)/VL53L0X.c
SRC_C += $(BOARD_DIR)/SSD1306.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/Motors.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/ADCs.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/LEDs.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/Buttons.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/INA219.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/preformatted_flash.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/stm32l4xx_hal_msp.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/MicroMouse_main.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/DMA.c
SRC_C += $(BOARD_DIR)/ZD25WQ80C.c
SRC_C += $(BOARD_DIR)/external_link/MicroMouseTemplate/MicroMouseProgramming_Code/Core/Src/VCP.c
SRC_C += $(BOARD_DIR)/bdev.c
SRC_C += lib/stm32lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
SRC_C += lib/stm32lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c
SRC_C += lib/stm32lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c
SRC_C += lib/stm32lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c
SRC_C += lib/stm32lib/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c




