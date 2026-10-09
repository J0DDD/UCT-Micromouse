#include "py/obj.h"
#include "storage.h"
#include "ZD25WQ80C.h"
#include <string.h>
#include "main.h"

// Systematically de-initialize custom peripherals and clear/disable NVIC interrupts before soft-reboot
void board_start_soft_reset(void) {
    extern volatile bool mouse_initialized;
    mouse_initialized = false;
    
    // Enable DMA clocks to safely access CCR registers without HardFault
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    // Stop all motor PWM actuation immediately and disable motor driver
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    
    // Force disable all DMA channels to prevent background transfers during reboot transition
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
    
    // Disable NVIC interrupts for custom peripherals (keep VCP/USB interrupts active)
    NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_DisableIRQ(TIM1_CC_IRQn);
    NVIC_DisableIRQ(TIM3_IRQn);
    NVIC_DisableIRQ(TIM4_IRQn);
    NVIC_DisableIRQ(TIM5_IRQn);
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
    NVIC_DisableIRQ(TIM7_IRQn);
    NVIC_DisableIRQ(ADC1_2_IRQn);
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    NVIC_DisableIRQ(DMA2_Channel3_IRQn);
    NVIC_DisableIRQ(DMA2_Channel6_IRQn);
    NVIC_DisableIRQ(DMA2_Channel7_IRQn);
    
    // Clear any pending interrupts to prevent immediately jumping into unmapped handlers in new vector table
    NVIC_ClearPendingIRQ(TIM1_UP_TIM16_IRQn);
    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    NVIC_ClearPendingIRQ(TIM4_IRQn);
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Channel3_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Channel6_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Channel7_IRQn);
}
