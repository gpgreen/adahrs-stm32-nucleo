/*
 * isr_def.h
 *
 *  Created on: Sep 8, 2017
 *      Author: ggreen
 */

#ifndef ISR_DEF_H_
#define ISR_DEF_H_

// ----------------------------------------------------------------------------
// Declaration of exception handlers
// ----------------------------------------------------------------------------
#if defined(__cplusplus)
extern "C"
{
#endif

    // TIM
    void TIM2_IRQHandler(void);

    // I2C
    void I2C1_EV_IRQHandler(void);
    void I2C2_EV_IRQHandler(void);

    // SPI
    void SPI1_IRQHandler(void);
    void SPI2_IRQHandler(void);

    // USART
    void USART1_IRQHandler(void);
    void USART2_IRQHandler(void);
    void USART3_IRQHandler(void);

    // DMA
    void DMA1_Channel1_IRQHandler(void);
    void DMA1_Channel2_IRQHandler(void);
    void DMA1_Channel3_IRQHandler(void);
    void DMA1_Channel4_IRQHandler(void);
    void DMA1_Channel5_IRQHandler(void);
    void DMA1_Channel6_IRQHandler(void);
    void DMA1_Channel7_IRQHandler(void);
    void DMA2_Channel1_IRQHandler(void);
    void DMA2_Channel2_IRQHandler(void);
    void DMA2_Channel3_IRQHandler(void);
    void DMA2_Channel4_5_IRQHandler(void);

    // EXTI
    void EXTI0_IRQHandler(void);
    void EXTI1_IRQHandler(void);
    
    void configure_nvic(uint8_t irq_no, uint8_t priority, uint8_t subpriority);

#if defined(__cplusplus)
}
#endif

// ----------------------------------------------------------------------------
#endif /* ISR_DEF_H_ */
