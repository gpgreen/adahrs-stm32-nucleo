/*
 * ADAHRSInit.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "adahrs_init.h"
#include "stm32_dma.h"
#include "stm32_usart.h"
#include "stm32_spi.h"
#include "stm32_i2c.h"
#include "work_queue.h"

// ----------------------------------------------------------------------------

ADAHRSInit::ADAHRSInit()
{
    // does nothing else
}

void ADAHRSInit::begin(void)
{
    // Enable Peripheral clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // assign all priority bits to preempt, none to subpriority
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // configure LED pin
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure pin in output push/pull mode
    GPIO_InitStructure.GPIO_Pin = LED_PIN_NUMBER;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT_NUMBER, &GPIO_InitStructure);

    // configure DMA channels
    DMA1Channel2.begin(3, 0);
    DMA1Channel3.begin(3, 0);
    DMA1Channel4.begin(3, 0);
    DMA1Channel5.begin(3, 0);
    DMA1Channel6.begin(3, 0);
    DMA1Channel7.begin(3, 0);

    // configure usart
    usart1.begin(115200, 4, 0);

    // configure spi1 in alternate pin mode
    spi1.begin(true, 3, 0);

    // configure i2c1
    i2c1.begin(false, 2, 0);
    
    // configure Timer2 for work queue
    // This is used to trigger the work queue periodically
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 2000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Enable the TIM2 global Interrupt and set at lowest priority.
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
        WORKQUEUE_IRQ_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // enable the timer
    TIM_Cmd(TIM2, ENABLE);
    // enable the update interrupt
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // Start with led turned off
    led_off();

}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // process the work queue
        g_work_queue.process();
        // clear pending interrupt bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
// ----------------------------------------------------------------------------

