/*
 * ADAHRSInit.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "adahrs_init.h"
#include "stm32_delaytimer.h"
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

void ADAHRSInit::begin(ADAHRSConfig* config, ADAHRSSensorData* state,
                       ADAHRSCommand* command, EKF* ekf)
{
    // assign all priority bits to preempt, none to subpriority
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // initialize the timer
    Timer::begin(TIMER_IRQ_PRIORITY, 0);

    // initialize the delaytimer
    delaytimer.begin();
    
    // initialize config from Flash
    config->begin();

    // initialize state from config
    state->begin(config);
    
    // configure DMA channels
    DMA1Channel2.begin(DMA1_IRQ_PRIORITY, 0);
    DMA1Channel3.begin(DMA1_IRQ_PRIORITY, 0);
    DMA1Channel4.begin(DMA1_IRQ_PRIORITY, 0);
    DMA1Channel5.begin(DMA1_IRQ_PRIORITY, 0);
    DMA1Channel6.begin(DMA1_IRQ_PRIORITY, 0);
    DMA1Channel7.begin(DMA1_IRQ_PRIORITY, 0);

    // configure usart
    usart1.begin(115200, USART_IRQ_PRIORITY, 0);

    // configure spi1 in alternate pin mode, hardware ss control
    spi1.begin(true, true, nullptr, SPI1_IRQ_PRIORITY, 0);

    // configure i2c1
    i2c1.begin(false, I2C1_IRQ_PRIORITY, 0);

    // initialize the kalman filter
    ekf->begin(config, state);

    // configure work queue
    g_work_queue.begin();
    
    // configure command
    command->begin(&usart1, config, state, ekf, COMMAND_IRQ_PRIORITY, 0);
    
    // setup the led
    configure_led();
}

void ADAHRSInit::configure_led()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // configure LED pin
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure pin in output push/pull mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Start with led turned off
    led_off();
}

// ----------------------------------------------------------------------------

void configure_nvic(uint8_t irq_no, uint8_t priority, uint8_t subpriority)
{
    // enable the IRQ
    NVIC_InitTypeDef NVIC_InitStructure;

    // enable the DMA Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = irq_no;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

// ----------------------------------------------------------------------------

