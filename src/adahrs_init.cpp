/*
 * ADAHRSInit.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "adahrs_init.h"
#include "stm32_dma.h"

// ----------------------------------------------------------------------------

ADAHRSInit::ADAHRSInit()
{
    // Enable GPIO Peripheral clock
    RCC_APB2PeriphClockCmd(ADAHRS_RCC_MASKx(LED_PORT_NUMBER), ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure pin in output push/pull mode
    GPIO_InitStructure.GPIO_Pin = ADAHRS_PIN_MASK(LED_PIN_NUMBER);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(ADAHRS_GPIOx(LED_PORT_NUMBER), &GPIO_InitStructure);
    
    DMA1Channel6.begin(2, 0);
    DMA1Channel7.begin(2, 0);

    // Start with led turned off
    led_off();
}

// ----------------------------------------------------------------------------




