/*
 * ADAHRSInit.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "ADAHRSInit.h"

// ----------------------------------------------------------------------------

ADAHRSInit::ADAHRSInit()
{
    // Enable GPIO Peripheral clock
    RCC_APB2PeriphClockCmd(ADAHRS_RCC_MASKx(LED_PORT_NUMBER), ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure pin in output push/pull mode
    GPIO_InitStructure.GPIO_Pin = ADAHRS_PIN_MASK(LED_PIN_NUMBER);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(ADAHRS_GPIOx(LED_PORT_NUMBER), &GPIO_InitStructure);
    
    // Start with led turned off
    led_off();
}

// ----------------------------------------------------------------------------




