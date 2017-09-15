/*
 * adahrs_init.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef ADAHRSINIT_H_
#define ADAHRSINIT_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "isr_def.h"
#include "adahrs_config.h"

// ----- LED definitions ------------------------------------------------------

// Adjust these definitions for your own board.

// stm32-nucleo64-F103RB definitions (the GREEN led, PA5, active low)
// (SEGGER J-Link device name: STM32F103RB).

// Port numbers: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, ...
#define LED_PORT_NUMBER               GPIOA
#define LED_PIN_NUMBER                GPIO_Pin_5
//#define LED_ACTIVE_LOW                (1)

// ----------------------------------------------------------------------------

class ADAHRSInit
{
public:
    explicit ADAHRSInit();

    void begin(ADAHRSConfig* config);

private:
    void configure_led();
    
    // define away copy constructor and assignment operator
    ADAHRSInit(const ADAHRSInit&);
    const ADAHRSInit& operator=(const ADAHRSInit&);
};

inline void
led_on(void);

inline void
led_off(void);

// ----------------------------------------------------------------------------

inline void
__attribute__((always_inline))
led_on(void)
{
#if (LED_ACTIVE_LOW)
  GPIO_ResetBits(LED_PORT_NUMBER, LED_PIN_NUMBER);
#else
  GPIO_WriteBit(LED_PORT_NUMBER, LED_PIN_NUMBER, Bit_SET);
#endif
}

inline void
__attribute__((always_inline))
led_off(void)
{
#if (LED_ACTIVE_LOW)
  GPIO_SetBits(LED_PORT_NUMBER, LED_PIN_NUMBER);
#else
  GPIO_WriteBit(LED_PORT_NUMBER, LED_PIN_NUMBER, Bit_RESET);
#endif
}

// ----------------------------------------------------------------------------
#endif
