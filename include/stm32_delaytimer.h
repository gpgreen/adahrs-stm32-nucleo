/*
 * stm32_delaytimer.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef STM32_DELAYTIMER_H_
#define STM32_DELAYTIMER_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------

class DelayTimer
{
public:
    typedef uint32_t timer_ticks_t;

    explicit DelayTimer(unsigned long timer_freq_hz);

    // startup DelayTimer subsystem
    void begin();

    // wait until number of timer ticks has passed
    void sleep(timer_ticks_t ticks);

    // delays CPU for 'delay_instr' nop instructions
    static void delay(int delay_instr);
    
private:
    // define away copy constructor and assignment operator
    DelayTimer(const DelayTimer&);
    const DelayTimer& operator=(const DelayTimer&);

    // members
    unsigned long _timer_freq_hz;
};

extern DelayTimer delaytimer;

// ----------------------------------------------------------------------------

#endif // STM32_DELAYTIMER_H_
