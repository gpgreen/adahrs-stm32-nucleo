/*
 * stm32_timer.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef STM32_TIMER_H_
#define STM32_TIMER_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------

class Timer
{
public:
    typedef uint32_t timer_ticks_t;

    explicit Timer(unsigned long timer_freq_hz);

    void start();
    void sleep(timer_ticks_t ticks);

private:
    unsigned long _timer_freq_hz;
};

// ----------------------------------------------------------------------------

#endif // STM32_TIMER_H_
