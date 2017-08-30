/*
 * Timer.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef TIMER_H_
#define TIMER_H_

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

#endif // TIMER_H_
