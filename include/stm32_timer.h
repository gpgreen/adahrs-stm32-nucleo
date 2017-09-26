/*
 * stm32_timer.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef STM32_TIMER_H_
#define STM32_TIMER_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"

// ----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class Timer
{
public:
    enum TimerState { Idle, Active, Done };
    enum TimerType { OneShot, Periodic };

    static void begin(uint8_t priority, uint8_t subpriority);
    
    explicit Timer();

    // start the timer, wait_for must be called on timers started this way
    // returns -1 if timer already active
    int start(uint32_t microseconds, TimerType = OneShot);

    // start the timer with a timeout function called when done
    // returns -1 if timer already active
    int start(uint32_t microseconds, void (*timeout_fn)(void*), void* timeout_fn_data,
              TimerType = OneShot);

    // blocking wait for timer to complete, returns -1 if timer not active or it has
    // a timeout function attached in start
    int wait_for();

    // cancel the timer
    void cancel();

private:
    
    volatile TimerState state;
    TimerType type;
    uint32_t length;
    volatile uint32_t count;
    Timer* next;
    void (*timer_fn)(void*);
    void* timer_fn_data;
    
private:

    // define away copy constructor and assignment operator
    Timer(const Timer&);
    Timer& operator=(const Timer&);

    friend class TimerList;
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

#endif /* STM32_TIMER_H_ */
