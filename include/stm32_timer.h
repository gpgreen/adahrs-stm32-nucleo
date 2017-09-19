/*
 * stm32_timer.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef STM32_TIMER_H_
#define STM32_TIMER_H_

// ----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class Timer
{
public:
    enum TimerState { Idle, Active, Done };
    enum TimerType { OneShot, Periodic };

    explicit Timer();
        
    int start(unsigned int microseconds, TimerType = OneShot);
    int wait_for();
    void cancel();
    
    volatile TimerState state;
    TimerType type;
    unsigned int length;
    volatile unsigned int count;
    Timer* next;

private:

    static void interrupt();
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

#endif /* STM32_TIMER_H_ */
