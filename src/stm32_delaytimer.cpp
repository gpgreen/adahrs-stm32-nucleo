/*
 * stm32_delaytimer.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "stm32_delaytimer.h"
#include "cortexm/ExceptionHandlers.h"
#include "adahrs_definitions.h"

DelayTimer delaytimer(TIMER_FREQUENCY_HZ);

// ----------------------------------------------------------------------------

#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

// global variable used to for counts
volatile DelayTimer::timer_ticks_t g_delay_count;

// ----------------------------------------------------------------------------

DelayTimer::DelayTimer(timer_ticks_t timer_freq_hz)
: _timer_freq_hz(timer_freq_hz)
{
    // does nothing else
}

// ----------------------------------------------------------------------------

void
DelayTimer::start (void)
{
    // Use SysTick as reference for the delay loops.
    SysTick_Config (SystemCoreClock / _timer_freq_hz);
}

void
DelayTimer::sleep (timer_ticks_t ticks)
{
    g_delay_count = ticks;

    // Busy wait until the SysTick decrements the counter to zero.
    while (g_delay_count != 0u)
	;
}

// interrupt function
static void
timer_tick (void)
{
    // Decrement to zero the counter used by the delay routine.
    if (g_delay_count != 0u)
    {
	--g_delay_count;
    }
}

// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void)
{
#if defined(USE_HAL_DRIVER)
    HAL_IncTick();
#endif
    timer_tick ();
}

// ----------------------------------------------------------------------------
