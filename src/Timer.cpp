/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

// global variable used to for counts
volatile Timer::timer_ticks_t g_delay_count;

// ----------------------------------------------------------------------------

Timer::Timer(timer_ticks_t timer_freq_hz)
: _timer_freq_hz(timer_freq_hz)
{
    // does nothing else
}

// ----------------------------------------------------------------------------

void
Timer::start (void)
{
    // Use SysTick as reference for the delay loops.
    SysTick_Config (SystemCoreClock / _timer_freq_hz);
}

void
Timer::sleep (timer_ticks_t ticks)
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
