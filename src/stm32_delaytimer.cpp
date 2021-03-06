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

#ifdef USE_DELAYTIMER_TRACE
static int tracer;
#endif

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
DelayTimer::begin()
{
#ifdef USE_DELAYTIMER_TRACE
    // setup pin
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // Configure PA2 as output push pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // set low
    GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    tracer = 0;
#endif
    
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

void
DelayTimer::delay (int delay_instr)
{
    for (int counter=0; counter<delay_instr; ++counter)
    {
        asm volatile("mov r0, r0");
    }
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

#ifdef USE_DELAYTIMER_TRACE

    if (tracer > 0)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        tracer = 0;
    }
    else
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
        tracer = 1;
    }

#endif
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
