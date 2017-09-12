/*
 * main.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 *
 * Original copyright header:
 *
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

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "adahrs_init.h"
#include "stm32_delaytimer.h"
#include "stm32_usart.h"
#include "stm32_i2c.h"
#include "adxl345.h"

// ----------------------------------------------------------------------------
//
// Components and Hardware utilization
//
// *** Hardware ***
//
// I2C1
//   - DMA1 Channel6
//   - DMA1 Channel7
//   - GPIO: PB6, PB7
// USART1
//   - DMA1 Channel4
//   - DMA1 Channel5
//   - GPIO: PA9, PA10
// SPI1
//   - DMA1 Channel2
//   - DMA1 Channel3
//   - Remap Pins
//   - GPIO: PA15, PB3, PB4, PB5
// Onboard LED
//   - GPIO: PA5
//
// *** Software Drivers ***
//
// WorkQueue (g_work_queue)
//   - TIM2
// ADXL345
//   - I2C1
//   - EXTI Line 0
//   - GPIO: PA0
// DelayTimer
//   - SysTick
// ----------------------------------------------------------------------------

int main(int, char**);

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"
//#pragma GCC diagnostic ignored "-Wmissing-declarations"
//#pragma GCC diagnostic ignored "-Wreturn-type"

ADAHRSInit init;

bool wait_for_accel_data = false;

// adjustments for accelerometer, which axis is x,y,z
// sign map to reverse acceleration on axis
// bias to apply to each axis
int accel_axis_map[3] = {0, 1, 2};
int16_t accel_sign_map[3] = {1, 1, 1};
int accel_bias[3] = {0, 0, 0};

int
main(int /*argc*/, char* /*argv*/[])
{
    // At this stage the system clock should have already been configured
    // at high speed.
    
    delaytimer.start();
    init.begin();

    uint32_t seconds = 0;

    // start the accel sensor
    ADXL345 adxl(&i2c1);
    adxl.begin(accel_sign_map, accel_axis_map, accel_bias,
	       ADXL_IRQ_PRIORITY, 0);
    
    // Infinite loop
    usart1.transmit("hello!\r\n", 8);
    while (1)
    {
    	led_on();
        delaytimer.sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);

        led_off();
        delaytimer.sleep(BLINK_OFF_TICKS);

        if (adxl.sensor_data_received())
        {
            adxl.correct_sensor_data();
            usart1.transmit("got some!\r\n", 11);
        }
        
        ++seconds;
    }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
