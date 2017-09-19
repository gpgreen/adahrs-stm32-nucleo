/*
 * main.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>

#include "adahrs_init.h"
#include "adahrs_config.h"
#include "adahrs_states.h"
#include "stm32_delaytimer.h"
#include "stm32_usart.h"
#include "stm32_i2c.h"
#include "adxl345.h"
#include "itg3200.h"
#include "adahrs_config_def.h"

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
//   - GPIO: PA3 (USE_WORKQUEUE_TRACE)
// ADXL345
//   - I2C1
//   - EXTI Line 0
//   - GPIO: PA0
// ITG3200
//   - I2C1
//   - EXTI Line 1
//   - GPIO: PA1
// DelayTimer
//   - SysTick
//   - GPIO: PA2 (USE_DELAYTIMER_TRACE)
// ----------------------------------------------------------------------------

int main(int, char**);

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

ADAHRSConfig config;
ADAHRSInit init;
ADAHRSSensorData state;

// adjustments for device axes
// axis_map - which axis is x,y,z
// sign_map - reverse direction on an axis if -1
uint8_t axis_map[3] = {0, 1, 2};
int16_t sign_map[3] = {1, 1, 1};

int
main(int /*argc*/, char* /*argv*/[])
{
    // At this stage the system clock should have already been configured
    // at high speed.
    // SYSCLK: 72 MHz
    // HCLK: 72 MHz
    // PCLK1: 36 MHz
    // PCLK2: 72 MHz
    // ADCCLK: 36 MHz
    
    delaytimer.begin();

    // initialize devices
    init.begin(&config, &state);
    
    // start the accel sensor
    ADXL345 adxl(&i2c1);
    adxl.begin(sign_map, axis_map, ADXL_IRQ_PRIORITY, 0);
    // now sleep for 50ms
    delaytimer.sleep(50);
    
    // start the gyro sensor
    ITG3200 gyro(&i2c1);
    gyro.begin(sign_map, axis_map, ITG_IRQ_PRIORITY, 0);
    // now sleep for 100ms
    delaytimer.sleep(100);

    // Infinite loop
    usart1.transmit("hello!\r\n", 8);

    uint32_t seconds = 0;

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
        if (gyro.sensor_data_received())
        {
            gyro.correct_sensor_data();
            usart1.transmit("got more!\r\n", 11);
        }
        ++seconds;
    }
}

// ----------------------------------------------------------------------------
