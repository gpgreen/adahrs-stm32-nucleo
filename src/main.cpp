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
#include "adahrs_ekf.h"
#include "stm32_timer.h"
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
//   - GPIO: PA0 (if interrupts)
// ITG3200
//   - I2C1
//   - EXTI Line 1
//   - GPIO: PA1 (if interrupts)
// DelayTimer
//   - SysTick
//   - GPIO: PA2 (USE_DELAYTIMER_TRACE)
// Timer
//   - TIM3
// EKF
//   - TIM4
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

// global flag for gyro data retrieval
volatile int g_get_gyro_data = 0;

static void gyro_timer_expired(void* /*data*/)
{
    g_get_gyro_data = 1;
}

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
    
    // initialize devices
    init.begin(&config, &state);
    
    Timer t0;

    // start the accel sensor
    ADXL345 adxl(&i2c1);
    adxl.begin(true, sign_map, axis_map, ADXL_IRQ_PRIORITY, 0);
    // now sleep for 50ms
    t0.start(50000, Timer::OneShot);
    t0.wait_for();
    
    // start the gyro sensor, no interrupts
    ITG3200 gyro(&i2c1);
    gyro.begin(false, sign_map, axis_map, ITG_IRQ_PRIORITY, 0);
    // now sleep for 100ms
    t0.start(100000, Timer::OneShot);
    t0.wait_for();

    // initialize the kalman filter
    EKF ekf;
    ekf.begin(&config, &state);

    // timer to trigger getting gyro data
    Timer gyrodata;
    gyrodata.start(100000, gyro_timer_expired, nullptr, Timer::Periodic);
    
    // Infinite loop
    usart1.transmit("hello!\r\n", 8);

    //uint32_t seconds = 0;

    while (1)
    {
    	//led_on();
        //t0.start(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);
        //t0.wait_for();

        //led_off();
        //t0.start(BLINK_OFF_TICKS);
        //t0.wait_for();

        // is there new data from the accelerometer?
        if (adxl.sensor_data_received())
        {
            adxl.correct_sensor_data();
            state.raw_data.accel_x = adxl.get_corrected_accel(0);
            state.raw_data.accel_y = adxl.get_corrected_accel(1);
            state.raw_data.accel_z = adxl.get_corrected_accel(2);
            state.raw_data.new_accel_data = 1;
        }
        
        // start gyro data retrieval
        if (g_get_gyro_data)
        {
            g_get_gyro_data = 0;
            gyro.start_get_sensor_data();
        }

        // is there data available from the gyros?
        if (gyro.sensor_data_received())
        {
            gyro.correct_sensor_data();
            state.raw_data.temperature = gyro.temperature();
            state.raw_data.gyro_x = gyro.get_corrected_gyro(0);
            state.raw_data.gyro_y = gyro.get_corrected_gyro(1);
            state.raw_data.gyro_z = gyro.get_corrected_gyro(2);
            state.raw_data.new_gyro_data = 1;
        }

        // do the EKF if required
        if (state.raw_data.new_gyro_data || state.raw_data.new_accel_data)
        {
            ekf.estimate_states();
            state.raw_data.new_gyro_data = 0;
            state.raw_data.new_accel_data = 0;
        }
        
        //++seconds;
    }
}

// ----------------------------------------------------------------------------
