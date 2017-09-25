/*
 * adxl345.h
 *
 *  Created on: Sep 7, 2017
 *      Author: ggreen
 */

#ifndef INCLUDE_ADXL345_H_
#define INCLUDE_ADXL345_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_i2c.h"
#include "isr_def.h"

// ----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class ADXL345
{
    
public:
    enum DataRate {HZ_400, HZ_200, HZ_100, HZ_50, HZ_25, HZ_12_5};

    explicit ADXL345(I2C* bus);

    // initialize the hardware
    void begin(DataRate rate, bool use_interrupt,
               uint8_t priority, uint8_t subpriority);

    // retrieve data, return false if not ready
    bool start_get_sensor_data();

    // true when data has been received
    bool sensor_data_received()
    {
        return _state == 12;
    }

    // convert the retrieved data to corrected values
    void retrieve_sensor_data(int16_t& accel_x,
                              int16_t& accel_y,
                              int16_t& accel_z);

private:

    void init_stage2();
    static void bus_callback(void* data);
    static void get_data_trigger(void* data);
    static void retry_send(void* data);
    
    // define away copy constructor and assignment operator
    ADXL345(const ADXL345&);
    const ADXL345& operator=(const ADXL345&);

private:
    I2C* _bus;
    volatile int _state;
    uint8_t _data[8];
    bool _use_interrupt;
    volatile uint32_t _retries;
    volatile uint32_t _missed_converts;

    // i2c transactions
    I2CMasterTxHeader _i2c_header;
    I2CMasterTxSegment _i2c_segments[3];

    friend void EXTI0_IRQHandler(void);
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

#endif /* INCLUDE_ADXL345_H_ */
