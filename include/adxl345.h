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
    explicit ADXL345(I2C* bus);

    // initialize the hardware
    void begin(bool use_interrupt,
               int16_t* sign_map, uint8_t* axis_map,
               uint8_t priority, uint8_t subpriority);

    // retrieve data, return false if not ready
    bool start_get_sensor_data();

    // true when data has been received
    bool sensor_data_received()
    {
        return _state == 12;
    }

    // convert the retrieved data to corrected values
    void correct_sensor_data();

    // the raw acceleration values per axis
    int16_t get_raw_accel(int axis)
    {
        return _raw_accel[axis];
    }

    // the corrected acceleration values per axis
    int16_t get_corrected_accel(int axis)
    {
        return _corrected_accel[axis];
    }

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
    uint8_t _axis_map[3];
    bool _use_interrupt;
    int16_t _sign_map[3];
    int16_t _raw_accel[3];
    int16_t _corrected_accel[3];
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
