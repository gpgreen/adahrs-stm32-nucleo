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

// ----------------------------------------------------------------------------

class ADXL345
{
public:
    explicit ADXL345(I2C* bus);

    // initialize the hardware
    void begin(int16_t* sign_map, int* axis_map, int* bias,
               uint8_t priority, uint8_t subpriority);

    // retrieve data, return false if not ready
    bool start_get_sensor_data();

    // true when data has been received
    bool sensor_data_received();

    // convert the retrieved data to corrected values
    void get_sensor_data();

    uint16_t get_raw_accel(int axis)
    {
        return _raw_accel[axis];
    }

    int get_corrected_accel(int axis)
    {
        return _corrected_accel[axis];
    }
    
    float get_accel(int axis)
    {
        return static_cast<float>(_corrected_accel[axis]);
    }
    
private:

    static void bus_callback(void *data);
    
    // define away copy constructor and assignment operator
    ADXL345(const ADXL345&);
    const ADXL345& operator=(const ADXL345&);

private:
    I2C* _bus;
    volatile int _state;
    uint8_t _data[8];
    uint16_t _raw_accel[3];
    int16_t _sign_map[3];
    int _corrected_accel[3];
    int _bias[3];
    int _axis_map[3];

    // i2c transactions
    I2CMasterTxHeader _i2c_header;
    I2CMasterTxSegment _i2c_segments[3];
};

// ----------------------------------------------------------------------------

#endif /* INCLUDE_ADXL345_H_ */
