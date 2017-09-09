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
    void begin(uint8_t priority, uint8_t subpriority);

    // retrieve data, return false if not ready
    bool start_get_sensor_data();

    // true when data has been received
    bool sensor_data_received();

    // copy the retrieved data to a buffer (length = 6)
    void get_sensor_data(uint8_t* buf);
    
private:

    static void bus_callback(void *data);
    
    // define away copy constructor and assignment operator
    ADXL345(const ADXL345&);
    const ADXL345& operator=(const ADXL345&);

private:
    I2C* _bus;
    volatile int _state;
    uint8_t _data[6];
    uint8_t padding[2];
};

// ----------------------------------------------------------------------------

#endif /* INCLUDE_ADXL345_H_ */
