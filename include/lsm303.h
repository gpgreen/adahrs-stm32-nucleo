/*
 * lsm303.h
 *
 *  Created on: Sep 7, 2017
 *      Author: ggreen
 */

#ifndef INCLUDE_LSM303_H_
#define INCLUDE_LSM303_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_i2c.h"

// ----------------------------------------------------------------------------

class LSM303
{
public:
    explicit LSM303(I2C* bus);

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
    LSM303(const LSM303&);
    const LSM303& operator=(const LSM303&);

private:
    I2C* _bus;
    volatile int _state;
    uint8_t _data[6];
    uint8_t padding[2];
};

// ----------------------------------------------------------------------------

#endif /* INCLUDE_LSM303_H_ */
