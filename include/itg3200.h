/* ------------------------------------------------------------------------------
  File: itg3200.h
  Author: Greg Green
  Version: 1.0
  
  Description: Code for interacting with the ITG3200 gyro
------------------------------------------------------------------------------ */ 

#ifndef	__ITG3200_H
#define __ITG3200_H

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_i2c.h"

// ----------------------------------------------------------------------------

class ITG3200
{
public:
    explicit ITG3200(I2C* bus);

    // initialize the hardware
    void begin(int16_t* sign_map, int* axis_map, int* bias,
               uint8_t priority, uint8_t subpriority);

    // retrieve data, return false if not ready
    bool start_get_sensor_data();

    // true when data has been received
    bool sensor_data_received();

    // convert the retrieved data to corrected values
    void get_sensor_data();

    uint16_t get_raw_gyros(int axis)
    {
        return _raw_gyro[axis];
    }

    int get_corrected_gyros(int axis)
    {
        return _corrected_gyro[axis];
    }
    
    float get_gyro(int axis)
    {
        return static_cast<float>(_corrected_gyro[axis]);
    }
    
private:

    static void bus_callback(void *data);
    
    // define away copy constructor and assignment operator
    ITG3200(const ITG3200&);
    const ITG3200& operator=(const ITG3200&);

private:
    I2C* _bus;
    volatile int _state;
    uint8_t _data[8];
    uint16_t _raw_gyro[3];
    int16_t _sign_map[3];
    int _corrected_gyro[3];
    int _bias[3];
    int _axis_map[3];

    // i2c transactions
    I2CMasterTxHeader _i2c_header;
    I2CMasterTxSegment _i2c_segments[3];
};

// ----------------------------------------------------------------------------

#endif
