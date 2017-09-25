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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class ITG3200
{
public:
    explicit ITG3200(I2C* bus);

    // initialize the hardware
    void begin(bool use_interrupt, uint8_t priority, uint8_t subpriority);

    // has setup been completed?
    bool setup_complete();
    
    // start data retrieval
    bool start_get_sensor_data();

    // true when data has been received
    bool sensor_data_received();

    // get the gyro data from i2c buffer
    void retrieve_sensor_data(int16_t& temp,
                              int16_t& gyro_x,
                              int16_t& gyro_y,
                              int16_t& gyro_z);
    
private:

    static void init_stage1(void* data);
    static void init_stage2(void* data);
    static void get_data_trigger(void* data);
    static void bus_callback(void* data);
    static void retry_send(void* data);
    
    // define away copy constructor and assignment operator
    ITG3200(const ITG3200&);
    const ITG3200& operator=(const ITG3200&);

private:
    I2C* _bus;
    volatile int _state;
    uint8_t _data[8];
    bool _use_interrupt;
    volatile uint32_t _retries;
    volatile uint32_t _missed_converts;

    // i2c transactions
    I2CMasterTxHeader _i2c_header;
    I2CMasterTxSegment _i2c_segments[2];

    friend void EXTI1_IRQHandler(void);
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

#endif
