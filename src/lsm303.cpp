/*
 * lsm303.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: ggreen
 */

#include "lsm303.h"
#include "work_queue.h"

// ----------------------------------------------------------------------------

#define	LSM_SLAVE_ADDRESS7	                (0x30)

#define	LSM_CTRL_REG1_A		                0x20
#define	LSM_CTRL_REG2_A		                0x21
#define	LSM_CTRL_REG3_A		                0x22
#define	LSM_CTRL_REG4_A		                0x23

#define	LSM_OUT_X_L				0x28
#define	LSM_OUT_X_H				0x29
#define	LSM_OUT_Y_L				0x2A
#define	LSM_OUT_Y_H				0x2B
#define	LSM_OUT_Z_L				0x2C
#define	LSM_OUT_Z_H				0x2D

// ----------------------------------------------------------------------------

LSM303::LSM303(I2C* bus)
    : _bus(bus), _state(0)
{
    _i2c_header.clock_speed = 100000;
    _i2c_header.first = &_i2c_segments[0];
    _i2c_header.slave_address = LSM_SLAVE_ADDRESS7;
}

void LSM303::begin(uint8_t /*priority*/, uint8_t /*subpriority*/)
{
    _state = 1;

    // normal power mode, 50Hz update rate, all channels enabled
    _data[0] = LSM_CTRL_REG1_A;
    _data[1] = 0x27;
    // no high-pass filter, no manual reboot
    _data[2] = LSM_CTRL_REG2_A;
    _data[3] = 0x00;
    // Active-low interrupts, data ready routed to interrupt pin 1
    _data[4] = LSM_CTRL_REG3_A;
    _data[5] = 0x82;
    // 2g full-scale range
    _data[6] = LSM_CTRL_REG4_A;
    _data[7] = 0x00;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitWithStop, &_data[0], 2, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], TransmitWithStop, &_data[2], 2, &_i2c_segments[2]);
    _bus->init_segment(&_i2c_segments[2], TransmitWithStop, &_data[4], 2, &_i2c_segments[3]);
    _bus->init_segment(&_i2c_segments[3], TransmitWithStop, &_data[6], 2, nullptr);
    
    _bus->send_receive(&_i2c_header, &LSM303::bus_callback, this);
}

bool LSM303::start_get_sensor_data()
{
    if (_state != 2)
        return false;

    // set read data register
    _data[0] = LSM_OUT_X_L | 0x80;

    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitNoStop, &_data[0], 1, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], ReceiveWithStop, &_data[0], 6, nullptr);

    _bus->send_receive(&_i2c_header, &LSM303::bus_callback, this);

    return true;
}

bool LSM303::sensor_data_received()
{
    return _state == 3;
}

void LSM303::get_sensor_data(uint8_t* buf)
{
    for (int i=0; i<6; i++)
        buf[i] = _data[i];

    _state = 2;
}

/**
 * callback for i2c bus, normally called from interrupt context
 * state machine
 *
 * state 0 - state at startup, transitions to 1 when begin is called
 * state 1 - completion of control register setup
 *       -> 2
 * state 2 - completion of data retrieval
 *       -> 3
 */
void LSM303::bus_callback(void *data)
{
    LSM303* lsm = reinterpret_cast<LSM303*>(data);

    if (lsm->_state == 1)
    {
        // set state to 2, completed initialization
        lsm->_state = 2;
    }
    else if (lsm->_state == 2)
    {
        // set state to 3, data has been retrieved
        lsm->_state = 3;
    }
}

// ----------------------------------------------------------------------------

