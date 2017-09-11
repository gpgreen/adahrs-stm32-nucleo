/*
 * adxl345.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: ggreen
 */

#include "adxl345.h"
#include "work_queue.h"

// ----------------------------------------------------------------------------

#define	ADXL_SLAVE_ADDRESS7             (0xa6)

/* registers */
#define ADXL_DEVID                      0x00
#define ADXL_THRESH_TAP                 0x1d
#define ADXL_OFSX                       0x1e
#define ADXL_OFSY                       0x1f
#define ADXL_OFSZ                       0x20
#define ADXL_DUR                        0x21
#define ADXL_LATENT                     0x22
#define ADXL_WINDOW                     0x23
#define ADXL_THRESH_ACT                 0x24
#define ADXL_THRESH_INACT               0x25
#define ADXL_TIME_INACT                 0x26
#define ADXL_ACT_INACT_CTL              0x27
#define ADXL_THRESH_FF                  0x28
#define ADXL_TIME_FF                    0x29
#define ADXL_TAP_AXES                   0x2a
#define ADXL_ACT_TAP_STATUS             0x2b
#define ADXL_BW_RATE                    0x2c
#define ADXL_POWER_CTL                  0x2d
#define ADXL_INT_ENABLE                 0x2e
#define ADXL_INT_MAP                    0x2f
#define ADXL_INT_SOURCE                 0x30
#define ADXL_DATA_FORMAT                0x31
#define ADXL_DATAX0                     0x32
#define ADXL_DATAX1                     0x33
#define ADXL_DATAY0                     0x34
#define ADXL_DATAY1                     0x35
#define ADXL_DATAZ0                     0x36
#define ADXL_DATAZ1                     0x37
#define ADXL_FIFO_CTL                   0x38
#define ADXL_FIFO_STATUS                0x39

// ----------------------------------------------------------------------------

ADXL345::ADXL345(I2C* bus)
    : _bus(bus), _state(0)
{
    _i2c_header.clock_speed = 0;
    _i2c_header.first = &_i2c_segments[0];
    _i2c_header.slave_address = ADXL_SLAVE_ADDRESS7;
}

void ADXL345::begin(int16_t* sign_map, int* axis_map, int* bias,
                    uint8_t /*priority*/, uint8_t /*subpriority*/)
{
    _sign_map[0] = sign_map[0];
    _sign_map[1] = sign_map[1];
    _sign_map[2] = sign_map[2];
    _axis_map[0] = axis_map[0];
    _axis_map[1] = axis_map[1];
    _axis_map[2] = axis_map[2];
    _bias[0] = bias[0];
    _bias[1] = bias[1];
    _bias[2] = bias[2];

    _state = 1;

    // measurement mode
    _data[0] = ADXL_POWER_CTL;
    _data[1] = 0x8;
    // full resolution, 16g
    _data[2] = ADXL_DATA_FORMAT;
    _data[3] = 0x0b;
    // select 100 Hz output data rate
    _data[4] = ADXL_BW_RATE;
    _data[5] = 0x0a;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitWithStop, &_data[0], 2, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], TransmitWithStop, &_data[2], 2, &_i2c_segments[2]);
    _bus->init_segment(&_i2c_segments[2], TransmitWithStop, &_data[4], 2, nullptr);
    
    _bus->send_receive(&_i2c_header, &ADXL345::bus_callback, this);
}

bool ADXL345::start_get_sensor_data()
{
    if (_state != 2)
        return false;

    // set read data register
    _data[0] = ADXL_DATAX0;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitNoStop, &_data[0], 1, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], ReceiveWithStop, &_data[0], 6, nullptr);

    _bus->send_receive(&_i2c_header, &ADXL345::bus_callback, this);

    return true;
}

bool ADXL345::sensor_data_received()
{
    return _state == 3;
}

void ADXL345::get_sensor_data()
{
    int x = _axis_map[0];
    int y = _axis_map[1];
    int z = _axis_map[2];
    
    // data is now in the buffer, do conversions
    _raw_accel[0] = static_cast<uint16_t>((_data[1] << 8) | _data[0]);
    _raw_accel[1] = static_cast<uint16_t>((_data[3] << 8) | _data[2]);
    _raw_accel[2] = static_cast<uint16_t>((_data[5] << 8) | _data[4]);

    _corrected_accel[0] = _sign_map[0] * _raw_accel[x] - _bias[x];
    _corrected_accel[1] = _sign_map[1] * _raw_accel[y] - _bias[y];
    _corrected_accel[2] = _sign_map[2] * _raw_accel[z] - _bias[z];
    
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
void ADXL345::bus_callback(void *data)
{
    ADXL345* adxl = reinterpret_cast<ADXL345*>(data);
    
    if (adxl->_state == 1)
    {
        // set state to 2, completed initialization
        adxl->_state = 2;
    }
    else if (adxl->_state == 2)
    {
        // set state to 3, data has been retrieved
        adxl->_state = 3;
    }
}

// ----------------------------------------------------------------------------

