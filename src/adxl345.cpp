/*
 * adxl345.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: ggreen
 */

#include "adxl345.h"

// ----------------------------------------------------------------------------

#define	ADXL_SLAVE_ADDRESS7	                (0xa6)

/* registers */
#define ADXL_DEVID           0x00
#define ADXL_THRESH_TAP      0x1d
#define ADXL_OFSX            0x1e
#define ADXL_OFSY            0x1f
#define ADXL_OFSZ            0x20
#define ADXL_DUR             0x21
#define ADXL_LATENT          0x22
#define ADXL_WINDOW          0x23
#define ADXL_THRESH_ACT      0x24
#define ADXL_THRESH_INACT    0x25
#define ADXL_TIME_INACT      0x26
#define ADXL_ACT_INACT_CTL   0x27
#define ADXL_THRESH_FF       0x28
#define ADXL_TIME_FF         0x29
#define ADXL_TAP_AXES        0x2a
#define ADXL_ACT_TAP_STATUS  0x2b
#define ADXL_BW_RATE         0x2c
#define ADXL_POWER_CTL       0x2d
#define ADXL_INT_ENABLE      0x2e
#define ADXL_INT_MAP         0x2f
#define ADXL_INT_SOURCE      0x30
#define ADXL_DATA_FORMAT     0x31
#define ADXL_DATAX0          0x32
#define ADXL_DATAX1          0x33
#define ADXL_DATAY0          0x34
#define ADXL_DATAY1          0x35
#define ADXL_DATAZ0          0x36
#define ADXL_DATAZ1          0x37
#define ADXL_FIFO_CTL        0x38
#define ADXL_FIFO_STATUS     0x39

// ----------------------------------------------------------------------------

ADXL345::ADXL345(I2C* bus)
    : _bus(bus), _state(0)
{
    // does nothing else
}

void ADXL345::begin(uint8_t /*priority*/, uint8_t /*subpriority*/)
{
    _state = 1;
    // measurement mode
    _data[0] = ADXL_POWER_CTL;
    _data[1] = 0x8;
    _bus->send_receive(ADXL_SLAVE_ADDRESS7, I2C_GENERATE_STOP, _data, 2,
                       &ADXL345::bus_callback, this);
}

bool ADXL345::start_get_sensor_data()
{
    if (_state != 4)
        return false;
    // set read data register to beginning of data, send that
    _data[0] = ADXL_DATAX0;
    _bus->send_receive(ADXL_SLAVE_ADDRESS7, 0, _data, 1,
                       &ADXL345::bus_callback, this);
    return true;
}

bool ADXL345::sensor_data_received()
{
    return _state == 6;
}

void ADXL345::get_sensor_data(uint8_t* buf)
{
    for (int i=0; i<6; i++)
        buf[i] = _data[i];
    _state = 4;
}
/**
 * callback for i2c bus, normally called from interrupt context
 * state machine
 *
 * state 0 - state at startup, transitions to 1 when begin is called
 * state 1 - completion of 1st control register, start i2c for 2nd register 
 *       -> 2
 * state 2 - completion of 2nd control register, start i2c for 3rd register 
 *       -> 3
 * state 3 - completion of 4th control register, transition to data ready state
 *       -> 4
 * state 4 - completion of data read register write, start i2c for data read
 *       -> 5
 */
void ADXL345::bus_callback(void *data)
{
    ADXL345* adxl = reinterpret_cast<ADXL345*>(data);

    // TODO: this needs to check result of send_receive and push to work_queue
    if (adxl->_state == 1) {
        // set state to 2
        adxl->_state = 2;
        // full resolution, 16g
        adxl->_data[0] = ADXL_DATA_FORMAT;
        adxl->_data[1] = 0x0b;
        adxl->_bus->send_receive(ADXL_SLAVE_ADDRESS7, I2C_GENERATE_STOP, adxl->_data, 2,
                                &ADXL345::bus_callback, adxl);
    }
    else if (adxl->_state == 2) {
        // set state to 3
        adxl->_state = 3;
        // select 100 Hz output data rate
        adxl->_data[0] = ADXL_BW_RATE;
        adxl->_data[1] = 0x0a;
        adxl->_bus->send_receive(ADXL_SLAVE_ADDRESS7, I2C_GENERATE_STOP, adxl->_data, 2,
                                &ADXL345::bus_callback, adxl);
    }
    else if (adxl->_state == 3) {
        // set state to 4, completed initialization
        adxl->_state = 4;
    }
    else if (adxl->_state == 4) {
        // set state to 5, read out the data
        adxl->_state = 5;
        adxl->_bus->send_receive(ADXL_SLAVE_ADDRESS7,
                                I2C_RECEIVE | I2C_GENERATE_STOP,
                                adxl->_data, 6, &ADXL345::bus_callback, adxl);
    }
    else if (adxl->_state == 5)
    {
        adxl->_state = 6;
    }
}

// ----------------------------------------------------------------------------

