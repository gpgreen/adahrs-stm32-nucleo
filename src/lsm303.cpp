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
    // does nothing else
}

void LSM303::begin(uint8_t /*priority*/, uint8_t /*subpriority*/)
{
    _state = 1;
    // normal power mode, 50Hz update rate, all channels enabled
    _data[0] = LSM_CTRL_REG1_A;
    _data[1] = 0x27;
    _bus->send_receive(LSM_SLAVE_ADDRESS7, I2C::TransmitWithStop,
                       _data, 2,
                       &LSM303::bus_callback, this);
}

bool LSM303::start_get_sensor_data()
{
    if (_state != 5)
        return false;
    // set read data register to beginning of data, send that
    _data[0] = LSM_OUT_X_L | 0x80;
    _bus->send_receive(LSM_SLAVE_ADDRESS7, I2C::TransmitNoStop,
                       _data, 1,
                       &LSM303::bus_callback, this);
    return true;
}

bool LSM303::sensor_data_received()
{
    return _state == 7;
}

void LSM303::get_sensor_data(uint8_t* buf)
{
    for (int i=0; i<6; i++)
        buf[i] = _data[i];
    _state = 5;
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
 * state 3 - completion of 3rd control register, start i2c for 4th register 
 *       -> 4
 * state 4 - completion of 4th control register, transition to data ready state
 *       -> 5
 * state 5 - completion of data read register write, start i2c for data read
 *       -> 6
 * state 6 - completion of data read
 *       -> 7
 */
void LSM303::bus_callback(void *data)
{
    LSM303* lsm = reinterpret_cast<LSM303*>(data);

    I2C::I2CTransfer txfr = I2C::TransmitWithStop;
    int buflen = 0;
    
    if (lsm->_state == 1)
    {
        // set state to 2
        lsm->_state = 2;
        // no high-pass filter, no manual reboot
        lsm->_data[0] = LSM_CTRL_REG2_A;
        lsm->_data[1] = 0x00;
        buflen = 2;
    }
    else if (lsm->_state == 2)
    {
        // set state to 3
        lsm->_state = 3;
        // Active-low interrupts, data ready routed to interrupt pin 1
        lsm->_data[0] = LSM_CTRL_REG3_A;
        lsm->_data[1] = 0x82;
        buflen = 2;
    }
    else if (lsm->_state == 3)
    {
        // set state to 4
        lsm->_state = 4;
        // 2g full-scale range
        lsm->_data[0] = LSM_CTRL_REG4_A;
        lsm->_data[1] = 0x00;
        buflen = 2;
    }
    else if (lsm->_state == 4)
    {
        // set state to 5, completed initialization
        lsm->_state = 5;
    }
    else if (lsm->_state == 5)
    {
        // set state to 6, read out the data
        lsm->_state = 6;
        txfr = I2C::ReceiveWithStop;
        buflen = 6;
    }
    else if (lsm->_state == 6)
    {
        lsm->_state = 7;
    }

    // now do the i2c if ready, put on work_queue if not
    if (buflen != 0 &&
        !lsm->_bus->send_receive(LSM_SLAVE_ADDRESS7, txfr,
                                 lsm->_data, buflen,
                                 &LSM303::bus_callback, lsm))
    {
        lsm->_state--;
        g_work_queue.add_work_irq(&LSM303::bus_callback, lsm);
    }

}

// ----------------------------------------------------------------------------

