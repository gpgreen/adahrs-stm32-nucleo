/* ------------------------------------------------------------------------------
  File: itg3200.cpp
  Author: Greg Green
  Version: 1.0
  
  Description: Functions for interacting with ITG-3200 gyro sensor
  Communicating with sensor and interrupt registers can be 20MHz for SPI
  Other's should be at 1MHz
------------------------------------------------------------------------------ */ 

#include "itg3200.h"
#include "work_queue.h"

#define	ITG_SLAVE_ADDRESS7	        0xD0

// Register addresses for the ITG3200
#define	ITG_REG_WHOAMI			0x0
#define	ITG_REG_SMPLRT_DIV	        0x15
#define ITG_REG_DLPF_FS                 0x16
#define	ITG_REG_INT_CFG                 0x17
#define	ITG_REG_INT_STATUS              0x1A
#define	ITG_REG_TEMP_OUT_H		0x1B
#define	ITG_REG_TEMP_OUT_L		0x1C
#define	ITG_REG_GYRO_XOUT_H	        0x1D
#define	ITG_REG_GYRO_XOUT_L	        0x1E
#define	ITG_REG_GYRO_YOUT_H	        0x1F
#define	ITG_REG_GYRO_YOUT_L	        0x20
#define	ITG_REG_GYRO_ZOUT_H	        0x21
#define	ITG_REG_GYRO_ZOUT_L	        0x22
#define	ITG_REG_PWR_MGMT		0x3E

// ----------------------------------------------------------------------------

ITG3200::ITG3200(I2C* bus)
    : _bus(bus), _state(0)
{
    _i2c_header.clock_speed = 0;
    _i2c_header.first = &_i2c_segments[0];
    _i2c_header.slave_address = ITG_SLAVE_ADDRESS7;
}

void ITG3200::begin(int16_t* sign_map, int* axis_map, int* bias,
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

    // device reset
    _data[0] = ITG_REG_PWR_MGMT;
    _data[1] = 0x80;
    // full resolution, 16g
    //_data[2] = ADXL_DATA_FORMAT;
    //_data[3] = 0x0b;
    // select 100 Hz output data rate
    //_data[4] = ADXL_BW_RATE;
    //_data[5] = 0x0a;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitWithStop, &_data[0], 2, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], TransmitWithStop, &_data[2], 2, &_i2c_segments[2]);
    _bus->init_segment(&_i2c_segments[2], TransmitWithStop, &_data[4], 2, nullptr);
    
    _bus->send_receive(&_i2c_header, &ITG3200::bus_callback, this);
}

bool ITG3200::start_get_sensor_data()
{
    if (_state != 2)
        return false;

    // set read data register
    _data[0] = ITG_REG_TEMP_OUT_H;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitNoStop, &_data[0], 1, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], ReceiveWithStop, &_data[0], 8, nullptr);

    _bus->send_receive(&_i2c_header, &ITG3200::bus_callback, this);

    return true;
}

bool ITG3200::sensor_data_received()
{
    return _state == 3;
}

void ITG3200::get_sensor_data()
{
    int x = _axis_map[0];
    int y = _axis_map[1];
    int z = _axis_map[2];
    
    // data is now in the buffer, do conversions
    _raw_gyro[0] = static_cast<uint16_t>((_data[1] << 8) | _data[0]);
    _raw_gyro[1] = static_cast<uint16_t>((_data[3] << 8) | _data[2]);
    _raw_gyro[2] = static_cast<uint16_t>((_data[5] << 8) | _data[4]);

    _corrected_gyro[0] = _sign_map[0] * _raw_gyro[x] - _bias[x];
    _corrected_gyro[1] = _sign_map[1] * _raw_gyro[y] - _bias[y];
    _corrected_gyro[2] = _sign_map[2] * _raw_gyro[z] - _bias[z];
    
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
void ITG3200::bus_callback(void *data)
{
    ITG3200* adxl = reinterpret_cast<ITG3200*>(data);
    
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

#if 0
/*******************************************************************************
* Function Name  : initializeITG
* Input          : None
* Output         : uint8_t* status_flag
* Return         : 0 if success, 1 if fail
* Description    : Initializes the ITG3400 rate gyro
						 
*******************************************************************************/
int32_t initializeITG( uint8_t* status_flag )
{
    uint8_t txBuf[4];

    // per the datasheet, to use SPI, do the following sequence:
    //  1 set DEVICE_RESET
    //  2 wait 100ms
    //  3 set GYRO_RST = TEMP_RST = 1
    //  4 wait 100ms
    txBuf[0] = 0x80;
    spi_write( ITG_REG_PWR_MGMT, txBuf, 1 );

    DelayMs( 100 );

    txBuf[0] = 0x5;
    spi_write( ITG_REG_SIGNAL_PATH_RESET, txBuf, 1 );

    DelayMs( 100 );

    // make sure i2c is disabled
    txBuf[0] = 0x10;            /* i2c slave module reset */
    txBuf[1] = 0x1;             /* auto select the best clock */
    spi_write( ITG_REG_USER_CTRL, txBuf, 2 );
    
    txBuf[0] = 0x00;     // No sample rate division (update registers with most recent data as often as possible)
    txBuf[1] = 0x4;      // FIFO_MODE=0, FSYNC-disabled Internal LPF at 20 Hz
    txBuf[2] = 0x8;      // +/- 500 deg/s full-scale range

    return spi_write( ITG_REG_SMPLRT_DIV, txBuf, 3 );

}

/*******************************************************************************
* Function Name  : getITGData
* Input          : None
* Output         : uint8_t* i2cBuf
* Return         : 0 if success, 1 if fail
* Description    : 
*******************************************************************************/
int32_t getITGData( )
{
    return spi_read( ITG_REG_TEMP_OUT_H, g_spiRxBuf, 8 );
}
#endif
