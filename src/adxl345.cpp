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

// static device pointer, used in interrupt handler, if we want
// more than one device, need to change this
static ADXL345* s_device_0;

// ----------------------------------------------------------------------------

ADXL345::ADXL345(I2C* bus)
    : _bus(bus), _state(0), _retries(0), _missed_converts(0)
{
    _i2c_header.clock_speed = 100000;
    _i2c_header.first = &_i2c_segments[0];
    _i2c_header.slave_address = ADXL_SLAVE_ADDRESS7;
    s_device_0 = this;
}

void ADXL345::begin(int16_t* sign_map, uint8_t* axis_map,
                    uint8_t priority, uint8_t subpriority)
{
    _sign_map[0] = sign_map[0];
    _sign_map[1] = sign_map[1];
    _sign_map[2] = sign_map[2];
    _axis_map[0] = axis_map[0];
    _axis_map[1] = axis_map[1];
    _axis_map[2] = axis_map[2];

    _state = 1;

    // setup interrupt pin
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    // Configure PA0 as input pull down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // enable interrupt handler
    configure_nvic(EXTI0_IRQn, priority, subpriority);
    
    // configure EXTI Line 0 as interrupt channel
    EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable external interrupt 0
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    
    // full resolution, 16g, interrupt ACTIVE_HIGH
    _data[0] = ADXL_DATA_FORMAT;
    _data[1] = 0x0b;
    // select 100 Hz output data rate
    _data[2] = ADXL_BW_RATE;
    _data[3] = 0x0a;
    // place in measurement mode, last
    _data[4] = ADXL_POWER_CTL;
    _data[5] = 0x8;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitWithStop, &_data[0], 2, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], TransmitWithStop, &_data[2], 2, &_i2c_segments[2]);
    _bus->init_segment(&_i2c_segments[2], TransmitWithStop, &_data[4], 2, nullptr);
    
    if (!_bus->send_receive(&_i2c_header, ADXL345::bus_callback, this))
    {
        g_work_queue.add_work_irq(ADXL345::retry_send, this);
    }
}

void ADXL345::init_stage2()
{

    // setup interrupt control registers
    
    // disable interrupts
    _data[0] = ADXL_INT_ENABLE;
    _data[1] = 0x00;
    // set interrupts to pin int1
    _data[2] = ADXL_INT_MAP;
    _data[3] = 0x0;
    // enable DATA_READY interrupt
    _data[4] = ADXL_INT_ENABLE;
    _data[5] = 0x80;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitWithStop, &_data[0], 2, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], TransmitWithStop, &_data[2], 2, &_i2c_segments[2]);
    _bus->init_segment(&_i2c_segments[2], TransmitWithStop, &_data[4], 2, nullptr);
    
    if (!_bus->send_receive(&_i2c_header, ADXL345::bus_callback, this))
    {
        g_work_queue.add_work_irq(ADXL345::retry_send, this);
    }
}

// static function to call start_get_sensor_data, will add to work
// queue if not ready to go
void ADXL345::get_data_trigger(void* data)
{
    ADXL345* adxl = reinterpret_cast<ADXL345*>(data);
    adxl->start_get_sensor_data();
}

// go get new sensor data, returns false if not ready to do so
bool ADXL345::start_get_sensor_data()
{
    // if current state is 10, set it to 11
    // if we fail, try to set it from 12 to 11, which means
    // the data was never converted after retrieval
    // if that fails, then return false
    if (!__sync_bool_compare_and_swap(&_state, 10, 11)) {
        if (!__sync_bool_compare_and_swap(&_state, 12, 11)) {
            return false;
        }
        ++_missed_converts;
    }
    
    // set read data register
    _data[0] = ADXL_DATAX0;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitNoStop, &_data[0], 1, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], ReceiveWithStop, &_data[0], 6, nullptr);

    if (!_bus->send_receive(&_i2c_header, ADXL345::bus_callback, this))
    {
        g_work_queue.add_work_irq(ADXL345::retry_send, this);
    }
    return true;
}

// try to send current i2c transfer setup, if i2c is busy, then schedule it for
// later
void ADXL345::retry_send(void* data)
{
    ADXL345* adxl = reinterpret_cast<ADXL345*>(data);
    ++adxl->_retries;
    if (!adxl->_bus->send_receive(&(adxl->_i2c_header), ADXL345::bus_callback, adxl))
    {
        g_work_queue.add_work_irq(ADXL345::retry_send, adxl);
    }
}

// take set of raw values retrieved from i2c, convert to corrected
// sensor readings, reset state so we can get new data
void ADXL345::correct_sensor_data()
{
    // data is now in the buffer, do conversions
    _raw_accel[0] = static_cast<int16_t>((_data[1] << 8) | _data[0]);
    _raw_accel[1] = static_cast<int16_t>((_data[3] << 8) | _data[2]);
    _raw_accel[2] = static_cast<int16_t>((_data[5] << 8) | _data[4]);

    _corrected_accel[0] = static_cast<int16_t>(_sign_map[0] * _raw_accel[_axis_map[0]]);
    _corrected_accel[1] = static_cast<int16_t>(_sign_map[1] * _raw_accel[_axis_map[1]]);
    _corrected_accel[2] = static_cast<int16_t>(_sign_map[2] * _raw_accel[_axis_map[2]]);
    
    _state = 10;
}

/**
 * callback for i2c bus, normally called from interrupt context
 *
 * state machine
 * state 0 - state at startup, transitions to 1 when begin is called
 * state 1 - completion of first set of control register setups, do second stage init
 *       -> 2
 * state 2 - completion of second set of control register setups
 *       -> 10
 * state 11 - completion of data retrieval
 *       -> 12
 */
void ADXL345::bus_callback(void *data)
{
    ADXL345* adxl = reinterpret_cast<ADXL345*>(data);
    
    if (adxl->_state == 1)
    {
        // set state to 2, next set of setup
        adxl->_state = 2;
        adxl->init_stage2();
    }
    else if (adxl->_state == 2)
    {
        // set state to 10, completed initialization
        adxl->_state = 10;
    }
    else if (adxl->_state == 11)
    {
        // set state to 12, data has been retrieved
        adxl->_state = 12;
    }
}

// ----------------------------------------------------------------------------
/**
 * Interrupt handler for EXTI0
 * this is triggered when accelerometer has data to read
 */
void EXTI0_IRQHandler(void)
{	 
    // Check for interrupt from accelerometer
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {		  
        EXTI_ClearITPendingBit(EXTI_Line0);
        ADXL345::get_data_trigger(s_device_0);
    }
	 
}

// ----------------------------------------------------------------------------

