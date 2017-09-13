/* ------------------------------------------------------------------------------
  File: itg3200.cpp
  Author: Greg Green
  Version: 1.0
  
  Description: Functions for interacting with ITG-3200 gyro sensor
  Communicating with sensor and interrupt registers can be 20MHz for SPI
  Other's should be at 1MHz
------------------------------------------------------------------------------ */ 

#include "itg3200.h"
#include "stm32_delaytimer.h"
#include "work_queue.h"

// ----------------------------------------------------------------------------
#define	ITG_SLAVE_ADDRESS7	        0xD0
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
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

// static device pointer, used in interrupt handler, if we want
// more than one device, need to change this
static ITG3200* s_device_0;

// ----------------------------------------------------------------------------

ITG3200::ITG3200(I2C* bus)
    : _bus(bus), _state(0)
{
    _i2c_header.clock_speed = 0;
    _i2c_header.first = &_i2c_segments[0];
    _i2c_header.slave_address = ITG_SLAVE_ADDRESS7;
    s_device_0 = this;
}

void ITG3200::begin(int16_t* sign_map, int* axis_map,
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
    
    // Configure PA1 as input pull down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // enable interrupt handler
    configure_nvic(priority, subpriority);
    
    // configure EXTI Line 1 as interrupt channel
    EXTI_ClearITPendingBit(EXTI_Line1);
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable external interrupt 1
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    first_stage_init(this);
}

void ITG3200::configure_nvic(uint8_t priority, uint8_t subpriority)
{
    // enable the IRQ
    NVIC_InitTypeDef NVIC_InitStructure;

    // enable the DMA Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ITG3200::first_stage_init(void* data)
{
    ITG3200* gyro = reinterpret_cast<ITG3200*>(data);

    // device reset
    gyro->_data[0] = ITG_REG_PWR_MGMT;
    gyro->_data[1] = 0x80;
    
    // setup i2c transfer
    gyro->_i2c_header.first = &(gyro->_i2c_segments[0]);
    gyro->_bus->init_segment(&(gyro->_i2c_segments[0]), TransmitWithStop,
                             &(gyro->_data[0]), 2, nullptr);

    if (!gyro->_bus->send_receive(&(gyro->_i2c_header), ITG3200::bus_callback, gyro))
    {
        g_work_queue.add_work_irq(ITG3200::first_stage_init, gyro);
    }
    else
    {
        // delay for 100ms
        delaytimer.sleep(100);
    }
}

void ITG3200::second_stage_init(void* data)
{
    ITG3200* gyro = reinterpret_cast<ITG3200*>(data);

    // select x gyro clock source
    gyro->_data[0] = ITG_REG_PWR_MGMT;
    gyro->_data[1] = 0x01;
    // auto-incrementing registers
    gyro->_data[2] = ITG_REG_SMPLRT_DIV;
    gyro->_data[3] = 0x07;     // sample rate division is 7+1 = 125Hz, 8ms per sample
    gyro->_data[4] = 0x1a;     // ITG_REG_DLPF_FS, FS_SEL=3 (2000deg/s),
                               //   DLPF_CFG=2, 98Hz band pass, 1kHz internal sample rate
    gyro->_data[5] = 0x11;     // ACTIVE_HIGH, DRIVE PUSH-PULL, ITG_REG_INT_CFG, RAW_RDY_EN,
                               //   INT_ANYRD_2CLEAR

    // setup i2c transfer
    gyro->_i2c_header.first = &(gyro->_i2c_segments[0]);
    gyro->_bus->init_segment(&(gyro->_i2c_segments[0]), TransmitWithStop,
                             &(gyro->_data[0]), 2, &(gyro->_i2c_segments[1]));
    gyro->_bus->init_segment(&(gyro->_i2c_segments[1]), TransmitWithStop,
                             &(gyro->_data[2]), 4, nullptr);
    
    if (!gyro->_bus->send_receive(&(gyro->_i2c_header), ITG3200::bus_callback, gyro))
    {
        g_work_queue.add_work_irq(ITG3200::second_stage_init, gyro);
    }
}

// static function to call start_get_sensor_data, will add to work
// queue if not ready to go
void ITG3200::get_data_trigger(void* data)
{
    ITG3200* gyro = reinterpret_cast<ITG3200*>(data);

    if (gyro->_state == 10 && !gyro->start_get_sensor_data())
    {
        g_work_queue.add_work_irq(ITG3200::get_data_trigger, gyro);
    }
}

// go get new sensor data, returns false if not ready to do so
bool ITG3200::start_get_sensor_data()
{
    if (_state != 10)
        return false;

    // set read data register
    _data[0] = ITG_REG_TEMP_OUT_H;
    
    // setup i2c transfer
    _i2c_header.first = &_i2c_segments[0];
    _bus->init_segment(&_i2c_segments[0], TransmitNoStop, &_data[0], 1, &_i2c_segments[1]);
    _bus->init_segment(&_i2c_segments[1], ReceiveWithStop, &_data[0], 8, nullptr);

    if (!_bus->send_receive(&_i2c_header, ITG3200::bus_callback, this))
    {
        return false;
    }

    return true;
}

bool ITG3200::sensor_data_received()
{
    return _state == 11;
}

void ITG3200::correct_sensor_data()
{
    int x = _axis_map[0];
    int y = _axis_map[1];
    int z = _axis_map[2];
    
    // data is now in the buffer, do conversions
    _raw_gyro[0] = static_cast<int16_t>((_data[1] << 8) | _data[0]);
    _raw_gyro[1] = static_cast<int16_t>((_data[3] << 8) | _data[2]);
    _raw_gyro[2] = static_cast<int16_t>((_data[5] << 8) | _data[4]);

    _corrected_gyro[0] = _sign_map[0] * _raw_gyro[x];
    _corrected_gyro[1] = _sign_map[1] * _raw_gyro[y];
    _corrected_gyro[2] = _sign_map[2] * _raw_gyro[z];
    
    _state = 10;
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
    ITG3200* gyro = reinterpret_cast<ITG3200*>(data);
    
    if (gyro->_state == 1)
    {
        // set state to 2, do second stage of initialization
        gyro->_state = 2;
        second_stage_init(gyro);
    }
    else if (gyro->_state == 2)
    {
        // set state to 20, initialization complete
        gyro->_state = 10;
    }
    else if (gyro->_state == 10)
    {
        // set state to 11, data has been retrieved
        gyro->_state = 11;
    }
}

// ----------------------------------------------------------------------------

/**
 * Interrupt handler for EXTI1
 * this is triggered when gyros have data to read
 */
void EXTI1_IRQHandler(void)
{	 
    // Check for interrupt from accelerometer
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {		  
        EXTI_ClearITPendingBit(EXTI_Line1);
        ITG3200::get_data_trigger(s_device_0);
    }
	 
}

// ----------------------------------------------------------------------------

