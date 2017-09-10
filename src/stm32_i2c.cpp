/*
 * stm32_i2c.cpp
 *
 *  Created on: Sep 2, 2017
 *      Author: ggreen
 *
 */

#include <string.h>
#include "stm32_i2c.h"
#include "stm32_dma.h"
#include "work_queue.h"

// ----------------------------------------------------------------------------
// flags used to control transfers

#define I2C_RECEIVE                     0x00000001
#define I2C_GENERATE_STOP               0x00000002
#define I2C_WAITING_FOR_ADDR            0x00000004
#define I2C_WAITING_FOR_DMA             0x00000008

// ----------------------------------------------------------------------------

I2C::I2C(int device_no)
    : _devno(device_no), _tx_dma(nullptr), _rx_dma(nullptr),
      _data_buffer(nullptr), _buffer_len(0), _flags(0), _address(0),
	  _irqno(0), _tx_busy(false), _alt_func(false),
      _send_completion_fn(nullptr), _send_completion_data(nullptr)
{
    if (device_no == 1)
    {
#ifdef DMA1_CHANNEL6_USED
        _tx_dma = &DMA1Channel6;
#endif
#ifdef DMA1_CHANNEL7_USED
        _rx_dma = &DMA1Channel7;
#endif
        _i2c = I2C1;
        _irqno = I2C1_EV_IRQn;
    }
    else if (device_no == 2)
    {
#ifdef DMA1_CHANNEL4_USED
        _tx_dma = &DMA1Channel4;
#endif
#ifdef DMA1_CHANNEL5_USED
        _rx_dma = &DMA1Channel5;
#endif
        _i2c = I2C2;
        _irqno = I2C2_EV_IRQn;
    }
    else
    {
        while (1)
            ;
    }
}

// ----------------------------------------------------------------------------

void I2C::begin(bool use_alternate, uint8_t priority, uint8_t subpriority)
{
    _alt_func = use_alternate;
    
    // enable the I2C pins
    uint16_t sda_pin = 0;
    uint16_t scl_pin = 0;
    GPIO_TypeDef* pinport = nullptr;

    if (_devno == 1)
    {
        if (!_alt_func)
        {
            sda_pin = GPIO_Pin_7;
            scl_pin = GPIO_Pin_6;
            pinport = GPIOB;
        }
        else 
        {
            sda_pin = GPIO_Pin_9;
            scl_pin = GPIO_Pin_8;
            pinport = GPIOB;
        }
    }
    else if (_devno == 2)
    {
        if (!_alt_func) {
            sda_pin = GPIO_Pin_11;
            scl_pin = GPIO_Pin_10;
            pinport = GPIOB;
        }
        else
        {
            while(1);
        }
    }

    // if using alternate, do the remap
    if (_alt_func) {
        GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    }
    
    // configure the pins
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure I2C SCL as alternate function open drain
    GPIO_InitStructure.GPIO_Pin = scl_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(pinport, &GPIO_InitStructure);

    // Configure I2C SDA as alternate function open drain
    GPIO_InitStructure.GPIO_Pin = sda_pin;
    GPIO_Init(pinport, &GPIO_InitStructure);

    /* I2C is configured as follows:
       - clock 200 KHz
       - i2c mode
       - Duty Cycle 2
       - Ack Enabled
       - 7bit address
    */
    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);

    I2C_InitStructure.I2C_ClockSpeed = 40; // 8MHz / 40 / 2 = 100 kHz
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    // configure interrupt vector
    configure_nvic(priority, subpriority);

    // configure I2C
    I2C_DeInit(_i2c);
    I2C_Init(_i2c, &I2C_InitStructure);

    // enable I2C
    I2C_Cmd(_i2c, ENABLE);
}

void I2C::configure_nvic(uint8_t priority, uint8_t subpriority)
{

    // enable the Event IRQ
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = _irqno;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

bool I2C::send_receive(uint8_t address, I2CTransfer txfr, uint8_t* databuf, int buflen,
                       void (*completed_fn)(void*), void* data)
{
    if (_tx_busy)
        return false;

    if (txfr == TransmitNoStop)
        _flags = 0;
    else if (txfr == TransmitWithStop)
        _flags = I2C_GENERATE_STOP;
    else if (txfr == ReceiveNoStop)
        _flags = I2C_RECEIVE;
    else if (txfr == ReceiveWithStop)
        _flags = I2C_RECEIVE | I2C_GENERATE_STOP;
    
    _address = address;
    _data_buffer = databuf;
    _buffer_len = buflen;
    _send_completion_fn = completed_fn;
    _send_completion_data = data;

    tx_start(false);

    return true;
}

void I2C::tx_start_irq(void* data)
{
    I2C* i2c = reinterpret_cast<I2C*>(data);
    i2c->tx_start(true);
}

void I2C::tx_start(bool in_irq)
{
    // check for legal starting conditions, not busy, or busy and waiting for DMA
    if (_buffer_len <= 0)
        return;
    if ((_tx_busy == true && ((_flags & I2C_WAITING_FOR_DMA) != I2C_WAITING_FOR_DMA)))
        return;

    _tx_busy = true;

    bool receive = (_flags & I2C_RECEIVE) == I2C_RECEIVE;

    // Configure the DMA controller to make the transfer
    DMA_InitTypeDef DMA_Init;

    // Configure the tx DMA controller to make the transfer
    if (_devno == 1)
        DMA_Init.DMA_PeripheralBaseAddr = (uint32_t) &(I2C1->DR);
    else if (_devno == 2)
        DMA_Init.DMA_PeripheralBaseAddr = (uint32_t) &(I2C2->DR);
    DMA_Init.DMA_MemoryBaseAddr = (uint32_t) _data_buffer;
    // sending or receiving
    DMA_Init.DMA_DIR = receive ? DMA_DIR_PeripheralSRC : DMA_DIR_PeripheralDST;
    DMA_Init.DMA_BufferSize = _buffer_len;
    DMA_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_Init.DMA_Mode = DMA_Mode_Normal;
    DMA_Init.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init.DMA_M2M = DMA_M2M_Disable;

    // make sure we can use the DMA
    DMA* which_dma = receive ? _rx_dma : _tx_dma;
    
    if (!which_dma->start(&DMA_Init, I2C::dma_complete, this))
    {
        _flags |= I2C_WAITING_FOR_DMA;
        if (in_irq)
            g_work_queue.add_work_irq(I2C::tx_start_irq, this);
        else
            g_work_queue.add_work(I2C::tx_start_irq, this);
        return;
    }
    _flags &= ~(I2C_WAITING_FOR_DMA);
    
    if (receive)
    {
        // Enable LAST bit in I2C_CR2 register so that a NACK is generated
        // on the last data read
        I2C_DMALastTransferCmd(_i2c, ENABLE);
    }
    
    // generate start condition and wait for response
    I2C_GenerateSTART(_i2c, ENABLE);
    wait_for_event(I2C_EVENT_MASTER_MODE_SELECT);

    // set flags to wait for address
    _flags |= I2C_WAITING_FOR_ADDR;
    
    // send i2c slave address
    I2C_Send7bitAddress(_i2c, _address, receive ?
                        I2C_Direction_Receiver : I2C_Direction_Transmitter);
    
    // wait for ADDR interrupt
    I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
}

// helper function to endlessly loop until event is received
void I2C::wait_for_event(uint32_t event)
{
    while (I2C_CheckEvent(_i2c, event) != SUCCESS);
}

// called when a DMA transfer is complete
void I2C::dma_complete(void* data)
{
    I2C* i2c = reinterpret_cast<I2C*>(data);

    // disable DMA requests
    I2C_DMACmd(i2c->_i2c, DISABLE);

    // if receiving, close out
    if ((i2c->_flags & I2C_RECEIVE) == I2C_RECEIVE)
    {
        i2c->priv_rx_complete();
    }
    // if transmitting, then enable interrupt to wait for last byte transmission
    else
    {
        I2C_ITConfig(i2c->_i2c, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
    }
}

// when all bytes have been received, this is called
void I2C::priv_rx_complete()
{
    _tx_busy = false;
    if ((_flags & I2C_GENERATE_STOP) == I2C_GENERATE_STOP) {
        I2C_GenerateSTOP(_i2c, ENABLE);
    }
    // trigger the callback if given
    if (_send_completion_fn != nullptr) {
        _send_completion_fn(_send_completion_data);
    }
}

// when all bytes have been transmitted, this is called
void I2C::priv_tx_complete()
{
    _tx_busy = false;
    if ((_flags & I2C_GENERATE_STOP) == I2C_GENERATE_STOP) {
        I2C_GenerateSTOP(_i2c, ENABLE);
    }
    // trigger the callback if given
    if (_send_completion_fn != nullptr) {
        _send_completion_fn(_send_completion_data);
    }
}

#ifdef I2C1_USED

I2C i2c1(1);

void I2C1_EV_IRQHandler(void)
{
    // first disable interrupts
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);

    // has the address been acknowledged?
    if ((i2c1._flags & I2C_WAITING_FOR_ADDR) == I2C_WAITING_FOR_ADDR)
    {
        if ((i2c1._flags & I2C_RECEIVE) == I2C_RECEIVE) {
            i2c1.wait_for_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
        }
        else
        {
            i2c1.wait_for_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
        }
        // i2c dma request enabled
        I2C_DMACmd(I2C1, ENABLE);
        i2c1._flags &= ~(I2C_WAITING_FOR_ADDR);
    }
    // if not waiting for the address, then we are waiting for last byte to finish transmission
    else if ((i2c1._flags & I2C_RECEIVE) != I2C_RECEIVE)
    {
        // check the event
        i2c1.wait_for_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        
        // execute callback
        i2c1.priv_tx_complete();
    }
}

#endif

#ifdef I2C2_USED

I2C i2c2(2);

void I2C2_EV_IRQHandler(void)
{
    // first disable interrupts
    I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, DISABLE);

    // has the address been acknowledged?
    if ((i2c2._flags & I2C_WAITING_FOR_ADDR) == I2C_WAITING_FOR_ADDR)
    {
        if ((i2c2._flags & I2C_RECEIVE) == I2C_RECEIVE) {
            i2c2.wait_for_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
        }
        else
        {
            i2c2.wait_for_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
        }
        // i2c dma request enabled
        I2C_DMACmd(I2C2, ENABLE);
        i2c2._flags &= ~(I2C_WAITING_FOR_ADDR);
    }
    // if not waiting for the address, then we are waiting for last byte to finish transmission
    else if ((i2c2._flags & I2C_RECEIVE) != I2C_RECEIVE)
    {
        // check the event
        i2c2.wait_for_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        
        // execute callback
        i2c2.priv_tx_complete();
    }
}

#endif

// ----------------------------------------------------------------------------
