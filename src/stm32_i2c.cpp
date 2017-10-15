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
#include "stm32_delaytimer.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------
// flags used to control transfers

#define I2C_RECEIVE                     0x00000001
#define I2C_GENERATE_STOP               0x00000002
#define I2C_HAVE_DMA                    0x00000004
#define I2C_WAITING_FOR_START           0x00000008
#define I2C_WAITING_FOR_ADDR            0x00000010

// ----------------------------------------------------------------------------

I2C::I2C(int device_no)
    : _devno(device_no), _tx_busy(0),
      _tx_dma(nullptr), _rx_dma(nullptr), _hdr(nullptr),
      _irqno(0), _alt_func(false), _restart_stuck(false),
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
        UsageFault_Handler();
    }
}

// ----------------------------------------------------------------------------

void I2C::begin(bool use_alternate, uint8_t priority, uint8_t subpriority)
{
    _alt_func = use_alternate;
    
    // start peripheral clocks
    enable_clocks();
    
    // disable the I2C if enabled
    I2C_Cmd(_i2c, DISABLE);
    delaytimer.delay(100);
    
    // enable the I2C pins
    setup_pins();

    /* I2C is configured as follows:
       - clock 100 KHz
       - i2c mode
       - Duty Cycle 2
       - Ack Enabled
       - 7bit address
    */
    I2C_StructInit(&_init);

    _init.I2C_ClockSpeed = 100000;
    _init.I2C_Mode = I2C_Mode_I2C;
    _init.I2C_DutyCycle = I2C_DutyCycle_2;
    _init.I2C_OwnAddress1 = 0;
    _init.I2C_Ack = I2C_Ack_Enable;
    _init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    // configure interrupt vector
    configure_nvic(_irqno, priority, subpriority);

    // configure I2C
    I2C_DeInit(_i2c);
    I2C_Init(_i2c, &_init);

    // enable I2C
    I2C_Cmd(_i2c, ENABLE);
}

// enable all clocks for needed peripherals
void I2C::enable_clocks()
{
    // enable dma clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    if (_devno == 1)
    {
        if (_alt_func)
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    }
    else if (_devno == 2)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    }
}

void I2C::get_port_pins(GPIO_TypeDef** port, uint16_t& sda, uint16_t& scl)
{
    if (_devno == 1)
    {
        if (!_alt_func)
        {
            sda = GPIO_Pin_7;
            scl = GPIO_Pin_6;
            *port = GPIOB;
        }
        else 
        {
            sda = GPIO_Pin_9;
            scl = GPIO_Pin_8;
            *port = GPIOB;
        }
    }
    else if (_devno == 2)
    {
        if (!_alt_func) {
            sda = GPIO_Pin_11;
            scl = GPIO_Pin_10;
            *port = GPIOB;
        }
        else
        {
            UsageFault_Handler();
        }
    }
}

void I2C::setup_pins()
{
    // determine the I2C pins
    uint16_t sda_pin = 0;
    uint16_t scl_pin = 0;
    GPIO_TypeDef* pinport = nullptr;

    get_port_pins(&pinport, sda_pin, scl_pin);

    // if using alternate, do the remap
    if (_alt_func)
    {
        GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
    }
    
    // configure the pins
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure I2C SCL, SDA as alternate function open drain
    GPIO_InitStructure.GPIO_Pin = scl_pin | sda_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(pinport, &GPIO_InitStructure);
}

// errata 2.14.7 describes how peripheral won't enter a START
// condition, usually after a MCU power-on reset. This happens
// randomly per the errata. This procedure resets the I2C analog
// filters to allow peripheral to enter the START condition.
void I2C::errata_2_14_7()
{
    // Step 1 disable
    I2C_Cmd(_i2c, DISABLE);
    delaytimer.delay(100);

    // get the I2C pins from driver config info
    uint16_t sda_pin = 0;
    uint16_t scl_pin = 0;
    GPIO_TypeDef* pinport = nullptr;

    get_port_pins(&pinport, sda_pin, scl_pin);
    
    // if using alternate, undo the remap
    if (_alt_func)
    {
        GPIO_PinRemapConfig(GPIO_Remap_I2C1, DISABLE);
    }
    
    // configure the pins as Output Open Drain

    GPIO_InitTypeDef GPIO_InitStructure;

    // Step 2 Configure I2C SCL, SDA as output open drain
    GPIO_InitStructure.GPIO_Pin = scl_pin | sda_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(pinport, &GPIO_InitStructure);

    // high level on both pins
    GPIO_WriteBit(pinport, scl_pin, Bit_SET);
    GPIO_WriteBit(pinport, sda_pin, Bit_SET);

    // Step 3 check for high level in IDR
    while (GPIO_ReadInputDataBit(pinport, scl_pin) == 0);
    while (GPIO_ReadInputDataBit(pinport, sda_pin) == 0);

    // Step 4 low level on sda
    GPIO_WriteBit(pinport, sda_pin, Bit_RESET);

    // Step 5 check for low level in sda IDR
    while (GPIO_ReadInputDataBit(pinport, sda_pin) == sda_pin);

    // Step 6 low level on scl
    GPIO_WriteBit(pinport, scl_pin, Bit_RESET);

    // Step 7 check for low level in scl IDR
    while (GPIO_ReadInputDataBit(pinport, scl_pin) == scl_pin);

    // Step 8 scl high level
    GPIO_WriteBit(pinport, scl_pin, Bit_SET);

    // Step 9 check for scl high
    while (GPIO_ReadInputDataBit(pinport, scl_pin) == 0);

    // Step 10 sda high level
    GPIO_WriteBit(pinport, sda_pin, Bit_SET);

    // Step 11 check sda high level
    while (GPIO_ReadInputDataBit(pinport, sda_pin) == 0);

    // Step 12 configure SCL and SDA to peripheral
    setup_pins();

    // Step 13 enable software reset
    I2C_SoftwareResetCmd(_i2c, ENABLE);

    delaytimer.delay(1000);

    // Step 14 clear software reset
    I2C_SoftwareResetCmd(_i2c, DISABLE);

    // Step 15 enable i2c
    I2C_Cmd(_i2c, ENABLE);

    delaytimer.delay(100);
}

bool I2C::send_receive(I2CMasterTxHeader* hdr,
                       void (*completed_fn)(void*), void* data)
{
    if (hdr->first == nullptr)
        return false;

    // set _tx_busy to true, or return false
    if (!__sync_bool_compare_and_swap(&_tx_busy, 0, 1))
        return false;

    // if the clock speed doesn't match what came before,
    // change it
    if (hdr->clock_speed != _init.I2C_ClockSpeed)
    {
        I2C_Cmd(_i2c, DISABLE);
        _init.I2C_ClockSpeed = hdr->clock_speed;
        I2C_Init(_i2c, &_init);
        I2C_Cmd(_i2c, ENABLE);
    }

    // set data for transmission
    _hdr = hdr;
    _hdr->num_txn_completed = 0;
    _send_completion_fn = completed_fn;
    _send_completion_data = data;

    // start it
    tx_start();

    return true;
}

void I2C::init_segment(I2CMasterTxSegment* segment, I2CTransferType type, uint8_t* databuffer,
                       uint32_t bufferlen, I2CMasterTxSegment* next)
{
    if (type == TransmitNoStop)
        segment->flags = 0;
    else if (type == TransmitWithStop)
        segment->flags = I2C_GENERATE_STOP;
    else if (type == ReceiveNoStop)
        segment->flags = I2C_RECEIVE;
    else if (type == ReceiveWithStop)
        segment->flags = I2C_RECEIVE | I2C_GENERATE_STOP;
    segment->databuf = databuffer;
    segment->buflen = bufferlen;
    segment->next = next;
}

void I2C::tx_start_irq(void* data)
{
    I2C* i2c = reinterpret_cast<I2C*>(data);
    i2c->tx_start();
}

void I2C::tx_start()
{
    // check for legal starting conditions
    if (_hdr->first == nullptr || _hdr->first->buflen == 0
        // if we have already gotten the DMA, no need to run this
        // method again, it may have gotten on the work queue multiple
        // times
        || (_hdr->first->flags & I2C_HAVE_DMA) == I2C_HAVE_DMA)
        return;
    
    I2CMasterTxSegment* seg = _hdr->first;

    bool receive = (seg->flags & I2C_RECEIVE) == I2C_RECEIVE;

    // Configure the DMA controller to make the transfer
    DMA_InitTypeDef DMA_Init;

    // Configure the tx DMA controller to make the transfer
    if (_devno == 1)
        DMA_Init.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(I2C1->DR));
    else if (_devno == 2)
        DMA_Init.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(I2C2->DR));
    DMA_Init.DMA_MemoryBaseAddr = reinterpret_cast<uint32_t>(seg->databuf);
    // sending or receiving
    DMA_Init.DMA_DIR = receive ? DMA_DIR_PeripheralSRC : DMA_DIR_PeripheralDST;
    DMA_Init.DMA_BufferSize = seg->buflen;
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
    	g_work_queue.add_work_irq(I2C::tx_start_irq, this);
        return;
    }
    seg->flags |= I2C_HAVE_DMA;
    
    if (receive)
    {
        // Enable LAST bit in I2C_CR2 register so that a NACK is generated
        // on the last data read
        I2C_DMALastTransferCmd(_i2c, ENABLE);
    }

    // setup the timeout timer
    _timeout.start(_hdr->timeout, I2C::timeout, this, Timer::OneShot);
    
    // generate start condition and wait for response
    seg->flags |= I2C_WAITING_FOR_START;
    I2C_GenerateSTART(_i2c, ENABLE);
    wait_for_event(I2C_EVENT_MASTER_MODE_SELECT);
    seg->flags &= ~(I2C_WAITING_FOR_START);

    // set flags to wait for address
    seg->flags |= I2C_WAITING_FOR_ADDR;
    
    // send i2c slave address
    I2C_Send7bitAddress(_i2c, _hdr->slave_address, receive ?
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
    if ((i2c->_hdr->first->flags & I2C_RECEIVE) == I2C_RECEIVE)
    {
        i2c->priv_rx_complete();
    }
    // if transmitting, then enable interrupt to wait for last byte transmission
    else
    {
        I2C_ITConfig(i2c->_i2c, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
    }
}

// called if the transfer times out
void I2C::timeout(void* data)
{
    I2C* i2c = reinterpret_cast<I2C*>(data);
    // if we are waiting for the peripheral to go into start
    // this may be condition shown in errata, try to run
    // the errata sequence (once)
    if (i2c->_hdr->first->flags & I2C_WAITING_FOR_START)
    {
        if (i2c->_restart_stuck)
        {
            UsageFault_Handler();
        }
        else
        {
            i2c->_restart_stuck = true;
        }
        // run the errata sequence
        i2c->errata_2_14_7();
        // remove the flags, release the DMA, and retry
        i2c->_hdr->first->flags &= ~(I2C_WAITING_FOR_START | I2C_HAVE_DMA);
        if (i2c->_hdr->first->flags & I2C_RECEIVE)
            i2c->_rx_dma->cancel();
        else
            i2c->_tx_dma->cancel();
        i2c->tx_start();
    }
}

// when receive transfer is complete, this is called
void I2C::priv_rx_complete()
{
    if ((_hdr->first->flags & I2C_GENERATE_STOP) == I2C_GENERATE_STOP)
    {
        I2C_GenerateSTOP(_i2c, ENABLE);
    }
    _timeout.cancel();
    ++_hdr->num_txn_completed;
    // advance the segment if there is more
    if (_hdr->first->next != nullptr)
    {
        _hdr->first = _hdr->first->next;
        tx_start();
    }
    // trigger the callback if given
    else if (_send_completion_fn != nullptr)
    {
        _tx_busy = 0;
        _send_completion_fn(_send_completion_data);
    }
}

// when transmit transfer is complete, this is called
void I2C::priv_tx_complete()
{
    if ((_hdr->first->flags & I2C_GENERATE_STOP) == I2C_GENERATE_STOP)
    {
        I2C_GenerateSTOP(_i2c, ENABLE);
    }
    _timeout.cancel();
    ++_hdr->num_txn_completed;
    // advance the segment if there is more
    if (_hdr->first->next != nullptr)
    {
        _hdr->first = _hdr->first->next;
        tx_start();
    }
    // trigger the callback if given
    else if (_send_completion_fn != nullptr)
    {
    	_tx_busy = 0;
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
    if ((i2c1._hdr->first->flags & I2C_WAITING_FOR_ADDR) == I2C_WAITING_FOR_ADDR)
    {
        if ((i2c1._hdr->first->flags & I2C_RECEIVE) == I2C_RECEIVE)
	{
            i2c1.wait_for_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
        }
        else
        {
            i2c1.wait_for_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
        }
        // i2c dma request enabled
        I2C_DMACmd(I2C1, ENABLE);
        i2c1._hdr->first->flags &= ~(I2C_WAITING_FOR_ADDR);
    }
    // if not waiting for the address, then we are waiting for last byte to finish transmission
    else if ((i2c1._hdr->first->flags & I2C_RECEIVE) != I2C_RECEIVE)
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
    if ((i2c2._hdr->first->flags & I2C_WAITING_FOR_ADDR) == I2C_WAITING_FOR_ADDR)
    {
        if ((i2c2._hdr->first->flags & I2C_RECEIVE) == I2C_RECEIVE)
	{
            i2c2.wait_for_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
        }
        else
        {
            i2c2.wait_for_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
        }
        // i2c dma request enabled
        I2C_DMACmd(I2C2, ENABLE);
        i2c2._hdr->first->flags &= ~(I2C_WAITING_FOR_ADDR);
    }
    // if not waiting for the address, then we are waiting for last byte to finish transmission
    else if ((i2c2._hdr->first->flags & I2C_RECEIVE) != I2C_RECEIVE)
    {
        // check the event
        i2c2.wait_for_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        
        // execute callback
        i2c2.priv_tx_complete();
    }
}

#endif

// put a start condition on the i2c bus
static void i2c_start(GPIO_TypeDef* port, uint16_t sda, uint16_t scl)
{
    // i2c_start, SDA 1, SCL hi-z, dly, SDA 1->0 dly, SCL 0, dly
    // SDA hi
    GPIO_WriteBit(port, sda, Bit_SET);
    // SCL hi-z
    GPIO_WriteBit(port, scl, Bit_SET);
    delaytimer.delay(320);
    
    // SDA low
    GPIO_WriteBit(port, sda, Bit_RESET);
    delaytimer.delay(320);

    // SCL lo
    GPIO_WriteBit(port, scl, Bit_RESET);
    delaytimer.delay(320);
}

// put a stop condition on the i2c bus
static void i2c_stop(GPIO_TypeDef* port, uint16_t sda, uint16_t scl)
{
    // i2c_stop, SCL 0, dly, SDA 0, dly, 0->1 on SCL, dly, then 0->1 on SDA

    // SCL lo
    GPIO_WriteBit(port, scl, Bit_RESET);
    delaytimer.delay(320);

    // SDA low
    GPIO_WriteBit(port, sda, Bit_RESET);
    delaytimer.delay(320);
    
    // SCL hi-z
    GPIO_WriteBit(port, scl, Bit_SET);
    delaytimer.delay(320);

    // SDA hi
    GPIO_WriteBit(port, sda, Bit_SET);
}

// function to temporarily disable the I2C port and then run sequence to 
// unstick any slave devices that may be hung in read mode waiting for an
// ACK to come. this recovery sequence consists of a start, >9 clocks and a 
// stop. This is followed again with another start/stop sequence that permits
// state machine logic reset in most slave peripheral chips
bool I2C::bus_recovery()
{
    // set _tx_busy to true, or return false
    if (!__sync_bool_compare_and_swap(&_tx_busy, 0, 1))
        return false;

    // determine the I2C pins
    uint16_t sda_pin = 0;
    uint16_t scl_pin = 0;
    GPIO_TypeDef* pinport = nullptr;

    get_port_pins(&pinport, sda_pin, scl_pin);
    
    // Disable I2C controller to free the I/O pins
    I2C_Cmd(_i2c, DISABLE);
    delaytimer.delay(320);

    // set the pins for manual control
    GPIO_InitTypeDef GPIO_InitStructure;

    // if using alternate, kill the remap
    if (_alt_func)
    {
        GPIO_PinRemapConfig(GPIO_Remap_I2C1, DISABLE);
    }
    
    // Configure I2C SCL & SDA as output open-drain
    GPIO_InitStructure.GPIO_Pin = scl_pin | sda_pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(pinport, &GPIO_InitStructure);

    i2c_start(pinport, sda_pin, scl_pin);

    // SCL hi-z
    GPIO_WriteBit(pinport, scl_pin, Bit_SET);
    delaytimer.delay(320);
    
    // loop to make at least 9 clocks
    for (uint32_t loop = 0; loop < 9; loop++)
    {
        // let SCL go high by pullup
        GPIO_WriteBit(pinport, scl_pin, Bit_SET);
        delaytimer.delay(320);
        // pull SCL back low
        GPIO_WriteBit(pinport, scl_pin, Bit_RESET);
        delaytimer.delay(320);
    }

    // run a start stop sequence. this performs a state machine reset
    // in most slave i2c devices
    i2c_start(pinport, sda_pin, scl_pin);
    i2c_stop(pinport, sda_pin, scl_pin);
    delaytimer.delay(320);

    // reset the pins back to i2c control
    setup_pins();
    
    // re-enable  I2C0 controller
    I2C_Cmd(_i2c, ENABLE);
    
    _tx_busy = 0;

    return true;
}

// ----------------------------------------------------------------------------
