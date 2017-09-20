/*
 * stm32_i2c.h
 *
 *  Created on: Sep 2, 2017
 *      Author: ggreen
 */

#ifndef INCLUDE_STM32_I2C_H_
#define INCLUDE_STM32_I2C_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_dma.h"
#include "isr_def.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

// ----------------------------------------------------------------------------

// which kind of i2c transmission
enum I2CTransferType {TransmitNoStop, TransmitWithStop, ReceiveNoStop, ReceiveWithStop};

// each segment of the transmission has one of these
struct I2CMasterTxSegment
{
    uint32_t flags;
    uint8_t* databuf;
    uint32_t buflen;
    struct I2CMasterTxSegment* next;
};

// header of transmission
struct I2CMasterTxHeader
{
    uint32_t clock_speed;
    struct I2CMasterTxSegment* first;
    uint8_t slave_address;
    uint8_t num_txn_completed;
};

// ----------------------------------------------------------------------------

class I2C
{
    
public:

    // constructor using device number
    explicit I2C(int device_no);

    // initialize the I2C hardware
    void begin(bool use_alternate, uint8_t priority, uint8_t subpriority);

    // send/receive segments, return false if transmission in progress, true if transmission
    // started or scheduled to start
    bool send_receive(I2CMasterTxHeader* header, void (*completed_fn)(void*), void* data);

    // setup flags in Segment based on type of transfer
    void init_segment(I2CMasterTxSegment* segment, I2CTransferType type, uint8_t* databuffer,
                      uint32_t bufferlen, I2CMasterTxSegment* next);

    // run a reset on the bus to unstick slave devices, returns false if i2c is busy
    // true if reset sequence completed
    bool bus_recovery();
    
private:

    static void tx_start_irq(void* data);
    static void dma_complete(void* data);

    void wait_for_event(uint32_t event);
    void tx_start();
    void priv_rx_complete();
    void priv_tx_complete();
    void enable_clocks();
    void get_port_pins(GPIO_TypeDef** port, uint16_t& sda, uint16_t& scl);
    void setup_pins();
    
    // define away copy constructor and assignment operator
    I2C(const I2C&);
    const I2C& operator=(const I2C&);

private:
    int _devno;
    volatile int _tx_busy;
    I2C_TypeDef* _i2c;
    I2C_InitTypeDef _init;
    DMA* _tx_dma;
    DMA* _rx_dma;
    I2CMasterTxHeader* _hdr;
    uint8_t _irqno;
    bool _alt_func;
    void (*_send_completion_fn)(void*);
    void* _send_completion_data;

    friend void I2C1_EV_IRQHandler(void);
    friend void I2C2_EV_IRQHandler(void);
};

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
// hardware instances
// ----------------------------------------------------------------------------

#ifdef I2C1_USED
extern I2C i2c1;
#endif

#ifdef I2C2_USED
extern I2C i2c2;
#endif

// ----------------------------------------------------------------------------

#endif /* INCLUDE_STM32_I2C_H_ */
