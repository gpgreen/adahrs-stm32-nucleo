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

// ----------------------------------------------------------------------------
// interrupt handlers
// ----------------------------------------------------------------------------

#if defined(__cplusplus)
extern "C"
{
#endif

    void I2C1_EV_IRQHandler(void);

    void I2C2_EV_IRQHandler(void);

#if defined(__cplusplus)
}
#endif

// ----------------------------------------------------------------------------

#define I2C_RECEIVE                     0x00000001
#define I2C_GENERATE_STOP               0x00000002

class I2C
{
public:
    explicit I2C(int device_no);

    // initialize the I2C hardware
    void begin(bool use_alternate, uint8_t priority, uint8_t subpriority);

    // send/receive some data, return false if dma's are busy
    bool send_receive(uint8_t address, int flags, uint8_t* databuf, int buflen,
                      void (*completed_fn)(void*), void* data);

private:

    static void tx_start_irq(void * data);
    static void dma_complete(void* data);

    void wait_for_event(uint32_t event);
    void tx_start(bool in_irq);
    void configure_nvic(uint8_t priority, uint8_t subpriority);
    void priv_rx_complete();
    void priv_tx_complete();

    // define away copy constructor and assignment operator
    I2C(const I2C&);
    const I2C& operator=(const I2C&);

private:
    int _devno;
    I2C_TypeDef* _i2c;
    DMA* _tx_dma;
    DMA* _rx_dma;
    // transmit buffer members
    volatile uint8_t* _data_buffer;
    volatile int _buffer_len;
    int _flags;
    uint8_t _address;
    uint8_t _irqno;
    volatile bool _tx_busy;
    bool _alt_func;
    void (*_send_completion_fn)(void*);
    void* _send_completion_data;

    friend void I2C1_EV_IRQHandler(void);
    friend void I2C2_EV_IRQHandler(void);
};

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
