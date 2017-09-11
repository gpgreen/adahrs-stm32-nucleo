/*
 * stm32_spi.h
 *
 *  Created on: Sep 2, 2017
 *      Author: ggreen
 */

#ifndef INCLUDE_STM32_SPI_H_
#define INCLUDE_STM32_SPI_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_dma.h"
#include "isr_def.h"

// ----------------------------------------------------------------------------

class SPI
{
public:
    explicit SPI(int device_no);

    // initialize the SPI hardware
    void begin(bool use_alternate, uint8_t priority, uint8_t subpriority);

    // send/receive some data, return false if dma's are busy
    bool send(uint8_t* txdata, int buflen, void (*completed_fn)(void*), void* data);

private:

    static void tx_start_irq(void * data);
    void tx_start(bool in_irq);
    static void rx_dma_complete(void* data);
    void priv_rx_complete();
    void configure_nvic(uint8_t priority, uint8_t subpriority);

    // define away copy constructor and assignment operator
    SPI(const SPI&);
    const SPI& operator=(const SPI&);

private:
    int _devno;
    SPI_TypeDef* _spi;
    DMA* _tx_dma;
    DMA* _rx_dma;
    // transmit buffer members
    volatile uint8_t* _tx_buffer;
    volatile int _buffer_len;
    uint8_t _irqno;
    volatile bool _tx_busy;
    bool _alt_func;
    uint8_t padding;
    void (*_send_completion_fn)(void*);
    void* _send_completion_data;

    friend void SPI1_IRQHandler(void);
    friend void SPI2_IRQHandler(void);
};

// ----------------------------------------------------------------------------
// hardware instances
// ----------------------------------------------------------------------------

#ifdef SPI1_USED
extern SPI spi1;
#endif

#ifdef SPI2_USED
extern SPI spi2;
#endif

#ifdef STM32F10X_HD_VL

#ifdef SPI3_USED
extern SPI spi3;
#endif

#endif // STM32F10X_HD_VL

// ----------------------------------------------------------------------------

#endif /* INCLUDE_STM32_SPI_H_ */
