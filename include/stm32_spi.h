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

// ----------------------------------------------------------------------------

class SPI
{
public:
    explicit SPI(int device_no);

    // initialize the SPI hardware
    void begin(uint8_t priority, uint8_t subpriority);

    // send/receive some data, return false if dma's are busy
    bool send(uint8_t* txdata, int buflen);

    // method called by external irq handler, not meant as part
    // of public interface
    void priv_rx_complete();

private:

    static void tx_start_irq(void * data);
    void tx_start(bool in_irq);
    static void tx_dma_complete(void* data);
    static void rx_dma_complete(void* data);
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
    uint8_t _irqno;
    volatile bool _tx_busy;
    volatile bool _rx_busy;
    uint8_t padding;
    volatile int _buffer_len;
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
// interrupt handlers
// ----------------------------------------------------------------------------

#if defined(__cplusplus)
extern "C"
{
#endif

    void SPI1_IRQHandler(void);

    void SPI2_IRQHandler(void);

#ifdef STM32F10X_HD_VL
    void SPI3_IRQHandler(void);
#endif // STM32F10X_HD_VL

#if defined(__cplusplus)
}
#endif

// ----------------------------------------------------------------------------

#endif /* INCLUDE_STM32_SPI_H_ */