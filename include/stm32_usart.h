/*
 * stm32_usart.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef USART_H_
#define USART_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_dma.h"

// ----------------------------------------------------------------------------

class USART
{
public:
    explicit USART(int device_no);

    // initialize the USART hardware
    void begin(int baud_rate, uint8_t priority, uint8_t subpriority);
    
    // transmit some data, return false if buffer full
    bool transmit(const char* txdata, int len);

    // received data?
    bool has_received_data();

    // get received data, returns count of data copied
    unsigned int get_received_data(uint8_t* buf, int buflen);

    // method called by external irq handler, not meant as part
    // of public interface
    void priv_rx_complete();

private:
    
    void tx_start(bool in_irq);
    void tx_dma_complete();
    void rx_dma_complete();
    void configure_nvic(uint8_t priority, uint8_t subpriority);
    
    // define away copy constructor and assignment operator
    USART(const USART&);
    const USART& operator=(const USART&);

private:
    int _devno;
    USART_TypeDef* _uart;
    DMA* _tx_dma;
    DMA* _rx_dma;
    // transmit buffer members
    volatile uint8_t _tx_buffer[TX_BUFFER_SIZE];
    volatile uint8_t* _tx_buf_p;
    volatile int _tx_buffer_start;
    uint8_t _irqno;
    volatile bool _tx_busy;
    volatile bool _rx_busy;
    uint8_t padding;
    // receive buffer members
    volatile uint8_t _rx_buffer[RX_BUFFER_SIZE];
    volatile int _rx_buffer_start;
};

// ----------------------------------------------------------------------------
// hardware instances
// ----------------------------------------------------------------------------

#ifdef USART1_USED
extern USART usart1;
#endif

#ifdef USART2_USED
extern USART usart2;
#endif

#ifdef USART3_USED
extern USART usart3;
#endif

// ----------------------------------------------------------------------------
// interrupt handlers
// ----------------------------------------------------------------------------

#if defined(__cplusplus)
extern "C"
{
#endif

void USART1_IRQHandler(void);

void USART2_IRQHandler(void);

void USART3_IRQHandler(void);

#if defined(__cplusplus)
}
#endif

// ----------------------------------------------------------------------------

#endif // STM32_USART_H_
