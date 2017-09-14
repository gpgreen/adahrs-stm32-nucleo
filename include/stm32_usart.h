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
#include "isr_def.h"

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

private:
    
    static void tx_start_irq(void * data);
    void tx_start();
    static void tx_dma_complete(void* data);
    void rx_dma_complete();
    void priv_rx_complete();
    
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

    friend void USART1_IRQHandler(void);
    friend void USART2_IRQHandler(void);
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

#endif // STM32_USART_H_
