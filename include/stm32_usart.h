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
    void begin(int baud_rate);
    
    // transmit some data, return false if buffer full
    bool transmit(const char* txdata, int len);

    // received data?
    bool has_received_data();

    // get received data, returns count of data copied
    unsigned int get_received_data(uint8_t* buf, int buflen);

private:
    
    void tx_start(bool in_irq);
    void tx_dma_complete();
    void rx_dma_complete();
    
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
    volatile uint8_t _tx_buffer_start;
    volatile bool _tx_busy;
    // receive buffer members
    volatile uint8_t _rx_buffer[RX_BUFFER_SIZE];
    volatile uint8_t _rx_buffer_start;
    volatile bool _rx_busy;
};

// ----------------------------------------------------------------------------

#endif // STM32_USART_H_
