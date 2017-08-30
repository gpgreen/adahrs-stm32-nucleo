/*
 * USART.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef USART_H_
#define USART_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "DMA.h"

// ----------------------------------------------------------------------------

class USART
{
public:
    explicit USART(int device_no);

    // initialize the USART hardware
    void begin(int baud_rate);
    
    // transmit some data, return false if buffer full
    bool transmit(const char* txdata, int len);

private:
    
    void tx_start();
    void tx_dma_complete();
    void rx_dma_complete();
    
    // define away copy constructor and assignment operator
    USART(const USART&);
    const USART& operator=(const USART&);

private:
    int _devno;
    DMA& _tx_dma;
    // transmit buffer members
    volatile uint8_t _tx_buffer[TX_BUFFER_SIZE];
    uint8_t* _tx_buf_p;
    volatile uint8_t _tx_buffer_start;
    volatile bool _tx_busy;
    // receive buffer members
    volatile uint8_t _rx_buffer[RX_BUFFER_SIZE];
    volatile uint8_t _rx_buffer_start;
    volatile bool _rx_busy;
};

// ----------------------------------------------------------------------------

#endif // TIMER_H_
