/*
 * stm32_usart.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include <functional>
#include "stm32_usart.h"

// ----------------------------------------------------------------------------

USART::USART(int device_no)
    : _devno(device_no), _tx_dma(nullptr), _rx_dma(nullptr),
      _tx_buf_p(nullptr), _tx_buffer_start(0), _tx_busy(false),
      _rx_buffer_start(0), _rx_busy(false)
{
#ifdef DMA1_CHANNEL4_USED
    if (device_no == 1)
        _tx_dma = &DMA1Channel4;
#endif
#ifdef DMA1_CHANNEL5_USED
    if (device_no == 1)
        _rx_dma = &DMA1Channel5;
#endif
#ifdef DMA1_CHANNEL7_USED
//    if (device_no == 2)
//        _tx_dma = &DMA1Channel7;
#endif
#ifdef DMA1_CHANNEL6_USED
//    if (device_no == 2)
//        _rx_dma = &DMA1Channel6;
#endif
#ifdef DMA1_CHANNEL2_USED
    if (device_no == 3)
        _tx_dma = &DMA1Channel2;
#endif
#ifdef DMA1_CHANNEL3_USED
    if (device_no == 3)
        _rx_dma = &DMA1Channel3;
#endif
}

// ----------------------------------------------------------------------------

void
USART::begin(int baud_rate)
{
    // enable the UART clock, get pins and port numbers
    USART_TypeDef* usart;
    uint16_t txpin;
    uint16_t rxpin;
    GPIO_TypeDef* pinport;
    
    if (_devno == 1) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	usart = USART1;
	txpin = GPIO_Pin_9;
	rxpin = GPIO_Pin_10;
	pinport = GPIOA;
    }
    if (_devno == 2) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	usart = USART2;
	txpin = GPIO_Pin_2;
	rxpin = GPIO_Pin_3;
	pinport = GPIOA;
    }
    if (_devno == 3) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	usart = USART3;
	txpin = GPIO_Pin_10;
	rxpin = GPIO_Pin_11;
	pinport = GPIOB;
    }
    
    // configure the pins
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Configure USART Rx as input, internal pullup
    GPIO_InitStructure.GPIO_Pin = rxpin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(pinport, &GPIO_InitStructure);

    // Configure USART Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = txpin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(pinport, &GPIO_InitStructure);

    /* USART1 is configured as followS:
	   - BaudRate = 'baud'  
	   - Word Length = 8 Bits
	   - One Stop Bit
	   - No parity
	   - Hardware flow control disabled (RTS and CTS signals)
	   - Receive and transmit enabled
    */
    USART_InitTypeDef USART_InitStructure;
    
    USART_InitStructure.USART_BaudRate = baud_rate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // configure USART
    USART_DeInit(usart);
    USART_Init(usart, &USART_InitStructure);
    
    // Enable USART TX DMA Requests
    USART_DMACmd(usart, USART_DMAReq_Tx, ENABLE);
    // Enable USART RX DMA Requests
//    USART_DMACmd(usart, USART_DMAReq_Rx, ENABLE);
    
    // enable USART
    USART_Cmd(usart, ENABLE);
}

bool
USART::transmit (const char* txdata, int len)
{
    if (len + _tx_buffer_start > TX_BUFFER_SIZE)
        return false;

    _tx_buf_p = &_tx_buffer[_tx_buffer_start];

    for (int i=0; i<len; ++i)
        _tx_buffer[_tx_buffer_start++] = txdata[i];
    
    tx_start();

    return true;
}

void
USART::tx_start (void)
{
    if (_tx_busy)
        return;

    if (_tx_buffer_start == 0)
        return;

    _tx_busy = true;

    // Configure the DMA controller to make the transfer
    DMA_InitTypeDef DMA_InitStructure;

    // Configure the DMA controller to make the transfer
    if (_devno == 1)
        DMA_InitStructure.DMA_PeripheralBaseAddr = USART1->DR;
    else if (_devno == 2)
        DMA_InitStructure.DMA_PeripheralBaseAddr = USART2->DR;
    else if (_devno == 3)
        DMA_InitStructure.DMA_PeripheralBaseAddr = USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = reinterpret_cast<uint32_t>(_tx_buf_p);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    // set buffer size to difference between end of data, and start of send
    DMA_InitStructure.DMA_BufferSize = (&_tx_buffer[_tx_buffer_start] - _tx_buf_p);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

    // this line is a problem if called within IRQ (tx_dma_complete)
    while (_tx_dma->is_busy() 
           || !_tx_dma->start(&DMA_InitStructure, 
                             std::bind(&USART::tx_dma_complete, this)));
}

// called from within IRQ
void USART::tx_dma_complete(void)
{
    _tx_busy = false;
    // if current buf ptr is the same, then no more data to send
    if (_tx_buf_p == &_tx_buffer[_tx_buffer_start]) {
        _tx_buffer_start = 0;
    } else {
        // more data to send, send it
        tx_start();
    }
}

void USART::rx_dma_complete(void)
{
    _rx_buffer_start = 0;
    _rx_busy = false;
}

// ----------------------------------------------------------------------------
