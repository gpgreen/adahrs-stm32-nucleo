/*
 * USART.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "USART.h"

// ----------------------------------------------------------------------------


USART::USART(int device_no)
    : _devno(device_no), _tx_buffer_start(0), _tx_busy(0),
      _rx_buffer_start(0)
{
    // does nothing else
}

// ----------------------------------------------------------------------------

int
USART::transmit (const char* txdata, int len)
{
    if (len + _tx_buffer_start > TX_BUFFER_SIZE)
	return 0;
    for (int i=0; i<len; ++i)
	_tx_buffer[_tx_buffer_start++] = txdata[i];

    tx_start();

    return 1;
}

void
USART::tx_start (void)
{
    if (_tx_busy)
	return;

    if (_tx_buffer_start == 0)
	return;

    _tx_busy = 1;

#if 0
    // Configure the DMA controller to make the transfer
    DMA_InitTypeDef DMA_InitStructure;

    // Configure the DMA controller to make the transfer
    DMA_DeInit(_tx_dma_channel);
    DMA_InitStructure.DMA_PeripheralBaseAddr = USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)_tx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = _tx_buffer_start;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(_tx_dma_channel, &DMA_InitStructure);
    
    // Enable the DMA controller to copy data from the TX buffer to the USART peripheral
    DMA_Cmd(_tx_dma_channel, ENABLE);

    // set the callback function for transfer complete
    
    // Enable TX DMA transfer complete interrupt
    DMA_ITConfig(_tx_dma_channel, DMA_IT_TC, ENABLE );
#endif
}

void USART::tx_dma_complete(void)
{
    _tx_buffer_start = 0;
    _tx_busy = 0;
}

void USART::rx_dma_complete(void)
{
    _rx_buffer_start = 0;
    _rx_busy = 0;
}

// ----------------------------------------------------------------------------
