/*
 * DMA.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "DMA.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------


DMA::DMA(int device_no, int channel)
    : _devno(device_no), _channel(channel), _busy(0)
{
    if (device_no == 1) {
	if (channel == 1)
	    _dma_channel_p = DMA1_Channel1;
	else if (channel == 2)
	    _dma_channel_p = DMA1_Channel2;
	else if (channel == 3)
	    _dma_channel_p = DMA1_Channel3;
	else if (channel == 4)
	    _dma_channel_p = DMA1_Channel4;
	else if (channel == 5)
	    _dma_channel_p = DMA1_Channel5;
	else if (channel == 6)
	    _dma_channel_p = DMA1_Channel6;
	else if (channel == 7)
	    _dma_channel_p = DMA1_Channel7;
	else
	    _dma_channel_p = 0;
    } else if (device_no == 2) {
	if (channel == 1)
	    _dma_channel_p = DMA2_Channel1;
	else if (channel == 2)
	    _dma_channel_p = DMA2_Channel2;
	else if (channel == 3)
	    _dma_channel_p = DMA2_Channel3;
	else if (channel == 4)
	    _dma_channel_p = DMA2_Channel4;
	else if (channel == 5)
	    _dma_channel_p = DMA2_Channel5;
	else
	    _dma_channel_p = 0;
    }
}

// ----------------------------------------------------------------------------

void
DMA::start(DMA_InitTypeDef* init, std::function<void(void)> cb)
{
    if (!_busy)
	return;

    _completed_fn = cb;

    DMA_DeInit(_dma_channel_p);
    DMA_Init(_dma_channel_p, init);

    // enable the DMA controller
    DMA_Cmd(_dma_channel_p, ENABLE);

    // enable DMA transfer complete interrupt
    DMA_ITConfig(_dma_channel_p, DMA_IT_TC, ENABLE);
}

void
DMA::complete_transaction()
{
    _busy = 0;
    if (_completed_fn != nullptr)
	_completed_fn();
    _completed_fn = nullptr;
}

DMA DMA1Channel1(1, 1);
DMA DMA1Channel2(1, 2);
DMA DMA1Channel3(1, 3);
DMA DMA1Channel4(1, 4);
DMA DMA1Channel5(1, 5);
DMA DMA1Channel6(1, 6);
DMA DMA1Channel7(1, 7);

// ----- DMA1_Channel1_IRQHandler() ----------------------------------------------------
void DMA1_Channel1_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC1 ) != RESET )
    {
	DMA1Channel1.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC1 );
    }
}

// ----- DMA1_Channel2_IRQHandler() ----------------------------------------------------
void DMA1_Channel2_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC2 ) != RESET )
    {
	DMA1Channel2.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC2 );
    }
}

// ----- DMA1_Channel3_IRQHandler() ----------------------------------------------------
void DMA1_Channel3_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC3 ) != RESET )
    {
	DMA1Channel3.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC3 );
    }
}

// ----- DMA1_Channel4_IRQHandler() ----------------------------------------------------
void DMA1_Channel4_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC4 ) != RESET )
    {
	DMA1Channel4.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC4 );
    }
}

// ----- DMA1_Channel5_IRQHandler() ----------------------------------------------------
void DMA1_Channel5_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC5 ) != RESET )
    {
	DMA1Channel5.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC5 );
    }
}

// ----- DMA1_Channel6_IRQHandler() ----------------------------------------------------
void DMA1_Channel6_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC6 ) != RESET )
    {
	DMA1Channel6.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC6 );
    }
}

// ----- DMA1_Channel7_IRQHandler() ----------------------------------------------------
void DMA1_Channel7_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC7 ) != RESET )
    {
	DMA1Channel7.complete_transaction();
	DMA_ClearFlag( DMA1_IT_TC7 );
    }
}

// ----------------------------------------------------------------------------
