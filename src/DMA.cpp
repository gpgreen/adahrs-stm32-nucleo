/*
 * DMA.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "DMA.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------


DMA::DMA(DMA_Channel_TypeDef* channel)
    : _channel(channel), _busy(0)
{
    // does nothing else
}

// ----------------------------------------------------------------------------

bool
DMA::start(DMA_InitTypeDef* init, std::function<void(void)> cb)
{
    if (_busy)
        return false;

    _completed_fn = cb;

    DMA_DeInit(_dma_channel_p);
    DMA_Init(_dma_channel_p, init);

    // enable the DMA controller
    DMA_Cmd(_dma_channel_p, ENABLE);

    // enable DMA transfer complete interrupt
    DMA_ITConfig(_dma_channel_p, DMA_IT_TC, ENABLE);

    return true;
}

// called when a transaction is done
void
DMA::complete_transaction()
{
    _busy = 0;
    if (_completed_fn)
        _completed_fn();
}

// instantiate an object for each channel
#if defined(DMA1_CHANNEL1_USED)
DMA DMA1Channel1(DMA1_Channel1);
#endif
#if defined(DMA1_CHANNEL2_USED)
DMA DMA1Channel2(DMA1_Channel2);
#endif
#if defined(DMA1_CHANNEL3_USED)
DMA DMA1Channel3(DMA1_Channel3);
#endif
#if defined(DMA1_CHANNEL4_USED)
DMA DMA1Channel4(DMA1_Channel4);
#endif
#if defined(DMA1_CHANNEL5_USED)
DMA DMA1Channel5(DMA1_Channel5);
#endif
#if defined(DMA1_CHANNEL6_USED)
DMA DMA1Channel6(DMA1_Channel6);
#endif
#if defined(DMA1_CHANNEL7_USED)
DMA DMA1Channel7(DMA1_Channel7);
#endif
#if defined(DMA2_CHANNEL1_USED)
DMA DMA2Channel1(DMA2_Channel1);
#endif
#if defined(DMA2_CHANNEL2_USED)
DMA DMA2Channel2(DMA2_Channel2);
#endif
#if defined(DMA2_CHANNEL3_USED)
DMA DMA2Channel3(DMA2_Channel3);
#endif
#if defined(DMA2_CHANNEL4_USED)
DMA DMA2Channel4(DMA2_Channel4);
#endif
#if defined(DMA2_CHANNEL5_USED)
DMA DMA2Channel5(DMA2_Channel5);
#endif

// ----- DMA1_Channel1_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL1_USED)
void DMA1_Channel1_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC1 ) != RESET )
    {
        DMA1Channel1.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC1 );
    }
}
#endif

// ----- DMA1_Channel2_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL2_USED)
void DMA1_Channel2_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC2 ) != RESET )
    {
        DMA1Channel2.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC2 );
    }
}
#endif

// ----- DMA1_Channel3_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL3_USED)
void DMA1_Channel3_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC3 ) != RESET )
    {
        DMA1Channel3.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC3 );
    }
}
#endif

// ----- DMA1_Channel4_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL4_USED)
void DMA1_Channel4_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC4 ) != RESET )
    {
        DMA1Channel4.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC4 );
    }
}
#endif

// ----- DMA1_Channel5_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL5_USED)
void DMA1_Channel5_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC5 ) != RESET )
    {
        DMA1Channel5.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC5 );
    }
}
#endif

// ----- DMA1_Channel6_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL6_USED)
void DMA1_Channel6_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC6 ) != RESET )
    {
        DMA1Channel6.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC6 );
    }
}
#endif

// ----- DMA1_Channel7_IRQHandler() ----------------------------------------------------
#if defined(DMA1_CHANNEL7_USED)
void DMA1_Channel7_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA1_IT_TC7 ) != RESET )
    {
        DMA1Channel7.complete_transaction();
        DMA_ClearFlag( DMA1_IT_TC7 );
    }
}
#endif

// ----- DMA2_Channel1_IRQHandler() ----------------------------------------------------
#if defined(DMA2_CHANNEL1_USED)
void DMA2_Channel1_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC1 ) != RESET )
    {
        DMA2Channel1.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC1 );
    }
}
#endif

// ----- DMA2_Channel2_IRQHandler() ----------------------------------------------------
#if defined(DMA2_CHANNEL2_USED)
void DMA2_Channel2_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC2 ) != RESET )
    {
        DMA2Channel2.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC2 );
    }
}
#endif

// ----- DMA2_Channel3_IRQHandler() ----------------------------------------------------
#if defined(DMA2_CHANNEL3_USED)
void DMA2_Channel3_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC3 ) != RESET )
    {
        DMA2Channel3.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC3 );
    }
}
#endif

// ----- DMA2_Channel4_IRQHandler() ----------------------------------------------------
#if defined(DMA2_CHANNEL4_USED)
void DMA2_Channel4_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC4 ) != RESET )
    {
        DMA2Channel4.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC4 );
    }
}
#endif

// ----- DMA2_Channel5_IRQHandler() ----------------------------------------------------
#if defined(DMA2_CHANNEL5_USED)
void DMA2_Channel5_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC5 ) != RESET )
    {
        DMA2Channel5.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC5 );
    }
}
#endif

// ----------------------------------------------------------------------------
