/*
 * stm32_dma.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "stm32_dma.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------


DMA::DMA(DMA_Channel_TypeDef* channel)
    : _busy(0), _dma_channel_p(channel)
{
    // does nothing else
}

// ----------------------------------------------------------------------------

void
DMA::begin(uint8_t priority, uint8_t subpriority)
{
    uint8_t irq = 0;
    if (_dma_channel_p == DMA1_Channel1) {
	irq = DMA1_Channel1_IRQn;
    }
    else if (_dma_channel_p == DMA1_Channel2) {
	irq = DMA1_Channel2_IRQn;
    }
    else if (_dma_channel_p == DMA1_Channel3) {
	irq = DMA1_Channel3_IRQn;
    }
    else if (_dma_channel_p == DMA1_Channel4) {
	irq = DMA1_Channel4_IRQn;
    }
    else if (_dma_channel_p == DMA1_Channel5) {
	irq = DMA1_Channel5_IRQn;
    }
    else if (_dma_channel_p == DMA1_Channel6) {
	irq = DMA1_Channel6_IRQn;
    }
    else if (_dma_channel_p == DMA1_Channel7) {
	irq = DMA1_Channel7_IRQn;
    }

#ifdef STM32F10X_HD_VL

    else if (_dma_channel_p == DMA2_Channel1) {
	irq = DMA2_Channel1_IRQn;
    }
    else if (_dma_channel_p == DMA2_Channel2) {
	irq = DMA2_Channel2_IRQn;
    }
    else if (_dma_channel_p == DMA2_Channel3) {
	irq = DMA2_Channel3_IRQn;
    }
    else if (_dma_channel_p == DMA2_Channel4) {
	irq = DMA2_Channel4_5_IRQn;
    }
    else if (_dma_channel_p == DMA2_Channel5) {
	irq = DMA2_Channel4_5_IRQn;
    }

#endif
    
    // enable the IRQ
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // enable the DMA Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

bool
DMA::start(DMA_InitTypeDef* init, std::function<void(void)> cb)
{
    if (_busy)
        return false;

    _busy = 1;

    _completed_fn = cb;

    DMA_DeInit(_dma_channel_p);
    DMA_Init(_dma_channel_p, init);

    // clear the completion flag
    DMA_ClearFlag(DMA1_IT_TC7);

    // enable DMA transfer complete interrupt
    DMA_ITConfig(_dma_channel_p, DMA_IT_TC, ENABLE);

    // enable the DMA controller
    DMA_Cmd(_dma_channel_p, ENABLE);

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
#ifdef DMA1_CHANNEL1_USED
DMA DMA1Channel1(DMA1_Channel1);
#endif
#ifdef DMA1_CHANNEL2_USED
DMA DMA1Channel2(DMA1_Channel2);
#endif
#ifdef DMA1_CHANNEL3_USED
DMA DMA1Channel3(DMA1_Channel3);
#endif
#ifdef DMA1_CHANNEL4_USED
DMA DMA1Channel4(DMA1_Channel4);
#endif
#ifdef DMA1_CHANNEL5_USED
DMA DMA1Channel5(DMA1_Channel5);
#endif
#ifdef DMA1_CHANNEL6_USED
DMA DMA1Channel6(DMA1_Channel6);
#endif
#ifdef DMA1_CHANNEL7_USED
DMA DMA1Channel7(DMA1_Channel7);
#endif

#ifdef STM32F10X_HD_VL

#ifdef DMA2_CHANNEL1_USED
DMA DMA2Channel1(DMA2_Channel1);
#endif
#ifdef DMA2_CHANNEL2_USED
DMA DMA2Channel2(DMA2_Channel2);
#endif
#ifdef DMA2_CHANNEL3_USED
DMA DMA2Channel3(DMA2_Channel3);
#endif
#ifdef DMA2_CHANNEL4_USED
DMA DMA2Channel4(DMA2_Channel4);
#endif
#ifdef DMA2_CHANNEL5_USED
DMA DMA2Channel5(DMA2_Channel5);
#endif

#endif

// ----- DMA1_Channel1_IRQHandler() ----------------------------------------------------
#ifdef DMA1_CHANNEL1_USED
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
#ifdef DMA1_CHANNEL2_USED
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
#ifdef DMA1_CHANNEL3_USED
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
#ifdef DMA1_CHANNEL4_USED
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
#ifdef DMA1_CHANNEL5_USED
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
#ifdef DMA1_CHANNEL6_USED
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
#ifdef DMA1_CHANNEL7_USED
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

#ifdef STM32F10X_HD_VL

// ----- DMA2_Channel1_IRQHandler() ----------------------------------------------------
#ifdef DMA2_CHANNEL1_USED
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
#ifdef DMA2_CHANNEL2_USED
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
#ifdef DMA2_CHANNEL3_USED
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

// ----- DMA2_Channel4_5_IRQHandler() ----------------------------------------------------
#ifdef DMA2_CHANNEL4_USED || defined(DMA2_CHANNEL5_USED)
void DMA2_Channel4_5_IRQHandler( void )
{
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC4 ) != RESET )
    {
        DMA2Channel4.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC4 );
    }
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC5 ) != RESET )
    {
        DMA2Channel5.complete_transaction();
        DMA_ClearFlag( DMA2_IT_TC5 );
    }
}
#endif

#endif // STM32F10X_HD_VL

// ----------------------------------------------------------------------------
