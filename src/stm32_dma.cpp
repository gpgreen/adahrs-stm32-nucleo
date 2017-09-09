/*
 * stm32_dma.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#include "stm32_dma.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

DMA::DMA(DMA_Channel_TypeDef* channel) :
    _completed_fn(nullptr), _completed_fn_data(nullptr),
    _dma_channel_p(channel), _busy(false)
{
    if (_dma_channel_p == DMA1_Channel1) {
        _irqno = DMA1_Channel1_IRQn;
        _flagmask = DMA1_IT_TE1 | DMA1_IT_HT1 | DMA1_IT_TC1;
    }
    else if (_dma_channel_p == DMA1_Channel2) {
        _irqno = DMA1_Channel2_IRQn;
        _flagmask = DMA1_IT_TE2 | DMA1_IT_HT2 | DMA1_IT_TC2;
    }
    else if (_dma_channel_p == DMA1_Channel3) {
        _irqno = DMA1_Channel3_IRQn;
        _flagmask = DMA1_IT_TE3 | DMA1_IT_HT3 | DMA1_IT_TC3;
    }
    else if (_dma_channel_p == DMA1_Channel4) {
        _irqno = DMA1_Channel4_IRQn;
        _flagmask = DMA1_IT_TE4 | DMA1_IT_HT4 | DMA1_IT_TC4;
    }
    else if (_dma_channel_p == DMA1_Channel5) {
        _irqno = DMA1_Channel5_IRQn;
        _flagmask = DMA1_IT_TE5 | DMA1_IT_HT5 | DMA1_IT_TC5;
    }
    else if (_dma_channel_p == DMA1_Channel6) {
        _irqno = DMA1_Channel6_IRQn;
        _flagmask = DMA1_IT_TE6 | DMA1_IT_HT6 | DMA1_IT_TC6;
    }
    else if (_dma_channel_p == DMA1_Channel7) {
        _irqno = DMA1_Channel7_IRQn;
        _flagmask = DMA1_IT_TE7 | DMA1_IT_HT7 | DMA1_IT_TC7;
    }

#ifdef STM32F10X_HD_VL

    else if (_dma_channel_p == DMA2_Channel1)
    {
        _irqno = DMA2_Channel1_IRQn;
//        _flagmask = DMA2_Channel1_IT_Mask;
    }
    else if (_dma_channel_p == DMA2_Channel2)
    {
        _irqno = DMA2_Channel2_IRQn;
//        _flagmask = DMA2_Channel2_IT_Mask;
    }
    else if (_dma_channel_p == DMA2_Channel3)
    {
        _irqno = DMA2_Channel3_IRQn;
//        _flagmask = DMA2_Channel3_IT_Mask;
    }
    else if (_dma_channel_p == DMA2_Channel4)
    {
        _irqno = DMA2_Channel4_5_IRQn;
//        _flagmask = DMA2_Channel4_IT_Mask;
    }
    else if (_dma_channel_p == DMA2_Channel5)
    {
        _irqno = DMA2_Channel4_5_IRQn;
//        _flagmask = DMA2_Channel5_IT_Mask;
    }

#endif

    else {
        while (1);
    }
}

// ----------------------------------------------------------------------------

void DMA::begin(uint8_t priority, uint8_t subpriority)
{
    // enable the IRQ
    NVIC_InitTypeDef NVIC_InitStructure;

    // clear DMA pending interrupts
    DMA_ClearFlag(_flagmask);

    // enable the DMA Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = _irqno;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    _busy = false;
}

bool DMA::start(DMA_InitTypeDef* init, void (*cb)(void *), void* data)
{
    if (_busy)
        return false;

    _busy = true;

    _completed_fn = cb;
    _completed_fn_data = data;

    DMA_DeInit(_dma_channel_p);
    DMA_Init(_dma_channel_p, init);

    // enable DMA transfer complete interrupt
    DMA_ITConfig(_dma_channel_p, DMA_IT_HT | DMA_IT_TE, DISABLE);
    DMA_ITConfig(_dma_channel_p, DMA_IT_TC, ENABLE);

    // enable the DMA controller
    DMA_Cmd(_dma_channel_p, ENABLE);

    return true;
}

void DMA::cancel()
{
    // disable the DMA controller and interrupts
    DMA_Cmd(_dma_channel_p, DISABLE);
    DMA_ITConfig(_dma_channel_p, DMA_IT_HT | DMA_IT_TE | DMA_IT_TC, DISABLE);
    _busy = false;
}

// called when a transaction is done
void DMA::priv_complete_transaction()
{
    _busy = false;
    if (_completed_fn != nullptr)
        _completed_fn(_completed_fn_data);
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
        DMA_ClearFlag( DMA1_IT_TC1 );
        DMA1Channel1.priv_complete_transaction();
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
        DMA_ClearFlag( DMA1_IT_TC2 );
        DMA1Channel2.priv_complete_transaction();
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
        DMA_ClearFlag( DMA1_IT_TC3 );
        DMA1Channel3.priv_complete_transaction();
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
        DMA_ClearFlag( DMA1_IT_TC4 );
        DMA1Channel4.priv_complete_transaction();
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
        DMA_ClearFlag( DMA1_IT_TC5 );
        DMA1Channel5.priv_complete_transaction();
    }
}
#endif

// ----- DMA1_Channel6_IRQHandler() ----------------------------------------------------
#ifdef DMA1_CHANNEL6_USED
void DMA1_Channel6_IRQHandler(void)
{
    // Indicates that the TX buffer contents have been transmitted.
    if (DMA_GetITStatus(DMA1_IT_TC6) != RESET)
    {
        DMA_ClearFlag(DMA1_IT_TC6);
        DMA1Channel6.priv_complete_transaction();
    }
}
#endif

// ----- DMA1_Channel7_IRQHandler() ----------------------------------------------------
#ifdef DMA1_CHANNEL7_USED
void DMA1_Channel7_IRQHandler(void)
{
    // Indicates that the TX buffer contents have been transmitted.
    if (DMA_GetITStatus(DMA1_IT_TC7) != RESET)
    {
        DMA_ClearFlag(DMA1_IT_TC7);
        DMA1Channel7.priv_complete_transaction();
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
        DMA_ClearFlag( DMA2_IT_TC1 );
        DMA2Channel1.priv_complete_transaction();
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
        DMA_ClearFlag( DMA2_IT_TC2 );
        DMA2Channel2.priv_complete_transaction();
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
        DMA_ClearFlag( DMA2_IT_TC3 );
        DMA2Channel3.priv_complete_transaction();
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
        DMA_ClearFlag( DMA2_IT_TC4 );
        DMA2Channel4.priv_complete_transaction();
    }
    // Indicates that the TX buffer contents have been transmitted.
    if( DMA_GetITStatus( DMA2_IT_TC5 ) != RESET )
    {
        DMA_ClearFlag( DMA2_IT_TC5 );
        DMA2Channel5.priv_complete_transaction();
    }
}
#endif

#endif // STM32F10X_HD_VL

// ----------------------------------------------------------------------------
