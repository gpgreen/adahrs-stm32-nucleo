/*
 * stm32_dma.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef STM32_DMA_H_
#define STM32_DMA_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "isr_def.h"

// ----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpadded"

class DMA
{
public:
    explicit DMA(DMA_Channel_TypeDef* channel);

    // initialize the hardware
    void begin(uint8_t priority, uint8_t subpriority);

    // start a transaction with given parameters and callback
    // returns false if DMA is busy
    bool start(DMA_InitTypeDef* init, void (*cb)(void *), void* data);

    // cancel an enabled transaction
    void cancel(void);
    
private:

    void priv_complete_transaction();

    // define away copy constructor and assignment operator
    DMA(const DMA&);
    const DMA& operator=(const DMA&);

    // members
    void (*_completed_fn)(void *);
    void *_completed_fn_data;
    DMA_Channel_TypeDef* _dma_channel_p;
    uint32_t _flagmask;
    uint8_t _irqno;
    volatile bool _busy;

    friend void DMA1_Channel1_IRQHandler(void);
    friend void DMA1_Channel2_IRQHandler(void);
    friend void DMA1_Channel3_IRQHandler(void);
    friend void DMA1_Channel4_IRQHandler(void);
    friend void DMA1_Channel5_IRQHandler(void);
    friend void DMA1_Channel6_IRQHandler(void);
    friend void DMA1_Channel7_IRQHandler(void);
    friend void DMA2_Channel1_IRQHandler(void);
    friend void DMA2_Channel2_IRQHandler(void);
    friend void DMA2_Channel3_IRQHandler(void);
    friend void DMA2_Channel4_5_IRQHandler(void);

};

#pragma GCC diagnostic pop

// instantiate objects for each channel
#ifdef DMA1_CHANNEL1_USED
extern DMA DMA1Channel1;
#endif
#ifdef DMA1_CHANNEL2_USED
extern DMA DMA1Channel2;
#endif
#ifdef DMA1_CHANNEL3_USED
extern DMA DMA1Channel3;
#endif
#ifdef DMA1_CHANNEL4_USED
extern DMA DMA1Channel4;
#endif
#ifdef DMA1_CHANNEL5_USED
extern DMA DMA1Channel5;
#endif
#ifdef DMA1_CHANNEL6_USED
extern DMA DMA1Channel6;
#endif
#ifdef DMA1_CHANNEL7_USED
extern DMA DMA1Channel7;
#endif

#ifdef STM32F10X_HD_VL

#ifdef DMA2_CHANNEL1_USED
extern DMA DMA2Channel1;
#endif
#ifdef DMA2_CHANNEL2_USED
extern DMA DMA2Channel2;
#endif
#ifdef DMA2_CHANNEL3_USED
extern DMA DMA2Channel3;
#endif
#ifdef DMA2_CHANNEL4_USED
extern DMA DMA2Channel4;
#endif
#ifdef DMA2_CHANNEL5_USED
extern DMA DMA2Channel5;
#endif

#endif

// ----------------------------------------------------------------------------

#endif // STM32_DMA_H_
