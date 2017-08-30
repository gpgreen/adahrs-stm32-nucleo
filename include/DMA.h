/*
 * DMA.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef DMA_H_
#define DMA_H_

#include <functional>

#include "cmsis_device.h"
#include "adahrs_definitions.h"

// ----------------------------------------------------------------------------

class DMA
{
public:
    explicit DMA(DMA_Channel_TypeDef* channel);

    // initalize the hardware
    void begin(void);
    // is the DMA busy?
    uint32_t is_busy() {return _busy;}
    // start a transaction with given parameters and callback
    // returns false if DMA is busy
    bool start(DMA_InitTypeDef* init, std::function<void(void)> cb);
    void complete_transaction();
    
private:

    std::function<void(void)> _completed_fn;
    volatile uint32_t _busy;
    DMA_Channel_TypeDef* _dma_channel_p;
};

// instantiate objects for each channel
#if defined(DMA1_CHANNEL1_USED)
extern DMA DMA1Channel1;
#endif
#if defined(DMA1_CHANNEL2_USED)
extern DMA DMA1Channel2;
#endif
#if defined(DMA1_CHANNEL3_USED)
extern DMA DMA1Channel3;
#endif
#if defined(DMA1_CHANNEL4_USED)
extern DMA DMA1Channel4;
#endif
#if defined(DMA1_CHANNEL5_USED)
extern DMA DMA1Channel5;
#endif
#if defined(DMA1_CHANNEL6_USED)
extern DMA DMA1Channel6;
#endif
#if defined(DMA1_CHANNEL7_USED)
extern DMA DMA1Channel7;
#endif
#if defined(DMA2_CHANNEL1_USED)
extern DMA DMA2Channel1;
#endif
#if defined(DMA2_CHANNEL2_USED)
extern DMA DMA2Channel2;
#endif
#if defined(DMA2_CHANNEL3_USED)
extern DMA DMA2Channel3;
#endif
#if defined(DMA2_CHANNEL4_USED)
extern DMA DMA2Channel4;
#endif
#if defined(DMA2_CHANNEL5_USED)
extern DMA DMA2Channel5;
#endif

// define the interrupt handlers
extern "C"
{
#if defined(DMA1_CHANNEL1_USED)
    void DMA1_Channel1_IRQHandler(void);
#endif
#if defined(DMA1_CHANNEL2_USED)
    void DMA1_Channel2_IRQHandler(void);
#endif
#if defined(DMA1_CHANNEL3_USED)
    void DMA1_Channel3_IRQHandler(void);
#endif
#if defined(DMA1_CHANNEL4_USED)
    void DMA1_Channel4_IRQHandler(void);
#endif
#if defined(DMA1_CHANNEL5_USED)
    void DMA1_Channel5_IRQHandler(void);
#endif
#if defined(DMA1_CHANNEL6_USED)
    void DMA1_Channel6_IRQHandler(void);
#endif
#if defined(DMA1_CHANNEL7_USED)
    void DMA1_Channel7_IRQHandler(void);
#endif
#if defined(DMA2_CHANNEL1_USED)
    void DMA2_Channel1_IRQHandler(void);
#endif
#if defined(DMA2_CHANNEL2_USED)
    void DMA2_Channel2_IRQHandler(void);
#endif
#if defined(DMA2_CHANNEL3_USED)
    void DMA2_Channel3_IRQHandler(void);
#endif
#if defined(DMA2_CHANNEL4_USED)
    void DMA2_Channel4_IRQHandler(void);
#endif
#if defined(DMA2_CHANNEL5_USED)
    void DMA2_Channel5_IRQHandler(void);
#endif
}

// ----------------------------------------------------------------------------

#endif // TIMER_H_
