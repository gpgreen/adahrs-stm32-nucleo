/*
 * DMA.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef DMA_H_
#define DMA_H_

#include "cmsis_device.h"
#include <functional>

// ----------------------------------------------------------------------------

class DMA
{
public:
    explicit DMA(int device_no, int channel);

    uint32_t is_busy() {return _busy;}
    void start(DMA_InitTypeDef* init, std::function<void(void)> cb);
    void complete_transaction();
    
private:

    int _devno;
    int _channel;
    std::function<void(void)> _completed_fn;
    volatile uint32_t _busy;
    DMA_Channel_TypeDef* _dma_channel_p;
};

extern DMA DMA1Channel1;
extern DMA DMA1Channel2;
extern DMA DMA1Channel3;
extern DMA DMA1Channel4;
extern DMA DMA1Channel5;
extern DMA DMA1Channel6;
extern DMA DMA1Channel7;

extern "C"
{
    void DMA1_Channel1_IRQHandler(void);
    void DMA1_Channel2_IRQHandler(void);
    void DMA1_Channel3_IRQHandler(void);
    void DMA1_Channel4_IRQHandler(void);
    void DMA1_Channel5_IRQHandler(void);
    void DMA1_Channel6_IRQHandler(void);
    void DMA1_Channel7_IRQHandler(void);
}

// ----------------------------------------------------------------------------

#endif // TIMER_H_
