/*
 * work_queue.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: ggreen
 */

#include "work_queue.h"
#include "isr_def.h"

// ----------------------------------------------------------------------------

// the global instance of the work queue
WorkQueue g_work_queue;

// ----------------------------------------------------------------------------

WorkQueue::WorkQueue()
    : _queue_start(0), _queue_end(0)
{
    // does nothing else
}

void WorkQueue::begin(uint8_t priority, uint8_t subpriority)
{
    // configure Timer2 for work queue
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // disable timer if enabled
    TIM_Cmd(TIM2, DISABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 2000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_DeInit(TIM2);
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // Enable the TIM2 update Interrupt and set at lowest priority.
    configure_nvic(TIM2_IRQn, priority, subpriority);

    // enable the timer
    TIM_Cmd(TIM2, ENABLE);

    // enable the timer update interrupt
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

// ----------------------------------------------------------------------------

// function called in TimerIRQ to process work, this work is a low priority
// irq, so queue and queue_end may be changed during execution of this method
// queue_start is only changed in this method, which is only called in the
// work process irq, so it cannot be changed elsewhere
void WorkQueue::process(void)
{
    uint32_t queue_end = __sync_fetch_and_add(&_queue_end, 0);

    // queue is empty
    if (_queue_start == queue_end)
        return;

    // get the pointer to callback and fix start of queue
    WorkCallback* cb = &_queue[_queue_start++];
    if (_queue_start == WORK_QUEUE_LENGTH)
        _queue_start = 0;

    // execute the callback, this may call add_work_irq which would
    // change the queue, but we are done changing the queue so it
    // won't cause problems
    if (cb->callback != nullptr)
    {
        cb->callback(cb->callback_data);
        cb->callback = nullptr;
        cb->callback_data = nullptr;
    }
}

// uses a critical section to ensure that no interrupts will happen
// while the queue is manipulated. 
void WorkQueue::add_work_irq(void (*work_fn)(void *), void* data)
{
    // critical sections created by setting BASEPRI to a level
    // above all irq's that could be adding work, that way it cannot be interrupted by
    // those irq

    // === START critical section
    __set_BASEPRI(WORKQUEUE_IRQ_MASKING);

    int end = _queue_end++;
    if (_queue_end == WORK_QUEUE_LENGTH)
    {
        _queue_end = 0;
    }
    if (_queue_end == _queue_start)
    {
        while (1);
    }
    else
    {
        WorkCallback* cb = &_queue[end];
        cb->callback = work_fn;
        cb->callback_data = data;
    }

    __DMB();
    __set_BASEPRI(0U);
    // === END critical section
}

// IRQ for work queue
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // process the work queue
        g_work_queue.process();
        // clear pending interrupt bit
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
