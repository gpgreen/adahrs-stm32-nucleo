/*
 * work_queue.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: ggreen
 */

#include "work_queue.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

// the global instance of the work queue
WorkQueue g_work_queue;

// ----------------------------------------------------------------------------

WorkQueue::WorkQueue()
    : _queue_start(0), _queue_end(0), _processed(0)
{
    // does nothing else
}

void WorkQueue::begin()
{
#ifdef USE_WORKQUEUE_TRACE
    // setup pin
    GPIO_InitTypeDef GPIO_InitStructure;

    // enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // Configure PA3 as output push pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // set low
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
#endif

    // workqueue will run every 20ms
    _timer.start(20000, WorkQueue::timeout, this, Timer::Periodic);
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

    // get the callback and fix start of queue
    WorkCallback* cb = &_queue[_queue_start++];
    if (_queue_start == WORK_QUEUE_LENGTH)
        _queue_start = 0;

    // execute the callback, this may call add_work_irq which would
    // change the queue, but we are done changing the queue so it
    // won't cause problems
    if (cb->callback != nullptr)
    {

        ++_processed;

#ifdef USE_WORKQUEUE_TRACE
        GPIO_SetBits(GPIOA, GPIO_Pin_3);
#endif

        cb->callback(cb->callback_data);
        cb->callback = nullptr;
        cb->callback_data = nullptr;

#ifdef USE_WORKQUEUE_TRACE
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);
#endif

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
        UsageFault_Handler();
    }
    else
    {
        WorkCallback* cb = &_queue[end];
        cb->callback = work_fn;
        cb->callback_data = data;
    }

    __set_BASEPRI(0U);
    // === END critical section
}

// function to call when the timer has expired
// process a work item
void WorkQueue::timeout(void* data)
{
    WorkQueue* wq = reinterpret_cast<WorkQueue*>(data);
    wq->process();
}
