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
	: _queue_start(0), _queue_end(0)
{
	// does nothing else
}

// ----------------------------------------------------------------------------

// function called in SysTick to process work, this work is in a low priority
// irq, so queue and queue_end may be changed during execution of this method
// queue_start is only changed in this method, which is only called in the
// work process irq, so it cannot be changed elsewhere
void WorkQueue::process(void)
{
	uint32_t queue_end = __sync_fetch_and_add(&_queue_end, 0);

	if (_queue_start == queue_end)
		return;
	WorkCallback* cb = &_queue[_queue_start++];
	if (cb->callback != nullptr)
	{
		cb->callback(cb->callback_data);
		cb->callback = nullptr;
		cb->callback_data = nullptr;
	}
	if (_queue_start == WORK_QUEUE_LENGTH)
	    _queue_start = 0;
}

// this can only be called from within irq
void WorkQueue::add_work_irq(void (*work_fn)(void *), void* data)
{
	// for this to work, we diable irq's
	// This is expensive, but we are only doing it when
	// fixing the queue.
	__disable_irq();
	WorkCallback* cb = &_queue[_queue_end++];
	cb->callback = work_fn;
	cb->callback_data = data;
	if (_queue_end == WORK_QUEUE_LENGTH)
	{
		_queue_end = 0;
	}
	if (_queue_end == _queue_start)
	{
		while(1);	// goto endless loop, queue is full, cannot add work
	}
	__DMB();
	__enable_irq();
}

// this can only be called outside irq - it uses a critical
// section to ensure that no interrupts will happen while
// the queue is manipulated. Returns 'true' if work
// added to queue, 'false' if queue is full and not added
bool WorkQueue::add_work(void (*work_fn)(void *), void* data)
{
	// critical sections created by setting BASEPRI to a level
	// above all irq's that could be adding work, that way it cannot be interrupted by
	// those irq
	bool retval = false;
	// === START critical section
	__set_BASEPRI(WORKQUEUE_IRQ_MASKING);
	uint32_t end = _queue_end++;
	if (_queue_end == WORK_QUEUE_LENGTH)
	{
		_queue_end = 0;
	}
	if (_queue_end == _queue_start)
	{
		_queue_end = end;
	}
	else
	{
		WorkCallback* cb = &_queue_start[end];
		cb->callback = work_fn;
		cb->callback_data = data;
		retval = true;
	}
	__DMB();
	__set_BASEPRI(0U);
	// === END critical section
	return retval;
}

