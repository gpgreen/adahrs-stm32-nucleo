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
WorkQueue g_work_queue();

// ----------------------------------------------------------------------------

WorkQueue::WorkQueue()
  : _queue_end(0)
{
    // does nothing else
}

// ----------------------------------------------------------------------------

// function called in SysTick to process work
void
WorkQueue::process (void)
{
  if (_queue_end == 0)
    return;
  if (_queue[0]) {
    _queue[0]();
    for (int i=1; i<_queue_end; ++i) {
      _queue[i-1] = _queue[i];
    }
    --_queue_end;
  }
}

// this can only be called in irq
void
WorkQueue::add_work_irq (std::function<void(void)> work_fn)
{
  if (_queue_end < WORK_QUEUE_LENGTH) {
    _queue[_queue_end++] = work_fn;
  } else {
    // we don't ever want to end up here
    while(1);
  }
}

// this can only be called outside irq
bool
WorkQueue::add_work (std::function<void(void)> work_fn)
{
  // critical sections created by setting BASEPRI to a level
  // above all irq's that could be adding work, that way it cannot be interrupted by
  // those irq
  bool retval = false;
  // === START critical section
  __set_BASEPRI(WORKQUEUE_PROCESS_IRQ_MASK);
  if (_queue_end < WORK_QUEUE_LENGTH) {
    _queue[_queue_end++] = work_fn;
    retval = true;
  }
  __DMB();
  __set_BASEPRI(0U);
  // === END critical section
  return retval;
}

