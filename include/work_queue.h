/*
 * work_queue.h
 *
 *  Created on: Aug 30, 2017
 *      Author: ggreen
 */

#ifndef WORK_QUEUE_H_
#define WORK_QUEUE_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "stm32_timer.h"

// ----------------------------------------------------------------------------

// structure to pass to workqueue to hold work callback and user data pointer
// for callback function argument
struct WorkCallback
{
    void (*callback)(void *);
    void* callback_data;
};

class WorkQueue
{
public:
    explicit WorkQueue();

    void begin();
    void process();
    void add_work_irq(void (*work_fn)(void *), void* data);

private:
    static void timeout(void*);
    
    // define away copy constructor and assignment operator
    WorkQueue(const WorkQueue&);
    const WorkQueue& operator=(const WorkQueue&);

    // members
    WorkCallback _queue[WORK_QUEUE_LENGTH];
    volatile uint32_t _queue_start;
    volatile uint32_t _queue_end;
    volatile uint32_t _processed;
    Timer _timer;
};

// ----------------------------------------------------------------------------

// Define one global work queue
extern WorkQueue g_work_queue;

// ----------------------------------------------------------------------------

#endif // WORK_QUEUE_H_
