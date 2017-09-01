/*
 * work_queue.h
 *
 *  Created on: Aug 30, 2017
 *      Author: ggreen
 */

#ifndef WORK_QUEUE_H_
#define WORK_QUEUE_H_

#include <functional>

#include "cmsis_device.h"
#include "adahrs_definitions.h"

// ----------------------------------------------------------------------------

class WorkQueue
{
public:
    explicit WorkQueue();

    void process();
    void add_work_irq(std::function<void(void)> work_fn);
    bool add_work(std::function<void(void)> work_fn);

private:
    // define away copy constructor and assignment operator
    WorkQueue(const WorkQueue&);
    const WorkQueue& operator=(const WorkQueue&);

    // members
    std::function<void(void)> _queue[WORK_QUEUE_LENGTH];
    volatile uint32_t _queue_start;
    volatile uint32_t _queue_end;
};

// ----------------------------------------------------------------------------

// Define one global work queue
extern WorkQueue g_work_queue;

// ----------------------------------------------------------------------------

#endif // WORK_QUEUE_H_
