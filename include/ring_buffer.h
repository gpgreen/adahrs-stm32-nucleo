/*
 * ring_buffer.h
 *
 *  Created on: Sep 22, 2017
 *      Author: ggreen
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

template <typename T>
class RingBuffer
{
public:
    explicit RingBuffer(void);

    uint32_t size() const;
    bool empty() const;

    T& top();
    const T& top() const;
    void pop();
    void push(T elem);
    
private:
    // define away copy constructor and assignment operator
    RingBuffer(const RingBuffer&);
    RingBuffer& operator=(const RingBuffer&);
    
    T _array[RING_BUFFER_SIZE];
    uint32_t _head;
    uint32_t _tail;
};

template <typename T>
RingBuffer<T>::RingBuffer(void)
    : _head(0), _tail(0)
{
    // does nothing else
}

template <typename T>
uint32_t RingBuffer<T>::size() const
{
    if (_head > _tail)
    {
        return RING_BUFFER_SIZE - _head + _tail;
    }
    else
    {
        return _tail - _head;
    }
}

template <typename T>
bool RingBuffer<T>::empty() const
{
    return _head == _tail;
}

template <typename T>
T& RingBuffer<T>::top()
{
    return _array[_head];
}

template <typename T>
const T& RingBuffer<T>::top() const
{
    return _array[_head];
}

template <typename T>
void RingBuffer<T>::pop()
{
    if (++_head == RING_BUFFER_SIZE)
        _head = 0;
}

template <typename T>
void RingBuffer<T>::push(T elem)
{
    if (size() == RING_BUFFER_SIZE)
    {
        UsageFault_Handler();
    }
    _array[_tail++] = elem;
    if (_tail == RING_BUFFER_SIZE)
        _tail = 0;
}

// ----------------------------------------------------------------------------
#endif /* RING_BUFFER_H_ */
