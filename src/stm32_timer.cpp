/*
 * stm32_timer.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#include "stm32_timer.h"
#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "isr_def.h"

#define MS_PER_TICK                     5

class TimerList
{
public:
    explicit TimerList();
    
    void insert(Timer* t);
    void remove(Timer* t);
    
    void tick();
    
private:
    Timer* head;
};

TimerList::TimerList()
    : head(nullptr)
{
    // does nothing else
}

void TimerList::insert(Timer* t)
{
    if (head == nullptr)
    {
        head = t;
        t->next = nullptr;
        t->count = t->length;
    }
    else
    {
    }
}

void TimerList::remove(Timer* t)
{
    if (head == t)
    {
        head = t->next;
        return;
    }
    Timer* prev = head;
    while (prev->next != nullptr)
    {
        if (prev->next == t)
        {
            prev->next = t->next;
            return;
        }
    }
}

void TimerList::tick()
{
    if (head == nullptr)
        return;
    if (--head->count == 0) {
        head->state = Timer::Done;
        head = head->next;
        while (head != nullptr && head->length == 0)
        {
            head->state = Timer::Done;
            head = head->next;
        }
    }
}

// declare a static timer list
static TimerList timer_list;

Timer::Timer()
    : state(Idle), type(OneShot), length(0), count(0), next(nullptr)
{
    static bool initialized = false;
    
    // Initialize the timer hardware, if not previously done
    if (!initialized)
    {
        // do hardware initialization

        // configure Timer3 for timer work
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

        // disable timer if enabled
        TIM_Cmd(TIM3, DISABLE);
    
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_TimeBaseStructure.TIM_Period = MS_PER_TICK - 1;
        TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        
        TIM_DeInit(TIM3);
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
        
        // Enable the TIM3 update Interrupt and set at lowest priority.
        configure_nvic(TIM3_IRQn, TIMER_IRQ_PRIORITY, 0);
        
        // enable the timer
        TIM_Cmd(TIM3, ENABLE);
        
        // enable the timer update interrupt
        TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

        initialized = true;
    }
}

int
Timer::start(unsigned int millis, TimerType timer_type)
{
    if (state != Idle)
    {
        return -1;
    }
    
    // initialize the software timer
    state = Active;
    type = timer_type;
    length = millis / MS_PER_TICK;
    
    // add the timer to the active timer list
    timer_list.insert(this);
    
    return 0;
}

int
Timer::wait_for()
{
    if (state != Active)
    {
        return -1;
    }
    
    // wait for the timer to expire
    while (state != Done);
    
    // restart or idle the timer
    if (type == Periodic)
    {
        state = Active;
        timer_list.insert(this);
    }
    else
    {
        state = Idle;
    }
    
    return 0;
}

void
Timer::cancel()
{
    // remove the timer from the timer list
    if (state == Active)
    {
        timer_list.remove(this);
    }
    
    // reset the timers state
    state = Idle;
}


// IRQ for timer
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        // decrement the active timer's count
        timer_list.tick();

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}
