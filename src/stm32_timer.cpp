/*
 * stm32_timer.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#include "stm32_timer.h"
#include "isr_def.h"

// ----------------------------------------------------------------------------
#define MS_PER_TICK                     100
// ----------------------------------------------------------------------------

/**
 * class to keep lists of timers
 * the list is ordered from soonest to expire to latest
 */
class TimerList
{
public:
    explicit TimerList();
    
    void insert(Timer* t);
    void remove(Timer* t);
    
    void tick();

    void start_critical_section();
    void end_critical_section();
    
private:
    Timer* head;
};

TimerList::TimerList()
    : head(nullptr)
{
    // does nothing else
}

// === START critical section
inline void TimerList::start_critical_section()
{
    // critical section created by setting BASEPRI to a level
    // above masked irq's
    __set_BASEPRI(TIMER_IRQ_MASKING);
}

// === END critical section
inline void TimerList::end_critical_section()
{
    __set_BASEPRI(0U);
}

void TimerList::insert(Timer* t)
{
    uint32_t ccount = t->length; // this will become the count for the inserted timer
                                 // also it determines where the timer fits into list

    start_critical_section();
    
    // find the correct spot to insert timer, and put it in list
    if (head == nullptr || head->count > ccount)
    {
        t->next = head;
        head = t;
    }
    else
    {
        Timer* prev = head;
        Timer* cur = head->next;
        ccount -= head->count;
        while (cur != nullptr && cur->count <= ccount)
        {
            ccount -= cur->count;
            prev = cur;
            cur = cur->next;
        }
        prev->next = t;
        t->next = cur;
    }
    t->count = ccount;
    // adjust count of all later timers
    Timer* cur = t->next;
    while (cur != nullptr)
    {
        if (cur->count > 0)
            cur->count -= ccount;
        cur = cur->next;
    }

    end_critical_section();
}

void TimerList::remove(Timer* t)
{
    start_critical_section();
    
    uint32_t ccount = t->count;
    if (head == t)
    {
        head = t->next;
        // add the count to the next item
        if (head != nullptr)
            head->count += ccount;
    }
    else
    {
        Timer* prev = head;
        while (prev->next != nullptr)
        {
            if (prev->next == t)
            {
                // remove t from the list
                prev->next = t->next;
                // add the count to following item
                if (t->next != nullptr)
                    t->next->count += ccount;
                break;
            }
        }
    }

    end_critical_section();
}

// called from IRQ
void TimerList::tick()
{
    if (head == nullptr)
        return;

    Timer* expired = nullptr;
    Timer* expired_tail = nullptr;
    
    start_critical_section();

    if (--head->count == 0) {
        while (head != nullptr && head->count == 0)
        {
            if (expired == nullptr)
            {
                expired = head;
                expired->next = nullptr;
            }
            else
                expired_tail->next = head;
            expired_tail = head;
            head->state = Timer::Done;
            head = head->next;
        }
    }

    end_critical_section();

    // now call completion functions on all timers that have them
    expired_tail = expired;
    while (expired_tail != nullptr)
    {
        // save the next pointer, it may get changed if timer reinserted
        // into timer list
        Timer* nxt = expired_tail->next;
        // if the function is non-null, execute it
        if (expired_tail->timer_fn != nullptr)
        {
            expired_tail->timer_fn(expired_tail->timer_fn_data);
            // if a periodic timer, put it back in the list
            if (expired_tail->type == Timer::Periodic)
            {
                expired_tail->state = Timer::Active;
                insert(expired_tail);
            }
            else
                expired_tail->state = Timer::Idle;
        }
        expired_tail = nxt;
    }
}

// ----------------------------------------------------------------------------

// declare a static timer list
static TimerList timer_list;

// ----------------------------------------------------------------------------

Timer::Timer()
    : state(Idle), type(OneShot), length(0), count(0), next(nullptr),
      timer_fn(nullptr), timer_fn_data(nullptr)
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
Timer::start(uint32_t microseconds, TimerType timer_type)
{
    if (state != Idle)
    {
        return -1;
    }
    
    // initialize the software timer
    state = Active;
    type = timer_type;
    length = microseconds / MS_PER_TICK;
    timer_fn = nullptr;
    timer_fn_data = nullptr;
    
    // add the timer to the active timer list
    timer_list.insert(this);
    
    return 0;
}

int
Timer::start(uint32_t microseconds, void (*timeout_fn)(void*), void* timeout_fn_data,
             TimerType timer_type)
{
    if (state != Idle)
    {
        return -1;
    }
    
    // initialize the software timer
    state = Active;
    type = timer_type;
    length = microseconds / MS_PER_TICK;
    timer_fn = timeout_fn;
    timer_fn_data = timeout_fn_data;
    
    // add the timer to the active timer list
    timer_list.insert(this);
    
    return 0;
}

int
Timer::wait_for()
{
    if (state != Active || timer_fn != nullptr)
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

// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------
