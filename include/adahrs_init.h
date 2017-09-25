/*
 * adahrs_init.h
 *
 *  Created on: Aug 28, 2017
 *      Author: ggreen
 */

#ifndef ADAHRSINIT_H_
#define ADAHRSINIT_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "isr_def.h"
#include "adahrs_config.h"
#include "adahrs_states.h"
#include "adahrs_command.h"
#include "adahrs_ekf.h"

// ----------------------------------------------------------------------------

class ADAHRSInit
{
public:
    explicit ADAHRSInit();

    void begin(ADAHRSConfig* config, ADAHRSSensorData* state,
               ADAHRSCommand* command, EKF* ekf);

private:
    void configure_led();
    
    // define away copy constructor and assignment operator
    ADAHRSInit(const ADAHRSInit&);
    const ADAHRSInit& operator=(const ADAHRSInit&);
};

inline void
led_on(void);

inline void
led_off(void);

// ----------------------------------------------------------------------------

inline void
__attribute__((always_inline))
led_on(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
}

inline void
__attribute__((always_inline))
led_off(void)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
}

// ----------------------------------------------------------------------------
#endif
