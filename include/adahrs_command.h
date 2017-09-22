/*
 * adahrs_command.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_COMMAND_H_
#define ADAHRS_COMMAND_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "adahrs_config.h"
#include "adahrs_states.h"

// ----------------------------------------------------------------------------

class ADAHRSCommand
{
public:
    explicit ADAHRSCommand();

    void begin(ADAHRSConfig* config, ADAHRSSensorData* state);
    void process_next_character();
    void send_next_packet();
    
private:
    
    // define away copy constructor and assignment operator
    ADAHRSCommand(const ADAHRSCommand&);
    const ADAHRSCommand& operator=(const ADAHRSCommand&);
};

// ----------------------------------------------------------------------------
#endif
