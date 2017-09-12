/*
 * adahrs_config.h
 *
 *  Created on: Sep 11, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_CONFIG_H_
#define ADAHRS_CONFIG_H_

#include "cmsis_device.h"
#include "adahrs_definitions.h"
#include "adahrs_config_def.h"

// ----------------------------------------------------------------------------

class ADAHRSConfig
{
public:
    explicit ADAHRSConfig();

    // initialize
    void begin();

    // size of register banks
    int config_size() { return CONFIG_ARRAY_SIZE; }
    int data_size() { return DATA_ARRAY_SIZE; } 
    int command_size() { return COMMAND_COUNT; }

    // get or set a register
    uint32_t get_register(int addr);
    uint32_t set_register(int addr, uint32_t data);

    // reset configuration data to factory values
    void reset_to_factory();

    // save configuration data to flash
    int write_config_to_flash(int write_location_flag);

    // clear all global data
    void clear_global_data();
    
private:

    // load configuration from flash
    void load_config_from_flash(int flash_address_flag);

    
    // define away copy constructor and assignment operator
    ADAHRSConfig(const ADAHRSConfig&);
    const ADAHRSConfig& operator=(const ADAHRSConfig&);

    // members
    uint32_t _config_reg[CONFIG_ARRAY_SIZE];
    uint32_t _data_reg[DATA_ARRAY_SIZE];
};

// ----------------------------------------------------------------------------

#endif // ADAHRS_CONFIG_H_
