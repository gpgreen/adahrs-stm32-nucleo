/*
 * adahrs_config.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: ggreen
 */

#include "adahrs_config.h"

extern "C"
{
    // start of flash block defined in linker script
    extern uint32_t __flashb1_start__;
    // start of factory flash block defined in linker script
    extern uint32_t __factory_flashb1_start__;
};

// variables to start of region(s) in flash block
static uint32_t* s_factory_start_address = (uint32_t*)&__flashb1_start__;
static uint32_t* s_flash_start_address = (uint32_t*)(&__factory_flashb1_start__);

// Macro for determining whether FLASH has been initialized
#define	FGET_FLASH_UNINITIALIZED()	(*s_flash_start_address == 0xFFFFFFFF)
#define	FGET_FACTORY_UNINITIALIZED()	(*s_factory_start_address == 0xFFFFFFFF)

// ----------------------------------------------------------------------------

ADAHRSConfig::ADAHRSConfig()
{
    // does nothing else
}

// ----------------------------------------------------------------------------

void ADAHRSConfig::begin()
{
    if (FGET_FLASH_UNINITIALIZED())
    {
        reset_to_factory();
    }
    else
    {
        load_config_from_flash(UM6_USE_CONFIG_ADDRESS);
    }
}

uint32_t ADAHRSConfig::get_register(int addr)
{
    if (addr >= DATA_REG_START_ADDRESS && addr < COMMAND_START_ADDRESS)
        return _data_reg[addr - DATA_REG_START_ADDRESS];
    else if (addr < COMMAND_START_ADDRESS)
        return _config_reg[addr];
    else
        while(1);
}

void ADAHRSConfig::set_register(int addr, uint32_t data)
{
    if (addr >= DATA_REG_START_ADDRESS && addr < COMMAND_START_ADDRESS)
        _data_reg[addr - DATA_REG_START_ADDRESS] = data;
    else if (addr < COMMAND_START_ADDRESS)
        _config_reg[addr] = data;
    else
        while(1);
}

void ADAHRSConfig::set_register(int addr, float data)
{
    float f = data;
    if (addr >= DATA_REG_START_ADDRESS && addr < COMMAND_START_ADDRESS)
        _data_reg[addr - DATA_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);
    else if (addr < COMMAND_START_ADDRESS)
        _config_reg[addr] = *reinterpret_cast<uint32_t*>(&f);
    else
        while(1);
}

void ADAHRSConfig::set_register(int addr, uint8_t b1, uint8_t b2,
                                uint8_t b3, uint8_t b4)
{
    uint32_t* reg;
    if (addr >= DATA_REG_START_ADDRESS && addr < COMMAND_START_ADDRESS)
        reg = &_data_reg[addr - DATA_REG_START_ADDRESS];
    else if (addr < COMMAND_START_ADDRESS)
        reg = &_config_reg[addr];
    else
        while(1);
    *reg = b1 | (b2 << 8) | (b3 << 16) | (b4 << 24);
}

// write all configuration data to flash memory
int ADAHRSConfig::write_config_to_flash(int write_location_flag)
{
    FLASH_Status FLASHStatus;
    
    uint32_t* flash_start_address;
    if (write_location_flag == UM6_USE_CONFIG_ADDRESS)
    {
        flash_start_address = s_flash_start_address;
    }
    else
    {
        flash_start_address = s_factory_start_address;
    }

    FLASH_Unlock();

    // Clear all pending flags
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	 
    // Erase FLASH page in preparation for write operation
    for (int i=0; i<CONFIG_ARRAY_SIZE; ++i)
    {
        FLASHStatus = FLASH_ErasePage((uint32_t)(flash_start_address + i));
		  
        if (FLASHStatus != FLASH_COMPLETE)
        {
            FLASH_Lock();
            return FLASHStatus;
        }
    }
	 
    // Write configuration data
    for (int i=0; i<CONFIG_ARRAY_SIZE; ++i)
    {		  
        // Write FLASH data
        FLASHStatus = FLASH_ProgramWord((uint32_t)(flash_start_address + i), _config_reg[i]);
	
        if (FLASHStatus != FLASH_COMPLETE)
        {
            FLASH_Lock();
            return FLASHStatus;
        }
        
        // Make sure new flash memory contents match
        if (_config_reg[i] != *(flash_start_address + i))
        {
            FLASH_Lock();
            return FLASH_TIMEOUT;
        }		  
    }
    
    FLASH_Lock();
    
    return FLASH_COMPLETE;
}

// clear all global data
void ADAHRSConfig::clear_global_data()
{
    for (int i=0; i<DATA_ARRAY_SIZE; ++i)
        _data_reg[i] = 0;
}

// load all configuration data from flash
void ADAHRSConfig::load_config_from_flash(int flash_address_flag)
{
    uint32_t* flash_start_address;
    if (flash_address_flag == UM6_USE_CONFIG_ADDRESS)
    {
        flash_start_address = s_flash_start_address;
    }
    else
    {
        flash_start_address = s_factory_start_address;
    }

    for (int i=0; i<CONFIG_ARRAY_SIZE; ++i)
        _config_reg[i] = *(flash_start_address + i);
}

void ADAHRSConfig::reset_to_factory()
{
    float f;
    // If flash has not been programmed yet, then use default
    // configuration.  Otherwise, load configuration from flash
    if (FGET_FACTORY_UNINITIALIZED())
    {
        // Communication configuration
        _config_reg[UM6_COMMUNICATION] =  UM6_BROADCAST_ENABLED
            | UM6_GYROS_PROC_ENABLED
            | UM6_GYROS_RAW_ENABLED
            | UM6_TEMPERATURE_ENABLED
            | UM6_ACCELS_PROC_ENABLED
            | UM6_MAG_PROC_ENABLED
            | UM6_EULER_ENABLED
            | UM6_GPS_POSITION_ENABLED
            | UM6_GPS_REL_POSITION_ENABLED
            | UM6_GPS_COURSE_SPEED_ENABLED
            | UM6_GPS_SAT_SUMMARY_ENABLED
            | UM6_GPS_SAT_DATA_ENABLED;
        _config_reg[UM6_COMMUNICATION] |= 0x0000;		// This is the broadcast rate (or, rather, what is used to set the broadcast rate)
        _config_reg[UM6_COMMUNICATION] |= (5 << UM6_BAUD_START_BIT);
        _config_reg[UM6_COMMUNICATION] |= (5 << UM6_GPS_BAUD_START_BIT);
		  
        // MISC configuration
        _config_reg[UM6_MISC_CONFIG] = UM6_MAG_UPDATE_ENABLED
            | UM6_ACCEL_UPDATE_ENABLED
            | UM6_QUAT_ESTIMATE_ENABLED
            | UM6_GYRO_STARTUP_CAL;
		  
        // Magnetometer reference vector
        f = 0.136f;
        _config_reg[UM6_MAG_REF_X] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.4065f;
        _config_reg[UM6_MAG_REF_Y] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.894f;
        _config_reg[UM6_MAG_REF_Z] = *reinterpret_cast<uint32_t*>(&f);
		  
        // Accelerometer reference vector
        f = 0.0f;
        _config_reg[UM6_ACCEL_REF_X] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_REF_Y] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = -1.0f;
        _config_reg[UM6_ACCEL_REF_Z] = *reinterpret_cast<uint32_t*>(&f);
		  
        // EKF variances
        f = 2.0f;
        _config_reg[UM6_EKF_MAG_VARIANCE] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 2.0f;
        _config_reg[UM6_EKF_ACCEL_VARIANCE] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.1f;
        _config_reg[UM6_EKF_PROCESS_VARIANCE] = *reinterpret_cast<uint32_t*>(&f);
		  
        // Gyro biases
        _config_reg[UM6_GYRO_BIAS_XY] = 0;
        _config_reg[UM6_GYRO_BIAS_Z] = 0;
            
        f = 0.0f;
        _config_reg[UM6_GYROX_BIAS_0] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROX_BIAS_1] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROX_BIAS_2] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROX_BIAS_3] = *reinterpret_cast<uint32_t*>(&f);
            
        _config_reg[UM6_GYROY_BIAS_0] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROY_BIAS_1] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROY_BIAS_2] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROY_BIAS_3] = *reinterpret_cast<uint32_t*>(&f);
            
        _config_reg[UM6_GYROZ_BIAS_0] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROZ_BIAS_1] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROZ_BIAS_2] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GYROZ_BIAS_3] = *reinterpret_cast<uint32_t*>(&f);
            
        // Accelerometer biases
        _config_reg[UM6_ACCEL_BIAS_XY] = 0;
        _config_reg[UM6_ACCEL_BIAS_Z] = 0;
		  
        // Magnetometer biases
        _config_reg[UM6_MAG_BIAS_XY] = 0;
        _config_reg[UM6_MAG_BIAS_Z] = 0;
		  
        // Accelerometer alignment matrix
        f = .0000625f;
        _config_reg[UM6_ACCEL_CAL_00] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_CAL_01] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_CAL_02] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_CAL_10] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = .0000625f;
        _config_reg[UM6_ACCEL_CAL_11] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_CAL_12] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_CAL_20] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_ACCEL_CAL_21] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = .0000625f;
        _config_reg[UM6_ACCEL_CAL_22] = *reinterpret_cast<uint32_t*>(&f);
		  
        // Rate gyro alignment matrix
        f = .06956f;
        _config_reg[UM6_GYRO_CAL_00] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_GYRO_CAL_01] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_GYRO_CAL_02] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_GYRO_CAL_10] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = .06956f;
        _config_reg[UM6_GYRO_CAL_11] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_GYRO_CAL_12] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_GYRO_CAL_20] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_GYRO_CAL_21] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = .06956f;
        _config_reg[UM6_GYRO_CAL_22] = *reinterpret_cast<uint32_t*>(&f);
		  
        // Magnetometer calibration matrix
        f = 0.00271f;
        _config_reg[UM6_MAG_CAL_00] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_MAG_CAL_01] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_MAG_CAL_02] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_MAG_CAL_10] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.00271f;
        _config_reg[UM6_MAG_CAL_11] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_MAG_CAL_12] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_MAG_CAL_20] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.0f;
        _config_reg[UM6_MAG_CAL_21] = *reinterpret_cast<uint32_t*>(&f);
		  
        f = 0.00271f;
        _config_reg[UM6_MAG_CAL_22] = *reinterpret_cast<uint32_t*>(&f);
		  
        // GPS configuration
        f = 0.0f;
        _config_reg[UM6_GPS_HOME_LAT-CONFIG_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GPS_HOME_LONG-CONFIG_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_GPS_HOME_ALTITUDE-CONFIG_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);

        // External magnetic sensor
        f = 0.0f;
        _config_reg[UM6_EXT_MAG_X-CONFIG_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_EXT_MAG_Y-CONFIG_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);
        _config_reg[UM6_EXT_MAG_Z-CONFIG_REG_START_ADDRESS] = *reinterpret_cast<uint32_t*>(&f);

        // CAN stack
        _config_reg[UM6_NODE_ID] = 4;

        // Equipment
        _config_reg[UM6_EQUIPMENT] = UM6_AOA_INSTALLED | UM6_STATIC_INSTALLED | UM6_PITOT_INSTALLED
            | UM6_AOA_ENABLED | UM6_STATIC_ENABLED | UM6_PITOT_ENABLED;
    }
    else
    {
        load_config_from_flash(UM6_USE_FACTORY_ADDRESS);
    }

}

// ----------------------------------------------------------------------------
