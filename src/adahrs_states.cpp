/*
 * ADAHRSStates.cpp
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#include <math.h>
#include "adahrs_states.h"
#include "adahrs_config_def.h"

// ----------------------------------------------------------------------------

ADAHRSSensorData::ADAHRSSensorData()
	: _mode(MODE_QUATERNION)
{
    // does nothing else
}

void ADAHRSSensorData::begin(ADAHRSConfig* config)
{
    copy_config_to_states(config);
}

void ADAHRSSensorData::copy_config_to_states(ADAHRSConfig* config)
{
    uint32_t v;
    
    // Magnetic field reference vector
    v = config->get_register(UM6_MAG_REF_X);
    state_data.mag_ref_x = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_MAG_REF_Y);
    state_data.mag_ref_y = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_MAG_REF_Z);
    state_data.mag_ref_z = *reinterpret_cast<float*>(&v);
	 
    // Accelerometer reference vector
    v = config->get_register(UM6_ACCEL_REF_X);
    state_data.accel_ref_x = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_ACCEL_REF_Y);
    state_data.accel_ref_y = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_ACCEL_REF_Z);
    state_data.accel_ref_z = *reinterpret_cast<float*>(&v);
	 
    // Magnetometer and accelerometer variances
    v = config->get_register(UM6_EKF_MAG_VARIANCE);
    state_data.mag_var = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_EKF_ACCEL_VARIANCE);
    state_data.accel_var = *reinterpret_cast<float*>(&v);
	 
    // Process variance matrix
    v = config->get_register(UM6_EKF_PROCESS_VARIANCE);
    if ((config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
    	state_data.R = Matrix(4, 4);
        state_data.R.set(0, 0, *reinterpret_cast<float*>(&v));
        state_data.R.set(1, 1, *reinterpret_cast<float*>(&v));
        state_data.R.set(2, 2, *reinterpret_cast<float*>(&v));
        state_data.R.set(3, 3, *reinterpret_cast<float*>(&v));
		  
        _mode = MODE_EULER;
    }
    else
    {
        // Process variance is scaled here so that the performance in Euler Angle mode and Quaternion mode is comparable
    	state_data.R = Matrix(4, 4);
        state_data.R.set(0, 0, *reinterpret_cast<float*>(&v) * 0.00001f);
        state_data.R.set(1, 1, *reinterpret_cast<float*>(&v) * 0.00001f);
        state_data.R.set(2, 2, *reinterpret_cast<float*>(&v) * 0.00001f);
        state_data.R.set(3, 3, *reinterpret_cast<float*>(&v) * 0.00001f);
		  
        _mode = MODE_QUATERNION;
    }
	  
    state_data.process_var = *reinterpret_cast<float*>(&v);
	 
    // Gyro biases
    state_data.beta_p = static_cast<int16_t>(config->get_register(UM6_GYRO_BIAS_XY) >> 16);
    state_data.beta_q = static_cast<int16_t>(config->get_register(UM6_GYRO_BIAS_XY) & 0x0FFFF);
    state_data.beta_r = static_cast<int16_t>(config->get_register(UM6_GYRO_BIAS_Z) >> 16);
      
    // X gyro temperature compensation
    v = config->get_register(UM6_GYROX_BIAS_0);
    state_data.beta_p0 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROX_BIAS_1);
    state_data.beta_p1 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROX_BIAS_2);
    state_data.beta_p2 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROX_BIAS_3);
    state_data.beta_p3 = *reinterpret_cast<float*>(&v);
      
    // Y gyro temperature compensation
    v = config->get_register(UM6_GYROY_BIAS_0);
    state_data.beta_q0 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROY_BIAS_1);
    state_data.beta_q1 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROY_BIAS_2);
    state_data.beta_q2 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROY_BIAS_3);
    state_data.beta_q3 = *reinterpret_cast<float*>(&v);
      
    // Z gyro temperature compensation
    v = config->get_register(UM6_GYROZ_BIAS_0);
    state_data.beta_r0 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROZ_BIAS_1);
    state_data.beta_r1 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROZ_BIAS_2);
    state_data.beta_r2 = *reinterpret_cast<float*>(&v);
      
    v = config->get_register(UM6_GYROZ_BIAS_3);
    state_data.beta_r3 = *reinterpret_cast<float*>(&v);
      
	 
    // Accel biases
    state_data.beta_acc_x = static_cast<int16_t>(config->get_register(UM6_ACCEL_BIAS_XY) >> 16);
    state_data.beta_acc_y = static_cast<int16_t>(config->get_register(UM6_ACCEL_BIAS_XY) & 0x0FFFF);
    state_data.beta_acc_z = static_cast<int16_t>(config->get_register(UM6_ACCEL_BIAS_Z) >> 16);
	 
    // Mag biases
    state_data.beta_mag_x = static_cast<int16_t>(config->get_register(UM6_MAG_BIAS_XY) >> 16);
    state_data.beta_mag_y = static_cast<int16_t>(config->get_register(UM6_MAG_BIAS_XY) & 0x0FFFF);
    state_data.beta_mag_z = static_cast<int16_t>(config->get_register(UM6_MAG_BIAS_Z) >> 16);
	 
    // Accelerometer calibration matrix
    state_data.accel_cal = Matrix(3, 3);
    v = config->get_register(UM6_ACCEL_CAL_00);
    state_data.accel_cal.set(0, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_01);
    state_data.accel_cal.set(0, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_02);
    state_data.accel_cal.set(0, 2, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_10);
    state_data.accel_cal.set(1, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_11);
    state_data.accel_cal.set(1, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_12);
    state_data.accel_cal.set(1, 2, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_20);
    state_data.accel_cal.set(2, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_21);
    state_data.accel_cal.set(2, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_ACCEL_CAL_22);
    state_data.accel_cal.set(2, 2, *reinterpret_cast<float*>(&v));	
	 
    // Rate gyro alignment matrix
    state_data.gyro_cal = Matrix(3, 3);
    v = config->get_register(UM6_GYRO_CAL_00);
    state_data.gyro_cal.set(0, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_01);
    state_data.gyro_cal.set(0, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_02);
    state_data.gyro_cal.set(0, 2, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_10);
    state_data.gyro_cal.set(1, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_11);
    state_data.gyro_cal.set(1, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_12);
    state_data.gyro_cal.set(1, 2, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_20);
    state_data.gyro_cal.set(2, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_21);
    state_data.gyro_cal.set(2, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_GYRO_CAL_22);
    state_data.gyro_cal.set(2, 2, *reinterpret_cast<float*>(&v));
	 
    // Magnetometer calibration matrix
    state_data.mag_cal = Matrix(3, 3);
    v = config->get_register(UM6_MAG_CAL_00);
    state_data.mag_cal.set(0, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_01);
    state_data.mag_cal.set(0, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_02);
    state_data.mag_cal.set(0, 2, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_10);
    state_data.mag_cal.set(1, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_11);
    state_data.mag_cal.set(1, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_12);
    state_data.mag_cal.set(1, 2, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_20);
    state_data.mag_cal.set(2, 0, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_21);
    state_data.mag_cal.set(2, 1, *reinterpret_cast<float*>(&v));
	 
    v = config->get_register(UM6_MAG_CAL_22);
    state_data.mag_cal.set(2, 2, *reinterpret_cast<float*>(&v));
	 
    // GPS home data
    v = config->get_register(UM6_GPS_HOME_LAT);
    state_data.GPS_lat_home = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_GPS_HOME_LONG);
    state_data.GPS_lon_home = *reinterpret_cast<float*>(&v);

    v = config->get_register(UM6_GPS_HOME_ALTITUDE);
    state_data.GPS_alt_home = *reinterpret_cast<float*>(&v);
	 
    // External Mag sensor data
    v = config->get_register(UM6_EXT_MAG_X);
    state_data.ext_mag_x = *reinterpret_cast<float*>(&v);
	 
    v = config->get_register(UM6_EXT_MAG_Y);
    state_data.ext_mag_y = *reinterpret_cast<float*>(&v);

    v = config->get_register(UM6_EXT_MAG_Z);
    state_data.ext_mag_z = *reinterpret_cast<float*>(&v);
	 
}

void ADAHRSSensorData::copy_states_to_config(ADAHRSConfig* config)
{
    int16_t converted_data1;
    int16_t converted_data2;
    
    // Raw gyro, accel, and mag data
    config->set_register(UM6_GYRO_RAW_XY, static_cast<uint32_t>((raw_data.gyro_x << 16)
                                                                | (raw_data.gyro_y & 0x0FFFF)));
    config->set_register(UM6_GYRO_RAW_Z, static_cast<uint32_t>(raw_data.gyro_z << 16));
	 
    config->set_register(UM6_ACCEL_RAW_XY, static_cast<uint32_t>((raw_data.accel_x << 16)
                                                                 | (raw_data.accel_y & 0x0FFFF)));
    config->set_register(UM6_ACCEL_RAW_Z, static_cast<uint32_t>(raw_data.accel_z << 16));
	 
    config->set_register(UM6_MAG_RAW_XY, static_cast<uint32_t>((raw_data.mag_x << 16)
                                                               | (raw_data.mag_y & 0x0FFFF)));
    config->set_register(UM6_MAG_RAW_Z, static_cast<uint32_t>(raw_data.mag_z << 16));
	 
    // Processed gyros
    // The maximum measurable rotation rate is +/-2000 deg/sec.  We have 16 bits to represent it as a signed integer
    // 2^16/4000 = 16.384
    converted_data1 = static_cast<int16_t>(roundf(state_data.gyro_x * 16.384f));
    converted_data2 = static_cast<int16_t>(roundf(state_data.gyro_y * 16.384f));
    config->set_register(UM6_GYRO_PROC_XY, static_cast<uint32_t>((converted_data1 << 16)
                                                                 | (converted_data2 & 0x0FFFF)));
	 
    converted_data1 = static_cast<int16_t>(roundf(state_data.gyro_z * 16.384f));
    config->set_register(UM6_GYRO_PROC_Z, static_cast<uint32_t>(converted_data1 << 16));
	 
    // Processed accels
    // The maximum measureable acceleration is assumed to be +/- 16g.  We have 16 bits to represent it as a signed integer
    // 2^16/32 = 2048
    converted_data1 = static_cast<int16_t>(roundf(state_data.accel_x * 2048.0f));
    converted_data2 = static_cast<int16_t>(roundf(state_data.accel_y * 2048.0f));
    config->set_register(UM6_ACCEL_PROC_XY, static_cast<uint32_t>((converted_data1 << 16)
                                                       | (converted_data2 & 0x0FFFF)));
	 
    converted_data1 = static_cast<int16_t>(roundf(state_data.accel_z * 5461.33333333f));
    config->set_register(UM6_ACCEL_PROC_Z, static_cast<uint32_t>(converted_data1 << 16));	
	 
    // Processed magnetic sensor data
    // If the magnetometer is even remotely calibrated correctly, the maximum value of any processed magnetometer axis is
    // equal to 1.0.  Can't guarantee, though, that calibration will always be good.  instead, assume maximum values of 10.
    // 2^16/20 = 3276.8
    converted_data1 = static_cast<int16_t>(roundf(state_data.mag_x * 3276.8f));
    converted_data2 = static_cast<int16_t>(roundf(state_data.mag_y * 3276.8f));
    config->set_register(UM6_MAG_PROC_XY, static_cast<uint32_t>((converted_data1 << 16)
                                                                | (converted_data2 & 0x0FFFF)));
	 
    converted_data1 = static_cast<int16_t>(roundf(state_data.mag_z * 3276.8f));
    config->set_register(UM6_MAG_PROC_Z, static_cast<uint32_t>(converted_data1 << 16));
	 
    // Euler angles
    // Maximum euler angle values are +/360 degrees.
    // 2^16/(360*2) = 91.0222222222
    converted_data1 = static_cast<int16_t>(roundf(state_data.phi * 91.0222222f));
    converted_data2 = static_cast<int16_t>(roundf(state_data.theta * 91.0222222f));
    config->set_register(UM6_EULER_PHI_THETA, static_cast<uint32_t>((converted_data1 << 16)
                                                                    | (converted_data2 & 0x0FFFF)));
	 
    converted_data1 = static_cast<int16_t>(roundf(state_data.psi * 91.0222222f));
    config->set_register(UM6_EULER_PSI, static_cast<uint32_t>(converted_data1 << 16));
	 
    // Quaternions
    // Quaternions are normalized, so the maximum value of any single element is 1.0.  However, let the value go to +/- 1.1 to 
    // to avoid overflow problems at 1.0
    // 2^16/2.2 = 29789.09091
    converted_data1 = static_cast<int16_t>(roundf(state_data.qib.a() * 29789.09091f));
    converted_data2 = static_cast<int16_t>(roundf(state_data.qib.b() * 29789.09091f));
    config->set_register(UM6_QUAT_AB, static_cast<uint32_t>((converted_data1 << 16)
                                                            | (converted_data2 & 0x0FFFF)));
	 
    converted_data1 = static_cast<int16_t>(roundf(state_data.qib.c() * 29789.09091f));
    converted_data2 = static_cast<int16_t>(roundf(state_data.qib.d() * 29789.09091f));
    config->set_register(UM6_QUAT_CD, static_cast<uint32_t>((converted_data1 << 16)
                                                            | (converted_data2 & 0x0FFFF)));

    float f;
    
    // Error covariance matrix
    f = state_data.Sigma.get(0, 0);
    config->set_register(UM6_ERROR_COV_00, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(0, 1);
    config->set_register(UM6_ERROR_COV_01, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(0, 2);
    config->set_register(UM6_ERROR_COV_02, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(0, 3);
    config->set_register(UM6_ERROR_COV_03, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(1, 0);
    config->set_register(UM6_ERROR_COV_10, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(1, 1);
    config->set_register(UM6_ERROR_COV_11, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(1, 2);
    config->set_register(UM6_ERROR_COV_12, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(1, 3);
    config->set_register(UM6_ERROR_COV_13, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(2, 0);
    config->set_register(UM6_ERROR_COV_20, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(2, 1);
    config->set_register(UM6_ERROR_COV_21, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(2, 2);
    config->set_register(UM6_ERROR_COV_22, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(2, 3);
    config->set_register(UM6_ERROR_COV_23, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(3, 0);
    config->set_register(UM6_ERROR_COV_30, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(3, 1);
    config->set_register(UM6_ERROR_COV_31, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(3, 2);
    config->set_register(UM6_ERROR_COV_32, *reinterpret_cast<uint32_t*>(&f));
	 
    f = state_data.Sigma.get(3, 3);
    config->set_register(UM6_ERROR_COV_33, *reinterpret_cast<uint32_t*>(&f));
      
    f = state_data.temperature;
    config->set_register(UM6_TEMPERATURE, *reinterpret_cast<uint32_t*>(&f));
	  
    // GPS data
    // Latitude
    f = raw_data.GPS_latitude;
    config->set_register(UM6_GPS_LATITUDE, *reinterpret_cast<uint32_t*>(&f));
    // Longitude
    f = raw_data.GPS_longitude;
    config->set_register(UM6_GPS_LONGITUDE, *reinterpret_cast<uint32_t*>(&f));	
    // Altitude
    f = raw_data.GPS_altitude;
    config->set_register(UM6_GPS_ALTITUDE, *reinterpret_cast<uint32_t*>(&f));
	  
    // Relative north position
    f = state_data.GPS_north;
    config->set_register(UM6_GPS_POSITION_N, *reinterpret_cast<uint32_t*>(&f));
	  
    // Relative east position
    f = state_data.GPS_east;
    config->set_register(UM6_GPS_POSITION_E, *reinterpret_cast<uint32_t*>(&f));
	  
    // Relative altitude
    f = state_data.GPS_h;
    config->set_register(UM6_GPS_POSITION_H, *reinterpret_cast<uint32_t*>(&f));
	  
    // GPS course and speed
    // The GPS course is in degrees, with 0.1 degree resolution.  Multiply the course by 10 and truncate.
    // The actual angle is given by dividing the register contents by 10. 
    // The speed is originally in km/hr or in knots.  The speed is converted to m/s and stored in the gStateData structure.
    // Maximum measureable velocity measured by the GPS is just over 514 m/s.  Multiply actual velocity by
    // 100 and truncate.  The actual velocity is given by dividing the register contents by 100
    uint32_t course_speed = static_cast<uint16_t>(roundf(raw_data.GPS_course * 10)) << 16;
    course_speed |= static_cast<uint16_t>(roundf(state_data.GPS_speed * 100));
    config->set_register(UM6_GPS_COURSE_SPEED, course_speed);
	  
    // Mode
    uint32_t mode = (raw_data.GPS_mode & UM6_GPS_MODE_MASK) << UM6_GPS_MODE_START_BIT;
    // Satellite count
    mode |= (raw_data.GPS_satellite_count & UM6_GPS_SAT_COUNT_MASK) << UM6_GPS_SAT_COUNT_START_BIT;
    // Satellite HDOP and VDOP
    uint16_t uint_data1 = static_cast<uint16_t>(raw_data.GPS_Hdop*10);
    uint16_t uint_data2 = static_cast<uint16_t>(raw_data.GPS_Vdop*10);
    mode |= (uint_data1 & UM6_GPS_HDOP_MASK) << UM6_GPS_HDOP_START_BIT;
    mode |= (uint_data2 & UM6_GPS_VDOP_MASK) << UM6_GPS_VDOP_START_BIT;
    config->set_register(UM6_GPS_SAT_SUMMARY, mode);

    // Individual satellite data
    config->set_register(UM6_GPS_SAT_1_2,
                         raw_data.GPS_SatSNR_2,
                         raw_data.GPS_SatID_2,
                         raw_data.GPS_SatSNR_1,
                         raw_data.GPS_SatID_1);
    config->set_register(UM6_GPS_SAT_3_4,
                         raw_data.GPS_SatSNR_4,
                         raw_data.GPS_SatID_4,
                         raw_data.GPS_SatSNR_3,
                         raw_data.GPS_SatID_3);
    config->set_register(UM6_GPS_SAT_5_6,
                         raw_data.GPS_SatSNR_6,
                         raw_data.GPS_SatID_6,
                         raw_data.GPS_SatSNR_5,
                         raw_data.GPS_SatID_5);
    config->set_register(UM6_GPS_SAT_7_8,
                         raw_data.GPS_SatSNR_8,
                         raw_data.GPS_SatID_8,
                         raw_data.GPS_SatSNR_7,
                         raw_data.GPS_SatID_7);
    config->set_register(UM6_GPS_SAT_9_10,
                         raw_data.GPS_SatSNR_10,
                         raw_data.GPS_SatID_10,
                         raw_data.GPS_SatSNR_9,
                         raw_data.GPS_SatID_9);
    config->set_register(UM6_GPS_SAT_11_12,
                         raw_data.GPS_SatSNR_12,
                         raw_data.GPS_SatID_12,
                         raw_data.GPS_SatSNR_11,
                         raw_data.GPS_SatID_11);
 
    // Status register
    // This register needn't be modified because it is always accessed directly.
	 
}

// ----------------------------------------------------------------------------

