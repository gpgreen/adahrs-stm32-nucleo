/*
 * adahrs_ekf.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: ggreen
 */

#include <math.h>
#include "adahrs_ekf.h"

// ----------------------------------------------------------------------------

EKF::EKF()
    : _config(nullptr), _states(nullptr)
{
    // does nothing else
}

// initialize the kalman filter
void EKF::begin(ADAHRSConfig* config, ADAHRSSensorData* estimated_states)
{
    _config = config;
    _states = estimated_states;

    ADAHRS_state_data* states = &(_states->state_data);
    
    states->phi = 0;
    states->theta = 0;
    states->psi = 0;

    states->phi_dot = 0;
    states->theta_dot = 0;
    states->psi_dot = 0;

    if (_states->mode() == ADAHRSSensorData::MODE_EULER)
    {
        states->Sigma = Matrix(3, 3);
        states->R = Matrix(3, 3);
        states->Sigma.set(0, 0, states->process_var);
        states->Sigma.set(1, 1, states->process_var);
        states->Sigma.set(2, 2, states->process_var);
        states->R.set(0, 0, states->process_var);
        states->R.set(1, 1, states->process_var);
        states->R.set(2, 2, states->process_var);
    }
    else
    {
        states->Sigma = Matrix(4, 4);
        states->R = Matrix(4, 4);
        // Process variance is scaled here so that the performance in Euler Angle mode and Quaternion mode is comparable
        states->Sigma.set(0, 0, states->process_var * 0.00001f);
        states->Sigma.set(1, 1, states->process_var * 0.00001f);
        states->Sigma.set(2, 2, states->process_var * 0.00001f);
        states->Sigma.set(3, 3, states->process_var * 0.00001f);
        states->R.set(0, 0, states->process_var * 0.00001f);
        states->R.set(1, 1, states->process_var * 0.00001f);
        states->R.set(2, 2, states->process_var * 0.00001f);
        states->R.set(3, 3, states->process_var * 0.00001f);
    }
    states->qib = Quaternion(1, 0, 0, 0);

    _states->copy_config_to_states(_config);

    // Enable TIM4 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // setup timer4 for prediction loop
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_DeInit(TIM4);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // enable the timer
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

// main Extended Kalman Filter process, get data, predict, update
void EKF::estimate_states()
{
    // First, convert raw sensor data to actual data (acceleration to gravities, gyro data
    // to angular rates, magnetometer to unit-norm data
    convert_raw_sensor_data();	 
    
    // Run EKF prediction step
    predict();
    
    // Run EKF update step
    update();
    
    // Copy the new states into the communication interface structures
    _states->copy_states_to_config(_config);
    
}

// Converts the raw sensor data in sensor_data to actual data (angular rates, 
// acceleration in gravities, etc. and stores in state_data.  Also performs
// calibration functions.
void EKF::convert_raw_sensor_data()
{
    RawSensorData* sensor_data = &(_states->raw_data);
    ADAHRS_state_data* state_data = &(_states->state_data);
    
    /// convert temperature data
    float temp = sensor_data->temperature * 0.00357143f + 70.00f;
    float temp2 = temp * temp;
    float temp3 = temp2 * temp;

    state_data->temperature = temp;

    // Rate gyros
    Matrix svec(3, 1);
    svec.set(0, 0, static_cast<float>(sensor_data->gyro_x)
             - static_cast<float>(state_data->beta_p)
             - (state_data->beta_p0 + state_data->beta_p1 * temp
                + state_data->beta_p2 * temp2 + state_data->beta_p3 * temp3));
    svec.set(1, 0, static_cast<float>(sensor_data->gyro_y)
             - static_cast<float>(state_data->beta_q)
             - (state_data->beta_q0 + state_data->beta_q1*temp
                + state_data->beta_q2 * temp2 + state_data->beta_q3 * temp3));
    svec.set(2, 0, static_cast<float>(sensor_data->gyro_z)
             - static_cast<float>(state_data->beta_r)
             - (state_data->beta_r0 + state_data->beta_r1 * temp
                + state_data->beta_r2 * temp2 + state_data->beta_r3 * temp3));

    // multiply gyro measuments by alignment matrix (fixes cross-axis alignment)
    svec *= state_data->gyro_cal;

    // Copy new gyro data to state_data structure
    state_data->gyro_x = svec.get(0, 0);
    state_data->gyro_y = svec.get(1, 0);
    state_data->gyro_z = svec.get(2, 0);
    
    // Now for accelerometers
    svec.set(0, 0, static_cast<float>(sensor_data->accel_x - state_data->beta_acc_x));
    svec.set(1, 0, static_cast<float>(sensor_data->accel_y - state_data->beta_acc_y));
    svec.set(2, 0, static_cast<float>(sensor_data->accel_z - state_data->beta_acc_z));
	 
    svec *= state_data->accel_cal;
	 
    state_data->accel_x = svec.get(0, 0);
    state_data->accel_y = svec.get(1, 0);
    state_data->accel_z = svec.get(2, 0);
             
    // Now the magnetometer
    svec.set(0, 0, static_cast<float>(sensor_data->mag_x - state_data->beta_mag_x));
    svec.set(1, 0, static_cast<float>(sensor_data->mag_y - state_data->beta_mag_y));
    svec.set(2, 0, static_cast<float>(sensor_data->mag_z - state_data->beta_mag_z));

    svec *= state_data->mag_cal;
	 
    state_data->mag_x = svec.get(0, 0);
    state_data->mag_y = svec.get(1, 0);
    state_data->mag_z = svec.get(2, 0);
}

// predict, use rate gyros to make new orientation estimate
void EKF::predict()
{
    ADAHRS_state_data* states = &(_states->state_data);

    // Get elapsed time since last prediction (Timer should be configured
    // to increment once every microsecond.  It is a 16-bit timer, which means
    // that a maximum of 2^16 = 65536 microseconds can pass before overflow.
    // The prediction step should thus be run at least once every 65 milliseconds (15.6 Hz),
    // but preferably more quickly.  This shouldn't be a problem - the prediction step
    // should nominally run at roughly 1000 Hz).
    uint16_t timer_value = TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);
    
    float T = 0.000001f * static_cast<float>(timer_value);
    
    // Copy body frame angular rates to local variables for convenience
    float p = states->gyro_x;
    float q = states->gyro_y;
    float r = states->gyro_z;

    // Euler Angle Estimation
    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
        Matrix A(3, 3);
        Matrix Atranspose(3, 3);
        
        // Precompute trigonometric functions - these will be used more than once
        float cos_phi = cos(states->phi * .01745329f);
        float sin_phi = sin(states->phi * .01745329f);
        float cos_theta = cos(states->theta * .01745329f);
        float sin_theta = sin(states->theta * .01745329f);	 
        float tan_theta = tan(states->theta * .01745329f);
		  
        // Compute rotation rates based on body frame angular rates measured by the rate gyros
        /*
          phi_dot = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta)
          theta_dot = q*cos(phi) - r*sin(phi)
          psi_dot = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)
        */
        states->phi_dot = p + r * cos_phi * tan_theta + q * sin_phi * tan_theta;
        states->theta_dot = q * cos_phi - r * sin_phi;
        states->psi_dot = (r * cos_phi) / cos_theta + (q * sin_phi) / cos_theta;

        // Use measured rotation rates in the body frame to compute new angle estimates
        states->phi += T * states->phi_dot;
        states->theta += T * states->theta_dot;
        states->psi += T * states->psi_dot;
		  
        // DISCRETE STATE TRANSITION
        //	 [ T*q*cos(phi)*tan(theta) - T*r*sin(phi)*tan(theta) + 1,               T*r*cos(phi)*(tan(theta)^2 + 1) + T*q*sin(phi)*(tan(theta)^2 + 1), 0]
        //	 [                         - T*q*sin(phi) - T*r*cos(phi),                                                                               1, 0]
        //	 [ (T*q*cos(phi))/cos(theta) - (T*r*sin(phi))/cos(theta), (T*r*cos(phi)*sin(theta))/cos(theta)^2 + (T*q*sin(phi)*sin(theta))/cos(theta)^2, 1]
        A.set(0, 0, T * (q * cos_phi * tan_theta - r * sin_phi * tan_theta) + 1);
        A.set(0, 1, T * (r * cos_phi * (tan_theta * tan_theta + 1)
                         + q * sin_phi * (tan_theta * tan_theta + 1)));
        A.set(0, 2, 0);
        A.set(1, 0, T * (-r * cos_phi - q * sin_phi));
        A.set(1, 1, 1);
        A.set(1, 2, 0);
        A.set(2, 0, T * ((q * cos_phi) / cos_theta - (r * sin_phi) / cos_theta));
        A.set(2, 1, T * ((r * cos_phi * sin_theta) / (cos_theta * cos_theta)
                         + (q * sin_phi * sin_theta) / (cos_theta * cos_theta))); 
        A.set(2, 2, 1);

        // Compute the new covariance estimate (discrete update: Sigma = A*Sigma*Atranspose + R
        Atranspose.transpose(A);
        states->Sigma = (A * states->Sigma * Atranspose) + states->R;
	
        // Finally, "unroll" states so that they range from -360 to 360 degrees
        unroll_states();
    }
    // Quaternion estimation
    else
    {
        Matrix A(4, 4);
        Matrix Atranspose(4, 4);
	
        // Convert p, q, and r to rad/s
        p = p * 3.14159f / 180.0f;
        q = q * 3.14159f / 180.0f;
        r = r * 3.14159f / 180.0f;
	
        // Create a quaternion to represent rotation rate
        Quaternion pqr_quat(0, p, q, r);
	
        // Predict new quaternion state based on gyro data
        Quaternion temp_quat = states->qib * pqr_quat;
        temp_quat = temp_quat * 0.5f * T;
        states->qib += temp_quat;
	
        // Normalize new predicted state
        states->qib.normalize();
	
        // PROPAGATE COVARIANCE
        // Compute linearized state transition matrix
        /*
          [       1, -(T*p)/2, -(T*q)/2, -(T*r)/2]
          [ (T*p)/2,        1,  (T*r)/2, -(T*q)/2]
          [ (T*q)/2, -(T*r)/2,        1,  (T*p)/2]
          [ (T*r)/2,  (T*q)/2, -(T*p)/2,        1]
        */
        A.set(0, 0, 1);
        A.set(0, 1, -(T * p) / 2);
        A.set(0, 2, -(T * q) / 2);
        A.set(0, 3, -(T * r) / 2);
	
        A.set(1, 0, (T * p) / 2);
        A.set(1, 1, 1);
        A.set(1, 2, (T * r) / 2);
        A.set(1, 3, -(T * q) / 2);
	
        A.set(2, 0, (T * q) / 2);
        A.set(2, 1, -(T * r) / 2);
        A.set(2, 2, 1);
        A.set(2, 3, (T * p) / 2);
	
        A.set(3, 0, (T * r) / 2);
        A.set(3, 1, (T * q) / 2);
        A.set(3, 2, -(T * p) / 2);
        A.set(3, 3, 1);
	
        // Compute the new covariance estimate (discrete update: Sigma = A*Sigma*Atranspose + R
        Atranspose.transpose(A);
        states->Sigma = (A * states->Sigma * Atranspose) + states->R;
	
        // Now use the new quaternion to compute Euler Angles
        compute_euler_angles();
    }
}

// update - use accelerations to correct pitch and roll errors and magnetic sensors to correct
// yaw errors. Compensation is only applied when new data is available. This is flagged in sensor_data.
void EKF::update()
{
    RawSensorData* sensor_data = &(_states->raw_data);
    ADAHRS_state_data* states = &(_states->state_data);
    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
		  
        // If there is new accelerometer data available and accelerometer state updates are enabled,
        // run an update using the new accelerometer sensor data
        if (sensor_data->new_accel_data &&
            (_config->get_register(UM6_MISC_CONFIG) & UM6_ACCEL_UPDATE_ENABLED))
        {
            Matrix C(1, 3);
				
            // Precompute trigonometric functions - these will be used more than once
            float cos_phi = cos(states->phi * .01745329f);
            float cos_theta = cos(states->theta * .01745329f);
            float cos_psi = cos(states->psi * .01745329f);
				
            float sin_phi = sin(states->phi * .01745329f);
            float sin_theta = sin(states->theta * .01745329f);
            float sin_psi = sin(states->psi * .01745329f);

            // Copy data into local variables for convenience		  
            float ax_ref = states->accel_ref_x;
            float ay_ref = states->accel_ref_y;
            float az_ref = states->accel_ref_z;
				
            // Compute expected accelerometer output based on current state estimates (the expected output is the accelerometer reference vector
            // rotated into the body-frame of the sensor
            float ax_hat = ax_ref*cos_psi*cos_theta - az_ref*sin_theta + ay_ref*cos_theta*sin_psi;
            float ay_hat = ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta)
                - ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + az_ref*cos_theta*sin_phi;
            float az_hat = ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta)
                - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta;
				
            // Compute linearized state transition matrix for each axis independently
            //		  [                                                                                                                                                  0,                          - az_ref*cos(theta) - ax_ref*cos(psi)*sin(theta) - ay_ref*sin(psi)*sin(theta),                                                                 ay_ref*cos(psi)*cos(theta) - ax_ref*cos(theta)*sin(psi)]
            //		  [ ax_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ay_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + az_ref*cos(phi)*cos(theta), ax_ref*cos(psi)*cos(theta)*sin(phi) - az_ref*sin(phi)*sin(theta) + ay_ref*cos(theta)*sin(phi)*sin(psi), - ax_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - ay_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))]
            //		  [ ax_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - ay_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - az_ref*cos(theta)*sin(phi), ax_ref*cos(phi)*cos(psi)*cos(theta) - az_ref*cos(phi)*sin(theta) + ay_ref*cos(phi)*cos(theta)*sin(psi),   ax_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + ay_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))]
				
            // x-axis
            C.set(0, 0, 0);
            C.set(0, 1, -az_ref*cos_theta - ax_ref*cos_psi*sin_theta - ay_ref*sin_psi*sin_theta);
            C.set(0, 2, ay_ref*cos_psi*cos_theta - ax_ref*cos_theta*sin_psi);
				
            // Do correction
            correction(C, states->accel_x, ax_hat, states->accel_var);
				
            // y-axis
            C.set(0, 0, ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta)
                  - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta);
            C.set(0, 1, ax_ref*cos_psi*cos_theta*sin_phi - az_ref*sin_phi*sin_theta
                  + ay_ref*cos_theta*sin_phi*sin_psi);
            C.set(0, 2, -ax_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta)
                  - ay_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta));
				
            // Do correction
            correction(C, states->accel_y, ay_hat, states->accel_var);
				
            // z-axis
            C.set(0, 0, ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta)
                  - ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - az_ref*cos_theta*sin_phi);
            C.set(0, 1, ax_ref*cos_phi*cos_psi*cos_theta - az_ref*cos_phi*sin_theta
                  + ay_ref*cos_phi*cos_theta*sin_psi);
            C.set(0, 2, ax_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta)
                  + ay_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta));
				
            // Do correction
            correction(C, states->accel_z, az_hat, states->accel_var);
				
            // Finally, "unroll" states so that they range from -360 to 360 degrees
            unroll_states();
        }	 
		  
        // Do magnetometer update if enabled and if there is new magnetic sensor data available
        if (sensor_data->new_mag_data
            && (_config->get_register(UM6_MISC_CONFIG) & UM6_MAG_UPDATE_ENABLED))
        {
            Matrix C(1, 3);

            // Precompute trigonometric functions - these will be used more than once
            float cos_phi = cos(states->phi * .01745329f);
            float cos_theta = cos(states->theta * .01745329f);
            float cos_psi = cos(states->psi * .01745329f);
				
            float sin_phi = sin(states->phi * .01745329f);
            float sin_theta = sin(states->theta * .01745329f);
            float sin_psi = sin(states->psi * .01745329f);
				
            // Copy data into local variables for convenience		  
            float mx_ref = states->mag_ref_x;
            float my_ref = states->mag_ref_y;
            //float mz_ref = states->mag_ref_z;
				
            // Rotate the sensor measurement into the vehicle-1 frame (undo pitch and roll)
            float mx_v1 = states->mag_x*cos_theta + states->mag_z*cos_phi*sin_theta + states->mag_y*sin_phi*sin_theta;
            float my_v1 = states->mag_y*cos_phi - states->mag_z*sin_phi;
            //float mz_v1 =  states->mag_z*cos_phi*cos_theta - states->mag_x*sin_theta + states->mag_y*cos_theta*sin_phi;
				
            // Compute expected magnetometer output based on current state estimates
            // (rotates the reference vector into the vehicle-1 frame)
            float mx_hat = mx_ref*cos_psi + my_ref*sin_psi;
            float my_hat = my_ref*cos_psi - mx_ref*sin_psi;
            //float mz_hat = mz_ref;
				
            // Compute linearized state transition matrix for each axis independently	  
            // x-axis
            C.set(0, 0, 0);
            C.set(0, 1, 0);
            C.set(0, 2, my_ref*cos_psi - mx_ref*sin_psi);
				
            // Do correction
            correction(C, mx_v1, mx_hat, states->mag_var);
				
            // y-axis
            C.set(0, 0, 0);
            C.set(0, 1, 0);
            C.set(0, 2, -mx_ref*cos_psi - my_ref*sin_psi);
				
            // Do correction
            correction(C, my_v1, my_hat, states->mag_var);
				
            /* z-axis doesn't do anything in the vehicle-1 frame
            // z-axis
            C.set(0, 0, 0);
            C.set(0, 1, 0);
            C.set(0, 2, 0);
				
            // Do correction
            correction(C, mz_v1, mz_hat, states->mag_var, states);
            */
				
            // Finally, "unroll" states so that they range from -360 to 360 degrees
            unroll_states();
        }
    }
    // QUATERNION CODE
    else
    {
        // Do accelerometer update if enabled and if there is new accelerometer sensor data available
        if (sensor_data->new_accel_data
            && (_config->get_register(UM6_MISC_CONFIG) & UM6_ACCEL_UPDATE_ENABLED))
        {
            float sensor_norm = sqrt(states->accel_x*states->accel_x + states->accel_y*states->accel_y + states->accel_z*states->accel_z);
            Matrix C(1, 4);
				
            // Make sure this accelerometer measurement is close to 1 G.  If not, ignore it.
            if (fabs(sensor_norm - 1) < 0.3)
            {
                // Make local copies of the current quaternion state estimate for convenience
                float a = states->qib.a();
                float b = states->qib.b();
                float c = states->qib.c();
                float d = states->qib.d();
					 
                // Make local copy of accelerometer reference vector for convenience
                float ax_ref = states->accel_ref_x;
                float ay_ref = states->accel_ref_y;
                float az_ref = states->accel_ref_z;
					 
                // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
                float ax_hat = ay_ref*(2*a*d + 2*b*c) - az_ref*(2*a*c - 2*b*d) + ax_ref*(a*a + b*b - c*c - d*d);
					 					 
                // Create linearized update matrix for x-axis accelerometer
                /* For all axes, C is given by:
                   [ 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d, 2*ay*b - 2*a*az - 2*ax*c, 2*a*ay + 2*az*b - 2*ax*d]
                   [ 2*a*ay + 2*az*b - 2*ax*d, 2*a*az - 2*ay*b + 2*ax*c, 2*ax*b + 2*ay*c + 2*az*d, 2*az*c - 2*a*ax - 2*ay*d]
                   [ 2*a*az - 2*ay*b + 2*ax*c, 2*ax*d - 2*az*b - 2*a*ay, 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d]					 
                */
                C.set(0, 0, 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d);
                C.set(0, 1, 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d);
                C.set(0, 2, 2*ay_ref*b - 2*a*az_ref - 2*ax_ref*c);
                C.set(0, 3, 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d);
					 
                // Do correction
                correction(C, states->accel_x, ax_hat, states->accel_var);
					 
                // REPEAT FOR Y-AXIS
                // Make local copies of the current quaternion state estimate for convenience
                a = states->qib.a();
                b = states->qib.b();
                c = states->qib.c();
                d = states->qib.d();
					 
                // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
                float ay_hat = az_ref*(2*a*b + 2*c*d) - ax_ref*(2*a*d - 2*b*c) + ay_ref*(a*a - b*b + c*c - d*d);
                float az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);
					 
                C.set(0, 0, 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d);
                C.set(0, 1, 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c);
                C.set(0, 2, 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d);
                C.set(0, 3, 2*az_ref*c - 2*a*ax_ref - 2*ay_ref*d);
					 
                // Do correction
                correction(C, states->accel_y, ay_hat, states->accel_var);
					 
                // REPEAT FOR Z-AXIS
                // Make local copies of the current quaternion state estimate for convenience
                a = states->qib.a();
                b = states->qib.b();
                c = states->qib.c();
                d = states->qib.d();
					 
                // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
                az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);
					 
                C.set(0, 0, 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c);
                C.set(0, 1, 2*ax_ref*d - 2*az_ref*b - 2*a*ay_ref);
                C.set(0, 2, 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d);
                C.set(0, 3, 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d);
					 
                // Do correction
                correction(C, states->accel_z, az_hat, states->accel_var);
            }
        }
		  
        // Do magnetometer update if enabled and if there is new magnetic sensor data available
        if (sensor_data->new_mag_data
            && (_config->get_register(UM6_MISC_CONFIG) & UM6_MAG_UPDATE_ENABLED))
        {
            Matrix C(1, 4);
				
            // Make local copies of the current quaternion state estimate for convenience
            float a = states->qib.a();
            float b = states->qib.b();
            float c = states->qib.c();
            float d = states->qib.d();
				
            // Make local copy of accelerometer reference vector for convenience
            float mx_ref = states->mag_ref_x;
            float my_ref = states->mag_ref_y;
            float mz_ref = states->mag_ref_z;
				
            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            float mx_hat = my_ref*(2*a*d + 2*b*c) - mz_ref*(2*a*c - 2*b*d) + mx_ref*(a*a + b*b - c*c - d*d);
									
            // Create linearized update matrix for x-axis accelerometer
            /* For all axes, C is given by:
               [ 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d, 2*ay*b - 2*a*az - 2*ax*c, 2*a*ay + 2*az*b - 2*ax*d]
               [ 2*a*ay + 2*az*b - 2*ax*d, 2*a*az - 2*ay*b + 2*ax*c, 2*ax*b + 2*ay*c + 2*az*d, 2*az*c - 2*a*ax - 2*ay*d]
               [ 2*a*az - 2*ay*b + 2*ax*c, 2*ax*d - 2*az*b - 2*a*ay, 2*a*ax - 2*az*c + 2*ay*d, 2*ax*b + 2*ay*c + 2*az*d]					 
            */
            C.set(0, 0, 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d);
            C.set(0, 1, 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d);
            C.set(0, 2, 2*my_ref*b - 2*a*mz_ref - 2*mx_ref*c);
            C.set(0, 3, 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*0);
				
            // Do correction
            correction(C, states->mag_x, mx_hat, states->mag_var);
				
            // REPEAT FOR Y-AXIS
            // Make local copies of the current quaternion state estimate for convenience
            a = states->qib.a();
            b = states->qib.b();
            c = states->qib.c();
            d = states->qib.d();
				
            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            float my_hat = mz_ref*(2*a*b + 2*c*d) - mx_ref*(2*a*d - 2*b*c) + my_ref*(a*a - b*b + c*c - d*d);
            float mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);
				
            C.set(0, 0, 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*d);
            C.set(0, 1, 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c);
            C.set(0, 2, 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d);
            C.set(0, 3, 2*mz_ref*c - 2*a*mx_ref - 2*my_ref*d);
				
            // Do correction
            correction(C, states->mag_y, my_hat, states->mag_var);
				
            // REPEAT FOR Z-AXIS
            // Make local copies of the current quaternion state estimate for convenience
            a = states->qib.a();
            b = states->qib.b();
            c = states->qib.c();
            d = states->qib.d();
				
            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);
				
            C.set(0, 0, 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c);
            C.set(0, 1, 2*mx_ref*d - 2*mz_ref*b - 2*a*my_ref);
            C.set(0, 2, 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d);
            C.set(0, 3, 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d);
				
            // Do correction
            correction(C, states->mag_z, mz_hat, states->mag_var);
        }
    }
}

void EKF::correction(const Matrix& C, float sensor_data, float sensor_hat, float sensor_covariance)
{
    Matrix temp1;
	 
    ADAHRS_state_data* states = &(_states->state_data);

    Matrix Ctranspose;
    Ctranspose.transpose(C);
	 
    // Compute Kalman Gain (L = Sigma*C'*(C*Sigma*C' + Q)^-1 )
    temp1 = C * states->Sigma * Ctranspose;
    float gain_scale = 1 / (temp1.get(0, 0) + sensor_covariance);
    Matrix L = (states->Sigma * Ctranspose) * gain_scale;
	 
    // Update state estimates
    float error = sensor_data - sensor_hat;
	 
    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
        states->phi += L.get(0, 0) * error;
        states->theta += L.get(1, 0) * error;
        states->psi += L.get(2, 0) * error;

        temp1 = Matrix(3, 3, true);
    }
    else
    {
        states->qib = Quaternion(
            states->qib.a() + L.get(0, 0) * error,
            states->qib.b() + L.get(1, 0) * error,
            states->qib.c() + L.get(2, 0) * error,
            states->qib.d() + L.get(3, 0) * error);
        temp1 = Matrix(4, 4, true);
    }
	 
    // Now update the covariance estimate (Sigma = (I - L*C)*Sigma
    states->Sigma = (temp1 - (L * C)) * states->Sigma;
}

// converts quaternion attitude estimate to euler angles (yaw, pitch, roll)
void EKF::compute_euler_angles()
{
    ADAHRS_state_data* states = &(_states->state_data);

    float q0 = states->qib.a();
    float q1 = states->qib.b();
    float q2 = states->qib.c();
    float q3 = states->qib.d();
    
    states->phi = atan2(2 * (q0 * q1 + q2 * q3), q3 * q3 - q2 * q2 - q1 * q1
                        + q0 * q0) * 180.0f / 3.14159f;
    states->theta = -asin(2 * (q1 * q3 - q0 * q2)) * 180.0f / 3.14159f;
    states->psi = atan2(2 * (q0 * q3 + q1 * q2),
                        q1 * q1 + q0 * q0 - q3 * q3 - q2 * q2) * 180.0f / 3.14159f;
}

// keeps all angle estimates in the range of -360 to 360 degrees
void EKF::unroll_states()
{
    ADAHRS_state_data* states = &(_states->state_data);

    while( states->phi > 360 )
    {
        states->phi -= 360;
    }
    while( states->phi < -360 )
    {
        states->phi += 360;
    }
    
    while( states->theta > 360 )
    {
        states->theta -= 360;
    }
    while( states->theta < -360 )
    {
        states->theta += 360;
    }
    
    while( states->psi > 360 )
    {
        states->psi -= 360;
    }
    while( states->psi < -360 )
    {
        states->psi += 360;
    }
}

// ----------------------------------------------------------------------------

