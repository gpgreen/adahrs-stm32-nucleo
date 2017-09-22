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

void EKF::begin(ADAHRSConfig* config, ADAHRSSensorData* states)
{
    _config = config;
    _states = states;

    ADAHRS_state_data* estimated_states = &(_states->state_data);
    
    estimated_states->phi = 0;
    estimated_states->theta = 0;
    estimated_states->psi = 0;

    estimated_states->phi_dot = 0;
    estimated_states->theta_dot = 0;
    estimated_states->psi_dot = 0;

    if (_states->mode() == ADAHRSSensorData::MODE_EULER)
    {
        estimated_states->Sigma = Matrix(3, 3);
        estimated_states->R = Matrix(3, 3);
        estimated_states->Sigma.set(0, 0, estimated_states->process_var);
        estimated_states->Sigma.set(1, 1, estimated_states->process_var);
        estimated_states->Sigma.set(2, 2, estimated_states->process_var);
        estimated_states->R.set(0, 0, estimated_states->process_var);
        estimated_states->R.set(1, 1, estimated_states->process_var);
        estimated_states->R.set(2, 2, estimated_states->process_var);
    }
    else
    {
        estimated_states->Sigma = Matrix(4, 4);
        estimated_states->R = Matrix(4, 4);
        // Process variance is scaled here so that the performance in Euler Angle mode and Quaternion mode is comparable
        estimated_states->Sigma.set(0, 0, estimated_states->process_var * 0.00001f);
        estimated_states->Sigma.set(1, 1, estimated_states->process_var * 0.00001f);
        estimated_states->Sigma.set(2, 2, estimated_states->process_var * 0.00001f);
        estimated_states->Sigma.set(3, 3, estimated_states->process_var * 0.00001f);
        estimated_states->R.set(0, 0, estimated_states->process_var * 0.00001f);
        estimated_states->R.set(1, 1, estimated_states->process_var * 0.00001f);
        estimated_states->R.set(2, 2, estimated_states->process_var * 0.00001f);
        estimated_states->R.set(3, 3, estimated_states->process_var * 0.00001f);
    }
    estimated_states->qib = Quaternion(1, 0, 0, 0);

    _states->copy_config_to_states(_config);
}

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
    svec.set(0, 0, (float)((float)sensor_data->gyro_x - (float)state_data->beta_p - (state_data->beta_p0 + state_data->beta_p1*temp + state_data->beta_p2*temp2 + state_data->beta_p3*temp3) ));
    svec.set(1, 0, (float)((float)sensor_data->gyro_y - (float)state_data->beta_q - (state_data->beta_q0 + state_data->beta_q1*temp + state_data->beta_q2*temp2 + state_data->beta_q3*temp3) ));
    svec.set(2, 0, (float)((float)sensor_data->gyro_z - (float)state_data->beta_r - (state_data->beta_r0 + state_data->beta_r1*temp + state_data->beta_r2*temp2 + state_data->beta_r3*temp3) ));

    // multiply gyro measuments by alignment matrix (fixes cross-axis alignment)
    svec *= state_data->gyro_cal;

    // Copy new gyro data to state_data structure
    state_data->gyro_x = svec.get(0, 0);
    state_data->gyro_y = svec.get(1, 0);
    state_data->gyro_z = svec.get(2, 0);
    
    // Now for accelerometers
    svec.set(0, 0, ((float)(sensor_data->accel_x - state_data->beta_acc_x)));
    svec.set(1, 0, ((float)(sensor_data->accel_y - state_data->beta_acc_y)));
    svec.set(2, 0, ((float)(sensor_data->accel_z - state_data->beta_acc_z)));
	 
    svec *= state_data->accel_cal;
	 
    state_data->accel_x = svec.get(0, 0);
    state_data->accel_y = svec.get(1, 0);
    state_data->accel_z = svec.get(2, 0);
             
    // Now the magnetometer
    svec.set(0, 0, ((float)(sensor_data->mag_x - state_data->beta_mag_x)));
    svec.set(1, 0, ((float)(sensor_data->mag_y - state_data->beta_mag_y)));
    svec.set(2, 0, ((float)(sensor_data->mag_z - state_data->beta_mag_z)));

    svec *= state_data->mag_cal;
	 
    state_data->mag_x = svec.get(0, 0);
    state_data->mag_y = svec.get(1, 0);
    state_data->mag_z = svec.get(2, 0);
}

void EKF::predict()
{
    //RawSensorData* sensor_data = &(_states->raw_data);
    ADAHRS_state_data* estimated_states = &(_states->state_data);

    // Get elapsed time since last prediction (Timer3 should be configured
    // to increment once every microsecond.  It is a 16-bit timer, which means
    // that a maximum of 2^16 = 65536 microseconds can pass before overflow.
    // The prediction step should thus be run at least once every 65 milliseconds (15.6 Hz),
    // but preferably more quickly.  This shouldn't be a problem - the prediction step
    // should nominally run at roughly 1000 Hz).
    uint16_t timer_value = TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);
    
    float T = 0.000001f * static_cast<float>(timer_value);
    
    // Copy body frame angular rates to local variables for convenience
    float p = estimated_states->gyro_x;
    float q = estimated_states->gyro_y;
    float r = estimated_states->gyro_z;

    // Euler Angle Estimation
    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
        Matrix A(3, 3);
        Matrix Atranspose(3, 3);
        Matrix temp1(3, 3);
        Matrix temp2(3, 3);
        
        // Precompute trigonometric functions - these will be used more than once
        float cos_phi = cos(estimated_states->phi * .01745329f);
        float sin_phi = sin(estimated_states->phi * .01745329f);
        float cos_theta = cos(estimated_states->theta * .01745329f);
        float sin_theta = sin(estimated_states->theta * .01745329f);	 
        float tan_theta = tan(estimated_states->theta * .01745329f);
		  
        // Compute rotation rates based on body frame angular rates measured by the rate gyros
        /*
          phi_dot = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta)
          theta_dot = q*cos(phi) - r*sin(phi)
          psi_dot = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)
        */
        estimated_states->phi_dot = p + r * cos_phi * tan_theta + q * sin_phi * tan_theta;
        estimated_states->theta_dot = q * cos_phi - r * sin_phi;
        estimated_states->psi_dot = (r * cos_phi) / cos_theta + (q * sin_phi) / cos_theta;

        // Use measured rotation rates in the body frame to compute new angle estimates
        estimated_states->phi += T * estimated_states->phi_dot;
        estimated_states->theta += T * estimated_states->theta_dot;
        estimated_states->psi += T * estimated_states->psi_dot;
		  
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
        // mat_transpose( &A, &Atranspose );
        Atranspose.transpose(A);
        //mat_mult( &A, &gStateData.Sigma, &temp1 );
        temp1 = A * estimated_states->Sigma;
        //mat_mult( &temp1, &Atranspose, &temp2 );
        temp2 = Atranspose * temp1;
        //mat_add( &temp2, &gStateData.R, &gStateData.Sigma );
        estimated_states->Sigma = temp2 + estimated_states->R;
	
        // Finally, "unroll" states so that they range from -360 to 360 degrees
        unroll_states();
    }
    // Quaternion estimation
    else
    {
        Matrix A(4, 4);
        Matrix Atranspose(4, 4);
        Matrix temp1(4, 4);
        Matrix temp2(4, 4);
	
        // Convert p, q, and r to rad/s
        p = p * 3.14159f / 180.0f;
        q = q * 3.14159f / 180.0f;
        r = r * 3.14159f / 180.0f;
	
        // Create a quaternion to represent rotation rate
        Quaternion pqr_quat(0, p, q, r);
	
        // Predict new quaternion state based on gyro data
        Quaternion temp_quat = estimated_states->qib * pqr_quat;
        temp_quat = temp_quat * 0.5f * T;
        estimated_states->qib += temp_quat;
	
        // Normalize new predicted state
        estimated_states->qib.normalize();
	
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
        temp1 = A * estimated_states->Sigma;
        temp2 = temp1 * Atranspose;
        estimated_states->Sigma = temp2  + estimated_states->R;
	
        // Now use the new quaternion to compute Euler Angles
        compute_euler_angles();
    }
}

void EKF::update()
{
    RawSensorData* sensor_data = &(_states->raw_data);
    ADAHRS_state_data* estimated_states = &(_states->state_data);
    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
		  
        // If there is new accelerometer data available and accelerometer state updates are enabled,
        // run an update using the new accelerometer sensor data
        if (sensor_data->new_accel_data &&
            (_config->get_register(UM6_MISC_CONFIG) & UM6_ACCEL_UPDATE_ENABLED))
        {
            Matrix C(1, 3);
				
            // Precompute trigonometric functions - these will be used more than once
            float cos_phi = cos(estimated_states->phi * .01745329f);
            float cos_theta = cos(estimated_states->theta * .01745329f);
            float cos_psi = cos(estimated_states->psi * .01745329f);
				
            float sin_phi = sin(estimated_states->phi * .01745329f);
            float sin_theta = sin(estimated_states->theta * .01745329f);
            float sin_psi = sin(estimated_states->psi * .01745329f);

            // Copy data into local variables for convenience		  
            float ax_ref = estimated_states->accel_ref_x;
            float ay_ref = estimated_states->accel_ref_y;
            float az_ref = estimated_states->accel_ref_z;
				
            // Compute expected accelerometer output based on current state estimates (the expected output is the accelerometer reference vector
            // rotated into the body-frame of the sensor
            float ax_hat = ax_ref*cos_psi*cos_theta - az_ref*sin_theta + ay_ref*cos_theta*sin_psi;
            float ay_hat = ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + az_ref*cos_theta*sin_phi;
            float az_hat = ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta;
				
            // Compute linearized state transition matrix for each axis independently
            //		  [                                                                                                                                                  0,                          - az_ref*cos(theta) - ax_ref*cos(psi)*sin(theta) - ay_ref*sin(psi)*sin(theta),                                                                 ay_ref*cos(psi)*cos(theta) - ax_ref*cos(theta)*sin(psi)]
            //		  [ ax_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ay_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + az_ref*cos(phi)*cos(theta), ax_ref*cos(psi)*cos(theta)*sin(phi) - az_ref*sin(phi)*sin(theta) + ay_ref*cos(theta)*sin(phi)*sin(psi), - ax_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - ay_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))]
            //		  [ ax_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - ay_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - az_ref*cos(theta)*sin(phi), ax_ref*cos(phi)*cos(psi)*cos(theta) - az_ref*cos(phi)*sin(theta) + ay_ref*cos(phi)*cos(theta)*sin(psi),   ax_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + ay_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))]
				
            // x-axis
            C.set(0, 0, 0);
            C.set(0, 1, -az_ref*cos_theta - ax_ref*cos_psi*sin_theta - ay_ref*sin_psi*sin_theta);
            C.set(0, 2, ay_ref*cos_psi*cos_theta - ax_ref*cos_theta*sin_psi);
				
            // Do correction
            correction(C, estimated_states->accel_x, ax_hat, estimated_states->accel_var);
				
            // y-axis
            C.set(0, 0, ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta);
            C.set(0, 1, ax_ref*cos_psi*cos_theta*sin_phi - az_ref*sin_phi*sin_theta + ay_ref*cos_theta*sin_phi*sin_psi);
            C.set(0, 2, -ax_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - ay_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta));
				
            // Do correction
            correction(C, estimated_states->accel_y, ay_hat, estimated_states->accel_var);
				
            // z-axis
            C.set(0, 0, ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) - ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - az_ref*cos_theta*sin_phi);
            C.set(0, 1, ax_ref*cos_phi*cos_psi*cos_theta - az_ref*cos_phi*sin_theta + ay_ref*cos_phi*cos_theta*sin_psi);
            C.set(0, 2, ax_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + ay_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta));
				
            // Do correction
            correction(C, estimated_states->accel_z, az_hat, estimated_states->accel_var);
				
            // Finally, "unroll" states so that they range from -360 to 360 degrees
            unroll_states();
        }	 
		  
        // Do magnetometer update if enabled and if there is new magnetic sensor data available
        if (sensor_data->new_mag_data
            && (_config->get_register(UM6_MISC_CONFIG) & UM6_MAG_UPDATE_ENABLED))
        {
            Matrix C(1, 3);

            // Precompute trigonometric functions - these will be used more than once
            float cos_phi = cos(estimated_states->phi * .01745329f);
            float cos_theta = cos(estimated_states->theta * .01745329f);
            float cos_psi = cos(estimated_states->psi * .01745329f);
				
            float sin_phi = sin(estimated_states->phi * .01745329f);
            float sin_theta = sin(estimated_states->theta * .01745329f);
            float sin_psi = sin(estimated_states->psi * .01745329f);
				
            // Copy data into local variables for convenience		  
            float mx_ref = estimated_states->mag_ref_x;
            float my_ref = estimated_states->mag_ref_y;
            //float mz_ref = estimated_states->mag_ref_z;
				
            // Rotate the sensor measurement into the vehicle-1 frame (undo pitch and roll)
            float mx_v1 = estimated_states->mag_x*cos_theta + estimated_states->mag_z*cos_phi*sin_theta + estimated_states->mag_y*sin_phi*sin_theta;
            float my_v1 = estimated_states->mag_y*cos_phi - estimated_states->mag_z*sin_phi;
            //float mz_v1 =  estimated_states->mag_z*cos_phi*cos_theta - estimated_states->mag_x*sin_theta + estimated_states->mag_y*cos_theta*sin_phi;
				
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
            correction(C, mx_v1, mx_hat, estimated_states->mag_var);
				
            // y-axis
            C.set(0, 0, 0);
            C.set(0, 1, 0);
            C.set(0, 2, -mx_ref*cos_psi - my_ref*sin_psi);
				
            // Do correction
            correction(C, my_v1, my_hat, estimated_states->mag_var);
				
            /* z-axis doesn't do anything in the vehicle-1 frame
            // z-axis
            C.set(0, 0, 0);
            C.set(0, 1, 0);
            C.set(0, 2, 0);
				
            // Do correction
            correction(C, mz_v1, mz_hat, estimated_states->mag_var, estimated_states);
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
            float sensor_norm = sqrt(estimated_states->accel_x*estimated_states->accel_x + estimated_states->accel_y*estimated_states->accel_y + estimated_states->accel_z*estimated_states->accel_z);
            Matrix C(1, 4);
				
            // Make sure this accelerometer measurement is close to 1 G.  If not, ignore it.
            if (fabs(sensor_norm - 1) < 0.3)
            {
                // Make local copies of the current quaternion state estimate for convenience
                float a = estimated_states->qib.a();
                float b = estimated_states->qib.b();
                float c = estimated_states->qib.c();
                float d = estimated_states->qib.d();
					 
                // Make local copy of accelerometer reference vector for convenience
                float ax_ref = estimated_states->accel_ref_x;
                float ay_ref = estimated_states->accel_ref_y;
                float az_ref = estimated_states->accel_ref_z;
					 
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
                correction(C, estimated_states->accel_x, ax_hat, estimated_states->accel_var);
					 
                // REPEAT FOR Y-AXIS
                // Make local copies of the current quaternion state estimate for convenience
                a = estimated_states->qib.a();
                b = estimated_states->qib.b();
                c = estimated_states->qib.c();
                d = estimated_states->qib.d();
					 
                // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
                float ay_hat = az_ref*(2*a*b + 2*c*d) - ax_ref*(2*a*d - 2*b*c) + ay_ref*(a*a - b*b + c*c - d*d);
                float az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);
					 
                C.set(0, 0, 2*a*ay_ref + 2*az_ref*b - 2*ax_ref*d);
                C.set(0, 1, 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c);
                C.set(0, 2, 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d);
                C.set(0, 3, 2*az_ref*c - 2*a*ax_ref - 2*ay_ref*d);
					 
                // Do correction
                correction(C, estimated_states->accel_y, ay_hat, estimated_states->accel_var);
					 
                // REPEAT FOR Z-AXIS
                // Make local copies of the current quaternion state estimate for convenience
                a = estimated_states->qib.a();
                b = estimated_states->qib.b();
                c = estimated_states->qib.c();
                d = estimated_states->qib.d();
					 
                // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
                az_hat = ax_ref*(2*a*c + 2*b*d) - ay_ref*(2*a*b - 2*c*d) + az_ref*(a*a - b*b - c*c + d*d);
					 
                C.set(0, 0, 2*a*az_ref - 2*ay_ref*b + 2*ax_ref*c);
                C.set(0, 1, 2*ax_ref*d - 2*az_ref*b - 2*a*ay_ref);
                C.set(0, 2, 2*a*ax_ref - 2*az_ref*c + 2*ay_ref*d);
                C.set(0, 3, 2*ax_ref*b + 2*ay_ref*c + 2*az_ref*d);
					 
                // Do correction
                correction(C, estimated_states->accel_z, az_hat, estimated_states->accel_var);
            }
        }
		  
        // Do magnetometer update if enabled and if there is new magnetic sensor data available
        if (sensor_data->new_mag_data
            && (_config->get_register(UM6_MISC_CONFIG) & UM6_MAG_UPDATE_ENABLED))
        {
            Matrix C(1, 4);
				
            // Make local copies of the current quaternion state estimate for convenience
            float a = estimated_states->qib.a();
            float b = estimated_states->qib.b();
            float c = estimated_states->qib.c();
            float d = estimated_states->qib.d();
				
            // Make local copy of accelerometer reference vector for convenience
            float mx_ref = estimated_states->mag_ref_x;
            float my_ref = estimated_states->mag_ref_y;
            float mz_ref = estimated_states->mag_ref_z;
				
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
            correction(C, estimated_states->mag_x, mx_hat, estimated_states->mag_var);
				
            // REPEAT FOR Y-AXIS
            // Make local copies of the current quaternion state estimate for convenience
            a = estimated_states->qib.a();
            b = estimated_states->qib.b();
            c = estimated_states->qib.c();
            d = estimated_states->qib.d();
				
            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            float my_hat = mz_ref*(2*a*b + 2*c*d) - mx_ref*(2*a*d - 2*b*c) + my_ref*(a*a - b*b + c*c - d*d);
            float mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);
				
            C.set(0, 0, 2*a*my_ref + 2*mz_ref*b - 2*mx_ref*d);
            C.set(0, 1, 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c);
            C.set(0, 2, 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d);
            C.set(0, 3, 2*mz_ref*c - 2*a*mx_ref - 2*my_ref*d);
				
            // Do correction
            correction(C, estimated_states->mag_y, my_hat, estimated_states->mag_var);
				
            // REPEAT FOR Z-AXIS
            // Make local copies of the current quaternion state estimate for convenience
            a = estimated_states->qib.a();
            b = estimated_states->qib.b();
            c = estimated_states->qib.c();
            d = estimated_states->qib.d();
				
            // Compute expected accelerometer measurements based on the current attitude quaternion and the accel reference vector
            mz_hat = mx_ref*(2*a*c + 2*b*d) - my_ref*(2*a*b - 2*c*d) + mz_ref*(a*a - b*b - c*c + d*d);
				
            C.set(0, 0, 2*a*mz_ref - 2*my_ref*b + 2*mx_ref*c);
            C.set(0, 1, 2*mx_ref*d - 2*mz_ref*b - 2*a*my_ref);
            C.set(0, 2, 2*a*mx_ref - 2*mz_ref*c + 2*my_ref*d);
            C.set(0, 3, 2*mx_ref*b + 2*my_ref*c + 2*mz_ref*d);
				
            // Do correction
            correction(C, estimated_states->mag_z, mz_hat, estimated_states->mag_var);
        }
    }
}

void EKF::correction(const Matrix& C, float sensor_data, float sensor_hat, float sensor_covariance)
{
    Matrix temp1,temp2;
	 
    ADAHRS_state_data* estimated_states = &(_states->state_data);

    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
        temp1 = Matrix(3, 1);
        temp2 = Matrix(3, 3);
    }
    else
    {
        temp1 = Matrix(4, 1);
        temp2 = Matrix(4, 4);
    }

    Matrix Ctranspose;
    Ctranspose.transpose(C);
	 
    // Compute Kalman Gain (L = Sigma*C'*(C*Sigma*C' + Q)^-1 )
    temp1 = estimated_states->Sigma * Ctranspose;
    temp2 = C * estimated_states->Sigma;
    temp2 *= Ctranspose;
    float gain_scale = 1 / (temp2.get(0, 0) + sensor_covariance);
    Matrix L = temp1 * gain_scale;
	 
    // Update state estimates
    float error = sensor_data - sensor_hat;
	 
    if ((_config->get_register(UM6_MISC_CONFIG) & UM6_QUAT_ESTIMATE_ENABLED) == 0)
    {
        estimated_states->phi += L.get(0, 0) * error;
        estimated_states->theta += L.get(1, 0) * error;
        estimated_states->psi += L.get(2, 0) * error;

        temp1 = Matrix(3, 3, true);
    }
    else
    {
        estimated_states->qib = Quaternion(
            estimated_states->qib.a() + L.get(0, 0) * error,
            estimated_states->qib.b() + L.get(1, 0) * error,
            estimated_states->qib.c() + L.get(2, 0) * error,
            estimated_states->qib.d() + L.get(3, 0) * error);
        temp1 = Matrix(4, 4, true);
    }
	 
    // Now update the covariance estimate (Sigma = (I - L*C)*Sigma
    temp2 = L * C;
    temp2 = temp2 * -1.0f;
    temp1 += temp2;
    estimated_states->Sigma = temp1 * estimated_states->Sigma;
}

void EKF::compute_euler_angles()
{
}

void EKF::unroll_states()
{
}

// ----------------------------------------------------------------------------

