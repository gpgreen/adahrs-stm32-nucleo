/*
 * adahrs_ekf.cpp
 *
 *  Created on: Sep 21, 2017
 *      Author: ggreen
 */

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
}

void EKF::predict()
{
}

void EKF::update()
{
}

void EKF::correction()
{
}

void EKF::compute_euler_angles()
{
}

void EKF::unroll_states()
{
}

// ----------------------------------------------------------------------------

