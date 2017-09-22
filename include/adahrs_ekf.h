/*
 * adahrs_ekf.h
 *
 *  Created on: Sep 17, 2017
 *      Author: ggreen
 */

#ifndef ADAHRS_EKF_H_
#define ADAHRS_EKF_H_

#include "cmsis_device.h"
#include "adahrs_config.h"
#include "adahrs_states.h"
#include "adahrs_matrix.h"

// ----------------------------------------------------------------------------

class EKF
{
public:

    EKF();

    // initialize the kalman filter to starting state
    void begin(ADAHRSConfig* config, ADAHRSSensorData* states);

    // do estimation using kalman filter, copy data
    void estimate_states();
    
private:

    void convert_raw_sensor_data();
    void predict();
    void update();
    void correction(const Matrix& C, float sensor_data, float sensor_hat, float sensor_covariance);
    void compute_euler_angles();
    void unroll_states();
    

    // define away copy constructor and assignment operator
    EKF(const EKF& src);
    EKF& operator =(const EKF& src);

    ADAHRSConfig* _config;
    ADAHRSSensorData* _states;
};

// ----------------------------------------------------------------------------
#endif /* ADAHRS_EKF_H_ */
