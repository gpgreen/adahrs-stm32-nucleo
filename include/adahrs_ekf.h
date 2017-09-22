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

// ----------------------------------------------------------------------------

class EKF
{
public:

    EKF();

    // initialize the kalman filter to starting state
    void begin(ADAHRSConfig* config, ADAHRSSensorData* states);

    // do estimation using kalman filter, copy data
    void estimate_states();
    
    void correction();
    void compute_euler_angles();

    // ensure angles are within 0-360 degrees
    void unroll_states();
    
private:

    void convert_raw_sensor_data();
    void predict();
    void update();

    // define away copy constructor and assignment operator
    EKF(const EKF& src);
    EKF& operator =(const EKF& src);

    ADAHRSConfig* _config;
    ADAHRSSensorData* _states;
};

// ----------------------------------------------------------------------------
#endif /* ADAHRS_EKF_H_ */
