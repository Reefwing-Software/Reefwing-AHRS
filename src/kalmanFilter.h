/******************************************************************
  @file       KalmanFilter.h
  @brief      Attitude determination using Kalman filtering 
  @authors    Callum Bruce & David Such
  @copyright  Please see the accompanying LICENSE file.

  Modified Code:  David Such
  Version:        1.0.0
  Date:           29/06/23

  1.0.0 Original Release.                         29/06/23

  Credits: - Kalman filter code is forked from kalman_filter (c5f4d8e)
             by Callum Bruce. 
             Ref: https://github.com/c-bruce/kalman_filter/tree/master
             
             Callum has written an article describing the theory
             behind the Kalman filter and his implementation
             at: https://betterprogramming.pub/let-build-an-arduino-based-kalman-filter-for-attitude-determination-a895263b172
         
******************************************************************/

#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <BasicLinearAlgebra.h>

/******************************************************************
 *
 * Global Variables - Basic Linear Algebra - 
 * 
 ******************************************************************/

//  Kalman Filter Variable - delta time
float dt;

//  Continuous Adjustment Variables
BLA::Matrix<1, 1> epsilon = {0.0}; // Normalized square of the residual

// Varying matricies
BLA::Matrix<4, 1> x = {0.0, 0.0, 0.0, 0.0}; // State [roll, pitch, roll_rate, pitch_rate]
BLA::Matrix<4, 1> z = {0.0, 0.0, 0.0, 0.0}; // Measurement state [roll, pitch, roll_rate, pitch_rate]
BLA::Matrix<4, 1> y = {0.0, 0.0, 0.0, 0.0}; // Residule [roll, pitch, roll_rate, pitch_rate]
BLA::Matrix<4, 4> P = {1.0, 0.0, 0.0, 0.0, 
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0}; // Covariance matrix
BLA::Matrix<4, 4> Q = {pow(dt, 4) / 4, 0.0, 0.0, 0.0, 
                       0.0, pow(dt, 4) / 4, 0.0, 0.0,
                       0.0, 0.0, pow(dt, 2), 0.0,
                       0.0, 0.0, 0.0, pow(dt, 2)};  // Process noise matrix
BLA::Matrix<4, 4> K = {0.0, 0.0, 0.0, 0.0, 
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0}; // Kalman gain matrix
BLA::Matrix<4, 4> inv = {0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0}; // Inverse matrix required to calculate K

// Constant matricies
const BLA::Matrix<4, 1> u = {0.0, 0.0, 0.0, 0.0}; // Control vector
const BLA::Matrix<4, 4> R = {50.0, 0.0, 0.0, 0.0, 
                             0.0, 50.0, 0.0, 0.0,
                             0.0, 0.0, 10.0, 0.0,
                             0.0, 0.0, 0.0, 10.0}; // Measurement covariance matrix
const BLA::Matrix<4, 4> Fstm = {1.0, 0.0, dt, 0.0, 
                                0.0, 1.0, 0.0, dt,
                                0.0, 0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0, 1.0}; // State transition matrix - F already defined in Arduino
const BLA::Matrix<4, 4> B = {0.0, 0.0, 0.0, 0.0, 
                             0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0}; // Control matrix
const BLA::Matrix<4, 4> H = {1.0, 0.0, 0.0, 0.0, 
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0}; // Observation matrix
const BLA::Matrix<4, 4> I = {1.0, 0.0, 0.0, 0.0, 
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0}; // Identity matrix

/******************************************************************
 *
 * Kalman Filter Class - 
 * 
 ******************************************************************/

class KalmanFilter {
  public:
    KalmanFilter();

    void get_prediction();
    void get_kalman_gain();
    void get_update();
    void get_residual();
    void get_epsilon();
    void scale_Q();

    //  Kalman Filter Variable
    float total_vector_acc;

    //  Continuous Adjustment Variables
    int count = 0;
    float epsilon_max = 1.0;
    float Q_scale_factor = 1000.0;

};

#endif