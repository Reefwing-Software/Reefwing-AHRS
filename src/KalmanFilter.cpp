/******************************************************************
  @file       KalmanFilter.cpp
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

#include "KalmanFilter.h"

/******************************************************************
 *
 * Kalman Filter Implementation - 
 * 
 ******************************************************************/

KalmanFilter::KalmanFilter() { }

void KalmanFilter::get_prediction() {
  x = (Fstm * x) + (B * u);
  P = (Fstm * (P * (~Fstm))) * 1.0 + Q;
}

void KalmanFilter::get_kalman_gain() {
  inv = BLA::Inverse((H * (P * (~H))) + R);
  K = ((P * (~H)) * inv);
}

void KalmanFilter::get_update() {
  x = x + (K * (z - (H * x)));
  P = (((I - (K * H)) * P) * (~(I - (K * H)))) + ((K * R) * (~K));
}

void KalmanFilter::get_residual() {
  y = z - x;
}

void KalmanFilter::get_epsilon() {
  epsilon = (~y * (inv * y));
}

void KalmanFilter::scale_Q() {
  if (epsilon.storage(0, 0) > epsilon_max) {
    Q *= Q_scale_factor;
    count += 1;
  }
  else if (epsilon.storage(0, 0) < epsilon_max && count > 0) {
    Q /= Q_scale_factor;
    count -= 1;
  }
}

