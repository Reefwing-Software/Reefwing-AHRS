// Copyright (c) 2024 David Such
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <math.h>

class ExtendedKalmanFilter {
public:
    // Constructor
    ExtendedKalmanFilter();

    // Initialize the filter with initial state and covariance
    void initialize(const float* initialState, const float* initialCovariance, int stateSize, int measurementSize);

    // Predict step: updates the state and covariance based on the system model
    void predict(const float* controlInput, float deltaTime);

    // Update step: incorporates measurements into the state estimate
    void update(const float* measurement);

    // Set user-defined state transition and measurement functions
    void setStateTransition(void (*stateTransitionFunc)(const float*, const float*, float, float*));
    void setMeasurementFunction(void (*measurementFunc)(const float*, float*));

    // Set user-defined Jacobians
    void setStateTransitionJacobian(void (*stateJacobianFunc)(const float*, const float*, float, float*));
    void setMeasurementJacobian(void (*measurementJacobianFunc)(const float*, float*));

    // Accessors for the current state and covariance
    const float* getState() const;
    const float* getCovariance() const;

private:
    int stateSize_;        // Number of state variables
    int measurementSize_;  // Number of measurement variables

    float* state_;             // State vector
    float* covariance_;        // Covariance matrix
    float* processNoise_;      // Process noise covariance matrix
    float* measurementNoise_;  // Measurement noise covariance matrix

    float* stateJacobian_;     // State transition Jacobian
    float* measurementJacobian_; // Measurement Jacobian

    float* kalmanGain_;        // Kalman gain
    float* residual_;          // Measurement residual
    float* residualCovariance_; // Residual covariance
    float* tempBuffer1_;       // Temporary buffer for calculations
    float* tempBuffer2_;       // Temporary buffer for calculations

    // Function pointers for the state transition and measurement models
    void (*stateTransitionFunc_)(const float*, const float*, float, float*);
    void (*measurementFunc_)(const float*, float*);

    // Function pointers for the Jacobians
    void (*stateJacobianFunc_)(const float*, const float*, float, float*);
    void (*measurementJacobianFunc_)(const float*, float*);
};

#endif