// Copyright (c) 2024 David Such
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <cstring>
#include "ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
    : state_(nullptr), covariance_(nullptr), processNoise_(nullptr), measurementNoise_(nullptr),
      stateJacobian_(nullptr), measurementJacobian_(nullptr), kalmanGain_(nullptr), residual_(nullptr), 
      residualCovariance_(nullptr), tempBuffer1_(nullptr), tempBuffer2_(nullptr),
      stateTransitionFunc_(nullptr), measurementFunc_(nullptr), 
      stateJacobianFunc_(nullptr), measurementJacobianFunc_(nullptr) {}

void ExtendedKalmanFilter::initialize(const float* initialState, const float* initialCovariance, int stateSize, int measurementSize) {
    // Release previously allocated memory (if any)
    if (state_ != nullptr) {
        delete[] state_;
        state_ = nullptr;
    }
    if (covariance_ != nullptr) {
        delete[] covariance_;
        covariance_ = nullptr;
    }
    if (processNoise_ != nullptr) {
        delete[] processNoise_;
        processNoise_ = nullptr;
    }
    if (measurementNoise_ != nullptr) {
        delete[] measurementNoise_;
        measurementNoise_ = nullptr;
    }
    if (stateJacobian_ != nullptr) {
        delete[] stateJacobian_;
        stateJacobian_ = nullptr;
    }
    if (measurementJacobian_ != nullptr) {
        delete[] measurementJacobian_;
        measurementJacobian_ = nullptr;
    }
    if (kalmanGain_ != nullptr) {
        delete[] kalmanGain_;
        kalmanGain_ = nullptr;
    }
    if (residual_ != nullptr) {
        delete[] residual_;
        residual_ = nullptr;
    }
    if (residualCovariance_ != nullptr) {
        delete[] residualCovariance_;
        residualCovariance_ = nullptr;
    }
    if (tempBuffer1_ != nullptr) {
        delete[] tempBuffer1_;
        tempBuffer1_ = nullptr;
    }
    if (tempBuffer2_ != nullptr) {
        delete[] tempBuffer2_;
        tempBuffer2_ = nullptr;
    }

    // Validate state and measurement sizes
    if (stateSize <= 0 || measurementSize <= 0) {
        return;
    }

    stateSize_ = stateSize;
    measurementSize_ = measurementSize;

    // Allocate memory
    state_ = new float[stateSize_]{};
    covariance_ = new float[stateSize_ * stateSize_]{};
    processNoise_ = new float[stateSize_ * stateSize_]{};
    measurementNoise_ = new float[measurementSize_ * measurementSize_]{};
    stateJacobian_ = new float[stateSize_ * stateSize_]{};
    measurementJacobian_ = new float[measurementSize_ * stateSize_]{};
    kalmanGain_ = new float[stateSize_ * measurementSize_]{};
    residual_ = new float[measurementSize_]{};
    residualCovariance_ = new float[measurementSize_ * measurementSize_]{};
    tempBuffer1_ = new float[stateSize_ * stateSize_]{};
    tempBuffer2_ = new float[stateSize_ * measurementSize_]{};

    // Copy initial state and covariance
    if (initialState != nullptr) {
        memcpy(state_, initialState, stateSize_ * sizeof(float));
    } else {
        memset(state_, 0, stateSize_ * sizeof(float)); // Default to zero if no initial state provided
    }

    if (initialCovariance != nullptr) {
        memcpy(covariance_, initialCovariance, stateSize_ * stateSize_ * sizeof(float));
    } else {
        // Initialize covariance matrix to small values along the diagonal if not provided
        for (int i = 0; i < stateSize_; i++) {
            covariance_[i * stateSize_ + i] = 1e-2; // Default small uncertainty
        }
    }

    // Initialize process noise and measurement noise to default values
    for (int i = 0; i < stateSize_ * stateSize_; i++) {
        processNoise_[i] = (i % (stateSize_ + 1) == 0) ? 1e-3 : 0.0f; // Small values along diagonal
    }
    for (int i = 0; i < measurementSize_ * measurementSize_; i++) {
        measurementNoise_[i] = (i % (measurementSize_ + 1) == 0) ? 1e-2 : 0.0f; // Larger values along diagonal
    }
}

void ExtendedKalmanFilter::predict(const float* controlInput, float deltaTime) {
    // Predict the state: x = f(x, u, dt)
    if (stateTransitionFunc_) {
        float tempState[stateSize_];
        stateTransitionFunc_(state_, controlInput, deltaTime, tempState);
        memcpy(state_, tempState, stateSize_ * sizeof(float));
    }

    // Predict the covariance: P = F * P * F^T + Q
    if (stateJacobianFunc_) {
        stateJacobianFunc_(state_, controlInput, deltaTime, stateJacobian_);

        // Compute P = F * P
        for (int i = 0; i < stateSize_; i++) {
            for (int j = 0; j < stateSize_; j++) {
                tempBuffer1_[i * stateSize_ + j] = 0;
                for (int k = 0; k < stateSize_; k++) {
                    tempBuffer1_[i * stateSize_ + j] += stateJacobian_[i * stateSize_ + k] * covariance_[k * stateSize_ + j];
                }
            }
        }

        // Add process noise
        for (int i = 0; i < stateSize_ * stateSize_; i++) {
            covariance_[i] = tempBuffer1_[i] + processNoise_[i];
        }
    }
}

void ExtendedKalmanFilter::update(const float* measurement) {
    // Compute the measurement residual: y = z - h(x)
    if (measurementFunc_) {
        float tempMeasurement[measurementSize_];
        measurementFunc_(state_, tempMeasurement);
        for (int i = 0; i < measurementSize_; i++) {
            residual_[i] = measurement[i] - tempMeasurement[i];
        }
    }

    // Compute the measurement Jacobian
    if (measurementJacobianFunc_) {
        measurementJacobianFunc_(state_, measurementJacobian_);

        // Compute the residual covariance: S = H * P * H^T + R
        for (int i = 0; i < measurementSize_; i++) {
            for (int j = 0; j < measurementSize_; j++) {
                residualCovariance_[i * measurementSize_ + j] = 0;
                for (int k = 0; k < stateSize_; k++) {
                    residualCovariance_[i * measurementSize_ + j] += measurementJacobian_[i * stateSize_ + k] * covariance_[k * stateSize_ + j];
                }
                residualCovariance_[i * measurementSize_ + j] += measurementNoise_[i * measurementSize_ + j];
            }
        }

        // Compute the Kalman gain: K = P * H^T * S^-1
        for (int i = 0; i < stateSize_; i++) {
            for (int j = 0; j < measurementSize_; j++) {
                kalmanGain_[i * measurementSize_ + j] = 0;
                for (int k = 0; k < measurementSize_; k++) {
                    kalmanGain_[i * measurementSize_ + j] += covariance_[i * stateSize_ + k] * measurementJacobian_[k * measurementSize_ + j];
                }
            }
        }

        // Update state: x = x + K * y
        for (int i = 0; i < stateSize_; i++) {
            for (int j = 0; j < measurementSize_; j++) {
                state_[i] += kalmanGain_[i * measurementSize_ + j] * residual_[j];
            }
        }

        // Update covariance: P = (I - K * H) * P
        for (int i = 0; i < stateSize_ * stateSize_; i++) {
            covariance_[i] -= kalmanGain_[i] * measurementJacobian_[i];
        }
    }
}

const float* ExtendedKalmanFilter::getState() const {
    return state_;
}

const float* ExtendedKalmanFilter::getCovariance() const {
    return covariance_;
}

void ExtendedKalmanFilter::setStateTransition(void (*stateTransitionFunc)(const float*, const float*, float, float*)) {
    stateTransitionFunc_ = stateTransitionFunc;
}

void ExtendedKalmanFilter::setMeasurementFunction(void (*measurementFunc)(const float*, float*)) {
    measurementFunc_ = measurementFunc;
}

void ExtendedKalmanFilter::setStateTransitionJacobian(void (*stateJacobianFunc)(const float*, const float*, float, float*)) {
    stateJacobianFunc_ = stateJacobianFunc;
}

void ExtendedKalmanFilter::setMeasurementJacobian(void (*measurementJacobianFunc)(const float*, float*)) {
    measurementJacobianFunc_ = measurementJacobianFunc;
}