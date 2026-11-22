// Copyright (c) 2024 David Such
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <string.h>   // not <cstring>: the AVR toolchain has no C++ standard library headers
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

// Invert a small n x n matrix with Gauss-Jordan elimination and partial
// pivoting. Returns false if the matrix is singular. n is tiny (the
// measurement dimension) so the O(n^3) cost is negligible.
static bool invertMatrix(const float* a, float* inv, int n) {
    float work[n * n];
    memcpy(work, a, n * n * sizeof(float));

    for (int i = 0; i < n * n; i++) {
        inv[i] = 0.0f;
    }
    for (int i = 0; i < n; i++) {
        inv[i * n + i] = 1.0f;
    }

    for (int col = 0; col < n; col++) {
        // Find the pivot row
        int pivot = col;
        float maxAbs = fabsf(work[col * n + col]);
        for (int r = col + 1; r < n; r++) {
            float v = fabsf(work[r * n + col]);
            if (v > maxAbs) { maxAbs = v; pivot = r; }
        }
        if (maxAbs < 1e-12f) {
            return false;   // Singular
        }
        if (pivot != col) {
            for (int c = 0; c < n; c++) {
                float t = work[col * n + c]; work[col * n + c] = work[pivot * n + c]; work[pivot * n + c] = t;
                t = inv[col * n + c];  inv[col * n + c]  = inv[pivot * n + c];  inv[pivot * n + c]  = t;
            }
        }
        // Normalise the pivot row
        float p = work[col * n + col];
        for (int c = 0; c < n; c++) {
            work[col * n + c] /= p;
            inv[col * n + c]  /= p;
        }
        // Eliminate the column from the other rows
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            float f = work[r * n + col];
            if (f == 0.0f) continue;
            for (int c = 0; c < n; c++) {
                work[r * n + c] -= f * work[col * n + c];
                inv[r * n + c]  -= f * inv[col * n + c];
            }
        }
    }
    return true;
}

void ExtendedKalmanFilter::predict(const float* controlInput, float deltaTime) {
    if (state_ == nullptr) { return; }

    const int n = stateSize_;

    // Predict the state: x = f(x, u, dt)
    if (stateTransitionFunc_) {
        float tempState[n];
        stateTransitionFunc_(state_, controlInput, deltaTime, tempState);
        memcpy(state_, tempState, n * sizeof(float));
    }

    // Predict the covariance: P = F * P * F^T + Q
    if (stateJacobianFunc_) {
        stateJacobianFunc_(state_, controlInput, deltaTime, stateJacobian_);

        // tempBuffer1_ = F * P
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                float sum = 0.0f;
                for (int k = 0; k < n; k++) {
                    sum += stateJacobian_[i * n + k] * covariance_[k * n + j];
                }
                tempBuffer1_[i * n + j] = sum;
            }
        }

        // P = (F * P) * F^T + Q
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                float sum = 0.0f;
                for (int k = 0; k < n; k++) {
                    sum += tempBuffer1_[i * n + k] * stateJacobian_[j * n + k];
                }
                covariance_[i * n + j] = sum + processNoise_[i * n + j];
            }
        }
    }
}

void ExtendedKalmanFilter::update(const float* measurement) {
    if (state_ == nullptr || !measurementFunc_ || !measurementJacobianFunc_) { return; }

    const int n = stateSize_;
    const int m = measurementSize_;

    // Measurement residual: y = z - h(x)
    float predictedMeasurement[m];
    measurementFunc_(state_, predictedMeasurement);
    for (int i = 0; i < m; i++) {
        residual_[i] = measurement[i] - predictedMeasurement[i];
    }

    // Measurement Jacobian H (m x n)
    measurementJacobianFunc_(state_, measurementJacobian_);

    // tempBuffer2_ = P * H^T  (n x m)
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += covariance_[i * n + k] * measurementJacobian_[j * n + k];
            }
            tempBuffer2_[i * m + j] = sum;
        }
    }

    // Residual covariance: S = H * (P * H^T) + R  (m x m)
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += measurementJacobian_[i * n + k] * tempBuffer2_[k * m + j];
            }
            residualCovariance_[i * m + j] = sum + measurementNoise_[i * m + j];
        }
    }

    // Kalman gain: K = (P * H^T) * S^-1  (n x m)
    float sInverse[m * m];
    if (!invertMatrix(residualCovariance_, sInverse, m)) {
        return;   // S is singular: skip this measurement update
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            float sum = 0.0f;
            for (int k = 0; k < m; k++) {
                sum += tempBuffer2_[i * m + k] * sInverse[k * m + j];
            }
            kalmanGain_[i * m + j] = sum;
        }
    }

    // Update state: x = x + K * y
    for (int i = 0; i < n; i++) {
        float sum = 0.0f;
        for (int j = 0; j < m; j++) {
            sum += kalmanGain_[i * m + j] * residual_[j];
        }
        state_[i] += sum;
    }

    // Update covariance: P = (I - K * H) * P
    // tempBuffer1_ = I - K * H  (n x n)
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float sum = 0.0f;
            for (int k = 0; k < m; k++) {
                sum += kalmanGain_[i * m + k] * measurementJacobian_[k * n + j];
            }
            tempBuffer1_[i * n + j] = ((i == j) ? 1.0f : 0.0f) - sum;
        }
    }
    float newCovariance[n * n];
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++) {
                sum += tempBuffer1_[i * n + k] * covariance_[k * n + j];
            }
            newCovariance[i * n + j] = sum;
        }
    }
    memcpy(covariance_, newCovariance, n * n * sizeof(float));
}

const float* ExtendedKalmanFilter::getState() const {
    return state_;
}

const float* ExtendedKalmanFilter::getCovariance() const {
    return covariance_;
}

const float* ExtendedKalmanFilter::getProcessNoise() const {
    return processNoise_;
}

const float* ExtendedKalmanFilter::getMeasurementNoise() const {
    return measurementNoise_;
}

void ExtendedKalmanFilter::setProcessNoise(const float* processNoise) {
    // Ignored until initialize() has allocated the filter
    if (processNoise_ == nullptr || processNoise == nullptr) { return; }
    memcpy(processNoise_, processNoise, stateSize_ * stateSize_ * sizeof(float));
}

void ExtendedKalmanFilter::setMeasurementNoise(const float* measurementNoise) {
    // Ignored until initialize() has allocated the filter
    if (measurementNoise_ == nullptr || measurementNoise == nullptr) { return; }
    memcpy(measurementNoise_, measurementNoise, measurementSize_ * measurementSize_ * sizeof(float));
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