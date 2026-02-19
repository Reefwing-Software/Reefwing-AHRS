# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Reefwing AHRS is an Arduino library providing an Attitude and Heading Reference System (AHRS) for Arduino-compatible boards. It implements multiple sensor fusion algorithms for IMU data. Published as an Arduino library (follows Arduino library conventions).

**Dependency**: Requires the [Reefwing_imuTypes](https://github.com/Reefwing-Software/Reefwing-imuTypes) library for `SensorData`, `EulerAngles`, `Quaternion`, and related types.

## Build & Test

This is an Arduino library — there is no standalone build system or unit test runner. Development workflow:

- **Compile/upload**: Use Arduino IDE or `arduino-cli` to compile example sketches targeting specific boards
- **Synthetic testing**: `examples/ekfTest/ekfTest.ino` runs without hardware, simulating roll/pitch motion profiles and outputting CSV for algorithm validation
- **Hardware testing**: Other examples (`nano33BLErev1`, `nano33BLErev2`, `nanoMPU6050`, `xiaoSense`, etc.) require the corresponding physical boards

To compile with arduino-cli:
```bash
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble examples/nano33BLErev1/
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble examples/ekfTest/
```

## Architecture

### Core Files

- `src/ReefwingAHRS.h` / `src/ReefwingAHRS.cpp` — Main library class
- `src/ExtendedKalmanFilter.h` / `src/ExtendedKalmanFilter.cpp` — Generic 2D EKF implementation

### Key Enums

```cpp
enum class BoardType { NANO, NANO33BLE, NANO33BLE_SENSE_R1, NANO33BLE_SENSE_R2, XIAO_SENSE, PORTENTA_H7, VIDOR_4000, NANO33IOT, NOT_DEFINED };
enum class ImuType  { LSM9DS1, LSM6DS3, BMI270_BMM150, MPU6050, MPU6500, UNKNOWN };
enum class DOF      { DOF_6, DOF_9 };
enum class SensorFusion { MADGWICK, MAHONY, COMPLEMENTARY, CLASSIC, EXTENDED_KALMAN, NONE };
```

### Sensor Fusion Algorithms

The `update()` method dispatches to the selected algorithm based on `_fusion`:

| Algorithm | Method | Key Parameters | Notes |
|-----------|--------|----------------|-------|
| MADGWICK | `madgwickUpdate()` | `_beta` (gyro meas error) | Quaternion gradient descent |
| MAHONY | `mahonyUpdate()` | `_Kp`, `_Ki` | Proportional-integral correction |
| COMPLEMENTARY | `complementaryUpdate()` | `_alpha` | Quaternion fusion |
| CLASSIC | `classicUpdate()` | `_alpha` | Euler angle complementary |
| EXTENDED_KALMAN | `extendedKalmanUpdate()` | Q, R matrices | 2-state (roll, pitch) |
| NONE | — | — | Raw gyro integration only |

### EKF Architecture

The `ExtendedKalmanFilter` class is generic (2D state, 2D measurement). Within `ReefwingAHRS`, it's configured for roll/pitch estimation:
- State transition via `imuStateTransitionFunc()` — integrates gyro rates
- Measurement via `imuMeasurementFunc()` — direct angle from state
- Jacobians (`imuStateJacobianFunc`, `imuMeasurementJacobianFunc`) return 2×2 identity matrices
- EKF noise matrices (`Q[2][2]`, `R[2][2]`) are private members of `ReefwingAHRS`

### Board Auto-Detection

`begin()` probes I2C to identify the connected IMU and infers `BoardType` and `ImuType`. Can be overridden manually via `setBoardType()` / `setImuType()`.

### 6-DOF vs 9-DOF

6-DOF IMUs (no magnetometer: MPU6050, MPU6500, LSM6DS3) only support CLASSIC, NONE, and EXTENDED_KALMAN fusion. Yaw drift is unavoidable without a magnetometer.

## Versioning Convention

Version is maintained in two places — keep them in sync:
- `library.properties` → `version=X.Y.Z`
- `README.md` → version history table at the top
