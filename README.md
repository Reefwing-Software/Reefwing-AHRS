![version](https://img.shields.io/github/v/tag/Reefwing-Software/Reefwing-AHRS) ![license](https://img.shields.io/badge/license-MIT-green) ![release](https://img.shields.io/github/release-date/Reefwing-Software/Reefwing-AHRS?color="red") ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# Reefwing AHRS
 
The Reefwing AHRS library provides an Attitude and Heading Reference System (AHRS) class for use with Arduino compatible boards. The library has been tested with the Arduino Nano, Nano 33 BLE, Nano 33 BLE SENSE (REV 1 and REV 2), Nano 33 IoT, MKR Vidor 4000, Portenta H7 and the Seeed XIAO nRF52840 Sense boards.

An Attitude and Heading Reference System (AHRS) takes information from the Inertial Measurement Unit (IMU) and processes it to provide reliable roll, pitch and yaw angles. Our library can be downloaded using the Arduino IDE Library Manager, or directly from the [Reefwing GitHub Repository](https://github.com/Reefwing-Software/Reefwing-AHRS).

 Version 1.1.0 of the library added the NONE option for Sensor Fusion. This option is used if you want Euler angles calculated but no sensor fusion filter applied. This release also fixed a bug in the Classic complementary filter calculation.

 Version 2.2.0 added support for the Nano 33 BLE Sense Rev. 2. Version 2 of the Nano 33 BLE Sense, replaces the LSM9DS1 9 axis IMU with a combination of two IMUs, the BMI270, a 6 axis gyro & accelerometer and the BMM150, a 3 axis magnetometer. In order to support the new hardware, it makes sense to separate the sensor processing from the sensor fusion algorithms. This library version also added the Kalman Filter Fusion option.

 A complete description of this library is available in our Medium article: [Reefwing AHRS Arduino Library for Drones](https://reefwing.medium.com/reefwing-ahrs-arduino-library-for-drones-part-1-6d6457231764).

## Reefwing IMU Types

[Reefwing IMU Types](https://github.com/Reefwing-Software/Reefwing-imuTypes) is a common library header which is used to prevent duplicate definition of similar types, classes and enums. It also ensures that changes will flow through to all of the Reefwing libraries that use it.

The libraries that use this definition header are:

- [ReefwingAHRS](https://github.com/Reefwing-Software/Reefwing-AHRS)
- [ReefwingLSM9DS1](https://github.com/Reefwing-Software/Reefwing-LSM9DS1)
- [Reefwing_xIMU3](https://github.com/Reefwing-Software/Reefwing-xIMU3)
- [ReefwingMPU6x00](https://github.com/Reefwing-Software/MPU6x00)
- [ReefwingMPU6050](https://github.com/Reefwing-Software/Reefwing-MPU6050)

The Structs Defined in the IMU Types Library are:

- `EulerAngles`
- `InertialMessage`
- `RawData`
- `ScaledData`
- `TempData`
- `SensorData`
- `VectorData`

The Class Defined in the IMU Types Library is:

- `Quaternion`

We have also included definitions of the I²C addresses for the following devices:

- `LSM9DS1AG_ADDRESS` - Address of the LSM9DS1 accelerometer & gyroscope
- `LSM9DS1M_ADDRESS` - Address of the LSM9DS1 magnetometer
- `HTS221_ADDRESS` - Nano 33 BLE Sense Rev 1 Sensor - temp/humidity
- `HS3003_ADDRESS` - Nano 33 BLE Sense Rev 2 Sensor - temp/humidity
- `LSM6DS3_ADDRESS` - Seeed Studios xiao Sense gyro/accelerometer
- `MPU6000_ADDRESS` - TDK InvenSense MPU6x00 gyro/accelerometer
- `MPU6050_ADDRESS` - TDK InvenSense MPU6050 gyro/accelerometer

## Library Dependencies

To use the Reefwing AHRS Library, you need to also install the Reefwing_imuTypes library. Both of these are able to be installed from the Arduino IDE Library Manager.

 ## Sensor Fusion & Free Parameters

 Sensor fusion is the process of combining sensory data or data derived from disparate sources such that the resulting information has less uncertainty than would be possible when these sources were used individually. With the gyroscope and accelerometer, we have two angle sensors which should be providing the same data but with different errors. The concept is to combine or fuse the data in such a way as to eliminate the errors and produce an accurate angle that we can use.

 In the original release of our AHRS Library, all the sensor fusion algorithms were contained within the IMU class. In version 2.2, these have been moved to the new ReefwingAHRS class. Now data from any IMU can be used as inputs to our sensor fusion algorithms.

 The Reefwing AHRS provides the following Sensor Fusion options:

 - Complementary Filter
 - Madgwick Filter
 - Kalman Filter
 - Mahony Filter
 - Classic Complementary Filter: Euler Angles are updated using trigonometry
 - NONE: Euler angles are calculated but no sensor fusion filter is used.

 We found that we experienced significant gyroscopic drift with the classic complementary filter. The Madgwick and Mahony filters fixes this issue but take a bit longer to settle on an angle. Of the two, Mahony is a bit faster than Madgwick, but the best filter and associated free parameter settings will depend on the application.

 A free parameter is defined as a variable in our sensor fusion algorithm which cannot be determined by the model and must be estimated experimentally or theoretically. The free parameters which need to be set for the three sensor fusion options are:

 - Alpha (𝛂) for the **Complementary filter**. Alpha is known as the filter co-efficient, smoothing factor or gain. It determines the cutoff frequency for the high pass filter, which we pass the gyro rate through. `𝛂 = 𝜏 / (𝜏 + ∆t)` where `𝜏 = filter time constant` and `∆t = imu sampling rate`. A typical value for `𝛂` is 0.95 to 0.98 but it depends on the sampling rate and application. A lot more detail is provided in our article [How to Write your own Flight Controller Software - Part 7]().
 - Kp and Ki for the **Mahony filter**. These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, and Ki is for integral. The Madgwick and Mahony filters differ with regards to the resolution of sensor biases. Mahony uses a proportional and integral controller to correct the gyroscope bias, whereas Madgwick uses only a proportional controller. The Mahony filter takes into consideration the disparity between the orientation from the gyroscope and the estimation from the magnetometer and accelerometer and weighs them according to its gains. The changes made to the gyroscope are given by: `Kp ∗ em + Ki ∗ ei`, where `em` is the sensor error of the gyroscope, and `ei` is the integral error, which is calculated by: `em = 1 / sampling frequency`. The filter default values are: `Kp = 10.0f` and `Ki = 0.0f`.
 - Beta (𝛃) for the **Madgwick filter**. The gain `𝛽` represents all mean zero gyroscope measurement errors, expressed as the magnitude of a quaternion derivative. It is defined using the angular velocity. Beta is based on our estimate of gyroscope measurement error (e.g., `GyroMeasError = 40` degrees per second is used as a default). We calculate beta using the code shown below.

 ```c++
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   
```

For the Madgwick filter there is a tradeoff in the beta parameter between accuracy and response speed. In the original Madgwick study, a beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy. However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion. Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter!

Increasing beta (`GyroMeasError`) by about a factor of fifteen, the response time is reduced to ~2 seconds. We haven't noticed any reduction in solution accuracy with this reduced lag. This is essentially the I coefficient in a PID control sense; the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. In any case, beta is the free parameter in the Madgwick filtering and fusion scheme.

For the Madgwick Filter use either `setBeta()` or `setGyroMeasError()`, **but not both**, since they both effect beta. 

## Inertial Coordinate System and NED

The inertial coordinate system is closely related to the NED (North-East-Down) coordinate system, which is a commonly used local-level coordinate system in navigation and aerospace applications.

The inertial coordinate system, also known as the Earth-Centered Earth-Fixed (ECEF) coordinate system, is a global reference frame with its origin located at the center of the Earth. It is fixed with respect to the Earth and does not rotate with respect to the Earth's rotation. The x-axis typically points towards the intersection of the equator and prime meridian (0 degrees latitude, 0 degrees longitude). The y-axis points eastward along the equator, and the z-axis points along the Earth's rotational axis, typically aligned with the North Pole.

The NED coordinate system, on the other hand, is a local-level coordinate system that is commonly used for navigation purposes. It is defined with respect to a specific location or observer on the Earth's surface. The origin of the NED coordinate system is typically set at the observer's position, and the NED axes are aligned such that:

- The North axis (N) points towards true North.
- The East axis (E) points towards true East.
- The Down axis (D) points vertically downward, perpendicular to the Earth's surface.

The NED coordinate system is relative to the observer's position and orientation, while the inertial coordinate system is fixed with respect to the Earth. However, the relationship between the two coordinate systems can be established through appropriate coordinate transformations.

To convert between NED and inertial coordinates, one typically needs to consider the observer's position and the rotation of the Earth. This transformation involves accounting for the observer's latitude, longitude, and altitude, as well as considering the Earth's rotation rate. For these reasons, we will stick with the inertial reference frame.

## The LSM9DS1 IMU

The LSM9DS1 is manufactured by STMicroelectronics (now known as ST). It has a 3D digital linear acceleration sensor, a 3D digital angular rate sensor (gyroscope), and a 3D digital magnetic sensor. It includes SPI and I2C (standard 100 kHz and fast mode 400 kHz) serial interfaces but the Arduino Nano 33 BLE uses I2C. Full details of this chip are available in the [LSM9DS1 data sheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf).

- accelerometer: 16 bit ADC with ±2g/±4g/±8/±16g ranges available;
- gyroscope: 16 bit ADC with ±245/±500/±2000 dps ranges.
- magnetometer: 16 bit ADC with range of ±4/±8/±12/±16 gauss.

The available IMU Output Data Rates (ODR) are 14.9, 59.5, 119, 238, 476 and 952 Hz. 

## The BMI270 and BMM150 IMUs

In Rev. 2 of the Nano 33 BLE SENSE, the LSM9DS1 IMU is replaced with the [BMI270](https://content.arduino.cc/assets/bmi270-ds000.pdf) and [BMM150](https://content.arduino.cc/assets/bmm150-ds001.pdf) IMUs from Bosch. The BMI270 is a 6-axis gyroscope and accelerometer.

- accelerometer: 16 bit ADC with ±2g/±4g/±8g/±16g ranges available;
- gyroscope: 16 bit ADC with ±125dps/±250dps/±500dps/±1000dps/±2000dps ranges.

The BMI270 is a 3-axis magnetometer. It has a measurement range of ±1300 µt (x-, y- axis) and ±2500µT (z-axis).

## IMU Orientation

IMU sensors don't have a standardised axis layout. Even if they did, it is the way that they are mounted on your device that is important. This matters most when we are combining data from different sensors like in the LSM9DS1 or the BMI270 and BMM150 on the Nano 33 BLE Rev 2.

In this situation we want to ensure that all the sensors are using the same reference frame. To this end, we have provided the `setData(SensorData d, bool axisAlign)` method. If `axisAlign = true` (the default), then when setting the sensor data, the following modifications are done based on IMU type:

- `LSM9DS1` - magnetometer x-axis value is reversed.
- `BMI270_BMM150` - magnetometer y-axis value is reversed.

Depending on the sensor combination you are using and their mounting orientation, you may need to do something similar in your sketch. If `axisAlign = false`, then no modification is made to the loaded sensor data.

Even though the Nano 33 BLE Sense Rev 2 mounts the two sensors to line up the x-axis, the magnetometer y-axis is still reversed according to the data sheet.

The convention that we use, consistent with the inertial frame euler angles, is that roll is associated with the x-axis, pitch is associated with the y-axis and yaw is associated with the z-axis.

## How to Use the Reefwing AHRS Library

In order to get the correct angles out of the AHRS, we need to take the following steps:

1. Calibrate the IMU (if available) and assign the appropriate offsets. This is normally possible in the IMU library.

2. In `setup()`, instantiate the `ReefwingAHRS` class and call `begin()` to detect the board type, and assign the default free parameters, fusion algorithm (Madgwick) and declination (Sydney, Australia). If you want to use something other than the defaults, then change those now.

3. On each iteration of `loop()`:

- If required adjust the sensor data to the correct format. Accelerometer - g, Gyroscope - DPS (degrees per second), and Magnetometer - can be anything (e.g., gauss or uT). The other thing to check is that the axis orientation is consistent for the three sensors. On the LSM9DS1, the magnetometer x-axis is inverted.
- Pass the latest formatted sensor data from the IMU to the AHRS class.

4. Update the AHRS model (call `update()`).

5. If the selected AHRS model uses quaternions (i.e., `COMPLEMENTARY`, `MADGWICK`, and `MAHONY`), convert the quaternion to a Euler Angle.

The best way to illustrate this process is with the provided examples.

## Examples - boardType.ino

This simple example attempts to detect the board type using the methodology explained in [our article on the nRF52840 boards](https://reefwing.medium.com/arduino-nano-33-ble-sense-rev-2-and-the-seeed-nrf52840-sense-3e16c5e25d95).

The getBoardTypeString() method will return one of the following:

- Nano
- Nano 33 BLE
- Nano 33 BLE Sense
- Nano 33 BLE Sense Rev 2
- Seeed XIAO nRF52840 Sense
- Portenta H7
- Not Defined Board Type

The related method 'getBoardType()', will return one of the BoardType enums.

```c++
enum class BoardType {
  NANO = 0,
  NANO33BLE,
  NANO33BLE_SENSE_R1,
  NANO33BLE_SENSE_R2,
  XIAO_SENSE,
  PORTENTA_H7,
  VIDOR_4000,
  NANO33IOT,
  NOT_DEFINED
};
```

This is useful if you are using different hardware with common firmware and need it to conditionally compile using different header files (e.g., IMU libraries).

## Examples - nanoMPU6050.ino

The vanilla Arduino Nano (i.e., ATMega328P version), could have any IMU connected to it. This example illustrates how to define the IMU to the AHRS class, so that it handles the calculations correctly. We are using the MPU-6050 IMU, so in `setup()` we assign:


```c++
ahrs.setDOF(DOF::DOF_6);
ahrs.setImuType(ImuType::MPU6050);
```

The most important setting is the `DOF`, so it doesn't matter if the IMU you use isn't one of the defined types. With IMUs that only have a gyroscope and accelerometer (6 DOF), you can only use the `CLASSIC`, `KALMAN` & `NONE` Sensor Fusion options. 

Set the algorithm that you wish to use with:

```c++
ahrs.setFusionAlgorithm(SensorFusion::CLASSIC);
```

The other Sensor Fusion algorithms require a 9 DOF IMU (i.e., a magnetometer as well).

## Examples - nano33BLErev1.ino

This example will print roll, pitch, yaw and heading angles to the Arduino IDE Serial Monitor. It is targeted at the LSM9DS1 IMU on the Nano 33 BLE or Nano 33 BLE Sense Revision 1 (rev1). If you run this sketch on the rev2 Sense (or any board that doesn't use the LSM9DS1), you will receive the message: `LSM9DS1 IMU Not Detected`.

This sketch is configured to work with the `MADGWICK`, `MAHONY`, `CLASSIC`, `COMPLEMENTARY` and `NONE` Sensor Fusion options:

- **Complementary Filter** - uses quaternions to find the weighted average of angles produced by the sensors.
- **Madgwick Filter** - uses quaternions and combines a complementary filter approach with adaptive gain to fuse the sensor data.
- **Mahony Filter** - uses quaternions, another riff on complementary fusion and includes an error correction step to compensate for gyroscope bias.
- **Classic Complementary Filter** - Euler Angles are updated using trigonometry, apart from this it is similar to the Quaternion version.
- **Kalman Filter** - can be used to minimise the impact of noise in sensors.
- **NONE** - Euler angles are calculated but no sensor fusion is used. This is useful for demonstrating why you need sensor fusion!

The loop frequency will vary with the board and fusion algorithm. The `COMPLEMENTARY` filter is a bit slower than the `MADGWICK` and `MAHONY`. We found that we experienced significant gyroscopic drift with the classic complementary filter. The Madgwick and Mahony filters fixes this issue but take a bit longer to settle on an angle. Of the two, Mahony is a bit faster than Madgwick, but the best filter and associated free parameter settings will depend on the application. The `NONE` option is also fast but has crazy gyroscopic drift.

## Examples - nano33BLErev2.ino

This example prints roll, pitch, yaw and heading angles on the Arduino Serial Monitor using the BMI270/BMM150 IMUs found on the Nano 33 BLE Sense Revison 2 (rev2). This example is similar to the last one, but we use the standard Arduino IMU library for the BMI270 and BMM150.

```c++
#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>
```

To detect the presense of the BMI270/BMM150 IMUs, we initially used:

```c++
if (IMU.begin()) {…}
```

However, this condition is always true if you use the `Arduino_BMI270_BMM150.h` library (The `begin()` method always returns 1, it doesn't check for a sensor connection). To fix this, we instead used:

```c++
if (IMU.begin() && ahrs.getBoardType() == BoardType::NANO33BLE_SENSE_R2) {…}
```

The IMU data is stored in a SensorData struct, to allow us to easily load these values into the AHRS for angle updates.

```c++
struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  uint32_t gTimeStamp, aTimeStamp, mTimeStamp;
};
```

The loop frequency for the combination of the Nano 33 BLE Sense Rev2 and the BMI270/BMM150 IMUs, is quite a bit slower than the earlier generation boards. We are not sure why this is, the hardware is operating at the same clock speed, so it is probably IMU library related,

It could be the 100 Hz sample rate for the gyro and accelerometer. The standard library doesn't provide a method to change this rate but the `interrupts_subclassing.ino` example shows how it can be done (gyro maximum ODR = 6.4 kHz and accelerometer maximum ODR = 1.6 kHz).

For comparison, the default sample rate for the gyro/accelerometer in the Reefwing LSM9DS1 Library is 119 Hz (and can be set to a maximum of 952 Hz).

As for the previous example, you can adjust the sensor fusion method using:

```c++
ahrs.setFusionAlgorithm(SensorFusion::MAHONY);
```

## Examples - xiaoSense.ino

There are no processor specific calls in our AHRS library, so theoretically it could be used with any Arduino compatible board and IMU. To demonstrate this, we have provided an example for the Seeed Studio xiao Sense. This board is tiny!

The xiao Sense uses the LSM6DS3 IMU (6 DOF). This IMU is referered to as 6-axis device because it only has a gyroscope and accelerometer (no magnetometer). Before using the AHRS example, make sure that you can run the `HighLevelExample.ino` sketch, found in the LSM6DS3 library examples folder. 

To do this, delete the `Arduino_LSM6DS3` library if you have it installed, and replace it with the Seeed Arduino LSM6DS3 library (version 2.0.3 or later). Also install the Seeed nrf52 mbed-enabled Boards package, version 2.7.2, using the Arduino Boards Manager.

The Seeed library is a fork of the SparkFun version. Note that the default constructor for this library is I²C mode, with address `0x6B`. This is not what we want, so we need to include the I²C address when we instantiate the IMU class:

```c++
LSM6DS3 imu(I2C_MODE, 0x6A);
```

The library API provides the following methods to read the IMU values. You have to call `imu.begin()`, before using these methods.

```c++
imu.readFloatAccelX();
imu.readFloatAccelY();
imu.readFloatAccelZ();
imu.readFloatGyroX();
imu.readFloatGyroY();
imu.readFloatGyroZ();
imu.readTempC();
```

However, it isn't obvious from the documentation what units are being returned (e.g., degrees per second, radians per second, g's, etc). To work out what is going on we need to have a look at the library code. From this we establish:

- The default sample rate (ODR) for the gyro and accelerometer is 416 Hz
- Gyro Range = 2000 degrees/second (DPS)
- Accelerometer range = 16 g's
- Gyro output = register * 4.375 * (gyroRangeDivisor) / 1000 (i.e., DPS)
- Accel output = register * 0.061 * (accelRange >> 1) / 1000 (i.e., g's)

As we have no magnetometer, we can only use the NONE, KALMAN, and CLASSIC sensor fusion algorithms. These all have a loop frequency of around 240 Hz. We can't calculate the tilt compensated yaw without a magnetometer, so the yaw results are useless. We had high hopes for the xiao Sense as a flight controller because of its small size and light weight. In practise, the results returned by the AHRS are pretty inaccurate and could only be used for applications like a pedometer.

The axis orientation of the LSM6DS3 IMU has the y-axis reversed compared to the LSM9DS1. This reversal is corrected by the library, when `setData()` is called.

## Examples - mkrMPU6500.ino

This example will print roll, pitch, yaw and heading angles to the Serial Monitor, using the MPU6500 IMU and an Arduino MKR board (e.g., Portenta H7 or Vidor 4000), connected via SPI.

The Arduino Portenta H7 is a beast of a board. Our initial plan was to port BetaFlight to this board, but that [didn't work out](https://reefwing.medium.com/the-new-arduino-portenta-h7-board-76040c3405f). It has two cores, and the one that we will be using is a Cortex M7 running at 480 MHz.

One downside of the Portenta is that it doesn't have an IMU. To address this, we created our own shield which includes the [MPU6500](https://invensense.tdk.com/download-pdf/mpu-6500-datasheet/). To maximise data transfer rates, the MPU6500 is connected to the Portenta using SPI (with a maximum clock speed of 20 MHz, schematic).

Like the LSM6DS3, the MPU6500 is a 6-axis IMU (i.e., no magnetometer) and has a similar axis orientation. You can use the the `CLASSIC`, `KALMAN` & `NONE` Sensor Fusion options in our AHRS library.

The Portenta H7 uses the Arduino MKR board layout, and we have tested this library with the MPU6500 mounted on the Portenta H7 and the MKR Vidor 4000. We are using the Reefwing MPU6500 IMU library in our example. Substitute the appropriate library and setup/calibration for your IMU.

## Examples - complementaryFilter.ino

This example is very similar to the `nano33BLErev1.ino` example but is targeted at the complementary filter sensor fusion algorithm. You can adjust the filter mix by setting alpha (𝛂).

```c++
ahrs.setAlpha(0.95);
```

Alpha (𝛂) in the Complementary filter is known as the filter co-efficient, smoothing factor or gain. It determines the cutoff frequency for the high pass filter, which we pass the gyro rate through. 

`𝛂 = 𝜏 / (𝜏 + ∆t) `

where 𝜏 = filter time constant and ∆t = imu sampling rate. 

A typical value for 𝛂 is 0.95 to 0.98 but it depends on the sampling rate and application. A large α implies that the output will decay very slowly but will also be strongly influenced by even small changes in input. The output of this filter will decay towards zero for an unchanging input.

In its simplest form, the complementary filter calculation is:

`Complementary Filter Result = 𝛂 × Input_A + (1 - 𝛂) × Input_B`

The complementary filter formula for the roll angle looks like:

`Roll 𝜽(t + ∆t) = 0.98 × Gyro Angle + 0.02 × Acc Angle`

So a simple way to think about alpha is it adjusts the contribution from different sensors towards the final angle. In the formula above 98% of the angle comes from the gyroscope (fast acting sensor) and 2% is from the accelerometer (slow but lower noise sensor).

## Examples - betaOptimisation.ino

This example sketch attempts to determine the optimum beta for the Madgwick filter. The Nano 33 BLE should be placed flat and not moved for the duration of the test.

The default sample rate for this library is 238 Hz for the gyro/accelerometer and 10 Hz for the magnetometer. Thus a new gyro/acc reading should be available every 4.2 ms. We delay for 10 ms between readings.

Beta (𝛃) is a free parameter used in the Madgwick filter. 

The gain 𝛽, represents all mean zero gyroscope measurement errors, 
expressed as the magnitude of a quaternion derivative. It is defined 
using the angular velocity. Beta is based on our estimate of gyroscope 
measurement error (e.g., GyroMeasError = 40 degrees per second is used 
as a default). 

We calculate beta using the code shown below.

```c++
float beta = sqrt(3.0f / 4.0f) * gyroMeasError;
```

`The other free parameter for the Madgwick filter is the 
filter gain ζ which represents the rate of convergence to remove gyroscope 
measurement errors which are not mean zero, also expressed as the 
magnitude of a quaternion derivative. `

There is a clear optimal value of β for each filter implementation; high enough to minimises errors due to integral drift but sufficiently low enough that unnecessary noise is not introduced by large steps of gradient descent iterations.

RMS is often used to compare a theoretical prediction against an actual result. The root mean square (RMS) error is defined as the arithmetic mean of the squares of a set of numbers. If we take n angle measurements {𝜽1, 𝜽2, …, 𝜽n}, the RMS is:

`𝜽rms = √[(𝜽1² + 𝜽2² + … + 𝜽n²)/n]`

This sketch in the examples folder can be used to assist with beta optimisation of your Madgwick filter. It will print to the serial port the value for beta between 0.0 and 0.5 and its associated static RMS error in CSV format. You can graph these results (using Excel for example). The sketch will record the beta with the minimum error in a static situation for pitch and roll. This sketch also demonstrates how you can adjust the filter gain in the Reefwing AHRS library.

