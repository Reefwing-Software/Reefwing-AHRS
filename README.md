![version](https://img.shields.io/github/v/tag/Reefwing-Software/Reefwing-AHRS) ![license](https://img.shields.io/badge/license-MIT-green) ![release](https://img.shields.io/github/release-date/Reefwing-Software/Reefwing-AHRS?color="red") ![open source](https://badgen.net/badge/open/source/blue?icon=github)

# Reefwing AHRS
 
 The Reefwing flight controller for quadcopters is based around the Arduino Nano 33 BLE board. This board includes the LSM9DS1 chip which we use as an Inertial Measurement Unit (IMU). The IMU determines the current orientation of the drone. 

 The Reefwing AHRS library provides an Attitude and Heading Reference System (AHRS) class for use with the Arduino Nano 33 BLE and the Nano 33 BLE SENSE boards.

 The AHRS will convert the gyroscope rate and accelerometer force data to a roll and pitch angle. The yaw angle is then calculated using the pitch, roll and magnetometer data. 

 Version 1.1.0 of the library added the NONE option for Sensor Fusion. This option is used if you want Euler angles calculated but no sensor fusion filter applied. This release also fixed a bug in the Classic complementary filter calculation.

 ## Sensor Fusion & Free Parameters

 Sensor fusion is the process of combining sensory data or data derived from disparate sources such that the resulting information has less uncertainty than would be possible when these sources were used individually. With the gyroscope and accelerometer, we have two angle sensors which should be providing the same data but with different errors. The concept is to combine or fuse the data in such a way as to eliminate the errors and produce an accurate angle that we can use.

 The Reefwing AHRS provides the following Sensor Fusion options:

 - Complementary Filter
 - Madgwick Filter
 - Fusion (Madgwick Filter v2)
 - Mahony Filter
 - Classic Complementary Filter: Euler Angles are updated using trigonometry
 - NONE: Euler angles are calculated but no sensor fusion filter is used.

 We found that we experienced significant gyroscopic drift with the classic complementary filter. The Madgwick and Mahony filters fixes this issue but take a bit longer to settle on an angle. Of the two, Mahony is a bit faster than Madgwick, but the best filter and associated free parameter settings will depend on the application.

 A free parameter is defined as a variable in our sensor fusion algorithm which cannot be determined by the model and must be estimated experimentally or theoretically. The free parameters which need to be set for the three sensor fusion options are:

 - Alpha (𝛂) for the **Complementary filter**. Alpha is known as the filter co-efficient, smoothing factor or gain. It determines the cutoff frequency for the high pass filter, which we pass the gyro rate through. `𝛂 = 𝜏 / (𝜏 + ∆t)` where `𝜏 = filter time constant` and `∆t = imu sampling rate`. A typical value for `𝛂` is 0.95 to 0.98 but it depends on the sampling rate and application. A lot more detail is provided in our article [How to Write your own Flight Controller Software - Part 7]().
 - Kp and Ki for the **Mahony filter**. These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, and Ki is for integral. The Madgwick and Mahony filters differ with regards to the resolution of sensor biases. Mahony uses a proportional and integral controller to correct the gyroscope bias, whereas Madgwick uses only a proportional controller. The Mahony filter takes into consideration the disparity between the orientation from the gyroscope and the estimation from the magnetometer and accelerometer and weighs them according to its gains. The changes made to the gyroscope are given by: `Kp ∗ em + Ki ∗ ei`, where `em` is the sensor error of the gyroscope, and `ei` is the integral error, which is calculated by: `em = 1 / sampling frequency`. The filter default values are: `Kp = 10.0f` and `Ki = 0.0f`.
 - Gain for the **Fusion filter**. Equivalent to beta in the original Madgwick filter. A low gain will decrease the influence of the accelerometer and magnetometer so that the algorithm will better reject disturbances causes by translational motion and temporary magnetic distortions. However, a low gain will also increase the risk of drift due to gyroscope calibration errors. The `default value is 0.5`.
 - Beta (𝛃) for the **Madgwick filter**. The gain `𝛽` represents all mean zero gyroscope measurement errors, expressed as the magnitude of a quaternion derivative. It is defined using the angular velocity. Beta is based on our estimate of gyroscope measurement error (e.g., `GyroMeasError = 40` degrees per second is used as a default). We calculate beta using the code shown below.

 ```c++
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   
```

For the Madgwick filter there is a tradeoff in the beta parameter between accuracy and response speed. In the original Madgwick study, a beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy. However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion. Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter!

Increasing beta (`GyroMeasError`) by about a factor of fifteen, the response time is reduced to ~2 seconds. We haven't noticed any reduction in solution accuracy with this reduced lag. This is essentially the I coefficient in a PID control sense; the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. In any case, beta is the free parameter in the Madgwick filtering and fusion scheme.

The Fusion Filter (Madgwick v2) operates similarly. The default gain is 0.5 but this results in a fair lag until the angle settles, increasing this to 7.5 provides significantly faster response.

The default free parameters are set in the LSM9DS1 `begin()` method, so custom values need to be assigned after this and before the `update()` method is called in `loop()`. The available `LSM9DS1` class set free parameter methods are:

```c++
void setAlpha(float a);           // Complementary Filter - value between 0 and 1.
void setBeta(float b);            // Madgwick Filter
void setGyroMeasError(float gme); // Madgwick Filter - value in degrees per second
void setKp(float p);              // Mahony Filter - proportional term
void setKi(float i);              // Mahony Filter - integral term
void setFusionGain(float g);      //  Fusion Filter - similar to beta
```

For the Madgwick Filter use either `setBeta()` or `setGyroMeasError()`, **but not both**, since they both effect beta. 

## Aircraft Reference Frame

Any control system needs to use a defined reference frame. Being a drone application, we will use the convention for aircraft. This is based on three body co-ordinates and three attitude angles roll, pitch and yaw.

The Aircraft Reference Frame (ARF) is relative to the drone itself and the origin is at the centre of gravity. Aircraft movement in the positive X, Y and Z directions translate to forward, right and down. In other words the positive X-Axis points towards the front of the drone.
Rotation around the X, Y and Z-Axes are referred to as roll, pitch and yaw. The direction of positive rotation is defined by the co-ordinate right hand rule.

Roll is defined as the angle between the Y Axis and the horizontal plane. When rotating the drone around the X Axis with the Y Axis moving downwards, roll is positive and increasing.

Pitch is defined as the angle between the X Axis and the horizontal plane. When rotating the drone around the Y Axis with the X Axis moving upwards, pitch is positive and increasing.

Yaw is defined as the angle between the X Axis and magnetic north on the horizontal plane measured in a clockwise direction when viewing from the top of the drone.

The aircraft reference frame is useful in a piloted flight control context, since we want command inputs to be in reference to the current drone position and attitude.

## World Reference Frame

Another possible reference frame is the world or ground. In this frame, the origin is defined as a location on Earth (e.g., the drone take off longitude, latitude and altitude) and the X, Y and Z-Axes correspond to North, East and Down. You may see this described as NED.

Heading is similar to yaw and defined as the angle between the X Axis and magnetic north on the horizontal plane measured in a clockwise direction when viewing from the top of the drone. Thus, a heading angle of 0° will point towards the North, and +90° towards the East.

The World Reference Frame is useful in flight planning or return to home failsafe scenarios where we need to know the location of the drone and where it came from. For any sort of accuracy you will need to complement the IMU with a GPS for this type of mission. Some filter libraries (e.g., the Madgewick fusion algorithm which we are using) also expect co-ordinates in the NED format.
 
## Inertial Navigation

Without a GPS, a drone relies on inertial navigation or dead reckoning to determine its position, velocity and orientation. It does this using an IMU to provide data to the Attitude and Heading Reference System (AHRS), which is part of the flight controller. The AHRS includes some sort of filtering or sensor fusion to deliver the most accurate roll, pitch and yaw data possible. Precision gyroscopes (e.g., ring lasers), are too expensive and bulky for drone applications and so less accurate MEMS (Micro Electrical Mechanical System) devices like the LSM9DS1 are used.

## The LSM9DS1 IMU

The LSM9DS1 is manufactured by STMicroelectronics (now known as ST). It has a 3D digital linear acceleration sensor, a 3D digital angular rate sensor (gyroscope), and a 3D digital magnetic sensor. It includes SPI and I2C (standard 100 kHz and fast mode 400 kHz) serial interfaces but the Arduino Nano 33 BLE uses I2C. Full details of this chip are available in the [LSM9DS1 data sheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf).

A gyroscope is a device that can measure the orientation and angular velocity of an object. Gyroscopes are more advanced than accelerometers, in that they can measure the tilt and lateral orientation, whereas an accelerometer can only measure its linear motion. Gyroscope sensors are also called “Angular Rate Sensors” or “Angular Velocity Sensors”. Angular velocity (measured in degrees per second) is the change in the rotational angle of the object per unit of time.

An accelerometer is an electromechanical device used to measure acceleration forces. Such forces may be static, like the continuous force of gravity or, as with a drone accelerating, dynamic to sense movement or vibrations. Changes in the acceleration force measured in the 3-axis directions can be used to determine the orientation of a drone. Accelerometer sensors are insensitive to rotation about the earth’s gravitational field vector.

A magnetometer is an instrument used for measuring magnetic forces, and in our context, the earth’s magnetism. The Earth’s magnetic field is a 3-dimensional vector that, like gravity, can be used to determine long-term orientation. The typical magnitude of the Earth’s magnetic field is between 20 µT and 70 µT.

## How to Use the Reefwing AHRS Library

The Reefwing AHRS Library includes the following classes:

  - Quaternion: converts Euler Angles to a Quaternion and includes open source complementary, madgwick and mahoney sensor fusion quaternion update methods.
  - LSM9DS1: for testing, calibrating and reading the IMU data from the sensor.
  - LPS22HB: to read the barometer pressure data if available (fitted to the Nano 33 BLE SENSE version only).

Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.

The Quaternion class is used by the LSM9DS1 class to provide fast updates to the roll, pitch and yaw Euler Angles derived from the IMU. The quaternion is a mathematical representation of the rotation matrix which describes a unique location in 3D space for our sensor.

At its simplest, a sketch will include:

```c++
#include <ReefwingAHRS.h>

LSM9DS1 imu;
EulerAngles angles;

void setup() {
  imu.begin();  //  Initialise the LSM9DS1 IMU
  imu.start();  //  Start processing sensor data
}

void loop() {
  angles = imu.update();  //  Check for new IMU data and update angles
}
```

Unless you are using the Fusion filter, it is strongly recommended that you run the test and calibrate sketch first to obtain the gyro, acc and mag biases for your board, see the `serialRollPitchYaw.ino` example sketch for how these are applied.

The Fusion filter has a gyroscope bias correction algorithm which provides run-time calibration of the gyroscope bias. The algorithm will detect when the gyroscope is stationary for a set period of time (`fusionThreshold`) and then begin to sample gyroscope measurements to calculate the bias as an average.

The EulerAngles data structure returned by the `imu.update()` method is defined as:

```c++
struct EulerAngles {
  //  Tait-Bryan Euler Angles, commonly used for aircraft orientation.
  float roll, pitch, yaw, heading;
  float rollRadians, pitchRadians, yawRadians;
};
```

You can also access the raw sensor data from the accelerometer, gyroscope and magnetometer using the `rawData()` method. For example,

```c++
SensorData data = imu.rawData();
```

The SensorData structure contains:

```c++
struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};
```

The LSM9DS1 class provides access to the quaternion which contains the latest fused sensor values with the `getQuaternion()` method. For example:

```c++
Quaternion quaternion = imu.getQuaternion();
```

The Quaternion class object includes the Euler Parameters:

```c++
float q0, q1, q2, q3;      //  Euler Parameters
```

The Quaternion class has a method for returning the associated Euler Angles:

```c++
EulerAngles toEulerAngles(float declination = 0.0);
```

The declination is an optional parameter in this method. The yaw angle is relative to magnetic north, including the magnetic declination for your area will correct this to a heading which indicates the angle to true north.

 ## Examples

 ### 1. IMU Configuration

The `imuConfig.ino` sketch will test if the gyroscope, magnetometer and accelerometer are connected via I2C. It does this by reading the value of the `WHO AM I` register and comparing this with the expected value. The expected values are defined as follows for the gyro/accelerometer, magnetometer and barometer. Note that a barometer is only fitted to the Nano 33 BLE SENSE board not the base Nano 33 BLE.

```c++
#define LSM9DS1XG_WHO_AM_I_VALUE    0x68
#define LSM9DS1M_WHO_AM_I_VALUE     0x3D
#define LPS22HB_WHO_AM_I_VALUE      0xB1
```

If the LSM9DS1 IMU is connected, the sketch will print out the Gyro chip temperature and default sensitivities for the three IMU sensors. The linear acceleration sensitivity is given in units of mg/LSB. This means milli-G's (1G = 9.8ms^-2) per Least Significant Bit. This is the smallest value that the sensor can return. Similarly, magnetic sensitivity is given in mGauss/LSB and angular rate sensitivity is mdps (milli degrees per second)/LSB. 

The sensitivity of the sensors depends on the full scale value selected (ref: Table 3 in the [LSM9DS1 Data Sheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)).

An angular rate gyroscope is device that produces a positive-going digital output for counterclockwise rotation around the axis considered. Sensitivity describes the gain of the sensor. This value changes very little over temperature and time. Magnetic sensor sensitivity also describes the gain of the sensor. 

### 2. Self Test and Calibration

Before you use an IMU for the first time, you need to calibrate it. The results wont be accurate if you don't do this. At the start of the self test, the Nano 33 BLE should be **level and not moving**.

#### 2.1 Gyroscope and Accelerometer

The LSM9DS1 includes a self test function that is not particularly well documented. To start the Gyro self-test, control register 10, bit ST_G needs to be set to 1. For the accelerometer, control register 10, bit ST_XL is set. The Reefwing AHRS library looks after this in the `selfTest` method. It calculates the average of the IMU at-rest readings and then loads these resulting offsets into the accelerometer and gyroscope bias registers.

An accelerometer in a steady state on a horizontal surface should measure 0 g on both the X-axis and Y-axis, whereas the Z-axis should measure 1 g. Ideally, the output is in the middle of the dynamic range of the sensor. A deviation from the ideal value is called a zero-g offset. Similarly, the gyro zero-rate level describes the actual output signal if there is no angular rate present and zero-gauss level offset describes the deviation of an actual output signal from the ideal output if no magnetic field is present.

Offset is to some extent a result of stress to MEMS sensor and therefore the offset can slightly change after mounting the sensor onto a printed circuit board or exposing it to extensive mechanical stress (like a drone crash). Offset changes little with temperature variation. 

From the [LSM9DS1 Data Sheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf), table 3, typical bias offsets are:

```
- Gyroscope (G_TyOff): ± 30 dps
- Accelerometer (LA_TyOff): ± 90 mg
- Magnetometer (M_TyOff): ± 1 gauss
```

This highlights the need for calibration. An uncorrected gyro offset of 30 degrees per second is material!

The results of the `selfTest()` function are loaded into a data structure, `SelfTestResults`, which is defined as:

```c++
struct SelfTestResults {
  float gyrodx;
  float gyrody;
  float gyrodz;
  float accdx;
  float accdy;
  float accdz;
};
```

A sensor will `PASS` the self test if these results are within the expected range:

- Gyroscope x, y and z axis expected range: 20 - 250 dps.
- Accelerometer x, y, and z axis expected range: 60 - 1700 mg.

#### 2.2 Magnetometer

The magnetometer included in the LSM9DS1 chip is the LIS3MDL. Its calibration is a bit different to the accelerometer and gyroscope. At the start of the magnetometer calibration, you will be asked to wave the Nano 33 BLE in a vertical figure of 8.

Since the magenetic field of the earth is relatively constant, the magnitude of the field measured by the magnetometer should also be constant, regardless of the orientation of the sensor. If one were to rotate the sensor and plot 𝑚𝑥, 𝑚𝑦, and 𝑚𝑧 in 3D, the paths should plot out the surface of a sphere with a constant radius.

In reality, due to hard and soft iron distortion, a 3D plot of magnetic field strength ends up as a deformed sphere. Calibration is used to adjust each of the three axis readings, so that the magnitude is constant regardless of orientation.

Performing the figure of 8 pattern 'traces out' part of our deformed sphere. From the coordinates obtained, the deformation of the sphere can be estimated, and the calibration coefficients calculated. A good pattern is one that traces through the greatest range of orientations and therefore estimates the greatest deviation from the true constant magnitude.

### 3. Roll, Pitch and Yaw Angles on Serial Port

This sketch is an example of a simple AHRS. It prints out the roll, pitch, yaw and heading angles to the Serial port every second. It is a good way to explore the capabilities of the library and for deciding which quaternion update method is best suited to your application. Running on a Nano 33 BLE SENSE, the loop frequency was just over 300 Hz for Mahony and Madgwick filters, and just under 300 Hz for the Complementary filter.

### 4. Complementary Filter

Largely the same as the previous sketch (roll, pitch and yaw on the serial port), it demonstrates how and where to set the filter free parameters. In this case, you can adjust `𝛂` and watch the effect on angle stability.

```c++
imu.setAlpha(0.95);
```

For the complementary filter, with `𝛂 = 0.0` the angle result will be  100% from the accelerometer and magnetometer. With `𝛂 = 1.0`, the result will be 100% from the gyroscope. A large `α` implies that the output will decay very slowly but will also be strongly influenced by even small changes in input. The output of this filter will decay towards zero for an unchanging input.

### 5. Beta Optimisation for the Madgwick Filter
The filter gain β represents all mean zero gyroscope measurement errors, expressed as the magnitude of a quaternion derivative. This error represents the gyroscope bias.

What this means is that beta is a parameter that you can tweak to optimise the filter results for your Arduino. In Madgwick's paper, he tests different values of 𝛃 (0 to 0.5) and graphs the results. A copy of this paper may be found in the theory folder of the Reefwing AHRS library. The y-axis on Madgwick's graph, [Effect of β on filter performance](https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf), represents the RMS error in degrees.

`There is a clear optimal value of β for each filter implementation; high enough to minimises errors due to integral drift but sufficiently low enough that unnecessary noise is not introduced by large steps of gradient descent iterations.`

RMS is often used to compare a theoretical prediction against an actual result. The root mean square (RMS) error is defined as the arithmetic mean of the squares of a set of numbers. If we take n angle measurements {𝜽1, 𝜽2, …, 𝜽n}, the RMS is:

`𝜽rms = √[(𝜽1² + 𝜽2² + … + 𝜽n²)/n]`

This sketch assists with beta optimisation of your Madgwick filter. It will print to the serial port the value for beta between 0.0 and 0.5 and its associated static RMS error in CSV format. You can graph these results. The sketch will record the beta with the minimum error in a static situation for pitch and roll. This sketch also demonstrates how you can adjust the filter gain in the Reefwing AHRS library. The optimum beta and minimum error is printed out at the end of the sketch.

```
Optimum Beta = 0.17, with an RMS error of 0.00 degrees. 
Default Beta = 0.60
```

You need to run the test a few times to get a feel for where the true optimum is. On our test board, the optimum was around 0.2.

However, for the Madgwick filter there is a tradeoff in the beta parameter between accuracy and response speed. In the original Madgwick study, a beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy. With this value, the LSM9SD1 response time is about 10 seconds to a stable initial quaternion. Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter!

Increasing beta (GyroMeasError) by about a factor of fifteen (beta = 0.6), the response time is reduced to ~2 seconds. We haven't noticed any reduction in solution accuracy with this reduced lag. This is essentially the I coefficient in a PID control sense; the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.

### 6. Fusion AHRS
[Fusion](https://github.com/xioTechnologies/Fusion) is the updated version of the Madgwick filter and was specifically developed for use with embedded systems and has been optimised for execution speed. 

```c++
  imu.setFusionAlgorithm(SensorFusion::FUSION);
  imu.setFusionPeriod(0.01f);   // Estimated sample period = 0.01 s = 100 Hz
  imu.setFusionGain(0.5);       // Default Fusion Filter Gain
```

The fusion AHRS sketch is similar to the other examples, apart from the Fusion specific configuration shown above. This is what we will focus on.

- `imu.setFusionPeriod(0.01f)`: One difference with this filter is that you need to know the frequency that the model will be updated in advance. This is normally either the sensor sample rate or the loop frequency of the Arduino board (whichever is slower). In this example we tweaked the delay amount in `loop()` to get 100 Hz (i.e., a period of 0.01 seconds).
- `imu.setFusionGain(0.5)`: The fusion algorithm is governed by a gain. A low gain will decrease the influence of the accelerometer and magnetometer so that the algorithm will better reject disturbances causes by translational motion and temporary magnetic distortions. However, a low gain will also increase the risk of drift due to gyroscope calibration errors. Madgwick suggests a gain value of 0.5, but this results in a lagging angle resolution. For a drone we think a gain of around 7.5 is better. This provides a much faster angle response without appearing to sacrifice too much accuracy.

For more details of how this algorithm operates, refer to the [Fusion Readme](theory/FUSION_README.md) in the theory folder of this repository.
