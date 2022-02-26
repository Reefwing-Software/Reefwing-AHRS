# Nexgen AHRS
 
 Our Magpie flight controller for quadcopters is based around the Arduino Nano 33 BLE board. This board includes the LSM9DS1 chip which we use as an Inertial Measurement Unit (IMU). In other words it determines the current orientation of the drone. 

 The Nexgen AHRS library provides an Attitude and Heading Reference System (AHRS) class for use with the Nano 33 BLE and the Nano 33 BLE SENSE.

 The AHRS will convert the gyroscope rate and accelerometer force data to a roll and pitch angle. The yaw angle is then calculated using the pitch, roll and magnetometer data. 

 ## Aircraft Reference Frame
 
 Any control system needs to use a defined reference frame. Being a drone application, we will use the convention for aircraft. This is based on three body co-ordinates and three attitude angles roll, pitch and yaw.

 The Aircraft Reference Frame (ARF) is relative to the drone itself and the origin is at the centre of gravity. Aircraft movement in the positive X, Y and Z directions translate to forward, right and down. In other words the positive X-Axis points towards the front of the drone.
 Rotation around the X, Y and Z-Axes are referred to as roll, pitch and yaw. The direction of positive rotation is defined by the co-ordinate right hand rule.

 Roll is defined as the angle between the Y Axis and the horizontal plane. When rotating the drone around the X Axis with the Y Axis moving downwards, roll is positive and increasing.

 Pitch is defined as the angle between the X Axis and the horizontal plane. When rotating the drone around the Y Axis with the X Axis moving upwards, pitch is positive and increasing.

 Yaw is defined as the angle between the X Axis and magnetic north on the
 horizontal plane measured in a clockwise direction when viewing from the top of the drone.

 The aircraft reference frame is useful in a piloted flight control context, since we want command inputs to be in reference to the current drone position and attitude.

 ## World Reference Frame

 Another possible reference frame is the world or ground. In this frame, the origin is defined as a location on Earth (e.g., the drone take off longitude, latitude and altitude) and the X, Y and Z-Axes correspond to North, East and Down. You may see this described as NED.

Heading is similar to yaw and defined as the angle between the X Axis and magnetic north on the horizontal plane measured in a clockwise direction when viewing from the top of the drone. Thus, a heading angle of 0° will point towards the North, and +90° towards the East.

The key difference between the Aircraft Reference Frame and the World Reference Frame is the origin. They can initially be the same point but once the drone moves, the co-ordinates will be different.

The World Reference Frame is useful in flight planning or return to home failsafe scenarios where we need to know the location of the drone and where it came from. For any sort of accuracy you will need to complement the IMU with a GPS for this type of mission. Some filter libraries (e.g., Madgewick fusion algorithm) also expect co-ordinates in the NED format.
 
## Inertial Navigation

Without a GPS, a drone relies on inertial navigation or dead reckoning to determine its position, velocity and orientation. It does this using an IMU to provide data to the Attitude and Heading Reference System (AHRS), which is part of the flight controller. The AHRS includes some sort of filtering or sensor fusion to deliver the most accurate roll, pitch and yaw data possible. Precision gyroscopes (e.g., ring lasers), are too expensive and bulky for drone applications and so less accurate MEMS (Micro Electrical Mechanical System) devices like the LSM9DS1 are used.

## The LSM9DS1 IMU

The LSM9DS1 is manufactured by STMicroelectronics (now known as ST). It has a 3D digital linear acceleration sensor, a 3D digital angular rate sensor (gyroscope), and a 3D digital magnetic sensor. It includes SPI and I2C (standard 100 kHz and fast mode 400 kHz) serial interfaces but the Arduino Nano 33 BLE uses I2C. Full details of this chip are available in the [LSM9DS1 data sheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf).

A gyroscope is a device that can measure the orientation and angular velocity of an object. Gyroscopes are more advanced than accelerometers, in that they can measure the tilt and lateral orientation, whereas an accelerometer can only measure its linear motion. Gyroscope sensors are also called “Angular Rate Sensors” or “Angular Velocity Sensors”. Angular velocity (measured in degrees per second) is the change in the rotational angle of the object per unit of time.

An accelerometer is an electromechanical device used to measure acceleration forces. Such forces may be static, like the continuous force of gravity or, as with a drone accelerating, dynamic to sense movement or vibrations. Changes in the acceleration force measured in the 3-axis directions can be used to determine the orientation of a drone. Accelerometer sensors are insensitive to rotation about the earth’s gravitational field vector.

A magnetometer is an instrument used for measuring magnetic forces, and in our context, the earth’s magnetism. The Earth’s magnetic field is a 3-dimensional vector that, like gravity, can be used to determine long-term orientation. The typical magnitude of the Earth’s magnetic field is between 20 µT and 70 µT.

 
