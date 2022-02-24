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
 
 
