/******************************************************************
  @file       boardType.ino
  @brief      Detect the Arduino Board Type
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.2.0
  Date:        10/02/23

  1.0.0 Original Release.                         22/02/22
  1.1.0 Added NONE fusion option.                 25/05/22
  2.0.0 Changed Repo & Branding                   15/12/22
  2.0.1 Invert Gyro Values PR                     24/12/22
  2.1.0 Updated Fusion Library                    30/12/22
  2.2.0 Add support for Nano 33 BLE Sense Rev. 2  10/02/23

  Version 2.2.0 added support for the Nano 33 BLE Sense Rev. 2, 
  which uses two IMU sensors to replace the 9-axis IMU used in 
  previous versions. We need to detect the Arduino board type
  in order to load the correct IMU libraries.

******************************************************************/

#include <ReefwingAHRS.h>

ReefwingAHRS ahrs;

void setup() {
    ahrs.begin();
    
    //  Start Serial and wait for connection
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Attempting to detect hardware...\n");
    Serial.print("Result - ");
    Serial.println(ahrs.getBoardTypeString());
}

void loop() { }

