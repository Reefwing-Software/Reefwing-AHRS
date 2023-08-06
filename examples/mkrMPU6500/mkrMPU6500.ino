/******************************************************************
  @file       mkrMPU6500.ino
  @brief      Print roll, pitch, yaw and heading angles using the
              MPU6500 IMU and a MKR board, connected via SPI.
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

  This sketch is configured to work with the CLASSIC, KALMAN & NONE Sensor 
  Fusion options. Set the algorithm that you wish to use with:

  imu.setFusionAlgorithm(SensorFusion::KALMAN);

  The other Sensor Fusion algorithms require a 9 DOF IMU (i.e., 
  they include a magnetometer).

  We are using the Reefwing MPU6500 library. Substitute the
  appropriate library and setup/calibration for your IMU.

******************************************************************/

#include <ReefwingAHRS.h>
#include <ReefwingMPU6x00.h>

ReefwingAHRS ahrs;
SensorData data;

//  SPI and LED Pins
static const uint8_t MOSI_PIN = 8;
static const uint8_t MISO_PIN = 10;
static const uint8_t SCLK_PIN = 9;
static const uint8_t CS_PIN  = 7;
static const uint8_t INT_PIN = 1;
static const uint8_t LED0_PIN = A4;
static const uint8_t LED1_PIN = A5;

static MPU6500 imu = MPU6500(SPI, CS_PIN);

//  Display and Loop Frequency
int loopFrequency = 0;
const long displayPeriod = 1000;
unsigned long previousMillis = 0;

static void blinkLED(void) {
    const auto msec = millis();
    static uint32_t prev;

    if (msec - prev > 500) {
        static bool on;

        digitalWrite(LED0_PIN, on);
        on = !on;
        prev = msec;
    }
}

static bool gotInterrupt;

static void handleInterrupt(void) {
    gotInterrupt = true;
}

void setup() {
  //  Pin Configuration
  pinMode(INT_PIN, INPUT);
  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);

  //  Initialise the AHRS
  //  begin() will detect the Portenta H7 or MKR Vidor 4000 
  //  but these have no default IMU.
  ahrs.begin();
  ahrs.setImuType(ImuType::MPU6500);
  ahrs.setDOF(DOF::DOF_6);
  ahrs.setFusionAlgorithm(SensorFusion::CLASSIC);
  ahrs.setDeclination(12.717);                      //  Sydney, Australia

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  // Initialise SPI and the MPU6500 IMU
  SPI.begin();

  if (imu.begin()) {
      Serial.println("MPU6500 IMU Connected.");
  }
  else {
      Serial.println("Error initializing MPU6500 IMU.");
      digitalWrite(LED1_PIN, HIGH);
      while(1);
  }

  attachInterrupt(INT_PIN, handleInterrupt, RISING);
}

void loop() {
  blinkLED();

  if (gotInterrupt) {
    data = imu.getSensorData();
    ahrs.setData(data);
    ahrs.update();
    gotInterrupt = false;
  }

  if (millis() - previousMillis >= displayPeriod) {
    //  Display sensor data every displayPeriod, non-blocking.
    Serial.print("--> Roll: ");
    Serial.print(ahrs.angles.roll, 2);
    Serial.print("\tPitch: ");
    Serial.print(ahrs.angles.pitch, 2);
    Serial.print("\tYaw: ");
    Serial.print(ahrs.angles.yaw, 2);
    Serial.print("\tHeading: ");
    Serial.print(ahrs.angles.heading, 2);
    Serial.print("\tLoop Frequency: ");
    Serial.print(loopFrequency);
    Serial.println(" Hz");

    loopFrequency = 0;
    previousMillis = millis();
  }

  loopFrequency++;
}