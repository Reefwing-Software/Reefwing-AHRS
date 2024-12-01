/******************************************************************
  @file       betaOptimisationR2.ino
  @brief      Determine optimal beta for Madgwick Filter
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        22/11/24

  1.0.0 Original Release.                         22/11/24

  This sketch targets the Nano 33 BLE Rev. 2 and attempts 
  to determine the optimum beta for the Madgwick filter. 
  The Nano 33 BLE should be placed flat and not moved for 
  the duration of the test.

  The default sample rate for this library is 100 Hz for the 
  gyro/accelerometer and 10 Hz for the magnetometer. Thus a 
  new gyro/acc reading should be available every 10 ms. We delay
  for 15 ms between readings.

******************************************************************/

#include <ReefwingAHRS.h>
#include <Arduino_BMI270_BMM150.h>

ReefwingAHRS ahrs;
SensorData data;

#define MIN_BETA          0.0
#define MAX_BETA          0.5
#define BETA_INC          0.01
#define SAMPLES_PER_BETA  20
#define GYRO_MEAS_ERROR   M_PI * (40.0f / 180.0f)               // gyroscope measurement error in rads/s (default is 40 deg/s)
#define DEFAULT_BETA      sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR

float beta = MIN_BETA;   //  Madgwick filter gain, Beta
float sampleArray[SAMPLES_PER_BETA];
float err, optimumBeta, minError = INFINITY;
int ctr = 0;           //  Track samples collected for each value of beta

float rms(float arr[], int n) {
    float square = 0.0, mean = 0.0;

    // Calculate square.
    for (int i = 0; i < n; i++) {
        square += pow(arr[i], 2);
    }

    mean = square / (float)n;   // Calculate Mean.
    return sqrt(mean);
}

void setup() {
  //  Initialise the AHRS
  ahrs.begin();

  //  Positive magnetic declination - Sydney, AUSTRALIA
  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  ahrs.setDeclination(12.717);

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  if (IMU.begin()) {
    Serial.println("BMI270 & BMM150 IMUs Connected."); 

    delay(20);
    //  Flush the first reading - this is important!
    //  Particularly after changing the configuration.
    IMU.readGyroscope(data.gx, data.gy, data.gz);
    IMU.readAcceleration(data.ax, data.ay, data.az);

    //  Start processing IMU data.
    Serial.println("Starting Beta Optimisation.\n"); 
    Serial.println("Nano 33 BLE should remain motionless for duration of test.\n");
    delay(1000); 
    Serial.print("Beta");  Serial.println("\tRMS Error");
  } 
  else {
    Serial.println("IMU Not Detected.");
    while(1);
  }

  while (beta < MAX_BETA) {
    while (ctr < SAMPLES_PER_BETA) {
      //  Check for new IMU data and update angles
      if (IMU.gyroscopeAvailable()) {  IMU.readGyroscope(data.gx, data.gy, data.gz);  }
      if (IMU.accelerationAvailable()) {  IMU.readAcceleration(data.ax, data.ay, data.az);  }
      if (IMU.magneticFieldAvailable()) {  IMU.readMagneticField(data.mx, data.my, data.mz);  }
  
      ahrs.setData(data);
      ahrs.update();

      //  load sampleArray with either pitch or roll angles
      sampleArray[ctr] = ahrs.angles.roll;
      ctr++;
      // sampleArray[ctr] = ahrs.angles.pitch;
      // ctr++;
      delay(15);  //  Wait for new sample
    }
    ctr = 0;
    err = rms(sampleArray, SAMPLES_PER_BETA);
    if (err < minError) {
      minError = err;
      optimumBeta = beta;
    }
    
    Serial.print(beta); Serial.print("\t"); Serial.println(err);
    beta += BETA_INC;
  }

  Serial.print("\nOptimum Beta = ");
  Serial.print(optimumBeta);
  Serial.print(", with an RMS error of ");
  Serial.print(minError);
  Serial.println(" degrees.");
  Serial.print("Default Beta = ");
  Serial.println(DEFAULT_BETA);
  Serial.println("\nTEST CONCLUDED");
}

void loop() { }