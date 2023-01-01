/******************************************************************
  @file       betaOptimisation.ino
  @brief      Determine optimal beta for Madgwick Filter
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.1.0
  Date:        15/12/22

  1.0.0 Original Release.           22/02/22
  1.1.0 Added NONE fusion option.   25/05/22
  2.0.0 Changed Repo & Branding     15/12/22
  2.0.1 Invert Gyro Values PR       24/12/22
  2.1.0 Updated Fusion Library      30/12/22

  This sketch attempts to determine the optimum beta for the
  Madgwick filter. The Nano 33 BLE should be placed flat and
  not moved for the duration of the test.

  The default sample rate for this library is 238 Hz for the 
  gyro/accelerometer and 10 Hz for the magnetometer. Thus a 
  new gyro/acc reading should be available every 4.2 ms. We delay
  for 10 ms between readings.

******************************************************************/

#include <ReefwingAHRS.h>

LSM9DS1 imu;
EulerAngles angles;

#define MIN_BETA          0.0
#define MAX_BETA          0.5
#define BETA_INC          0.01
#define SAMPLES_PER_BETA  20
#define GYRO_MEAS_ERROR   M_PI * (40.0f / 180.0f)               // gyroscope measurement error in rads/s (default is 40 deg/s)
#define DEFAULT_BETA      sqrt(3.0f / 4.0f) * GYRO_MEAS_ERROR

float beta = MIN_BETA;   //  Madgwick filter gain, Beta
float sampleArray[SAMPLES_PER_BETA];
float error, optimumBeta, minError = INFINITY;
int ctr = 0;           //  Track samples collected for each value of beta

float rms(float arr[]) {
    float square = 0.0, mean = 0.0;
    int n = sizeof(arr) / sizeof(arr[0]);
 
    // Calculate square.
    for (int i = 0; i < n; i++) {
        square += pow(arr[i], 2);
    }
 
    mean = (square / (float)(n));   // Calculate Mean.
 
    return sqrt(mean);
}

void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();

  //  Positive magnetic declination - Sydney, AUSTRALIA
  imu.setDeclination(12.717);
  imu.setFusionAlgorithm(SensorFusion::MADGWICK);
  imu.setBeta(beta);

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 

    //  Paste your calibration bias offset HERE
    //  This information comes from the testAndCalibrate.ino 
    //  sketch in the library examples sub-directory.
    imu.loadAccBias(0.070862, -0.046570, -0.016907);
    imu.loadGyroBias(0.800018, 0.269165, -0.097198);
    imu.loadMagBias(-0.192261, -0.012085, 0.118652);

    //  Start processing IMU data.
    Serial.println("Starting Beta Optimisation.\n"); 
    Serial.println("Nano 33 BLE should remain motionless for duration of test.\n");
    delay(1000); 
    Serial.print("Beta");  Serial.println("\tRMS Error");
    imu.start();
  }
  else {
    Serial.println("LSM9DS1 Accelerometer, Magnetometer and Gyroscope not found.");
    while (1);
  }

  while (beta < MAX_BETA) {
    while (ctr < SAMPLES_PER_BETA) {
      //  load sampleArray with measured pitch and roll angles
      angles = imu.update();  //  Check for new IMU data and update angles
      sampleArray[ctr] = angles.roll;
      ctr++;
      sampleArray[ctr] = angles.pitch;
      ctr++;
      delay(10);  //  Wait for new sample
    }
    ctr = 0;
    error = rms(sampleArray);
    if (error < minError) {
      minError = error;
      optimumBeta = beta;
    }
    
    Serial.print(beta); Serial.print("\t"); Serial.println(error);
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