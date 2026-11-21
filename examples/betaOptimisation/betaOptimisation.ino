/******************************************************************
  @file       betaOptimisation.ino
  @brief      Determine optimal beta for Madgwick Filter
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     2.3.8
  Date:        22/11/26

  1.0.0 Original Release.                         22/02/22
  1.1.0 Added NONE fusion option.                 25/05/22
  2.0.0 Changed Repo & Branding                   15/12/22
  2.0.1 Invert Gyro Values PR                     24/12/22
  2.1.0 Updated Fusion Library                    30/12/22
  2.2.0 Add support for Nano 33 BLE Sense Rev. 2  10/02/23
  2.3.0 Fixed RMS Error calculation               22/11/24
  2.3.8 Apply beta to filter, reset per candidate  22/11/26

  This sketch attempts to determine the optimum beta for the
  Madgwick filter. The Nano 33 BLE R1 should be placed flat and
  not moved for the duration of the test.

  The default sample rate for this library is 238 Hz for the 
  gyro/accelerometer and 10 Hz for the magnetometer. Thus a 
  new gyro/acc reading should be available every 4.2 ms. We delay
  for 10 ms between readings.

******************************************************************/

#include <ReefwingAHRS.h>
#include <ReefwingLSM9DS1.h>

ReefwingLSM9DS1 imu;
ReefwingAHRS ahrs;

#define MIN_BETA          0.0
#define MAX_BETA          0.5
#define BETA_INC          0.01
#define SAMPLES_PER_BETA  20
#define SETTLE_MS         2000                                  // settling time allowed for each beta before sampling
#define SAMPLE_DELAY_MS   10                                    // delay between IMU reads
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
  //  Initialise the LSM9DS1 IMU & AHRS
  imu.begin();
  ahrs.begin();

  ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
  ahrs.setDeclination(12.717); //  Sydney, AUSTRALIA

  //  Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  Serial.print("Detected Board - ");
  Serial.println(ahrs.getBoardTypeString());

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    Serial.println("Calibrating IMU...\n"); 
    imu.start();
    imu.calibrateGyro();
    imu.calibrateAccel();
    imu.calibrateMag();

    delay(20);
    //  Flush the first reading - this is important!
    //  Particularly after changing the configuration.
    imu.readGyro();
    imu.readAccel();
    imu.readMag();

    //  Start processing IMU data.
    Serial.println("Starting Beta Optimisation.\n"); 
    Serial.println("Nano 33 BLE should remain motionless for duration of test.\n");
    Serial.print("Each beta is given "); Serial.print(SETTLE_MS / 1000.0); Serial.println(" s to settle before sampling.\n");
    delay(1000); 
    Serial.print("Beta");  Serial.println("\tRMS Error");
  } 
  else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

  while (beta < MAX_BETA) {
    //  Start every candidate from the same filter state, apply the
    //  candidate gain to the filter (previously the printed beta changed
    //  but the filter kept its default gain), then allow the same settling
    //  time before sampling so the comparison between betas is fair.
    ahrs.reset();
    ahrs.setFusionAlgorithm(SensorFusion::MADGWICK);
    ahrs.setBeta(beta);

    unsigned long settleStart = millis();

    while (millis() - settleStart < SETTLE_MS) {
      imu.updateSensorData();
      ahrs.setData(imu.data);
      ahrs.update();
      delay(SAMPLE_DELAY_MS);  //  Wait for new sample
    }

    while (ctr < SAMPLES_PER_BETA) {
      //  Check for new IMU data and update angles
      imu.updateSensorData();
      ahrs.setData(imu.data);
      ahrs.update();

      //  load sampleArray with either pitch or roll angles
      sampleArray[ctr] = ahrs.angles.roll;
      ctr++;
      // sampleArray[ctr] = ahrs.angles.pitch;
      // ctr++;
      delay(SAMPLE_DELAY_MS);  //  Wait for new sample
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