/******************************************************************
  @file       filter_test.ino
  @brief      The sketch simulates IMU data with and without noise, 
              evaluates singularities, and measures accuracy and drift.
              
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0.0
  Date:        23/11/24

  1.0.0 Original Release.                         23/11/24

******************************************************************/

#include <ReefwingAHRS.h>

// Select the simulation parameters
#define SIMULATION_DURATION 20.0f // Seconds
#define SIMULATION_STEP 0.01f     // Seconds (100 Hz sampling rate)
#define NOISE_STDDEV 0.02f        // Standard deviation of noise for acceleration and gyro (rad/s and g)
#define ACTUAL_YAW 30.0f          // Ground truth yaw (degrees) - fixed
#define MAX_ROLL  30.0f           // Limit of synthesized roll angle ±MAX_ROLL (degrees)
#define MAX_PITCH 20.0f           // Limit of synthesized pitch angle ±MAX_PITCH (degrees)

// Instantiate AHRS
ReefwingAHRS ahrs;
SensorData simulatedData;

// Timing variables
unsigned long startTime;
float currentTime;

// Output variables
float rollError, pitchError, yawError;

// Function to add Gaussian noise
float addNoise(float value, float stddev) {
  return value + (stddev * ((float)random(-1000, 1000) / 1000.0f)); // Simulate noise
}

// Function to simulate ideal IMU data
void simulateIMUData(float t, bool addNoiseFlag, float &actualRoll, float &actualPitch, float &actualYaw) {
  // Ground truth: sinusoidal roll, pitch; constant yaw
  actualRoll = MAX_ROLL * sin(2 * PI * 0.1f * t);   // Roll in degrees
  actualPitch = MAX_PITCH * cos(2 * PI * 0.1f * t); // Pitch in degrees
  actualYaw = ACTUAL_YAW;                           // Yaw in degrees

  // Simulate gyro data (rad/s)
  simulatedData.gx = radians(2 * PI * 0.1f * MAX_ROLL * cos(2 * PI * 0.1f * t));   // d(roll)/dt
  simulatedData.gy = radians(-2 * PI * 0.1f * MAX_PITCH * sin(2 * PI * 0.1f * t)); // d(pitch)/dt
  simulatedData.gz = 0.0f; // d(yaw)/dt

  // Simulate accelerometer data (g)
  simulatedData.ax = sin(radians(actualPitch));
  simulatedData.ay = -sin(radians(actualRoll)) * cos(radians(actualPitch));
  simulatedData.az = cos(radians(actualRoll)) * cos(radians(actualPitch));

  // Simulate magnetometer data (arbitrary units)
  simulatedData.mx = cos(radians(actualYaw)) * cos(radians(actualPitch));
  simulatedData.my = sin(radians(actualYaw)) * cos(radians(actualPitch));
  simulatedData.mz = -sin(radians(actualPitch));

  // Add noise if required
  if (addNoiseFlag) {
    simulatedData.gx = addNoise(simulatedData.gx, NOISE_STDDEV);
    simulatedData.gy = addNoise(simulatedData.gy, NOISE_STDDEV);
    simulatedData.gz = addNoise(simulatedData.gz, NOISE_STDDEV);

    simulatedData.ax = addNoise(simulatedData.ax, NOISE_STDDEV);
    simulatedData.ay = addNoise(simulatedData.ay, NOISE_STDDEV);
    simulatedData.az = addNoise(simulatedData.az, NOISE_STDDEV);

    simulatedData.mx = addNoise(simulatedData.mx, NOISE_STDDEV);
    simulatedData.my = addNoise(simulatedData.my, NOISE_STDDEV);
    simulatedData.mz = addNoise(simulatedData.mz, NOISE_STDDEV);
  }

  // Set the simulated data in the AHRS library
  ahrs.setData(simulatedData);
}

// Test a single filter
void testFilter(SensorFusion filter, bool addNoiseFlag) {
  ahrs.reset();
  ahrs.setFusionAlgorithm(filter);
  ahrs.setDOF(DOF::DOF_9);

  Serial.print("Testing filter: ");
  Serial.print((int)filter);
  Serial.println(addNoiseFlag ? " with noise" : " without noise");
  Serial.println("Time,Roll Error,Pitch Error,Yaw Error");

  currentTime = 0.0f;
  startTime = millis();

  while (currentTime < SIMULATION_DURATION) {
    float actualRoll, actualPitch, actualYaw;

    simulateIMUData(currentTime, addNoiseFlag, actualRoll, actualPitch, actualYaw);
    ahrs.update();

    // Calculate errors using ground-truth angles
    rollError = ahrs.angles.roll - actualRoll;   // Error in roll angle
    pitchError = ahrs.angles.pitch - actualPitch; // Error in pitch angle
    yawError = ahrs.angles.yaw - actualYaw;       // Error in yaw angle

    // Output results
    Serial.print(currentTime, 3);
    Serial.print(",");
    Serial.print(rollError, 3);
    Serial.print(",");
    Serial.print(pitchError, 3);
    Serial.print(",");
    Serial.println(yawError, 3);

    currentTime += SIMULATION_STEP;
    delay(SIMULATION_STEP * 1000);
  }
}

void setup() {
  // Start Serial and wait for connection
  Serial.begin(115200);
  while (!Serial);

  // Initialize AHRS
  ahrs.begin();

  // Test each filter
  for (int i = 0; i <= (int)SensorFusion::NONE; i++) {
    testFilter(static_cast<SensorFusion>(i), false); // Test without noise
    testFilter(static_cast<SensorFusion>(i), true);  // Test with noise
  }
}

void loop() { }