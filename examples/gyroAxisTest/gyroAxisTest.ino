// Copyright (c) 2026 David Such
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

/*****************************************************************************************
  Reefwing-AHRS Gyro Axis Test (NONE fusion, synthetic data)

  Purpose
  - Drive the NONE fusion path (ReefwingAHRS::updateEulerAngles(), also used by CLASSIC)
    with a synthetic gyroscope, one body axis at a time, and check that the integrated
    angle lands on the right Euler angle with the right magnitude.
  - Guards against the bug fixed in 2.3.8, where the X and Y gyro inputs were swapped
    and degrees per second were integrated as if they were radians per second. With that
    bug, 1 deg/s about X for one second produced ~57.3 deg of pitch instead of 1 deg of roll.

  Test cases (each starts from a reset, level attitude)
  1. Rotate about body X at +10 deg/s for 1 s  -> roll  = +10, pitch = 0,   yaw = 0
  2. Rotate about body Y at +10 deg/s for 1 s  -> roll  = 0,   pitch = +10, yaw = 0
  3. Rotate about body Z at +10 deg/s for 1 s  -> roll  = 0,   pitch = 0,   yaw = +10
  4. Roll to +90 deg, then rotate about body Y at +10 deg/s for 1 s
                                                -> roll  = +90, pitch = 0,   yaw = +10
     (at 90 deg of bank a body pitch rate changes heading, not pitch: this checks the
      Euler rate transformation, not just the axis mapping)

  Wiring
  - None. This sketch does not call ahrs.begin() and does not touch I2C.

  Output
  - Serial @ 115200. One PASS/FAIL line per case plus a summary.

******************************************************************************************/

#include <Arduino.h>
#include <ReefwingAHRS.h>

// ---------------------------- Configuration ----------------------------

static constexpr float    DT      = 0.01f;   // 10 ms - a typical 100 Hz IMU update rate
static constexpr uint32_t DT_US   = 10000;   // Same period in microseconds
static constexpr float    RATE    = 10.0f;   // Test rotation rate in deg/s
static constexpr float    TOL_DEG = 0.5f;    // Pass tolerance (allows for micros() jitter)

ReefwingAHRS ahrs;
int failures = 0;

// -------------------------------- Helpers --------------------------------

// Put the AHRS back to a known, level state running the NONE fusion path.
void resetLevel() {
  ahrs.reset();
  ahrs.setFusionAlgorithm(SensorFusion::NONE);
  ahrs.setDOF(DOF::DOF_6);   // No magnetometer: yaw comes from gyro integration only
}

// Feed a constant body rate (deg/s) to the AHRS for the given number of seconds at 1/DT Hz.
// ahrs.update() measures deltaT with micros(), so a fixed-rate ticker is used to pace it.
void spin(float gxDps, float gyDps, float gzDps, float seconds) {
  SensorData d{};
  d.az = 1.0f;              // 1 g straight down - not used by NONE, kept for realism
  d.gx = gxDps;
  d.gy = gyDps;
  d.gz = gzDps;

  const uint32_t steps = (uint32_t)(seconds / DT + 0.5f);
  uint32_t lastTickUs = micros();

  for (uint32_t i = 0; i < steps; i++) {
    while ((uint32_t)(micros() - lastTickUs) < DT_US) { /* wait for next tick */ }
    lastTickUs += DT_US;
    ahrs.setData(d, false);
    ahrs.update();
  }
}

void printAngle(const char* label, float got, float expected) {
  Serial.print(label); Serial.print(" "); Serial.print(got, 2);
  Serial.print(" (expected "); Serial.print(expected, 2); Serial.print(")");
}

bool check(const char* name, float expRoll, float expPitch, float expYaw) {
  const bool ok = fabs(ahrs.angles.roll  - expRoll)  < TOL_DEG &&
                  fabs(ahrs.angles.pitch - expPitch) < TOL_DEG &&
                  fabs(ahrs.angles.yaw   - expYaw)   < TOL_DEG;

  Serial.print(ok ? "PASS  " : "FAIL  ");
  Serial.print(name); Serial.print(": ");
  printAngle("roll",  ahrs.angles.roll,  expRoll);  Serial.print(", ");
  printAngle("pitch", ahrs.angles.pitch, expPitch); Serial.print(", ");
  printAngle("yaw",   ahrs.angles.yaw,   expYaw);
  Serial.println();

  if (!ok) { failures++; }
  return ok;
}

// ---------------------------------- Main ----------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait on boards that need it */ }

  Serial.println(F("Reefwing AHRS - Gyro Axis Test (NONE fusion)"));
  Serial.print(F("Rate = ")); Serial.print(RATE); Serial.print(F(" deg/s for 1 s, tolerance = "));
  Serial.print(TOL_DEG); Serial.println(F(" deg\n"));

  // 1. Body X -> roll
  resetLevel();
  spin(RATE, 0.0f, 0.0f, 1.0f);
  check("X axis -> roll ", RATE, 0.0f, 0.0f);

  // 2. Body Y -> pitch
  resetLevel();
  spin(0.0f, RATE, 0.0f, 1.0f);
  check("Y axis -> pitch", 0.0f, RATE, 0.0f);

  // 3. Body Z -> yaw
  resetLevel();
  spin(0.0f, 0.0f, RATE, 1.0f);
  check("Z axis -> yaw  ", 0.0f, 0.0f, RATE);

  // 4. Euler coupling: at 90 deg of roll, a body pitch rate is a heading change
  resetLevel();
  spin(90.0f, 0.0f, 0.0f, 1.0f);   // roll to +90 deg
  spin(0.0f, RATE, 0.0f, 1.0f);    // then pitch about body Y
  check("Y at 90 roll   ", 90.0f, 0.0f, RATE);

  Serial.println();
  if (failures == 0) {
    Serial.println(F("ALL TESTS PASSED"));
  } else {
    Serial.print(failures); Serial.println(F(" TEST(S) FAILED"));
  }
  Serial.println(F("TEST CONCLUDED"));
}

void loop() { }
