#pragma once
#include <Arduino.h>
#include "Adafruit_Sensor.h"

// Minimal stub for Adafruit_ISM330DHCX to satisfy includes and basic usage in project.
// This is a temporary shim — replace with the real library for full functionality.

#define ISM330DHCX_ACCEL_RANGE_4_G 4
#define ISM330DHCX_GYRO_RANGE_500_DPS 500

class Adafruit_ISM330DHCX {
public:
  Adafruit_ISM330DHCX() {}
  bool begin_I2C() { return true; }
  void setAccelRange(int) {}
  void setGyroRange(int) {}
  bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp) {
    if (accel) {
      // Stub: report level orientation (gravity on Z) so pitch/roll
      // calculations don't operate on uninitialized values.
      accel->acceleration.x = 0.0f;
      accel->acceleration.y = 0.0f;
      accel->acceleration.z = 9.8f;
      accel->gyro.x = 0.0f;
      accel->gyro.y = 0.0f;
      accel->gyro.z = 0.0f;
    }
    if (gyro) {
      gyro->gyro.x = 0.0f;
      gyro->gyro.y = 0.0f;
      gyro->gyro.z = 0.0f;
    }
    if (temp) {
      temp->temperature = 0.0f;
    }
    return true;
  }
};
