#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>

PCA10059Board board;

NRF52WirelessRadio radio_driver;

VolatileRTCClock rtc_clock;
SensorManager sensors;

bool radio_init() {
  return radio_driver.init();
}

mesh::LocalIdentity radio_new_identity() {
  NRF52HardwareRNG rng;
  return mesh::LocalIdentity(&rng);  // create new random identity
}
