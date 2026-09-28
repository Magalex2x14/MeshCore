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

// nRF52 hardware RNG (available as the SoftDevice is not enabled)
class NRF52_RNG : public mesh::RNG {
public:
  void random(uint8_t* dest, size_t sz) override {
    NRF_RNG->CONFIG = RNG_CONFIG_DERCEN_Msk;
    NRF_RNG->TASKS_START = 1;
    for (size_t i = 0; i < sz; i++) {
      NRF_RNG->EVENTS_VALRDY = 0;
      while (!NRF_RNG->EVENTS_VALRDY) {
      }
      dest[i] = NRF_RNG->VALUE;
    }
    NRF_RNG->TASKS_STOP = 1;
  }
};

mesh::LocalIdentity radio_new_identity() {
  NRF52_RNG rng;
  return mesh::LocalIdentity(&rng);  // create new random identity
}
