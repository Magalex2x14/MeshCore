#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>

RAK3401Board board;

#ifndef PIN_USER_BTN
  #define PIN_USER_BTN (-1)
#endif

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true, true);

  #if defined(PIN_USER_BTN_ANA)
  MomentaryButton analog_btn(PIN_USER_BTN_ANA, 1000, 20);
  #endif
#endif

#ifdef NRF52_WIRELESS_RADIO
WRAPPER_CLASS radio_driver;
#else
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);
#endif

VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);

#if ENV_INCLUDE_GPS
  #include <helpers/sensors/MicroNMEALocationProvider.h>
  MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1, &rtc_clock);
  EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
  EnvironmentSensorManager sensors;
#endif

bool radio_init() {
  rtc_clock.begin(Wire);
#ifdef NRF52_WIRELESS_RADIO
  digitalWrite(SX126X_POWER_EN, LOW);  // LoRa FEM (SKY66122) not used
  return radio_driver.init();
#else
  return radio.std_init(&SPI);
#endif
}

mesh::LocalIdentity radio_new_identity() {
#ifdef NRF52_WIRELESS_RADIO
  NRF52HardwareRNG rng;
#else
  RadioNoiseListener rng(radio);
#endif
  return mesh::LocalIdentity(&rng);  // create new random identity
}

