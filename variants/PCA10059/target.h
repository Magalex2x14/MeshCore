#pragma once

#include <PCA10059Board.h>
#include <helpers/nrf52/NRF52WirelessRadio.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>

extern PCA10059Board board;
extern NRF52WirelessRadio radio_driver;
extern VolatileRTCClock rtc_clock;
extern SensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();
