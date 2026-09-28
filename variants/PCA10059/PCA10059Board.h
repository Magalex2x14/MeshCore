#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/NRF52Board.h>

class PCA10059Board : public NRF52Board {
public:
  PCA10059Board() : NRF52Board("PCA10059_OTA") {}

  uint16_t getBattMilliVolts() override {
    return 0;  // USB powered, no battery
  }

  const char* getManufacturerName() const override {
    return "nRF52840 Dongle";
  }
};
