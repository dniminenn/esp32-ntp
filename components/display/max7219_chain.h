// SPDX-License-Identifier: MIT-0
#pragma once
#include <stdint.h>
#include "driver/spi_master.h"

class Max7219Chain {
public:
  Max7219Chain(spi_host_device_t host, int csPin, int cascadeSize, int clockHz);
  esp_err_t begin();
  esp_err_t setIntensity(uint8_t intensity);
  esp_err_t clear();
  esp_err_t setColumn(int col, uint8_t value);
  esp_err_t flush();
  esp_err_t setUpdateMode(bool enabled);
  int columns() const { return cascade * 8; }

private:
  spi_host_device_t host;
  int csPin;
  int cascade;
  int clockHz;
  spi_device_handle_t spi;
  uint8_t* shadowBuffer;
  esp_err_t sendCommandAll(uint8_t reg, uint8_t data);
  esp_err_t writeRow(int deviceIndex, int row, uint8_t value);
};


