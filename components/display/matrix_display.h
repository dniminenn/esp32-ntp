// SPDX-License-Identifier: Unlicense
#pragma once
#include <stdint.h>
#include "display.h"
#include "max7219_chain.h"

class MatrixDisplay : public Display {
public:
  MatrixDisplay(spi_host_device_t host, int csPin, int devices, int clockHz);
  esp_err_t begin() override;
  void clear() override;
  void setIntensity(uint8_t intensity) override;
  void drawTime(int hours, int minutes, int seconds) override;
  void drawTopRowFromCentiseconds(unsigned long long centiseconds) override;
  void drawPreSyncGlyph() override;
  void push() override;

private:
  Max7219Chain chain;
  int devices;
  uint8_t* screenBuffer;
  static uint8_t reverseByte(uint8_t b);
  void drawCharToBuffer(char c, int startCol);
  void drawDigit(int digit, int startCol);
  void drawColon(int col);
};

