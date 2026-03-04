// SPDX-License-Identifier: MIT-0
#pragma once
#include <stdint.h>
#include "max7219_chain.h"

class Display {
public:
  Display(spi_host_device_t host, int csPin, int devices, int clockHz);
  esp_err_t begin();
  void clear();
  void setIntensity(uint8_t intensity);
  void drawTime(int hours, int minutes, int seconds);
  void drawTopRowFromCentiseconds(unsigned long long centiseconds);
  void drawPreSyncGlyph();
  void push();

private:
  Max7219Chain chain;
  int devices;
  uint8_t* screenBuffer;
  static uint8_t reverseByte(uint8_t b);
  void drawCharToBuffer(char c, int startCol);
  void drawDigit(int digit, int startCol);
  void drawColon(int col);
};

