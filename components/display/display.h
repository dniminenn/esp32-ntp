#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include "esp_err.h"

class Display {
public:
  virtual ~Display() = default;
  virtual esp_err_t begin() = 0;
  virtual void clear() = 0;
  virtual void setIntensity(uint8_t intensity) = 0;
  virtual void drawTime(int hours, int minutes, int seconds) = 0;
  virtual void drawTopRowFromCentiseconds(unsigned long long centiseconds) = 0;
  virtual void drawPreSyncGlyph() = 0;
  virtual void push() = 0;
};
