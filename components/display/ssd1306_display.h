#pragma once
// SPDX-License-Identifier: Unlicense
#include "display.h"
#include "driver/i2c_master.h"

/* SSD1306 / SH1106 over I2C, 128x32 or 128x64. Same three drawings as the matrix, scaled to fit. */
class Ssd1306Display : public Display {
public:
  Ssd1306Display(int sdaPin, int sclPin, uint8_t addr, int rows, bool flip, bool sh1106 = false);
  esp_err_t begin() override;
  void clear() override;
  void setIntensity(uint8_t intensity) override;
  void drawTime(int hours, int minutes, int seconds) override;
  void drawTopRowFromCentiseconds(unsigned long long centiseconds) override;
  void drawPreSyncGlyph() override;
  void push() override;

private:
  static constexpr int kWidth = 128;
  int sdaPin, sclPin, rows, pages, scale;
  uint8_t addr;
  bool flip;
  bool sh1106;
  i2c_master_dev_handle_t dev = nullptr;
  uint8_t fb[kWidth * 8];
  uint8_t sent[kWidth * 8];
  bool sentValid = false;
  esp_err_t cmd(const uint8_t* bytes, size_t n);
  void setPixel(int x, int y, bool on);
  void fillRect(int x, int y, int w, int h, bool on);
  int drawGlyph(char c, int x, int y);
};
