// SPDX-License-Identifier: MIT-0

#include "display.h"
#include "config.h"
#include <string.h>

static const uint8_t font3x5_digits[10][3] = {
  {0x1F, 0x11, 0x1F},
  {0x00, 0x1F, 0x00},
  {0x1D, 0x15, 0x17},
  {0x11, 0x15, 0x1F},
  {0x07, 0x04, 0x1F},
  {0x17, 0x15, 0x1D},
  {0x1F, 0x15, 0x1D},
  {0x01, 0x01, 0x1F},
  {0x1F, 0x15, 0x1F},
  {0x17, 0x15, 0x1F}
};

Display::Display(spi_host_device_t host, int csPin, int devs, int clockHz)
  : chain(host, csPin, devs, clockHz), devices(devs) {
  screenBuffer = (uint8_t*)malloc(devices * 8);
}

esp_err_t Display::begin() {
  return chain.begin();
}

void Display::clear() {
  if (screenBuffer) memset(screenBuffer, 0x00, devices * 8);
  chain.clear();
}

void Display::setIntensity(uint8_t intensity) { chain.setIntensity(intensity); }

uint8_t Display::reverseByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

void Display::drawDigit(int digit, int startCol) {
  if (digit < 0 || digit > 9 || startCol + 3 > devices * 8) return;
  char digitChar = '0' + digit;
  drawCharToBuffer(digitChar, startCol);
}

void Display::drawCharToBuffer(char c, int startCol) {
  if (!screenBuffer) return;
  if (startCol + 3 > devices * 8) return;
  if (c >= '0' && c <= '9') {
    int digit = c - '0';
    for (int i = 0; i < 3; i++) {
      // Preserve bit 0 (top row), overwrite bits 1-7
      uint8_t topBit = screenBuffer[startCol + i] & 0x01;
      screenBuffer[startCol + i] = (uint8_t)((font3x5_digits[digit][i] << 2) | topBit);
    }
  }
}

void Display::drawColon(int col) {
  if (!screenBuffer) return;
  if (col >= devices * 8) return;
  // Preserve bit 0 (top row)
  uint8_t topBit = screenBuffer[col] & 0x01;
  screenBuffer[col] = (uint8_t)((0x14 << 1) | topBit);
}

void Display::drawTime(int hours, int minutes, int seconds) {
  if (!screenBuffer) return;
  int col = 2;
  drawDigit(hours / 10, col); col += 3; col += 1;
  drawDigit(hours % 10, col); col += 3; col += 1;
  drawColon(col);              col += 1; col += 1;
  drawDigit(minutes / 10, col); col += 3; col += 1;
  drawDigit(minutes % 10, col); col += 3; col += 1;
  drawColon(col);               col += 1; col += 1;
  drawDigit(seconds / 10, col); col += 3; col += 1;
  drawDigit(seconds % 10, col);
}

void Display::drawTopRowFromCentiseconds(unsigned long long centiseconds) {
  if (!screenBuffer) return;
  int totalCols = devices * 8;
  
  for (int col = 0; col < totalCols; col++) {
    // Show bits 31-0, MSB first
    uint8_t bit = (uint8_t)((centiseconds >> (31 - col)) & 0x1);
    screenBuffer[col] = (uint8_t)((screenBuffer[col] & 0xFE) | bit);
  }
}

void Display::drawPreSyncGlyph() {
  if (!Config::getUsePreSyncGlyph()) return;
  if (!screenBuffer) return;
  if (devices < 4) return;
  // Panels are wired right-to-left in setColumn/flush. Compute device indices.
  int rightmost = devices - 1;
  int secondRight = devices - 2;
  int secondLeft = 1;
  int leftmost = 0;

  // Write 8 bytes per device into the shadow buffer via columns.
  auto writeDeviceRows = [&](int deviceIndex, const uint8_t rows[8]) {
    // For each column in this device (0..7), build column bit from rows' bits
    for (int colInDevice = 0; colInDevice < 8; colInDevice++) {
      uint8_t colBits = 0;
      for (int row = 0; row < 8; row++) {
        uint8_t rowByte = rows[row];
        // Bit 7 is leftmost pixel in a row definition; map to column
        int bitIndex = 7 - colInDevice;
        uint8_t on = (rowByte >> bitIndex) & 0x1;
        colBits |= (uint8_t)(on << (7 - row));
      }
      int globalCol = deviceIndex * 8 + colInDevice;
      if (globalCol >= 0 && globalCol < devices * 8) {
        screenBuffer[globalCol] = colBits;
      }
    }
  };

  const uint8_t glyph_rightmost[8] = {0x00,0x00,0x1e,0xe1,0x07,0xe1,0x1e,0x00};
  const uint8_t glyph_middle[8]    = {0x00,0x00,0x00,0xff,0x00,0xff,0x00,0x00};
  const uint8_t glyph_leftmost[8]  = {0x70,0x88,0x88,0x7f,0x70,0x8f,0x88,0x70};

  writeDeviceRows(rightmost, glyph_rightmost);
  writeDeviceRows(secondRight, glyph_middle);
  writeDeviceRows(secondLeft, glyph_middle);
  writeDeviceRows(leftmost, glyph_leftmost);
}

void Display::push() {
  if (!screenBuffer) return;
  chain.setUpdateMode(false);
  for (int col = 0; col < (devices * 8); col++) {
    chain.setColumn(col, screenBuffer[col]);
  }
  chain.flush();
  chain.setUpdateMode(true);
}


