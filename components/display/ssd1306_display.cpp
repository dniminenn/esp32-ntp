// SPDX-License-Identifier: Unlicense
#include "ssd1306_display.h"
#include "presync_glyph.h"
#include "config.h"
#include "i2c_bus.h"
#include <string.h>

/* 5x7, column major, bit 0 is the top row. Colon is two columns wide. */
static const uint8_t kFont[11][5] = {
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46},
  {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
  {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03}, {0x36,0x49,0x49,0x49,0x36},
  {0x06,0x49,0x49,0x29,0x1E}, {0x36,0x36,0x00,0x00,0x00},
};
static const int kTimeoutMs = 50;

Ssd1306Display::Ssd1306Display(int sda, int scl, uint8_t a, int r, bool f)
  : sdaPin(sda), sclPin(scl), rows(r == 32 ? 32 : 64), pages(rows / 8), scale(rows / 8 > 4 ? 3 : 2),
    addr(a), flip(f) {
  memset(fb, 0, sizeof(fb));
}

esp_err_t Ssd1306Display::cmd(const uint8_t* bytes, size_t n) {
  uint8_t buf[16];
  if (!dev || n + 1 > sizeof(buf)) return ESP_ERR_INVALID_ARG;
  buf[0] = 0x00;
  memcpy(buf + 1, bytes, n);
  return i2c_master_transmit(dev, buf, n + 1, kTimeoutMs);
}

esp_err_t Ssd1306Display::begin() {
  i2c_master_bus_handle_t bus;
  esp_err_t err = i2c_bus_get(sdaPin, sclPin, &bus);
  if (err != ESP_OK) return err;
  err = i2c_master_probe(bus, addr, kTimeoutMs);
  if (err != ESP_OK) return ESP_ERR_NOT_FOUND;
  i2c_device_config_t devcfg = {};
  devcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devcfg.device_address = addr;
  devcfg.scl_speed_hz = 400000;
  err = i2c_master_bus_add_device(bus, &devcfg, &dev);
  if (err != ESP_OK) return err;

  const uint8_t init[] = {
    0xAE,
    0xD5, 0x80,
    0xA8, (uint8_t)(rows - 1),
    0xD3, 0x00,
    0x40,
    0x8D, 0x14,
    0x20, 0x00,
    (uint8_t)(flip ? 0xA0 : 0xA1),
    (uint8_t)(flip ? 0xC0 : 0xC8),
    0xDA, (uint8_t)(rows == 64 ? 0x12 : 0x02),
    0x81, 0x7F,
    0xD9, 0xF1,
    0xDB, 0x40,
    0xA4,
    0xA6,
    0x2E,
  };
  for (size_t i = 0; i < sizeof(init); ) {
    size_t n = 1;
    if (init[i] == 0xD5 || init[i] == 0xA8 || init[i] == 0xD3 || init[i] == 0x8D || init[i] == 0x20 ||
        init[i] == 0xDA || init[i] == 0x81 || init[i] == 0xD9 || init[i] == 0xDB) n = 2;
    if ((err = cmd(init + i, n)) != ESP_OK) return err;
    i += n;
  }
  clear();
  sentValid = false;
  push();
  const uint8_t on = 0xAF;
  return cmd(&on, 1);
}

void Ssd1306Display::clear() { memset(fb, 0, sizeof(fb)); }

void Ssd1306Display::setIntensity(uint8_t intensity) {
  if (intensity > 15) intensity = 15;
  const uint8_t c[] = { 0x81, (uint8_t)((intensity + 1) * 16 - 1) };
  cmd(c, 2);
}

void Ssd1306Display::setPixel(int x, int y, bool on) {
  if (x < 0 || x >= kWidth || y < 0 || y >= rows) return;
  uint8_t& b = fb[(y >> 3) * kWidth + x];
  if (on) b |= (uint8_t)(1 << (y & 7)); else b &= (uint8_t)~(1 << (y & 7));
}

void Ssd1306Display::fillRect(int x, int y, int w, int h, bool on) {
  for (int j = 0; j < h; ++j)
    for (int i = 0; i < w; ++i) setPixel(x + i, y + j, on);
}

int Ssd1306Display::drawGlyph(char c, int x, int y) {
  int idx = (c >= '0' && c <= '9') ? c - '0' : (c == ':' ? 10 : -1);
  if (idx < 0) return 0;
  int cols = idx == 10 ? 2 : 5;
  for (int col = 0; col < cols; ++col)
    for (int row = 0; row < 7; ++row)
      fillRect(x + col * scale, y + row * scale, scale, scale, (kFont[idx][col] >> row) & 1);
  return cols * scale;
}

void Ssd1306Display::drawTime(int hours, int minutes, int seconds) {
  char text[9];
  snprintf(text, sizeof(text), "%02d:%02d:%02d", hours, minutes, seconds);
  int bar = scale;
  int y = bar + ((rows - bar) - 7 * scale) / 2;
  int x = (kWidth - 41 * scale) / 2;
  fillRect(0, bar, kWidth, rows - bar, false);
  for (const char* p = text; *p; ++p) x += drawGlyph(*p, x, y) + scale;
}

void Ssd1306Display::drawTopRowFromCentiseconds(unsigned long long centiseconds) {
  for (int bit = 0; bit < 32; ++bit)
    fillRect(bit * 4, 0, 3, scale, (centiseconds >> (31 - bit)) & 1);
}

void Ssd1306Display::drawPreSyncGlyph() {
  if (!Config::getUsePreSyncGlyph()) return;
  int x0 = (kWidth - 32 * scale) / 2;
  int y0 = (rows - 8 * scale) / 2;
  for (int tile = 0; tile < 4; ++tile)
    for (int row = 0; row < 8; ++row)
      for (int col = 0; col < 8; ++col)
        fillRect(x0 + (tile * 8 + col) * scale, y0 + row * scale, scale, scale,
                 (kPreSyncTiles[tile][row] >> (7 - col)) & 1);
}

void Ssd1306Display::push() {
  if (!dev) return;
  uint8_t buf[1 + kWidth];
  for (int page = 0; page < pages; ++page) {
    const uint8_t* src = fb + page * kWidth;
    if (sentValid && memcmp(src, sent + page * kWidth, kWidth) == 0) continue;
    const uint8_t win[] = { 0x21, 0x00, (uint8_t)(kWidth - 1), 0x22, (uint8_t)page, (uint8_t)page };
    if (cmd(win, 3) != ESP_OK || cmd(win + 3, 3) != ESP_OK) return;
    buf[0] = 0x40;
    memcpy(buf + 1, src, kWidth);
    if (i2c_master_transmit(dev, buf, sizeof(buf), kTimeoutMs) != ESP_OK) return;
    memcpy(sent + page * kWidth, src, kWidth);
  }
  sentValid = true;
}
