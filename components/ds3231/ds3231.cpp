// SPDX-License-Identifier: Unlicense

#include "ds3231.h"
#include <string.h>
#include "esp_log.h"
#include "civil_time.h"

static const char* TAG = "DS3231";

static const uint8_t kAddr = 0x68;
static const int kTimeoutMs = 50;   // bounded; protects task WDT

// Registers
static const uint8_t kRegTime    = 0x00;  // sec min hour day date month year
static const uint8_t kRegControl = 0x0E;
static const uint8_t kRegStatus  = 0x0F;
static const uint8_t kRegTemp    = 0x11;  // MSB, LSB (two fraction bits)

static const uint8_t kStatusOSF     = 0x80;
static const uint8_t kStatusEn32k   = 0x08;
static const uint8_t kControlEOSC   = 0x80;

static uint8_t bcd2bin(uint8_t v) { return (uint8_t)((v >> 4) * 10 + (v & 0x0F)); }
static uint8_t bin2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }


esp_err_t Ds3231::begin(int sdaPin, int sclPin) {
  i2c_master_bus_config_t buscfg = {};
  buscfg.i2c_port = -1;                       // any free controller
  buscfg.sda_io_num = (gpio_num_t)sdaPin;
  buscfg.scl_io_num = (gpio_num_t)sclPin;
  buscfg.clk_source = I2C_CLK_SRC_DEFAULT;
  buscfg.glitch_ignore_cnt = 7;
  // internal pull-up, harmless extra
  buscfg.flags.enable_internal_pullup = 1;
  esp_err_t err = i2c_new_master_bus(&buscfg, &bus);
  if (err != ESP_OK) return err;

  err = i2c_master_probe(bus, kAddr, kTimeoutMs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "no DS3231 at 0x%02x (SDA=%d SCL=%d): %s",
             kAddr, sdaPin, sclPin, esp_err_to_name(err));
    i2c_del_master_bus(bus);
    bus = nullptr;
    return ESP_ERR_NOT_FOUND;
  }

  i2c_device_config_t devcfg = {};
  devcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  devcfg.device_address = kAddr;
  devcfg.scl_speed_hz = 100000;               // 100k for long wires
  err = i2c_master_bus_add_device(bus, &devcfg, &dev);
  if (err != ESP_OK) {
    i2c_del_master_bus(bus);
    bus = nullptr;
    return err;
  }
  return ESP_OK;
}

esp_err_t Ds3231::rd(uint8_t reg, uint8_t* buf, size_t n) {
  if (!dev) return ESP_ERR_INVALID_STATE;
  return i2c_master_transmit_receive(dev, &reg, 1, buf, n, kTimeoutMs);
}

esp_err_t Ds3231::wr(uint8_t reg, const uint8_t* buf, size_t n) {
  if (!dev) return ESP_ERR_INVALID_STATE;
  uint8_t tmp[8];
  if (n + 1 > sizeof(tmp)) return ESP_ERR_INVALID_ARG;
  tmp[0] = reg;
  memcpy(tmp + 1, buf, n);
  return i2c_master_transmit(dev, tmp, n + 1, kTimeoutMs);
}

bool Ds3231::readTime(time_t& out) {
  uint8_t st;
  if (rd(kRegStatus, &st, 1) != ESP_OK) return false;
  if (st & kStatusOSF) return false;          // oscillator stopped since last set
  uint8_t b[7];
  if (rd(kRegTime, b, 7) != ESP_OK) return false;
  int sec = bcd2bin(b[0] & 0x7F);
  int min = bcd2bin(b[1] & 0x7F);
  int hour;
  if (b[2] & 0x40) {                          // handle 12-hour mode
    hour = bcd2bin(b[2] & 0x1F) % 12 + ((b[2] & 0x20) ? 12 : 0);
  } else {
    hour = bcd2bin(b[2] & 0x3F);
  }
  int date  = bcd2bin(b[4] & 0x3F);
  int month = bcd2bin(b[5] & 0x1F);
  int year  = 2000 + bcd2bin(b[6]) + ((b[5] & 0x80) ? 100 : 0);
  if (month < 1 || month > 12 || date < 1 || date > 31) return false;
  out = civil_to_unix(year, month, date, hour, min, sec);
  return true;
}

esp_err_t Ds3231::setTime(time_t t) {
  struct tm tmv;
  gmtime_r(&t, &tmv);
  uint8_t b[7];
  b[0] = bin2bcd((uint8_t)tmv.tm_sec);
  b[1] = bin2bcd((uint8_t)tmv.tm_min);
  b[2] = bin2bcd((uint8_t)tmv.tm_hour);       // 24-hour mode
  b[3] = (uint8_t)(tmv.tm_wday + 1);
  b[4] = bin2bcd((uint8_t)tmv.tm_mday);
  b[5] = (uint8_t)(bin2bcd((uint8_t)(tmv.tm_mon + 1)) | (tmv.tm_year >= 200 ? 0x80 : 0));
  b[6] = bin2bcd((uint8_t)(tmv.tm_year % 100));
  esp_err_t err = wr(kRegTime, b, 7);
  if (err != ESP_OK) return err;
  // clear OSF
  uint8_t st;
  err = rd(kRegStatus, &st, 1);
  if (err != ESP_OK) return err;
  st &= (uint8_t)~kStatusOSF;
  return wr(kRegStatus, &st, 1);
}

bool Ds3231::readTempC(float& out) {
  uint8_t b[2];
  if (rd(kRegTemp, b, 2) != ESP_OK) return false;
  out = (float)(int8_t)b[0] + (float)(b[1] >> 6) * 0.25f;
  return true;
}

esp_err_t Ds3231::enable32kOutput() {
  // keep oscillator running on battery
  uint8_t ctl;
  esp_err_t err = rd(kRegControl, &ctl, 1);
  if (err != ESP_OK) return err;
  if (ctl & kControlEOSC) {
    ctl &= (uint8_t)~kControlEOSC;
    err = wr(kRegControl, &ctl, 1);
    if (err != ESP_OK) return err;
  }
  uint8_t st;
  err = rd(kRegStatus, &st, 1);
  if (err != ESP_OK) return err;
  if (!(st & kStatusEn32k)) {
    st |= kStatusEn32k;
    err = wr(kRegStatus, &st, 1);
  }
  return err;
}
