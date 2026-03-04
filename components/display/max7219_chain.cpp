// SPDX-License-Identifier: MIT-0

#include "max7219_chain.h"
#include "driver/gpio.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>

// MAX7219 registers
static const uint8_t REG_NOOP = 0x00;
static const uint8_t REG_DIGIT0 = 0x01; // up to 0x08
static const uint8_t REG_DECODE_MODE = 0x09;
static const uint8_t REG_INTENSITY = 0x0A;
static const uint8_t REG_SCAN_LIMIT = 0x0B;
static const uint8_t REG_SHUTDOWN = 0x0C;
static const uint8_t REG_DISPLAY_TEST = 0x0F;

Max7219Chain::Max7219Chain(spi_host_device_t h, int cs, int casc, int clk)
  : host(h), csPin(cs), cascade(casc), clockHz(clk), spi(nullptr) {
  shadowBuffer = (uint8_t*)malloc(cascade * 8);
  if (shadowBuffer) memset(shadowBuffer, 0, cascade * 8);
}

esp_err_t Max7219Chain::begin() {
  gpio_set_direction((gpio_num_t)csPin, GPIO_MODE_OUTPUT);
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = Config::getSpiMosiPin();
  buscfg.miso_io_num = Config::getSpiMisoPin();  // -1 for display (no MISO needed)
  buscfg.sclk_io_num = Config::getSpiSclkPin();
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 0;
  esp_err_t err;
  err = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = clockHz;
  devcfg.mode = 0;
  devcfg.spics_io_num = csPin;
  devcfg.queue_size = 4;
  devcfg.flags = SPI_DEVICE_NO_DUMMY;
  err = spi_bus_add_device(host, &devcfg, &spi);
  if (err != ESP_OK) return err;

  // Init sequence
  sendCommandAll(REG_DISPLAY_TEST, 0x00);
  sendCommandAll(REG_SCAN_LIMIT, 0x07);
  sendCommandAll(REG_DECODE_MODE, 0x00);
  sendCommandAll(REG_SHUTDOWN, 0x01);
  clear();
  setIntensity(1);
  return ESP_OK;
}

esp_err_t Max7219Chain::sendCommandAll(uint8_t reg, uint8_t data) {
  // For N cascaded, we must send N pairs (reg,data), most significant device first
  const int bytes = cascade * 2;
  uint8_t buf[16];
  if (bytes > (int)sizeof(buf)) return ESP_ERR_INVALID_SIZE;
  for (int i = 0; i < cascade; ++i) {
    buf[i*2 + 0] = reg;
    buf[i*2 + 1] = data;
  }
  spi_transaction_t t = {};
  t.length = bytes * 8;
  t.tx_buffer = buf;
  return spi_device_transmit(spi, &t);
}

esp_err_t Max7219Chain::setIntensity(uint8_t intensity) {
  if (intensity > 0x0F) intensity = 0x0F;
  return sendCommandAll(REG_INTENSITY, intensity);
}

esp_err_t Max7219Chain::setUpdateMode(bool enabled) {
  return sendCommandAll(REG_SHUTDOWN, enabled ? 0x01 : 0x00);
}

esp_err_t Max7219Chain::clear() {
  if (shadowBuffer) memset(shadowBuffer, 0, cascade * 8);
  
  for (int digit = 0; digit < 8; ++digit) {
    const int bytes = cascade * 2;
    uint8_t buf[16];
    for (int i = 0; i < cascade; ++i) {
      buf[i*2 + 0] = REG_DIGIT0 + digit;
      buf[i*2 + 1] = 0x00;
    }
    spi_transaction_t t = {};
    t.length = bytes * 8;
    t.tx_buffer = buf;
    esp_err_t err = spi_device_transmit(spi, &t);
    if (err != ESP_OK) return err;
  }
  return ESP_OK;
}

esp_err_t Max7219Chain::writeRow(int deviceIndex, int row, uint8_t value) {
  // FC16_HW: DIGIT registers control rows
  int digitReg = REG_DIGIT0 + row;

  const int bytes = cascade * 2;
  uint8_t buf[16];
  for (int i = 0; i < cascade; ++i) {
    if (i == deviceIndex) {
      buf[i*2 + 0] = digitReg;
      buf[i*2 + 1] = value;
    } else {
      buf[i*2 + 0] = REG_NOOP;
      buf[i*2 + 1] = 0x00;
    }
  }
  spi_transaction_t t = {};
  t.length = bytes * 8;
  t.tx_buffer = buf;
  return spi_device_transmit(spi, &t);
}

esp_err_t Max7219Chain::setColumn(int col, uint8_t value) {
  if (!shadowBuffer) return ESP_ERR_NO_MEM;
  
  // FC16_HW: transpose column data into row registers
  // col is the global column [0..cascade*8-1]
  // value has 8 bits, one per row (bit 0 = top row before reverseByte)
  
  int deviceIndex = (cascade - 1) - (col / 8);
  int colInDevice = 7 - (col % 8);
  
  // Update shadow buffer: each of the 8 rows for this device
  for (int row = 0; row < 8; row++) {
    uint8_t bit = (value >> (7 - row)) & 0x01;
    int shadowIdx = deviceIndex * 8 + row;
    
    if (bit) {
      shadowBuffer[shadowIdx] |= (1 << colInDevice);
    } else {
      shadowBuffer[shadowIdx] &= ~(1 << colInDevice);
    }
  }
  
  return ESP_OK;
}

static uint8_t reverseByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

esp_err_t Max7219Chain::flush() {
  if (!shadowBuffer) return ESP_ERR_NO_MEM;

  // Send one transaction per row across the entire cascade (8 transactions total)
  for (int row = 0; row < 8; row++) {
    const int bytes = cascade * 2;
    uint8_t buf[16];
    for (int i = 0; i < cascade; ++i) {
      buf[i*2 + 0] = REG_DIGIT0 + row;
      buf[i*2 + 1] = reverseByte(shadowBuffer[i * 8 + row]);
    }
    spi_transaction_t t = {};
    t.length = bytes * 8;
    t.tx_buffer = buf;
    esp_err_t err = spi_device_transmit(spi, &t);
    if (err != ESP_OK) return err;
  }

  return ESP_OK;
}


