// SPDX-License-Identifier: Unlicense
#include "i2c_bus.h"
#include "soc/soc_caps.h"

typedef struct { int sda, scl; i2c_master_bus_handle_t bus; } slot_t;
static slot_t s_slots[SOC_I2C_NUM];

esp_err_t i2c_bus_get(int sda, int scl, i2c_master_bus_handle_t* out) {
  if (!out || sda < 0 || scl < 0) return ESP_ERR_INVALID_ARG;
  slot_t* spare = NULL;
  for (int i = 0; i < SOC_I2C_NUM; ++i) {
    if (s_slots[i].bus && s_slots[i].sda == sda && s_slots[i].scl == scl) { *out = s_slots[i].bus; return ESP_OK; }
    if (!s_slots[i].bus && !spare) spare = &s_slots[i];
  }
  if (!spare) return ESP_ERR_NOT_FOUND;
  i2c_master_bus_config_t cfg = {0};
  cfg.i2c_port = -1;
  cfg.sda_io_num = (gpio_num_t)sda;
  cfg.scl_io_num = (gpio_num_t)scl;
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.glitch_ignore_cnt = 7;
  cfg.flags.enable_internal_pullup = 1;
  esp_err_t err = i2c_new_master_bus(&cfg, &spare->bus);
  if (err != ESP_OK) return err;
  spare->sda = sda; spare->scl = scl;
  *out = spare->bus;
  return ESP_OK;
}
