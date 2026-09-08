#pragma once
// SPDX-License-Identifier: Unlicense
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One master bus per SDA/SCL pair, shared by every device wired to it. Boot-time only, never deleted. */
esp_err_t i2c_bus_get(int sda, int scl, i2c_master_bus_handle_t* out);

#ifdef __cplusplus
}
#endif
