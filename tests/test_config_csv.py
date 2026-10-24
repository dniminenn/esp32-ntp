#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense
"""Host regression test: python3 tests/test_config_csv.py.

Run with ESP-IDF's Python to also generate and inspect a real NVS image.
The actual field table, configuration accessors and CSV exporter are compiled;
only ESP-IDF hardware/flash dependencies are stubbed.
"""

import csv
import importlib.util
import io
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
import sys

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "components/config"

STUBS = {
    "esp_err.h": """
#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_NO_MEM 2
#define ESP_ERR_NVS_NOT_FOUND 3
static inline const char* esp_err_to_name(int e) { (void)e; return "stub"; }
""",
    "sdkconfig.h": """
#define CONFIG_IDF_TARGET_ESP32S3 1
#define CONFIG_SOC_WIFI_SUPPORTED 1
#define CONFIG_APP_TZ "UTC0"
#define CONFIG_APP_NTP_PORT 123
#define CONFIG_APP_SPI_HOST 2
#define CONFIG_APP_CS_PIN 5
#define CONFIG_APP_SPI_MOSI_PIN 23
#define CONFIG_APP_SPI_SCLK_PIN 18
#define CONFIG_APP_SPI_CLOCK_HZ 1000000
#define CONFIG_APP_MAX_DEVICES 4
""",
    "hal/spi_types.h": "#define SPI2_HOST 1\n",
    "esp_log.h": """
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
""",
    "esp_attr.h": "",
    "nvs_flash.h": "",
    "nvs.h": """
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
typedef int nvs_handle_t;
#define NVS_READONLY 0
#define NVS_READWRITE 1
static inline int nvs_open(const char* ns, int mode, nvs_handle_t* h) {
  (void)ns; (void)mode; *h = 0; return ESP_ERR_NVS_NOT_FOUND;
}
static inline void nvs_close(nvs_handle_t h) { (void)h; }
static inline int nvs_get_str(int h, const char* key, char* s, size_t* n) {
  (void)h; (void)key; (void)s; (void)n; return ESP_ERR_NVS_NOT_FOUND;
}
static inline int nvs_set_str(int h, const char* key, const char* s) {
  (void)h; (void)key; (void)s; return ESP_OK;
}
#define STUB_NVS_NUM(suffix, type) \
static inline int nvs_get_##suffix(int h, const char* key, type* v) { \
  (void)h; (void)key; (void)v; return ESP_ERR_NVS_NOT_FOUND; \
} \
static inline int nvs_set_##suffix(int h, const char* key, type v) { \
  (void)h; (void)key; (void)v; return ESP_OK; \
}
STUB_NVS_NUM(i32, int32_t)
STUB_NVS_NUM(u8, uint8_t)
static inline int nvs_commit(int h) { (void)h; return ESP_OK; }
static inline int nvs_erase_all(int h) { (void)h; return ESP_OK; }
""",
}

HARNESS = r'''
#include "config_store.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void) {
  char out[8192];
  size_t length = 99;
  cfg_init(true);
  assert(cfg_export_csv(out, sizeof(out), &length) == ESP_OK);
  assert(strstr(out, "sys.tz,data,string,\"UTC0\"\r\n"));
  assert(strstr(out, "wifi.pass,data,string,\"\"\r\n"));
  cfg_init(false);  // no NVS keys: export build defaults plus staged values
  assert(cfg_stage(CFG_WIFI_SSID, "CR\r\nLF"));
  assert(cfg_export_csv(out, sizeof(out), &length) == ESP_OK);
  assert(strstr(out, "wifi.ssid,data,string,\"CR\r\nLF\"\r\n"));
  assert(cfg_stage(CFG_SYS_TZ, "CET-1CEST,M3.5.0,M10.5.0/3"));
  assert(cfg_stage(CFG_WIFI_SSID, " #clock, \"café\"\nline two "));
  assert(cfg_stage(CFG_WIFI_PASS, "quote\"comma,secret"));
  assert(cfg_stage(CFG_UI_PASS, " management secret "));
  assert(cfg_stage(CFG_NET_MODE, "1"));
  assert(cfg_stage(CFG_NET_DHCP, "0"));
  assert(cfg_stage(CFG_SERVE_CAL, "-100000"));
  assert(cfg_stage(CFG_GPS_BAUD, "921600"));
  assert(cfg_export_csv(out, sizeof(out), &length) == ESP_OK);
  assert(length == strlen(out));

  // Every insufficient capacity must fail without writing past its boundary.
  char limited[8192];
  for (size_t cap = 1; cap <= length; ++cap) {
    memset(limited, 'X', sizeof(limited));
    size_t n = 99;
    assert(cfg_export_csv(limited, cap, &n) == ESP_ERR_NO_MEM);
    assert(n == 0 && limited[0] == '\0' && limited[cap] == 'X');
  }
  size_t n = 0;
  memset(limited, 'X', sizeof(limited));
  assert(cfg_export_csv(limited, length + 1, &n) == ESP_OK);
  assert(n == length && strcmp(out, limited) == 0 && limited[length + 1] == 'X');
  assert(cfg_export_csv(NULL, sizeof(out), &n) == ESP_ERR_INVALID_ARG && n == 0);
  assert(cfg_export_csv(limited, 0, &n) == ESP_ERR_INVALID_ARG && n == 0);
  assert(cfg_export_csv(limited, sizeof(limited), NULL) == ESP_ERR_INVALID_ARG);
  fwrite(out, 1, length, stdout);
  return 0;
}
'''


class ConfigCsvTest(unittest.TestCase):
    def test_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            for name, contents in STUBS.items():
                path = tmp / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(contents)
            harness = tmp / "harness.c"
            harness.write_text(HARNESS)
            binary = tmp / "test_csv"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Wno-unused-variable",
                "-Wno-unused-but-set-variable",
                "-I", str(tmp), "-I", str(CONFIG), str(harness),
                str(CONFIG / "config_store.c"), str(CONFIG / "config_csv.c"),
                "-o", str(binary),
            ], check=True)
            exported = subprocess.check_output([str(binary)])
            rows = list(csv.reader(io.StringIO(exported.decode(), newline="")))
            self.assertEqual(rows[0], ["key", "type", "encoding", "value"])
            self.assertEqual(rows[1], ["ntpcfg", "namespace", "", ""])
            fields = dict(re.findall(
                r'\.key="([^"]+)"[^\n]*\.type=(CF_\w+)',
                (CONFIG / "config_store.c").read_text(),
            ))
            self.assertEqual(len(rows), len(fields) + 2)
            values = {}
            for key, kind, encoding, value in rows[2:]:
                self.assertNotIn(key, values)
                self.assertEqual(kind, "data")
                self.assertEqual(encoding, "string" if fields[key] in
                                 {"CF_STR", "CF_PASS"} else "i32")
                values[key] = value
            self.assertEqual(set(values), set(fields))
            self.assertEqual(values["sys.tz"], "CET-1CEST,M3.5.0,M10.5.0/3")
            self.assertEqual(values["wifi.ssid"], ' #clock, "café"\nline two ')
            self.assertEqual(values["wifi.pass"], 'quote"comma,secret')
            self.assertEqual(values["ui.pass"], " management secret ")
            self.assertEqual(values["net.mode"], "1")
            self.assertEqual(values["net.dhcp"], "0")
            self.assertEqual(values["serve.cal"], "-100000")
            self.assertEqual(values["gps.baud"], "921600")
            self.assertEqual(values["rtc.sda"], "-1")
            self.assertNotIn("boot.fail", values)

            if importlib.util.find_spec("esp_idf_nvs_partition_gen"):
                source = tmp / "settings.csv"
                source.write_bytes(exported)
                image = tmp / "settings.bin"
                subprocess.run([sys.executable, "-m", "esp_idf_nvs_partition_gen",
                                "generate", str(source), str(image), "0x6000"], check=True)
                data = image.read_bytes()
                self.assertEqual(len(data), 0x6000)
                # Ensure CSV quoting was removed and original strings reached NVS.
                for key in ["sys.tz", "wifi.ssid", "wifi.pass", "ui.pass"]:
                    self.assertTrue(values[key].encode() + b"\0" in data,
                                    f"{key} was changed by the NVS generator")
            else:
                print("NVS generator unavailable; binary compatibility check skipped.")


if __name__ == "__main__":
    unittest.main()
