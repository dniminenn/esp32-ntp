// SPDX-License-Identifier: Unlicense

#include "config_store.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "hal/spi_types.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char* TAG = "CFGSTORE";
static const char* NVS_NS = "ntpcfg";

/* Kconfig entries that only exist under one branch of the network choice. */
#ifndef CONFIG_APP_USE_PRESYNC_GLYPH
#define CONFIG_APP_USE_PRESYNC_GLYPH 0
#endif
#ifndef CONFIG_APP_WIFI_SSID
#define CONFIG_APP_WIFI_SSID ""
#endif
#ifndef CONFIG_APP_WIFI_PASSWORD
#define CONFIG_APP_WIFI_PASSWORD ""
#endif
#ifndef CONFIG_APP_STATIC_IP_ADDR
#define CONFIG_APP_STATIC_IP_ADDR "192.168.1.100"
#endif
#ifndef CONFIG_APP_STATIC_GW
#define CONFIG_APP_STATIC_GW "192.168.1.1"
#endif
#ifndef CONFIG_APP_STATIC_NETMASK
#define CONFIG_APP_STATIC_NETMASK "255.255.255.0"
#endif
#ifdef CONFIG_APP_NETWORK_WIFI
#define DEF_NET_MODE 1
#else
#define DEF_NET_MODE 0
#endif
#ifdef CONFIG_APP_USE_STATIC_IP
#define DEF_NET_DHCP 0
#else
#define DEF_NET_DHCP 1
#endif
#ifdef CONFIG_APP_USE_DISPLAY
#define DEF_DISP_EN 1
#else
#define DEF_DISP_EN 0
#endif

#define PIN_OUT -1, 33
#define PIN_IN  -1, 39

#ifdef CONFIG_SOC_WIFI_SUPPORTED
static const char* const kNetModes[] = { "wiznet", "wifi" };
#define NET_MODE_MAX 1
#else
static const char* const kNetModes[] = { "wiznet" };
#define NET_MODE_MAX 0
#endif

static const char* const kTzOpts[] = {
  "UTC0",                                    "UTC",
  "GMT0BST,M3.5.0/1,M10.5.0",                "UK - London",
  "CET-1CEST,M3.5.0,M10.5.0/3",              "Central Europe - Paris, Berlin, Madrid",
  "EET-2EEST,M3.5.0/3,M10.5.0/4",            "Eastern Europe - Athens, Helsinki",
  "<-01>1<+00>,M3.5.0/0,M10.5.0/1",          "Azores",
  "NST3:30NDT,M3.2.0,M11.1.0",               "Canada - Newfoundland",
  "AST4ADT,M3.2.0,M11.1.0",                  "Canada - Atlantic",
  "EST5EDT,M3.2.0,M11.1.0",                  "US/Canada - Eastern",
  "CST6CDT,M3.2.0,M11.1.0",                  "US/Canada - Central",
  "MST7MDT,M3.2.0,M11.1.0",                  "US/Canada - Mountain",
  "MST7",                                    "US - Arizona (no DST)",
  "PST8PDT,M3.2.0,M11.1.0",                  "US/Canada - Pacific",
  "AKST9AKDT,M3.2.0,M11.1.0",                "US - Alaska",
  "HST10",                                   "US - Hawaii (no DST)",
  "<-03>3",                                  "Brazil - Sao Paulo",
  "<-03>3",                                  "Argentina - Buenos Aires",
  "<-06>6",                                  "Mexico - Mexico City",
  "MSK-3",                                   "Russia - Moscow",
  "<+04>-4",                                 "Gulf - Dubai",
  "IST-5:30",                                "India",
  "<+07>-7",                                 "Thailand, Vietnam",
  "CST-8",                                   "China, Singapore, Hong Kong",
  "JST-9",                                   "Japan",
  "KST-9",                                   "Korea",
  "AWST-8",                                  "Australia - Perth",
  "ACST-9:30ACDT,M10.1.0,M4.1.0/3",          "Australia - Adelaide",
  "AEST-10AEDT,M10.1.0,M4.1.0/3",            "Australia - Sydney, Melbourne",
  "AEST-10",                                 "Australia - Brisbane (no DST)",
  "NZST-12NZDT,M9.5.0,M4.1.0/3",             "New Zealand",
  "SAST-2",                                  "South Africa",
  NULL
};

#define PIN_HELP "ESP32 GPIO. 34-39 are input-only."

const cfg_field_t g_cfg_fields[CFG_COUNT] = {
  [CFG_NET_MODE] = { .key="net.mode", .label="Interface", .group="Network", .type=CF_ENUM,
                     .imin=0, .imax=NET_MODE_MAX, .idef=DEF_NET_MODE, .names=kNetModes, .reboot=true },
  [CFG_NET_DHCP] = { .key="net.dhcp", .label="Use DHCP", .group="Network", .type=CF_BOOL,
                     .imin=0, .imax=1, .idef=DEF_NET_DHCP,
                     .help="Off means the three fields below are used.", .reboot=true },
  [CFG_NET_IP]   = { .key="net.ip", .label="Static IP", .group="Network", .type=CF_STR,
                     .sdef=CONFIG_APP_STATIC_IP_ADDR, .reboot=true },
  [CFG_NET_GW]   = { .key="net.gw", .label="Gateway", .group="Network", .type=CF_STR,
                     .sdef=CONFIG_APP_STATIC_GW, .reboot=true },
  [CFG_NET_MASK] = { .key="net.mask", .label="Netmask", .group="Network", .type=CF_STR,
                     .sdef=CONFIG_APP_STATIC_NETMASK, .reboot=true },
  [CFG_WIFI_SSID]= { .key="wifi.ssid", .label="WiFi SSID", .group="Network", .type=CF_STR,
                     .sdef=CONFIG_APP_WIFI_SSID,
                     .help="Only used when the interface is WiFi.", .reboot=true },
  [CFG_WIFI_PASS]= { .key="wifi.pass", .label="WiFi password", .group="Network", .type=CF_PASS,
                     .sdef=CONFIG_APP_WIFI_PASSWORD, .reboot=true },

  [CFG_SYS_TZ]     = { .key="sys.tz", .label="Timezone", .group="System", .type=CF_STR,
                       .sdef=CONFIG_APP_TZ, .opts=kTzOpts,
                       .help="Affects the LED display only. NTP always serves UTC." },
  [CFG_STATS_PORT] = { .key="stats.port", .label="Management port", .group="System", .type=CF_INT,
                       .imin=1, .imax=65535, .idef=8080,
                       .help="This page and /metrics.", .reboot=true },
  [CFG_UI_PASS]    = { .key="ui.pass", .label="Management password", .group="System", .type=CF_PASS,
                       .sdef="", .help="Blank leaves this page open to anyone on the network." },
  [CFG_UI_LOCK]    = { .key="ui.lock", .label="Lock settings permanently", .group="System", .type=CF_BOOL,
                       .imin=0, .imax=1, .idef=0,
                       .help="One way. Removes the settings page for good; only erasing NVS over "
                             "USB brings it back. Metrics keep working. Needs a password set "
                             "first, which only stops you doing this by accident." },

  [CFG_DISP_EN]    = { .key="disp.en", .label="Enable display", .group="Display", .type=CF_BOOL,
                       .imin=0, .imax=1, .idef=DEF_DISP_EN, .reboot=true },
  [CFG_DISP_GLYPH] = { .key="disp.glyph", .label="Show presync glyph", .group="Display", .type=CF_BOOL,
                       .imin=0, .imax=1, .idef=CONFIG_APP_USE_PRESYNC_GLYPH,
                       .help="Marker shown until the GPS locks." },

  [CFG_NTP_PORT]   = { .key="ntp.port", .label="NTP port", .group="Service", .type=CF_INT,
                       .imin=1, .imax=65535, .idef=CONFIG_APP_NTP_PORT,
                       .help="123 is the standard. Clients will not find it anywhere else.",
                       .reboot=true, .advanced=true },
  [CFG_SERVE_CAL]  = { .key="serve.cal", .label="Serve calibration (us)", .group="Service", .type=CF_INT,
                       .imin=-100000, .imax=100000, .idef=-5,
                       .help="Subtracted from t2 and t3. Re-derive before changing.",
                       .advanced=true },

  [CFG_DISP_HOST]  = { .key="disp.host", .label="SPI host", .group="Display wiring", .type=CF_INT,
                       .imin=1, .imax=2, .idef=CONFIG_APP_SPI_HOST,
                       .help="1 = SPI2, 2 = SPI3. Must differ from the W5500 host.",
                       .reboot=true, .advanced=true },
  [CFG_DISP_CS]    = { .key="disp.cs", .label="CS pin", .group="Display wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=CONFIG_APP_CS_PIN, .help=PIN_HELP,
                       .reboot=true, .advanced=true },
  [CFG_DISP_MOSI]  = { .key="disp.mosi", .label="MOSI pin", .group="Display wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=CONFIG_APP_SPI_MOSI_PIN,
                       .reboot=true, .advanced=true },
  [CFG_DISP_SCLK]  = { .key="disp.sclk", .label="SCLK pin", .group="Display wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=CONFIG_APP_SPI_SCLK_PIN,
                       .reboot=true, .advanced=true },
  [CFG_DISP_HZ]    = { .key="disp.hz", .label="SPI clock (Hz)", .group="Display wiring", .type=CF_INT,
                       .imin=100000, .imax=20000000, .idef=CONFIG_APP_SPI_CLOCK_HZ,
                       .reboot=true, .advanced=true },
  [CFG_DISP_NDEV]  = { .key="disp.ndev", .label="Cascaded modules", .group="Display wiring", .type=CF_INT,
                       .imin=1, .imax=16, .idef=CONFIG_APP_MAX_DEVICES,
                       .reboot=true, .advanced=true },

  [CFG_W5_HOST]    = { .key="w5.host", .label="SPI host", .group="W5500 wiring", .type=CF_INT,
                       .imin=1, .imax=2, .idef=SPI2_HOST,
                       .help="1 = SPI2, 2 = SPI3. Must differ from the display host.",
                       .reboot=true, .advanced=true },
  [CFG_W5_CS]      = { .key="w5.cs", .label="CS pin", .group="W5500 wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=25, .help=PIN_HELP, .reboot=true, .advanced=true },
  [CFG_W5_MOSI]    = { .key="w5.mosi", .label="MOSI pin", .group="W5500 wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=33, .reboot=true, .advanced=true },
  [CFG_W5_MISO]    = { .key="w5.miso", .label="MISO pin", .group="W5500 wiring", .type=CF_INT,
                       .imin=-1, .imax=39, .idef=35, .reboot=true, .advanced=true },
  [CFG_W5_SCLK]    = { .key="w5.sclk", .label="SCLK pin", .group="W5500 wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=32, .reboot=true, .advanced=true },
  [CFG_W5_INT]     = { .key="w5.int", .label="INT pin", .group="W5500 wiring", .type=CF_INT,
                       .imin=-1, .imax=39, .idef=34,
                       .help="Hardware RX timestamping depends on this.", .reboot=true, .advanced=true },
  [CFG_W5_RST]     = { .key="w5.rst", .label="RST pin", .group="W5500 wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=26, .reboot=true, .advanced=true },
  [CFG_W5_HZ]      = { .key="w5.hz", .label="SPI clock (Hz)", .group="W5500 wiring", .type=CF_INT,
                       .imin=1000000, .imax=20000000, .idef=20000000,
                       .help="20 MHz is the proven ceiling on GPIO-matrix pins. Reads corrupt silently above it.",
                       .reboot=true, .advanced=true },

  [CFG_GPS_UART]   = { .key="gps.uart", .label="UART port", .group="GPS wiring", .type=CF_INT,
                       .imin=0, .imax=2, .idef=2, .reboot=true, .advanced=true },
  [CFG_GPS_RX]     = { .key="gps.rx", .label="RX pin (GPS TX)", .group="GPS wiring", .type=CF_INT,
                       .imin=-1, .imax=39, .idef=16, .help=PIN_HELP, .reboot=true, .advanced=true },
  [CFG_GPS_TX]     = { .key="gps.tx", .label="TX pin", .group="GPS wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=17, .reboot=true, .advanced=true },
  [CFG_GPS_BAUD]   = { .key="gps.baud", .label="Baud rate", .group="GPS wiring", .type=CF_INT,
                       .imin=1200, .imax=921600, .idef=9600, .reboot=true, .advanced=true },
  [CFG_PPS_GPIO]   = { .key="pps.gpio", .label="PPS pin", .group="GPS wiring", .type=CF_INT,
                       .imin=-1, .imax=39, .idef=19,
                       .help="Hardware-captured by MCPWM. The whole clock rides on this.",
                       .reboot=true, .advanced=true },
  [CFG_PPS_CAL]    = { .key="pps.cal", .label="PPS calibration (us)", .group="GPS wiring", .type=CF_INT,
                       .imin=-1000000, .imax=1000000, .idef=0, .advanced=true },

  [CFG_RTC_SDA]    = { .key="rtc.sda", .label="SDA pin", .group="DS3231 wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=-1,
                       .help="-1 = no DS3231 fitted. With one: battery-backed time at boot.",
                       .reboot=true, .advanced=true },
  [CFG_RTC_SCL]    = { .key="rtc.scl", .label="SCL pin", .group="DS3231 wiring", .type=CF_INT,
                       .imin=-1, .imax=33, .idef=-1, .help=PIN_HELP, .reboot=true, .advanced=true },
  [CFG_RTC_32K]    = { .key="rtc.32k", .label="32kHz pin", .group="DS3231 wiring", .type=CF_INT,
                       .imin=-1, .imax=39, .idef=-1,
                       .help="TCXO output, captured as the holdover frequency reference. -1 = not wired.",
                       .reboot=true, .advanced=true },
};

typedef struct {
  int32_t i;
  char s[CFG_STR_MAX];
} cfg_val_t;

static cfg_val_t s_val[CFG_COUNT];
static bool s_fdirty[CFG_COUNT];
static bool s_stored[CFG_COUNT];   /* key exists in NVS rather than tracking the build default */
static bool s_safe = false;
static bool s_dirty = false;
static bool s_provisioned = false;
static bool s_ready = false;

static bool is_str(cfg_type_t t) { return t == CF_STR || t == CF_PASS; }

static void load_defaults(void) {
  for (int i = 0; i < CFG_COUNT; ++i) {
    const cfg_field_t* f = &g_cfg_fields[i];
    if (is_str(f->type)) {
      s_val[i].i = 0;
      snprintf(s_val[i].s, CFG_STR_MAX, "%s", f->sdef ? f->sdef : "");
    } else {
      s_val[i].i = f->idef;
      s_val[i].s[0] = '\0';
    }
  }
}

void cfg_init(bool safe_mode) {
  load_defaults();
  s_safe = safe_mode;
  s_dirty = false;
  s_ready = true;

  if (safe_mode) {
    ESP_LOGW(TAG, "safe mode: using build-time defaults, NVS not read");
    return;
  }

  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "no stored settings (%s), using build-time defaults",
             esp_err_to_name(err));
    return;
  }

  int loaded = 0;
  for (int i = 0; i < CFG_COUNT; ++i) {
    const cfg_field_t* f = &g_cfg_fields[i];
    if (is_str(f->type)) {
      size_t len = CFG_STR_MAX;
      char tmp[CFG_STR_MAX];
      if (nvs_get_str(h, f->key, tmp, &len) == ESP_OK) {
        memcpy(s_val[i].s, tmp, len > CFG_STR_MAX ? CFG_STR_MAX : len);
        s_val[i].s[CFG_STR_MAX - 1] = '\0';
        s_stored[i] = true;
        loaded++;
      }
    } else {
      int32_t v;
      if (nvs_get_i32(h, f->key, &v) == ESP_OK) {
        if (v >= f->imin && v <= f->imax) s_val[i].i = v;
        s_stored[i] = true;
        loaded++;
      }
    }
  }
  nvs_close(h);
  s_provisioned = loaded > 0;
  ESP_LOGI(TAG, "loaded %d stored setting(s)", loaded);
}

bool cfg_safe_mode(void)   { return s_safe; }
bool cfg_provisioned(void) { return s_provisioned; }
bool cfg_dirty(void)       { return s_dirty; }

int32_t cfg_int(cfg_id_t id) {
  if (id >= CFG_COUNT) return 0;
  if (!s_ready) return g_cfg_fields[id].idef;
  return s_val[id].i;
}

const char* cfg_str(cfg_id_t id) {
  if (id >= CFG_COUNT) return "";
  if (!s_ready) {
    const char* d = g_cfg_fields[id].sdef;
    return d ? d : "";
  }
  return s_val[id].s;
}

cfg_id_t cfg_lookup(const char* key) {
  for (int i = 0; i < CFG_COUNT; ++i)
    if (strcmp(g_cfg_fields[i].key, key) == 0) return (cfg_id_t)i;
  return CFG_COUNT;
}

static bool valid_ipv4(const char* s) {
  int parts = 0;
  while (*s) {
    if (*s < '0' || *s > '9') return false;
    int v = 0, digits = 0;
    while (*s >= '0' && *s <= '9') {
      v = v * 10 + (*s - '0');
      if (++digits > 3) return false;
      s++;
    }
    if (v > 255) return false;
    parts++;
    if (*s == '.') { s++; if (!*s) return false; }
    else if (*s) return false;
  }
  return parts == 4;
}

bool cfg_stage(cfg_id_t id, const char* value) {
  if (id >= CFG_COUNT || !value) return false;
  const cfg_field_t* f = &g_cfg_fields[id];

  if (id == CFG_UI_LOCK && value[0] == '1' && s_val[CFG_UI_PASS].s[0] == '\0')
    return false;

  if (is_str(f->type)) {
    if (strlen(value) >= CFG_STR_MAX) return false;
    if (id == CFG_NET_IP || id == CFG_NET_GW || id == CFG_NET_MASK) {
      if (!valid_ipv4(value)) return false;
    }
    if (f->type == CF_PASS && value[0] == '\0') return true;
    if (strcmp(s_val[id].s, value) != 0) {
      snprintf(s_val[id].s, CFG_STR_MAX, "%s", value);
      s_fdirty[id] = true;
      s_dirty = true;
    }
    return true;
  }

  char* end = NULL;
  long v = strtol(value, &end, 10);
  if (end == value || (end && *end != '\0')) return false;
  if (v < f->imin || v > f->imax) return false;
  if (s_val[id].i != (int32_t)v) {
    s_val[id].i = (int32_t)v;
    s_fdirty[id] = true;
    s_dirty = true;
  }
  return true;
}

bool cfg_clear(cfg_id_t id) {
  if (id >= CFG_COUNT || !is_str(g_cfg_fields[id].type)) return false;
  if (s_val[id].s[0] == '\0') return true;
  s_val[id].s[0] = '\0';
  s_fdirty[id] = true;
  s_dirty = true;
  return true;
}

esp_err_t cfg_commit(void) {
  bool anyUnpinned = false;
  for (int i = 0; i < CFG_COUNT; ++i)
    if (!s_stored[i]) { anyUnpinned = true; break; }

  if (!s_dirty && !anyUnpinned) {
    ESP_LOGI(TAG, "no changes staged, flash untouched");
    return ESP_OK;
  }

  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) return err;

  int written = 0;
  for (int i = 0; i < CFG_COUNT; ++i) {
    if (!s_fdirty[i] && s_stored[i]) continue;
    const cfg_field_t* f = &g_cfg_fields[i];
    if (is_str(f->type)) err = nvs_set_str(h, f->key, s_val[i].s);
    else                 err = nvs_set_i32(h, f->key, s_val[i].i);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "set %s failed: %s", f->key, esp_err_to_name(err));
      nvs_close(h);
      return err;
    }
    s_stored[i] = true;
    written++;
  }
  err = nvs_commit(h);
  nvs_close(h);
  if (err == ESP_OK) {
    memset(s_fdirty, 0, sizeof(s_fdirty));
    s_dirty = false;
    s_provisioned = true;
    ESP_LOGI(TAG, "%d setting(s) committed to flash", written);
  }
  return err;
}

esp_err_t cfg_factory_reset(void) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
  if (err != ESP_OK) return err;
  err = nvs_erase_all(h);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  if (err == ESP_OK) ESP_LOGW(TAG, "settings erased, reverting to build defaults");
  return err;
}

bool cfg_locked(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
  int32_t v = 0;
  esp_err_t err = nvs_get_i32(h, g_cfg_fields[CFG_UI_LOCK].key, &v);
  nvs_close(h);
  return err == ESP_OK && v != 0;
}

#define BOOT_KEY "boot.fail"

uint8_t cfg_boot_begin(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return 1;
  uint8_t n = 0;
  nvs_get_u8(h, BOOT_KEY, &n);
  if (n < 255) n++;
  nvs_set_u8(h, BOOT_KEY, n);
  nvs_commit(h);
  nvs_close(h);
  return n;
}

void cfg_boot_healthy(void) {
  nvs_handle_t h;
  if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
  uint8_t n = 0;
  nvs_get_u8(h, BOOT_KEY, &n);
  if (n != 0) {
    nvs_set_u8(h, BOOT_KEY, 0);
    nvs_commit(h);
  }
  nvs_close(h);
}
