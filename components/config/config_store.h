#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_STR_MAX 64

typedef enum {
  CFG_NET_MODE = 0,
  CFG_NET_DHCP,
  CFG_NET_IP,
  CFG_NET_GW,
  CFG_NET_MASK,
  CFG_WIFI_SSID,
  CFG_WIFI_PASS,
  CFG_SYS_TZ,
  CFG_NTP_PORT,
  CFG_STATS_PORT,
  CFG_UI_PASS,
  CFG_UI_LOCK,
  CFG_DISP_EN,
  CFG_DISP_GLYPH,
  CFG_DISP_HOST,
  CFG_DISP_CS,
  CFG_DISP_MOSI,
  CFG_DISP_SCLK,
  CFG_DISP_HZ,
  CFG_DISP_NDEV,
  CFG_W5_HOST,
  CFG_W5_CS,
  CFG_W5_MOSI,
  CFG_W5_MISO,
  CFG_W5_SCLK,
  CFG_W5_INT,
  CFG_W5_RST,
  CFG_W5_HZ,
  CFG_GPS_UART,
  CFG_GPS_RX,
  CFG_GPS_TX,
  CFG_GPS_BAUD,
  CFG_PPS_GPIO,
  CFG_PPS_CAL,
  CFG_SERVE_CAL,
  CFG_RTC_SDA,
  CFG_RTC_SCL,
  CFG_RTC_32K,
  CFG_COUNT
} cfg_id_t;

typedef enum { CF_BOOL, CF_INT, CF_STR, CF_PASS, CF_ENUM } cfg_type_t;

typedef struct {
  const char* key;               /* NVS key, <= 15 chars                     */
  const char* label;             /* shown in the management UI               */
  const char* group;             /* UI section heading                       */
  cfg_type_t type;
  int32_t imin, imax;            /* inclusive range for CF_INT/CF_ENUM       */
  int32_t idef;                  /* build-time default for numeric types     */
  const char* sdef;              /* build-time default for string types      */
  const char* const* names;      /* CF_ENUM labels, imax+1 entries           */
  const char* const* opts;       /* CF_STR dropdown: value,label pairs, NULL-terminated */
  const char* help;              /* one line under the field in the UI       */
  bool reboot;                   /* change only takes effect after restart   */
  bool advanced;                 /* wiring and timing: hidden behind a disclosure */
} cfg_field_t;

extern const cfg_field_t g_cfg_fields[CFG_COUNT];

/* safe_mode: serve every field from its build-time default and never read NVS.
 * Writes are still allowed, so the UI can repair a bad config in place. */
void cfg_init(bool safe_mode);
bool cfg_safe_mode(void);
bool cfg_provisioned(void);

int32_t     cfg_int(cfg_id_t id);
const char* cfg_str(cfg_id_t id);

/* Look a field up by NVS key; CFG_COUNT when unknown. */
cfg_id_t cfg_lookup(const char* key);

bool cfg_stage(cfg_id_t id, const char* value);
bool cfg_clear(cfg_id_t id);
bool cfg_dirty(void);

esp_err_t cfg_commit(void);          /* persist every staged value to NVS */
esp_err_t cfg_factory_reset(void);   /* erase the namespace entirely      */

/* Read straight from NVS, deliberately ignoring safe mode: the lock is a fuse,
 * so booting on build-time defaults must not hand the settings page back. */
bool cfg_locked(void);

#define CFG_SAFE_MODE_FAILS 3

uint8_t cfg_boot_begin(void);
void    cfg_boot_healthy(void);

#ifdef __cplusplus
}
#endif
