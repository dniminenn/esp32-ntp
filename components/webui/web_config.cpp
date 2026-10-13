// SPDX-License-Identifier: Unlicense

#include "web_internal.h"
#include "web_log.h"
#include "config.h"
#include "config_store.h"
#include "gps.h"
#include "ntp_server.h"
#include "ntp_prof.h"
#include "w5500_eth.h"
#include "wifi_sta.h"
#include "w5k_tcp_wrapper.h"
#include "w5500_drv.h"
#include "w5500_dhcp.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <stdlib.h>
#include <fcntl.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"


static const char* TAG = "WEBUI";
extern volatile uint32_t g_mainLoopBeats;
extern uint32_t g_bootCount;

static void url_decode(char* s) {
  char* out = s;
  for (; *s; ++s) {
    if (*s == '+') {
      *out++ = ' ';
    } else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      char hex[3] = { s[1], s[2], '\0' };
      *out++ = (char)strtol(hex, nullptr, 16);
      s += 2;
    } else {
      *out++ = *s;
    }
  }
  *out = '\0';
}

static void html_escape(const char* in, char* out, size_t cap) {
  size_t o = 0;
  for (const char* p = in; *p && o + 7 < cap; ++p) {
    switch (*p) {
      case '&':  memcpy(out + o, "&amp;", 5);  o += 5; break;
      case '<':  memcpy(out + o, "&lt;", 4);   o += 4; break;
      case '>':  memcpy(out + o, "&gt;", 4);   o += 4; break;
      case '"':  memcpy(out + o, "&quot;", 6); o += 6; break;
      default:   out[o++] = *p;
    }
  }
  out[o] = '\0';
}

static const char* const kGroupOrder[] = {
  "Network", "System", "Display", "Service",
  "Display wiring", "W5500 wiring", "GPS wiring", "DS3231 wiring", nullptr
};

void WebServer::renderField(char** pp, char* end, int i) {
  char* p = *pp;
  const cfg_field_t* f = &g_cfg_fields[i];

  p += snprintf(p, end - p, "<tr><td class=l><label for=\"%s\">%s</label>%s",
                f->key, f->label, f->reboot ? "<span class=r>*</span>" : "");
  if (f->help) p += snprintf(p, end - p, "<div class=h>%s</div>", f->help);
  p += snprintf(p, end - p, "</td><td>");

  switch (f->type) {
    case CF_BOOL:
      p += snprintf(p, end - p,
        "<input type=hidden name=\"%s\" value=\"0\">"
        "<input type=checkbox id=\"%s\" name=\"%s\" value=\"1\"%s>",
        f->key, f->key, f->key, cfg_int((cfg_id_t)i) ? " checked" : "");
      break;
    case CF_ENUM:
      p += snprintf(p, end - p, "<select id=\"%s\" name=\"%s\">", f->key, f->key);
      for (int v = f->imin; v <= f->imax; ++v)
        p += snprintf(p, end - p, "<option value=\"%d\"%s>%s</option>",
                      v, cfg_int((cfg_id_t)i) == v ? " selected" : "", f->names[v]);
      p += snprintf(p, end - p, "</select>");
      break;
    case CF_PASS:
      p += snprintf(p, end - p,
        "<input type=password id=\"%s\" name=\"%s\" value=\"\" placeholder=\"%s\">",
        f->key, f->key, cfg_str((cfg_id_t)i)[0] ? "unchanged" : "not set");
      if (cfg_str((cfg_id_t)i)[0])
        p += snprintf(p, end - p,
          "<label class=h><input type=checkbox name=\"%s.clr\" value=\"1\"> remove</label>",
          f->key);
      break;
    case CF_STR: {
      const char* cur = cfg_str((cfg_id_t)i);
      if (f->opts) {
        bool matched = false;
        for (int k = 0; f->opts[k]; k += 2)
          if (strcmp(f->opts[k], cur) == 0) { matched = true; break; }
        p += snprintf(p, end - p, "<select id=\"%s\" name=\"%s\">", f->key, f->key);
        if (!matched) {
          char esc[CFG_STR_MAX * 6];
          html_escape(cur, esc, sizeof(esc));
          p += snprintf(p, end - p, "<option value=\"%s\" selected>%s (custom)</option>", esc, esc);
        }
        for (int k = 0; f->opts[k]; k += 2)
          p += snprintf(p, end - p, "<option value=\"%s\"%s>%s</option>",
                        f->opts[k], strcmp(f->opts[k], cur) == 0 ? " selected" : "",
                        f->opts[k + 1]);
        p += snprintf(p, end - p, "</select>");
      } else {
        char esc[CFG_STR_MAX * 6];
        html_escape(cur, esc, sizeof(esc));
        p += snprintf(p, end - p,
          "<input id=\"%s\" name=\"%s\" value=\"%s\" maxlength=%d>",
          f->key, f->key, esc, CFG_STR_MAX - 1);
      }
      break;
    }
    case CF_INT:
      p += snprintf(p, end - p,
        "<input type=number id=\"%s\" name=\"%s\" value=\"%d\" min=%d max=%d>",
        f->key, f->key, (int)cfg_int((cfg_id_t)i), (int)f->imin, (int)f->imax);
      break;
  }
  p += snprintf(p, end - p, "</td></tr>");
  *pp = p;
}

void WebServer::renderSection(char** pp, char* end, bool advanced) {
  for (int g = 0; kGroupOrder[g]; ++g) {
    bool opened = false;
    for (int i = 0; i < CFG_COUNT; ++i) {
      const cfg_field_t* f = &g_cfg_fields[i];
      if (f->advanced != advanced) continue;
      if (strcmp(f->group, kGroupOrder[g]) != 0) continue;
      if (!opened) {
        *pp += snprintf(*pp, end - *pp, "<h2>%s</h2><table>", kGroupOrder[g]);
        opened = true;
      }
      renderField(pp, end, i);
    }
    if (opened) *pp += snprintf(*pp, end - *pp, "</table>");
  }
}

void WebServer::sendConfigPage(const char* notice) {
  char* p = g_resp;
  char* end = g_resp + sizeof(g_resp);

  p += snprintf(p, end - p,
    "<!doctype html><meta charset=utf-8>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>esp32-ntp settings</title>"
    "<style>"
    ":root{color-scheme:light dark;--b:#8883;--m:#8888}"
    "*{box-sizing:border-box}"
    "body{font:15px/1.5 ui-sans-serif,system-ui,sans-serif;max-width:44rem;margin:0 auto;padding:1.5rem 1rem 4rem}"
    "h1{font-size:1.35rem;margin:0}"
    "h2{font-size:.8rem;text-transform:uppercase;letter-spacing:.06em;color:var(--m);"
    "margin:1.75rem 0 .35rem;font-weight:600}"
    "table{width:100%%;border-collapse:collapse}"
    "tr{border-top:1px solid var(--b)}"
    "tr:first-child{border-top:0}"
    "td{padding:.55rem 0;vertical-align:top}"
    "td.l{width:54%%;padding-right:1rem}"
    "label{font-weight:500}"
    ".h{color:var(--m);font-size:.82rem;line-height:1.35;margin-top:.15rem}"
    ".r{color:#d97706;margin-left:.25rem;font-weight:700}"
    "input,select{width:100%%;padding:.4rem .5rem;font:inherit;"
    "border:1px solid var(--b);border-radius:.375rem;background:transparent;color:inherit}"
    "input:focus,select:focus{outline:2px solid #3b82f6;outline-offset:1px}"
    "input[type=checkbox]{width:1.1rem;height:1.1rem;margin-top:.3rem}"
    ".bar{display:flex;align-items:baseline;justify-content:space-between;gap:1rem;flex-wrap:wrap}"
    ".st{color:var(--m);font-size:.85rem;font-variant-numeric:tabular-nums}"
    ".note{padding:.65rem .85rem;border-radius:.375rem;margin:1rem 0;font-size:.9rem}"
    ".warn{background:#fef3c7;color:#713f12;border:1px solid #fcd34d}"
    ".ok{background:#dcfce7;color:#14532d;border:1px solid #86efac}"
    "details{margin-top:2rem;border-top:1px solid var(--b);padding-top:1rem}"
    "summary{cursor:pointer;font-weight:600;font-size:.9rem}"
    "button{font:inherit;font-weight:500;padding:.5rem 1.1rem;border-radius:.375rem;"
    "border:1px solid var(--b);background:#3b82f6;color:#fff;cursor:pointer}"
    "button.sec{background:transparent;color:inherit}"
    ".acts{position:sticky;bottom:0;background:Canvas;padding:1rem 0;margin-top:1.5rem;"
    "border-top:1px solid var(--b);display:flex;gap:.6rem;align-items:center;flex-wrap:wrap}"
    "a{color:#3b82f6}"
    "</style>");

  uint32_t ipv = 0;
  if (eth) eth->getIpAddr(ipv);
#if CONFIG_SOC_WIFI_SUPPORTED
  else if (wifi) {
    wifi->getIpAddr(ipv);
    ipv = ntohl(ipv);
  }
#endif
  uint8_t mac[6] = {0};
  if (eth) eth->getMacAddr(mac);

  p += snprintf(p, end - p,
    "<div class=bar><h1>esp32-ntp</h1>"
    "<div class=st>%u.%u.%u.%u &middot; %02x:%02x:%02x:%02x:%02x:%02x &middot; up %us</div></div>",
    (unsigned)((ipv >> 24) & 0xff), (unsigned)((ipv >> 16) & 0xff),
    (unsigned)((ipv >> 8) & 0xff), (unsigned)(ipv & 0xff),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    (unsigned)(esp_timer_get_time() / 1000000));

  if (notice)
    p += snprintf(p, end - p, "<div class=\"note ok\">%s</div>", notice);

  if (Config::isSafeMode())
    p += snprintf(p, end - p,
      "<div class=\"note warn\"><b>Safe mode.</b> Stored settings were ignored this boot "
      "because %d starts in a row did not reach the network. These are the build-time "
      "defaults; your stored values are still in flash, and saving from here overwrites "
      "them. A restart that reaches the network clears this on its own.</div>",
      CFG_SAFE_MODE_FAILS);

  if (cfg_str(CFG_UI_PASS)[0] == '\0')
    p += snprintf(p, end - p,
      "<div class=\"note warn\">No management password is set, so anyone who can reach this "
      "address can change these settings or reboot the clock.</div>");

  p += snprintf(p, end - p, "<form method=post action=/config>");
  renderSection(&p, end, false);

  p += snprintf(p, end - p,
    "<details><summary>Advanced: wiring, pins and timing</summary>"
    "<div class=\"note warn\">These describe how the board is physically wired and how the "
    "clock is calibrated. If you built it to the published wiring, leave them alone. A wrong "
    "pin here takes the clock off the network on the next restart; interrupt startup twice, "
    "with RESET or the power lead, and the third boot comes up on the build-time defaults "
    "so you can undo it.</div>");
  renderSection(&p, end, true);
  p += snprintf(p, end - p, "</details>");

  p += snprintf(p, end - p,
    "<div class=acts><button type=submit>Save to flash</button>"
    "<span class=st><span class=r>*</span> needs a restart</span></div></form>"
    "<form method=post action=/factory-reset "
    "onsubmit=\"return confirm('Erase all stored settings and reboot?')\">"
    "<button class=sec type=submit>Erase stored settings</button></form>"
    "<p class=st><a href=/metrics>/metrics</a>");
  if (weblog_active()) p += snprintf(p, end - p, " &middot; <a href=/log>/log</a>");
  p += snprintf(p, end - p, "</p>");

  int blen = (int)(p - g_resp);
  if (blen >= (int)sizeof(g_resp) - 1) blen = (int)sizeof(g_resp) - 1;

  char hdr[160];
  int hlen = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Content-Length: %d\r\n"
    "\r\n", blen);
  sendAll(hdr, hlen);
  sendAll(g_resp, blen);
  closeConn();
}

/*
 * Stage every field the form sent, then commit once. A value that fails
 * validation is reported and nothing is written, so a typo in one pin cannot
 * half-apply a settings change.
 */
void WebServer::handleConfigPost(char* body) {
  char* buf = body;

  char bad[64] = {0};
  cfg_id_t badId = CFG_COUNT;
  int staged = 0;

  char* saveptr = nullptr;
  for (char* tok = strtok_r(buf, "&", &saveptr); tok; tok = strtok_r(nullptr, "&", &saveptr)) {
    char* eq = strchr(tok, '=');
    if (!eq) continue;
    *eq = '\0';
    char* key = tok;
    char* val = eq + 1;
    url_decode(key);
    url_decode(val);

    size_t klen = strlen(key);
    if (klen > 4 && strcmp(key + klen - 4, ".clr") == 0) {
      key[klen - 4] = '\0';
      cfg_id_t cid = cfg_lookup(key);
      if (cid != CFG_COUNT && strcmp(val, "1") == 0) {
        cfg_clear(cid);
        staged++;
      }
      continue;
    }

    cfg_id_t id = cfg_lookup(key);
    if (id == CFG_COUNT) continue;
    if (!cfg_stage(id, val)) {
      snprintf(bad, sizeof(bad), "%s", g_cfg_fields[id].label);
      badId = id;
      break;
    }
    staged++;
  }

  if (bad[0]) {
    char msg[220];
    if (badId == CFG_UI_LOCK)
      snprintf(msg, sizeof(msg),
               "Rejected: set a management password before locking settings, "
               "otherwise anyone on this network could lock you out. Nothing was written.");
    else
      snprintf(msg, sizeof(msg),
               "Rejected: %s is out of range. Nothing was written.", bad);
    sendStatus("400 Bad Request", "text/plain", msg);
    return;
  }

  bool changed = cfg_dirty();
  esp_err_t err = cfg_commit();
  if (err != ESP_OK) {
    sendStatus("500 Internal Server Error", "text/plain", "Could not write to flash");
    return;
  }

  if (!changed) {
    sendConfigPage("No changes; flash was not written.");
    return;
  }

  sendStatus("200 OK", "text/html",
    "<!doctype html><meta charset=utf-8>"
    "<meta http-equiv=refresh content=\"8;url=/\">"
    "<p>Saved to flash. Restarting.</p>"
    "<p>If the address or interface changed, the device will come back somewhere else.</p>");
  ESP_LOGW(TAG, "settings saved over the management UI, restarting");
  vTaskDelay(pdMS_TO_TICKS(300));
  esp_restart();
}
