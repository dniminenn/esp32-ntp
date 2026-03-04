// SPDX-License-Identifier: MIT-0

#include "wifi_sta.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include <string.h>

static const char* TAG = "WIFI_STA";

static void on_wifi_event(void* arg, esp_event_base_t base, int32_t id, void* data) {
  WifiSta* self = (WifiSta*)arg;
  if (base != WIFI_EVENT || self == nullptr) return;
  if (id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
  }
}

static void on_ip_event(void* arg, esp_event_base_t base, int32_t id, void* data) {
  WifiSta* self = (WifiSta*)arg;
  if (base != IP_EVENT || id != IP_EVENT_STA_GOT_IP || self == nullptr) return;
  ip_event_got_ip_t* event = (ip_event_got_ip_t*)data;
  self->onGotIp(event->ip_info.ip.addr);
}

void WifiSta::onGotIp(uint32_t ip) {
  cached_ip = ip;
  ip_valid = true;
  ESP_LOGI(TAG, "Got IP: %lu.%lu.%lu.%lu",
           (cached_ip >> 24) & 0xff, (cached_ip >> 16) & 0xff,
           (cached_ip >> 8) & 0xff, (cached_ip >> 0) & 0xff);
}

WifiSta::WifiSta() : started(false), cached_ip(0), ip_valid(false) {}

WifiSta::~WifiSta() {
  if (started) {
    esp_wifi_stop();
    started = false;
  }
}

esp_err_t WifiSta::begin(const char* ssid, const char* password,
                         bool use_static_ip,
                         const char* static_ip,
                         const char* static_gw,
                         const char* static_netmask) {
  esp_netif_t* netif = esp_netif_create_default_wifi_sta();
  if (use_static_ip && netif && static_ip && static_gw && static_netmask) {
    esp_netif_dhcpc_stop(netif);
    ip4_addr_t ip, gw, nm;
    if (ip4addr_aton(static_ip, &ip) && ip4addr_aton(static_gw, &gw) && ip4addr_aton(static_netmask, &nm)) {
      esp_netif_ip_info_t info = {};
      info.ip.addr = ip.addr;
      info.gw.addr = gw.addr;
      info.netmask.addr = nm.addr;
      esp_err_t err = esp_netif_set_ip_info(netif, &info);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_set_ip_info failed: %s", esp_err_to_name(err));
      } else {
        cached_ip = ip.addr;
        ip_valid = true;
        ESP_LOGI(TAG, "Static IP: %s GW: %s NM: %s", static_ip, static_gw, static_netmask);
      }
    } else {
      ESP_LOGE(TAG, "Invalid static IP/gw/netmask");
    }
  }
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, on_wifi_event, this, nullptr);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, this, nullptr);
  wifi_config_t wcfg = {};
  size_t ssid_len = strlen(ssid);
  if (ssid_len >= sizeof(wcfg.sta.ssid)) ssid_len = sizeof(wcfg.sta.ssid) - 1;
  memcpy(wcfg.sta.ssid, ssid, ssid_len);
  wcfg.sta.ssid[ssid_len] = '\0';
  if (password) {
    size_t pass_len = strlen(password);
    if (pass_len >= sizeof(wcfg.sta.password)) pass_len = sizeof(wcfg.sta.password) - 1;
    memcpy(wcfg.sta.password, password, pass_len);
    wcfg.sta.password[pass_len] = '\0';
  }
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
  return ESP_OK;
}

esp_err_t WifiSta::start() {
  esp_err_t err = esp_wifi_start();
  if (err == ESP_OK) {
    started = true;
    esp_wifi_connect();
  }
  return err;
}

void WifiSta::loop() {
  (void)0;
}

bool WifiSta::getIpAddr(uint32_t& ip) const {
  if (!ip_valid) return false;
  ip = cached_ip;
  return true;
}
