#pragma once
// SPDX-License-Identifier: MIT-0
#include <stdint.h>
#include "esp_err.h"

class WifiSta {
public:
  WifiSta();
  ~WifiSta();
  esp_err_t begin(const char* ssid, const char* password,
                  bool use_static_ip = false,
                  const char* static_ip = nullptr,
                  const char* static_gw = nullptr,
                  const char* static_netmask = nullptr);
  esp_err_t start();
  void loop();
  bool getIpAddr(uint32_t& ip) const;
  void onGotIp(uint32_t ip);

private:
  bool started;
  uint32_t cached_ip;
  bool ip_valid;
};
