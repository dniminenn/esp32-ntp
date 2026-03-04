#pragma once
// SPDX-License-Identifier: MIT-0

#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "driver/spi_master.h"

class W5500Eth {
public:
  W5500Eth();
  ~W5500Eth();
  
  esp_err_t begin(spi_host_device_t spiHost, int mosiPin, int misoPin, int sclkPin, int csPin, int intPin, int rstPin, int clockHz);
  esp_err_t start(bool use_static_ip = false,
                  const char* static_ip = nullptr,
                  const char* static_gw = nullptr,
                  const char* static_netmask = nullptr);
  esp_err_t stop();
  void loop();
  bool isLinkUp() const;
  esp_err_t getMacAddr(uint8_t mac[6]) const;
  bool getIpAddr(uint32_t& ip) const;

private:
  esp_netif_t* eth_netif;
  bool linkUp;
  int intPin;
  int rstPin;
};
