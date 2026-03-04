#pragma once
// SPDX-License-Identifier: MIT-0
#include <stdint.h>
#include "esp_err.h"

class GpsDiscipline;

class NtpServer {
public:
  NtpServer();
  esp_err_t begin(int port, GpsDiscipline* gps);
  void loop();
  uint32_t getRequestCount() const { return requestCount; }

private:
  void computeNtpTimestamp(uint64_t monoUs, bool locked, uint32_t& sec1900, uint32_t& frac);

  int sock;
  int port;
  GpsDiscipline* gps;
  uint32_t requestCount;
  bool useWifi;

  // Non-blocking send state for ARP delay detection
  bool sendPending;
  uint64_t sendStartUs;
  uint8_t pendingRsp[48];
  uint8_t pendingIp[4];
  uint16_t pendingPort;
  bool pendingLocked;
};


