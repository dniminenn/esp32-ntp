#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include "esp_err.h"

class GpsDiscipline;

class NtpServer {
public:
  NtpServer();
  esp_err_t begin(int port, GpsDiscipline* gps);
  void loop();
  uint32_t getRequestCount() const { return requestCount; }
  uint32_t getRxIrqCount() const;
  double getTxCorrectionUs() const;

private:
  void setupRxInterrupt();

  void computeNtpTimestamp(uint64_t monoUs, bool locked, uint32_t& sec1900, uint32_t& frac);

  int sock;
  int port;
  GpsDiscipline* gps;
  uint32_t requestCount;
  uint32_t lastRxIrqConsumed;
  bool useWifi;
};


