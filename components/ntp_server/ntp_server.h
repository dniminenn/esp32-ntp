#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include "esp_err.h"

class GpsDiscipline;

/* Wake-on-interrupt plumbing, used by the NTP task in app_main. */
bool ntp_wait_for_packet(uint32_t timeout_ms);
void ntp_register_task(void* handle);

class NtpServer {
public:
  NtpServer();
  esp_err_t begin(int port, GpsDiscipline* gps);
  bool loop();   // true = consumed a datagram
  void reopenSocket();   // after a W5500 chip re-init: hardware sockets are gone
  uint32_t getRequestCount() const { return requestCount; }
  uint32_t getPrimeSkips() const;
  uint32_t getCapRejects() const;
  uint32_t getRxIrqCount() const;
  uint32_t getLateStampOk() const;
  uint32_t getInterleavedServed() const;
  uint32_t getLateStampFallbacks() const;
  int getLastStageRc() const;
  int getLastWrDelta() const;
  uint32_t getTurnSamples() const;
  double getTurnUs() const;
  double getTxCorrectionUs() const;

private:
  void setupRxInterrupt();

  void computeNtpTimestamp(uint64_t monoUs, bool locked, uint32_t& sec1900, uint32_t& frac);

  int sock;
  int port;
  GpsDiscipline* gps;
  uint32_t requestCount;
  uint32_t lastRxIrqConsumed;
  uint32_t lastRxCapSeq;   // last MCPWM-latched arrival consumed
  bool useWifi;
};


