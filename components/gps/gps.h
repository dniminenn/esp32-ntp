#pragma once
// SPDX-License-Identifier: MIT-0
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"

struct GpsStats {
    double lastOffsetSec;     // last PPS-vs-system offset before correction
    double rmsOffsetSec;      // exponentially-weighted RMS
    double frequencyPpm;      // estimated local oscillator freq error
    double ppsJitterSec;      // inter-PPS interval jitter (sigma)
    uint32_t ppsCount;        // total PPS edges received
    uint32_t ppsRejectCount;  // PPS pulses rejected as outliers
};

class GpsDiscipline {
public:
  GpsDiscipline();
  esp_err_t begin(int uartPort, int baud, int txPin, int rxPin, int ppsGpio);
  bool isLocked() const { return gpsLock; }
  // Get last PPS in NTP epoch seconds and fractional 32-bit
  bool getLastPps(uint32_t& sec1900, uint32_t& frac) const;
  uint64_t getLastPpsMonotonicUs() const { return lastPpsMonotonicUs; }
  double getFrequencyPpm() const { return statFrequencyPpm; }
  uint64_t getLastNmeaUpdateUs() const { return lastNmeaUpdateUs; }
  void getStats(GpsStats& out) const;
  double getRootDispersion() const;
  void loop();

private:
  // UART/NMEA
  int uartPort;
  int baud;
  int txPin;
  int rxPin;
  int ppsGpio;

  // State
  volatile bool gpsLock;
  volatile uint32_t lastPpsSec1900;
  volatile uint32_t lastPpsFrac;
  volatile uint64_t lastPpsMonotonicUs;
  volatile time_t lastNmeaUnixSec;
  volatile uint64_t lastNmeaUpdateUs;
  volatile bool ppsPending;
  volatile uint64_t ppsEdgeUs;

  // Statistics tracking
  double statLastOffsetSec;
  double statRmsOffsetSec;
  double statFrequencyPpm;
  double statPpsJitterSec;
  uint32_t statPpsCount;
  uint32_t ppsRejectCount;       // PPS pulses rejected as outliers
  uint64_t prevPpsMonotonicUs;   // previous PPS edge for jitter calc
  double ppsIntervalMeanUs;      // EWMA mean of inter-PPS interval
  double ppsJitterVarUs2;        // EWMA variance of inter-PPS interval (µs²)
  int64_t firstPpsMonotonicUs;   // first PPS edge for freq estimation
  uint32_t firstPpsSec;          // first GPS second for freq estimation
  double clockCorrectionUs;      // integral servo correction applied to settimeofday
  uint64_t prevPpsEdgeForOffset;  // previous PPS edge for analytical offset
  double lastAppliedTotalCorrUs;  // total correction applied at previous settimeofday

  // Sliding-window frequency estimation (replaces boot-anchored estimator for dispersion)
  uint64_t freqWindowStartUs;
  uint32_t freqWindowStartSec;
  uint32_t freqWindowSamples;
  double filteredFrequencyPpm;   // EWMA-smoothed frequency for dispersion calc
  double filteredRmsOffsetSec;   // outlier-immune RMS for dispersion calc

  static void uart_task(void* arg);
  static bool parse_rmc_line(const char* line, int len, time_t& outUnixSec);
  static int from_hex(char c);
  static bool verify_nmea_checksum(const char* line, int len);
  static void IRAM_ATTR pps_isr_handler(void* arg);
  void handle_pps_deferred();
};


