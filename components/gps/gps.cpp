// SPDX-License-Identifier: MIT-0

#include "gps.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "GPS";

static uint32_t unix_to_ntp_seconds(time_t unixSec) {
  return (uint32_t)((uint64_t)unixSec + 2208988800ULL);
}

GpsDiscipline::GpsDiscipline()
  : uartPort(-1), baud(0), txPin(-1), rxPin(-1), ppsGpio(-1),
    gpsLock(false), lastPpsSec1900(0), lastPpsFrac(0), lastPpsMonotonicUs(0),
    lastNmeaUnixSec(0), lastNmeaUpdateUs(0), ppsPending(false), ppsEdgeUs(0),
    statLastOffsetSec(0), statRmsOffsetSec(0), statFrequencyPpm(0),
    statPpsJitterSec(0), statPpsCount(0), ppsRejectCount(0), prevPpsMonotonicUs(0),
    ppsIntervalMeanUs(0), ppsJitterVarUs2(0),
    firstPpsMonotonicUs(0), firstPpsSec(0), clockCorrectionUs(0),
    prevPpsEdgeForOffset(0), lastAppliedTotalCorrUs(0),
    freqWindowStartUs(0), freqWindowStartSec(0), freqWindowSamples(0),
    filteredFrequencyPpm(0), filteredRmsOffsetSec(0) {}

esp_err_t GpsDiscipline::begin(int uartPort_, int baud_, int txPin_, int rxPin_, int ppsGpio_) {
  uartPort = uartPort_;
  baud = baud_;
  txPin = txPin_;
  rxPin = rxPin_;
  ppsGpio = ppsGpio_;

  // UART setup for NMEA
  uart_config_t cfg = {};
  cfg.baud_rate = baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_APB;
  ESP_ERROR_CHECK(uart_param_config((uart_port_t)uartPort, &cfg));
  ESP_ERROR_CHECK(uart_set_pin((uart_port_t)uartPort, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_driver_install((uart_port_t)uartPort, 2048, 0, 0, nullptr, 0));

  xTaskCreatePinnedToCore(&GpsDiscipline::uart_task, "gps_uart", 4096, this, 5, nullptr, 1);

  // PPS GPIO interrupt
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << ppsGpio);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_ERROR_CHECK(gpio_config(&io_conf));

  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)ppsGpio, &GpsDiscipline::pps_isr_handler, this));

  return ESP_OK;
}

void IRAM_ATTR GpsDiscipline::pps_isr_handler(void* arg) {
  GpsDiscipline* self = reinterpret_cast<GpsDiscipline*>(arg);
  self->ppsEdgeUs = esp_timer_get_time();
  self->ppsPending = true;
}

static bool parse_two(const char* s, int& v) {
  if (!isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[1])) return false;
  v = (s[0]-'0')*10 + (s[1]-'0');
  return true;
}

// Convert date/time to unix epoch (UTC)
static time_t make_unix_time_utc(int year, int month, int day, int hour, int min, int sec) {
  // year full (e.g., 2024)
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;
  int JDN = day + (153*m + 2)/5 + 365*y + y/4 - y/100 + y/400 - 32045;
  // days since Unix epoch (1970-01-01 JDN 2440588)
  int days = JDN - 2440588;
  return (time_t)days * 86400 + hour*3600 + min*60 + sec;
}

int GpsDiscipline::from_hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

bool GpsDiscipline::verify_nmea_checksum(const char* line, int len) {
  // line includes optional CRLF; find '*' and compute XOR of chars between '$' and '*'
  if (len < 4) return false;
  const char* star = nullptr;
  int iStar = -1;
  for (int i = 0; i < len; ++i) {
    if (line[i] == '*') { star = &line[i]; iStar = i; break; }
    if (line[i] == '\0') break;
  }
  if (!star || iStar < 1 || line[0] != '$') return false;
  uint8_t chk = 0;
  for (int i = 1; i < iStar; ++i) chk ^= (uint8_t)line[i];
  if (iStar + 2 >= len) return false;
  int h1 = from_hex(line[iStar+1]);
  int h2 = from_hex(line[iStar+2]);
  if (h1 < 0 || h2 < 0) return false;
  uint8_t want = (uint8_t)((h1 << 4) | h2);
  return chk == want;
}

bool GpsDiscipline::parse_rmc_line(const char* line, int len, time_t& outUnixSec) {
  if (len < 6 || line[0] != '$') return false;
  // Accept GPRMC, GNRMC
  if (!(strncmp(line+1, "GPRMC", 5) == 0 || strncmp(line+1, "GNRMC", 5) == 0)) return false;
  if (!verify_nmea_checksum(line, len)) return false;

  // Tokenize by ',' but stop at '*'
  // Fields: 1:time(hhmmss.sss), 2:status(A/V), 9:date(ddmmyy)
  const int MAXF = 20;
  const char* f[MAXF];
  int nf = 0;
  f[nf++] = line; // '$GPRMC...'
  for (int i = 0; i < len && nf < MAXF; ++i) {
    if (line[i] == ',') f[nf++] = &line[i+1];
    if (line[i] == '*') break;
  }
  if (nf < 10) return false;
  const char* times = f[1];
  const char* status = f[2];
  const char* dates = f[9];
  if (!times || !status || !dates) return false;
  if (status[0] != 'A') return false;

  int hh=0, mm=0, ss=0;
  if (!parse_two(times, hh) || !parse_two(times+2, mm) || !parse_two(times+4, ss)) return false;
  int dd=0, MM=0, yy=0;
  if (!parse_two(dates, dd) || !parse_two(dates+2, MM) || !parse_two(dates+4, yy)) return false;
  int fullYear = (yy >= 70 ? 1900 + yy : 2000 + yy);

  outUnixSec = make_unix_time_utc(fullYear, MM, dd, hh, mm, ss);
  return true;
}

void GpsDiscipline::uart_task(void* arg) {
  auto* self = reinterpret_cast<GpsDiscipline*>(arg);
  const int maxLine = 128;
  char line[maxLine];
  int pos = 0;
  while (true) {
    uint8_t ch;
    int n = uart_read_bytes((uart_port_t)self->uartPort, &ch, 1, pdMS_TO_TICKS(100));
    if (n <= 0) continue;
    if (ch == '\n' || ch == '\r') {
      if (pos > 0) {
        time_t unixSec;
        if (GpsDiscipline::parse_rmc_line(line, pos, unixSec)) {
          self->lastNmeaUnixSec = unixSec;
          self->lastNmeaUpdateUs = esp_timer_get_time();
        }
      }
      pos = 0;
    } else {
      if (pos < maxLine-1) line[pos++] = (char)ch;
      else pos = 0; // reset on overflow
    }
  }
}

bool GpsDiscipline::getLastPps(uint32_t& sec1900, uint32_t& frac) const {
  if (lastPpsSec1900 == 0) return false;
  sec1900 = lastPpsSec1900;
  frac = lastPpsFrac;
  return true;
}

void GpsDiscipline::handle_pps_deferred() {
  if (!ppsPending) return;
  ppsPending = false;
  uint64_t ppsEdgeCapture = ppsEdgeUs;
  lastPpsMonotonicUs = ppsEdgeCapture;

  uint64_t nowUs = esp_timer_get_time();
  uint64_t ageUs = nowUs - lastNmeaUpdateUs;
  bool wasLocked = gpsLock;
  
  if (ageUs < 1500000ULL && lastNmeaUnixSec > 0) {
    time_t ppsSec = lastNmeaUnixSec + 1;

    // --- PPS outlier rejection gate ---
    // Detect missed or spurious PPS by checking interval sanity.
    // If the interval is outside +/-10% of 1s, skip servo/stats updates
    // but still set the clock from NMEA and update the edge reference.
    bool outlier = false;
    double offset = 0;
    if (prevPpsEdgeForOffset != 0) {
      double ppsIntervalUs = (double)((int64_t)(ppsEdgeCapture - prevPpsEdgeForOffset));
      if (ppsIntervalUs < 900000.0 || ppsIntervalUs > 1100000.0) {
        outlier = true;
        ppsRejectCount++;
        ESP_LOGW(TAG, "PPS outlier rejected: interval=%.0fus (expected ~1000000us)", ppsIntervalUs);
      } else {
        offset = (ppsIntervalUs - 1000000.0 + lastAppliedTotalCorrUs) / 1e6;
      }
    }

    // PI servo: only update on clean (non-outlier) pulses
    double proportionalUs = 0;
    if (!outlier && statPpsCount >= 20 && prevPpsEdgeForOffset != 0) {
      statLastOffsetSec = offset;
      double offsetUs = offset * 1e6;
      proportionalUs = -0.5 * offsetUs;
      clockCorrectionUs -= 0.05 * offsetUs;
      if (clockCorrectionUs > 50)  clockCorrectionUs = 50;
      if (clockCorrectionUs < -50) clockCorrectionUs = -50;
    }

    // Set system time: elapsed + PI servo + frequency feedforward
    uint64_t setUs = esp_timer_get_time();
    int64_t rawElapsed = (int64_t)(setUs - ppsEdgeCapture);
    double freqCompUs = (statPpsCount >= 30) ? -filteredFrequencyPpm : 0;
    double totalCorrUs = clockCorrectionUs + proportionalUs + freqCompUs;
    int64_t elapsedUs = rawElapsed + (int64_t)totalCorrUs;
    if (elapsedUs < 0) elapsedUs = 0;
    struct timeval tv;
    tv.tv_sec = ppsSec + (time_t)(elapsedUs / 1000000LL);
    tv.tv_usec = (suseconds_t)(elapsedUs % 1000000LL);
    settimeofday(&tv, nullptr);
    prevPpsEdgeForOffset = ppsEdgeCapture;
    lastAppliedTotalCorrUs = outlier ? 0 : totalCorrUs;

    // Store PPS reference timestamp with calibration offset applied
    int64_t calibrationUs = Config::getPpsCalibrationUs();
    int64_t adjustedPpsSec = (int64_t)ppsSec + (calibrationUs / 1000000);
    int64_t adjustedPpsFracUs = calibrationUs % 1000000;
    if (adjustedPpsFracUs < 0) {
      adjustedPpsSec -= 1;
      adjustedPpsFracUs += 1000000;
    }
    lastPpsSec1900 = unix_to_ntp_seconds((time_t)adjustedPpsSec);
    lastPpsFrac = (uint32_t)(((uint64_t)adjustedPpsFracUs << 32) / 1000000ULL);

    gpsLock = true;
    if (!wasLocked) {
      ESP_LOGI(TAG, "GPS locked - PPS disciplining active (latency: %" PRId64 "us)", elapsedUs);
    }

    // RMS offset: only include clean samples
    if (!outlier && prevPpsEdgeForOffset != 0) {
      const double alpha = 0.1;
      statRmsOffsetSec = sqrt(alpha * offset * offset + (1.0 - alpha) * statRmsOffsetSec * statRmsOffsetSec);
      // Separate filtered RMS for dispersion (slower decay, outlier-immune)
      const double alphaFilt = 0.05;
      filteredRmsOffsetSec = sqrt(alphaFilt * offset * offset + (1.0 - alphaFilt) * filteredRmsOffsetSec * filteredRmsOffsetSec);
    }

    statPpsCount++;

    // PPS jitter: EWMA variance on inter-PPS interval (~20-sample window)
    // Only include non-outlier intervals to avoid corrupting the jitter estimate.
    // Secondary micro-outlier gate: reject intervals where deviation from mean
    // exceeds max(10µs, 5σ) — catches ISR latency spikes that pass the coarse gate.
    if (prevPpsMonotonicUs != 0 && !outlier) {
      double intervalUs = (double)(ppsEdgeCapture - prevPpsMonotonicUs);
      if (ppsIntervalMeanUs == 0) {
        ppsIntervalMeanUs = intervalUs;
      } else {
        double diff = intervalUs - ppsIntervalMeanUs;
        double absDiff = fabs(diff);
        double sigma = sqrt(ppsJitterVarUs2);
        double threshold = (sigma > 2.0) ? 5.0 * sigma : 10.0;  // max(10µs, 5σ)
        if (absDiff < threshold) {
          const double alphaJ = 0.05;
          ppsIntervalMeanUs += alphaJ * diff;
          ppsJitterVarUs2 = (1.0 - alphaJ) * (ppsJitterVarUs2 + alphaJ * diff * diff);
          statPpsJitterSec = sqrt(ppsJitterVarUs2) / 1e6;
        } else {
          ESP_LOGD(TAG, "PPS jitter micro-outlier: dev=%.1fus thresh=%.1fus", absDiff, threshold);
        }
      }
    }
    prevPpsMonotonicUs = ppsEdgeCapture;

    // --- Sliding-window frequency estimator ---
    // Resets every ~300 seconds so stale boot data doesn't dominate.
    // Only non-outlier pulses advance the window sample count.
    if (!outlier) {
      if (freqWindowStartUs == 0) {
        freqWindowStartUs = ppsEdgeCapture;
        freqWindowStartSec = (uint32_t)ppsSec;
        freqWindowSamples = 0;
      }
      freqWindowSamples++;
      if (freqWindowSamples > 300) {
        // Reset window for fresh estimate
        freqWindowStartUs = ppsEdgeCapture;
        freqWindowStartSec = (uint32_t)ppsSec;
        freqWindowSamples = 0;
      } else if (freqWindowSamples > 30) {
        double monoElapsedSec = (double)(ppsEdgeCapture - freqWindowStartUs) / 1e6;
        double gpsElapsedSec = (double)((uint32_t)ppsSec - freqWindowStartSec);
        if (gpsElapsedSec > 0) {
          double rawPpm = ((monoElapsedSec - gpsElapsedSec) / gpsElapsedSec) * 1e6;
          statFrequencyPpm = rawPpm;  // raw for stats display
          // EWMA smooth for dispersion / feedforward
          if (filteredFrequencyPpm == 0 && statPpsCount > 30) {
            filteredFrequencyPpm = rawPpm;
          } else {
            filteredFrequencyPpm += 0.02 * (rawPpm - filteredFrequencyPpm);
          }
        }
      }
    }

    // Legacy boot-anchored estimator (kept for stats, replaced by window for servo)
    if (firstPpsMonotonicUs == 0) {
      firstPpsMonotonicUs = (int64_t)ppsEdgeCapture;
      firstPpsSec = (uint32_t)ppsSec;
    }

    // After warmup, reset accumulators so boot noise doesn't persist
    static const uint32_t kWarmup = 20;
    if (statPpsCount == kWarmup) {
      statRmsOffsetSec = 0;
      filteredRmsOffsetSec = 0;
      ppsIntervalMeanUs = 0;
      ppsJitterVarUs2 = 0;
      prevPpsMonotonicUs = 0;
      firstPpsMonotonicUs = (int64_t)ppsEdgeCapture;
      firstPpsSec = (uint32_t)ppsSec;
      statPpsJitterSec = 0;
      statFrequencyPpm = 0;
      filteredFrequencyPpm = 0;
      clockCorrectionUs = 0;
      freqWindowStartUs = ppsEdgeCapture;
      freqWindowStartSec = (uint32_t)ppsSec;
      freqWindowSamples = 0;
    }
  } else {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    lastPpsSec1900 = unix_to_ntp_seconds(tv.tv_sec);
    lastPpsFrac = 0;
    prevPpsEdgeForOffset = 0;
    lastAppliedTotalCorrUs = 0;
    if (wasLocked) {
      ESP_LOGW(TAG, "GPS lock lost - NMEA stale or missing");
    }
    gpsLock = false;
  }
}

void GpsDiscipline::getStats(GpsStats& out) const {
  out.lastOffsetSec = statLastOffsetSec;
  out.rmsOffsetSec = statRmsOffsetSec;
  out.frequencyPpm = statFrequencyPpm;
  out.ppsJitterSec = statPpsJitterSec;
  out.ppsCount = statPpsCount;
  out.ppsRejectCount = ppsRejectCount;
}

double GpsDiscipline::getRootDispersion() const {
  if (!gpsLock) return 1.0;
  uint64_t now = esp_timer_get_time();
  double sincePpsSec = (double)(now - lastPpsMonotonicUs) / 1e6;
  // Use filtered (outlier-immune) RMS and frequency for stable dispersion
  return fabs(filteredRmsOffsetSec) + fabs(filteredFrequencyPpm) * 1e-6 * sincePpsSec + 1e-6;
}

void GpsDiscipline::loop() {
  handle_pps_deferred();
}


