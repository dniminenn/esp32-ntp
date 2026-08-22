// SPDX-License-Identifier: Unlicense

#include "gps.h"
#include <stdlib.h>
#include "config.h"
#include "civil_time.h"
#include "nvs.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "GPS";

// Holdover policy: after GPS is lost the clock coasts on the disciplined
// oscillator. We keep claiming sync (stratum 1) while the predicted error
// stays bounded, with root dispersion growing to tell clients honestly.
static const int64_t kFreshPpsUs = 2500000;            // disciplining considered live within 2.5s of a good PPS
/*
 * While disciplined, dispersion must grow at the rate our frequency estimate
 * could be WRONG, not at the raw frequency error. The fit measures the crystal
 * at ~27ppm and compensates it; what remains uncertain is that estimate, whose
 * observed instability is ~0.1ppm/hour. Using the uncorrected 27ppm here
 * overstated dispersion by two orders of magnitude and pushed the advertised
 * value to ~31us when the protocol floor is 15.26us anyway.
 */
static const double  kDisciplinedDriftPpm = 0.1;
static const double  kHoldoverMaxDispersionSec = 0.01; // drop the lock once predicted error exceeds 10ms
static const int64_t kHoldoverMaxUs = 3600LL * 1000000LL; // ...or after 1 hour, whichever first
/*
 * Holdover error model.
 *
 * The same argument as kDisciplinedDriftPpm above, carried into holdover: the
 * crystal's offset is COMPENSATED, and the compensation keeps being applied
 * while coasting. What degrades is not that offset but the rate at which the
 * frozen estimate goes stale, which is the frequency drift. That drift is
 * thermal, so it depends on your crystal, your enclosure and your room --
 * statFreqDriftPerSec measures it live rather than assuming a figure.
 *
 * Error grows in two terms, not one. The estimate is already wrong by
 * drift x (the estimator's lag) at the moment GPS is lost, and it keeps going
 * wrong, which integrates to the quadratic term.
 *
 * The lag is DERIVED in getRootDispersion() from kFitWin and kFreqEwmaAlpha
 * rather than written down, because a hardcoded figure silently goes wrong the
 * moment someone retunes either for a different GPS module or a noisier pulse.
 *
 * For scale, one deployment (bare board, unheated shed) measured 0.075 ppm/h
 * median and 0.42 ppm/h at p95, over a 1.04 ppm peak-to-peak daily swing. That
 * is a wide-swinging environment rather than a gentle one, so treat it as a
 * rough ceiling for a climate-controlled room and as no guide at all for a
 * sealed enclosure or an outdoor mount. Nothing here depends on the figure.
 */
static const double  kFreqEwmaAlpha = 0.05;               // fit -> filtered frequency
static const double  kHoldoverDriftFloorPpmPerHour = 0.5; // never claim better than this

/* DS3231 TCXO holdover constants. */
static const uint32_t kTcxoPrescale = 256;   // real divisor calibrated at runtime
static const double   kTcxoHoldoverFloorPpm = 0.05;        // never claim better than this
// dispersion ceiling governs (~55 h)
static const int64_t  kHoldoverMaxTcxoUs = 72LL * 3600LL * 1000000LL;
static const uint32_t kTcxoMinLearnSamples = 60;           // before trusting a correction
/* GPS flywheel: serve smoothed pulses. */
static const double   kFwAlpha = 1.0 / 32.0;   // ~30 s pulse averaging
static const double   kFwClampNs = 500.0;      // beyond this, serve raw
static const uint32_t kFwSettle = 64;          // pulses before trusting
static const float    kTcxoTempMin = -40.0f;               // full rated range, 0.5C step

static uint32_t unix_to_ntp_seconds(time_t unixSec) {
  return (uint32_t)((uint64_t)unixSec + 2208988800ULL);
}

GpsDiscipline::GpsDiscipline()
  : uartPort(-1), baud(0), txPin(-1), rxPin(-1), ppsGpio(-1),
    gpsLock(false), holdover(false), lastGoodPpsUs(0), lockExpiredLogged(false),
    statMispairCount(0), mispairStreak(0),
    lastPpsSec1900(0), lastPpsFrac(0), lastPpsMonotonicUs(0),
    lastNmeaUnixSec(0), lastNmeaUpdateUs(0), ppsSeq(0), ppsEdgeUs(0),
    ppsCapValue(0), ppsSeqSeen(0), prevPpsCapValue(0),
    fitHead(0), fitCount(0), tickExt(0), tickLastRaw(0), tickInit(false),
    fitTicksPerSec(80000000.0), fitResidualTicks(0), fitValid(false),
    statLastOffsetSec(0), statRmsOffsetSec(0), statFrequencyPpm(0),
    statPpsJitterSec(0), statPpsCount(0), ppsRejectCount(0), prevPpsMonotonicUs(0),
    ppsIntervalMeanUs(0), ppsJitterVarUs2(0),
    clockCorrectionUs(0),
    prevPpsEdgeForOffset(0), lastAppliedTotalCorrUs(0),
    statPpsHandleLatUs(0), statPpsHandleLatMaxUs(0),
    statFreqDriftPerSec(0), slopeLagged(0), slopeLagCount(0),
    filteredFrequencyPpm(0), filteredRmsOffsetSec(0),
    capTimer(nullptr), capChannel(nullptr), rxCapChannel(nullptr),
    rxCapTick(0), rxCapSeq(0),
    tcxoCapChannel(nullptr), tcxoSeq(0), tcxoTickExt(0), tcxoCount(0),
    tcxoRawPub(0), tcxoLastRaw(0), tcxoTickInit(false),
    tcxoRingHead(0), tcxoRingN(0), tcxoLastSnapUs(0),
    tcxoCyclesPerEvent(0), tcxoApbPerSec(0), tcxoRatioValid(false),
    rtcTempC(0), rtcTempUs(0),
    tcxoGlobalPpm(0), tcxoGlobalN(0), tcxoLiveErrPpm(0), tcxoResidualPpm(0),
    tcxoHoldoverPpm(0), tcxoHoldoverGood(false), tcxoCorrFromBin(false),
    tcxoHoldStartUs(0), tcxoHoldLastUs(0), tcxoHoldPpmSecIntegral(0), tcxoAvgPpm(0),
    tcxoPpsPrevValid(false), tcxoPpsPrevCnt(0), tcxoPpsPrevSub(0),
    tcxoPpsPrevSec(0), tcxoPpsFreqInit(false), tcxoPpsFreqNsPerS(0),
    tcxoPpsDevNs(0), tcxoPpsRmsNs(0),
    tcxoFlywheelNs(0), tcxoFwSettle(0), tcxoFwValid(false),
    anchorSeq(0), anchorCapTick(0), anchorSec1900(0), anchorFrac(0) {
  memset(tcxoBinPpm, 0, sizeof(tcxoBinPpm));
  memset(tcxoBinN, 0, sizeof(tcxoBinN));
}

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

  // PPS capture via MCPWM hardware — latches timer at exact GPIO edge
  mcpwm_capture_timer_config_t cap_timer_cfg = {};
  cap_timer_cfg.group_id = 0;
  cap_timer_cfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_cfg, &capTimer));

  mcpwm_capture_channel_config_t cap_ch_cfg = {};
  cap_ch_cfg.gpio_num = ppsGpio;
  cap_ch_cfg.prescale = 1;
  cap_ch_cfg.flags.pos_edge = 1;
  cap_ch_cfg.flags.neg_edge = 0;
  ESP_ERROR_CHECK(mcpwm_new_capture_channel(capTimer, &cap_ch_cfg, &capChannel));

  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = &GpsDiscipline::pps_capture_callback;
  ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(capChannel, &cbs, this));

  ESP_ERROR_CHECK(mcpwm_capture_channel_enable(capChannel));
  ESP_ERROR_CHECK(mcpwm_capture_timer_enable(capTimer));
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(capTimer));

  return ESP_OK;
}

/* Discipline task to wake when the PPS edge is captured. */
static volatile TaskHandle_t s_disciplineTask = nullptr;

void gps_register_task(void* handle) { s_disciplineTask = (TaskHandle_t)handle; }

bool gps_wait_for_pps(uint32_t timeout_ms) {
  if (s_disciplineTask == nullptr) gps_register_task(xTaskGetCurrentTaskHandle());
  return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0;
}

bool IRAM_ATTR GpsDiscipline::pps_capture_callback(
    mcpwm_cap_channel_handle_t cap_channel,
    const mcpwm_capture_event_data_t* edata,
    void* user_ctx) {
  GpsDiscipline* self = reinterpret_cast<GpsDiscipline*>(user_ctx);
  // Seqlock publish: bump to odd, write, bump to even. The task retries if
  // the counter moved or is odd, so it can never consume a torn snapshot.
  self->ppsSeq = self->ppsSeq + 1;
  __sync_synchronize();
  self->ppsCapValue = edata->cap_value;
  self->ppsEdgeUs = esp_timer_get_time();
  __sync_synchronize();
  self->ppsSeq = self->ppsSeq + 1;
  /*
   * Wake the discipline task on the edge itself rather than letting it discover
   * the pulse on its next poll.
   *
   * NOTE ON WHAT THIS DOES AND DOES NOT FIX. Polling every 2 ms meant the
   * handler ran up to ~2 ms after the capture (measured: 659 us mean, 1977 us
   * max). That latency does NOT enter the reported offset: handle_pps_deferred()
   * computes it as fitResidualTicks / fitTicksPerSec, and both come only from
   * MCPWM capture ticks and NMEA second labels. It does not enter the system
   * clock either — settimeofday() adds back (now - ppsEdgeCapture) exactly, as
   * does the PPS/NMEA mispair guard. What it did cost is anchor freshness: for
   * the first couple of milliseconds after each pulse, lastPpsMonotonicUs and
   * the capture anchor still described the PREVIOUS pulse, so a request landing
   * in that window extrapolated a full second instead of nearly zero. Small
   * (~10 ns at the disciplined frequency error) but free to remove.
   */
  BaseType_t hpw = pdFALSE;
  if (s_disciplineTask) vTaskNotifyGiveFromISR((TaskHandle_t)s_disciplineTask, &hpw);
  return hpw == pdTRUE;
}

// --- Phase/frequency fit ------------------------------------------------
// wall(tick) is linear in GPS seconds while the oscillator is stable, so a
// least-squares line through (GPS second, capture tick) gives the frequency
// as its slope and each pulse's phase error as its residual. Averaging over
// the window suppresses per-pulse capture noise by ~sqrt(N), and — unlike the
// old 300-sample hard reset — the frequency estimate is never discarded, so
// holdover starts from the best estimate available rather than a blind one.

void GpsDiscipline::fitReset() {
  fitHead = 0;
  fitCount = 0;
  fitValid = false;
  fitTicksPerSec = 80000000.0;
  fitResidualTicks = 0;
}

void GpsDiscipline::fitPush(uint32_t gpsSec, int64_t tick) {
  fitRing[fitHead].sec = gpsSec;
  fitRing[fitHead].tick = tick;
  fitHead = (fitHead + 1) % kFitWin;
  if (fitCount < kFitWin) fitCount++;
}

bool GpsDiscipline::fitSolve() {
  if (fitCount < kFitMinSamples) return false;
  int idx = (fitHead + kFitWin - fitCount) % kFitWin;
  // Rebase on the oldest sample: doubles hold small deltas exactly, whereas
  // absolute ticks (2^32+) and GPS seconds (1.7e9) lose sub-tick resolution.
  const FitSample& base = fitRing[idx];
  double sx = 0, sy = 0;
  for (int i = 0; i < fitCount; i++) {
    const FitSample& s = fitRing[(idx + i) % kFitWin];
    sx += (double)(int32_t)(s.sec - base.sec);
    sy += (double)(s.tick - base.tick);
  }
  double mx = sx / fitCount, my = sy / fitCount;
  double sxx = 0, sxy = 0;
  for (int i = 0; i < fitCount; i++) {
    const FitSample& s = fitRing[(idx + i) % kFitWin];
    double dx = (double)(int32_t)(s.sec - base.sec) - mx;
    double dy = (double)(s.tick - base.tick) - my;
    sxx += dx * dx;
    sxy += dx * dy;
  }
  if (sxx <= 0) return false;
  double slope = sxy / sxx;
  // Sanity: an ESP32 crystal is spec'd ±10 ppm and ages to maybe ±50; a slope
  // outside ±500 ppm means the ring holds garbage (missed pulses, a step).
  if (slope < 79960000.0 || slope > 80040000.0) return false;
  // Residual of the newest sample = its phase error against the model.
  const FitSample& last = fitRing[(fitHead + kFitWin - 1) % kFitWin];
  double lx = (double)(int32_t)(last.sec - base.sec) - mx;
  double ly = (double)(last.tick - base.tick) - my;
  fitTicksPerSec = slope;
  fitResidualTicks = ly - slope * lx;
  return true;
}

// --- RX packet-arrival capture -----------------------------------------
// The W5500 asserts INTn when a datagram has landed in its buffer. Feeding
// that line to a second MCPWM capture channel latches the arrival in the same
// 80 MHz counter that timestamps PPS: 12.5 ns resolution and, crucially, no
// interrupt-latency term at all, where the previous GPIO ISR contributed
// ~1-2 us of jitter to every t2. The ISR here only publishes the already-
// latched value.
bool IRAM_ATTR GpsDiscipline::rx_capture_callback(
    mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t* edata,
    void* ctx) {
  GpsDiscipline* self = reinterpret_cast<GpsDiscipline*>(ctx);
  // Same odd/even protocol as the PPS callback: writing the tick first and
  // then bumping a single counter lets a reader pair a NEW tick with an OLD
  // sequence number, which either drops the capture or misattributes it.
  self->rxCapSeq = self->rxCapSeq + 1;
  __sync_synchronize();
  self->rxCapTick = edata->cap_value;
  __sync_synchronize();
  self->rxCapSeq = self->rxCapSeq + 1;
  return false;
}

esp_err_t GpsDiscipline::beginRxCapture(int intGpio) {
  if (intGpio < 0 || capTimer == nullptr) return ESP_ERR_INVALID_ARG;
  mcpwm_capture_channel_config_t cfg = {};
  cfg.gpio_num = intGpio;
  cfg.prescale = 1;
  cfg.flags.neg_edge = 1;   // INTn is active low
  cfg.flags.pos_edge = 0;
  // No pull-up flag: GPIO34-39 have no pull resistors in silicon, so the
  // W5500 module's external pull-up is what holds INTn high. (The flag was
  // also removed from mcpwm_capture_channel_config_t in IDF v6.)
  esp_err_t err = mcpwm_new_capture_channel(capTimer, &cfg, &rxCapChannel);
  if (err != ESP_OK) return err;
  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = &GpsDiscipline::rx_capture_callback;
  err = mcpwm_capture_channel_register_event_callbacks(rxCapChannel, &cbs, this);
  if (err != ESP_OK) return err;
  return mcpwm_capture_channel_enable(rxCapChannel);
}

// --- DS3231 TCXO capture -------------------------------------------------
bool IRAM_ATTR GpsDiscipline::tcxo_capture_callback(
    mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t* edata,
    void* ctx) {
  GpsDiscipline* self = reinterpret_cast<GpsDiscipline*>(ctx);
  uint32_t raw = edata->cap_value;
  if (!self->tcxoTickInit) {
    self->tcxoTickInit = true;
    self->tcxoLastRaw = raw;
    return false;
  }
  int32_t d = (int32_t)(raw - self->tcxoLastRaw);   // wrap-safe: ~625k ticks apart
  self->tcxoLastRaw = raw;
  self->tcxoSeq = self->tcxoSeq + 1;
  __sync_synchronize();
  self->tcxoTickExt += d;
  self->tcxoCount = self->tcxoCount + 1;
  self->tcxoRawPub = raw;
  __sync_synchronize();
  self->tcxoSeq = self->tcxoSeq + 1;
  return false;
}

esp_err_t GpsDiscipline::beginTcxoCapture(int gpio) {
  if (gpio < 0 || capTimer == nullptr) return ESP_ERR_INVALID_ARG;
  mcpwm_capture_channel_config_t cfg = {};
  cfg.gpio_num = gpio;
  cfg.prescale = kTcxoPrescale;
  // falling edge; rising ramps slowly
  cfg.flags.pos_edge = 0;
  cfg.flags.neg_edge = 1;
  // last channel, shared 80MHz counter
  esp_err_t err = mcpwm_new_capture_channel(capTimer, &cfg, &tcxoCapChannel);
  if (err != ESP_OK) return err;
  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = &GpsDiscipline::tcxo_capture_callback;
  err = mcpwm_capture_channel_register_event_callbacks(tcxoCapChannel, &cbs, this);
  if (err != ESP_OK) return err;
  err = mcpwm_capture_channel_enable(tcxoCapChannel);
  if (err != ESP_OK) return err;
  // open-drain 32K needs pull-up
  gpio_set_pull_mode((gpio_num_t)gpio, GPIO_PULLUP_ONLY);
  return ESP_OK;
}

void GpsDiscipline::setRtcTemp(float tempC) {
  rtcTempC = tempC;
  rtcTempUs = esp_timer_get_time();
}

// correction for current die temperature
bool GpsDiscipline::tcxoCorrPpm(double& out, bool& fromBin) const {
  fromBin = false;
  bool tempFresh = rtcTempUs != 0 &&
                   esp_timer_get_time() - rtcTempUs < 300000000LL;   // 5 min
  if (tempFresh) {
    int bin = (int)((rtcTempC - kTcxoTempMin) * 2.0f);
    for (int r = 0; r <= 4; ++r) {           // search outward to ±2C
      for (int s = -1; s <= 1; s += 2) {
        int b = bin + (s < 0 ? -r : r);
        if (b < 0 || b >= kTcxoBins) continue;
        if (tcxoBinN[b] >= 32) { out = tcxoBinPpm[b]; fromBin = true; return true; }
        if (r == 0) break;                   // don't visit bin twice
      }
    }
  }
  if (tcxoGlobalN >= kTcxoMinLearnSamples) { out = tcxoGlobalPpm; return true; }
  return false;
}

// PPS timestamped against TCXO ticks
void GpsDiscipline::tcxoPpsSample(uint32_t ppsCapTick, uint32_t gpsSec) {
  if (tcxoCapChannel == nullptr || !tcxoRatioValid) {
    tcxoPpsPrevValid = false;
    tcxoFwValid = false;
    return;
  }
  uint32_t cnt = 0, raw = 0, s;
  int tries = 0;
  for (;;) {
    if (++tries > 8) return;
    s = tcxoSeq;
    if (s & 1) continue;
    __sync_synchronize();
    cnt = tcxoCount;
    raw = tcxoRawPub;
    __sync_synchronize();
    if (tcxoSeq == s) break;
  }
  // ticks from newest 32k capture
  // bound: one capture period max
  int32_t sub = (int32_t)(ppsCapTick - raw);
  if (sub > 2000000 || sub < -2000000) { tcxoPpsPrevValid = false; return; }

  if (tcxoPpsPrevValid) {
    uint32_t dSec = gpsSec - tcxoPpsPrevSec;
    if (dSec >= 1 && dSec <= 5) {
      double cycles = (double)(uint32_t)(cnt - tcxoPpsPrevCnt) * tcxoCyclesPerEvent
                    + (double)(sub - tcxoPpsPrevSub) * (32768.0 / tcxoApbPerSec);
      double devNs = (cycles / 32768.0 - (double)dSec) * 1e9;
      double devPerSec = devNs / (double)dSec;
      if (!tcxoPpsFreqInit) {
        tcxoPpsFreqInit = true;
        tcxoPpsFreqNsPerS = devPerSec;
      } else {
        double resid = devNs - tcxoPpsFreqNsPerS * (double)dSec;
        tcxoPpsDevNs = resid;
        tcxoPpsRmsNs = sqrt(0.05 * resid * resid + 0.95 * tcxoPpsRmsNs * tcxoPpsRmsNs);
        tcxoPpsFreqNsPerS += (1.0 / 240.0) * (devPerSec - tcxoPpsFreqNsPerS);
        // flywheel: smoothed minus raw
        if (resid > -1000.0 && resid < 1000.0) {
          tcxoFlywheelNs = (1.0 - kFwAlpha) * (tcxoFlywheelNs - resid);
          if (tcxoFlywheelNs > -kFwClampNs && tcxoFlywheelNs < kFwClampNs) {
            if (tcxoFwSettle < kFwSettle) tcxoFwSettle++;
            else tcxoFwValid = true;
          } else {
            tcxoFlywheelNs = 0;   // ramp too fast; serve raw
            tcxoFwSettle = 0;
            tcxoFwValid = false;
          }
        } else {
          tcxoFlywheelNs = 0;
          tcxoFwSettle = 0;
          tcxoFwValid = false;
        }
      }
    } else {
      tcxoPpsFreqInit = false;   // gap: trend stale
      tcxoFlywheelNs = 0;
      tcxoFwSettle = 0;
      tcxoFwValid = false;
    }
  }
  tcxoPpsPrevValid = true;
  tcxoPpsPrevCnt = cnt;
  tcxoPpsPrevSub = sub;
  tcxoPpsPrevSec = gpsSec;
}

// 1 Hz: ratio, learning, holdover
void GpsDiscipline::tcxoUpdate() {
  if (tcxoCapChannel == nullptr) return;
  int64_t nowUs = esp_timer_get_time();
  if (nowUs - tcxoLastSnapUs < 1000000) return;
  tcxoLastSnapUs = nowUs;

  int64_t tick = 0;
  uint32_t cnt = 0, s;
  int tries = 0;
  for (;;) {
    if (++tries > 8) return;
    s = tcxoSeq;
    if (s & 1) continue;
    __sync_synchronize();
    tick = tcxoTickExt;
    cnt = tcxoCount;
    __sync_synchronize();
    if (tcxoSeq == s) break;
  }

  int oldest = (tcxoRingHead + kTcxoRing - tcxoRingN) % kTcxoRing;
  bool haveBase = tcxoRingN >= 2;
  int64_t baseTick = haveBase ? tcxoRing[oldest].tick : 0;
  uint32_t baseCnt = haveBase ? tcxoRing[oldest].count : 0;
  tcxoRing[tcxoRingHead] = { tick, cnt };
  tcxoRingHead = (tcxoRingHead + 1) % kTcxoRing;
  if (tcxoRingN < kTcxoRing) tcxoRingN++;

  if (haveBase) {
    uint32_t dCnt = cnt - baseCnt;
    if (dCnt == 0) {
      // 32k stopped; restart accumulation
      static bool stallWarned = false;
      if (!stallWarned) {
        stallWarned = true;
        ESP_LOGW(TAG, "32kHz input silent — no TCXO edges (check 32K wiring/pull-up)");
      }
      tcxoRingN = 0;
      tcxoRatioValid = false;
      tcxoHoldoverGood = false;
      return;
    }
    // integer cycles per event
    if (tcxoCyclesPerEvent == 0) {
      if (dCnt < 128) return;
      double kf = (double)(tick - baseTick) / (double)dCnt * 32768.0 / 80000000.0;
      double kr = floor(kf + 0.5);
      if (kr < 1 || kr > 2048 || fabs(kf - kr) > 0.25) {
        ESP_LOGW(TAG, "32kHz event spacing not an integer cycle count (%.3f) — wrong signal?", kf);
        tcxoRingN = 0;
        return;
      }
      tcxoCyclesPerEvent = kr;
      ESP_LOGI(TAG, "TCXO capture calibrated: %.0f cycles/event (%.1f events/s)",
               kr, 32768.0 / kr);
    }
    double tcxoSec = (double)dCnt * tcxoCyclesPerEvent / 32768.0;
    double r = (double)(tick - baseTick) / tcxoSec;
    if (r > 79960000.0 && r < 80040000.0) {   // ±500 ppm sanity
      if (!tcxoRatioValid)
        ESP_LOGI(TAG, "TCXO reference live: crystal at %+.3f ppm vs 32kHz",
                 (r / 80000000.0 - 1.0) * 1e6);
      tcxoApbPerSec = r;
      tcxoRatioValid = true;
    } else {
      tcxoRatioValid = false;
    }
  }

  // learn when disciplined; clamp ±50ppm
  if (tcxoRatioValid && gpsLock && !holdover && fitValid && fitCount >= 120) {
    double e = (fitTicksPerSec / tcxoApbPerSec - 1.0) * 1e6;  // ppm; fast TCXO > 0
    if (e > -50.0 && e < 50.0) {
      tcxoLiveErrPpm = e;
      // residual against pre-fold correction
      double corr;
      bool fromBin;
      if (tcxoCorrPpm(corr, fromBin))
        tcxoResidualPpm += 0.01 * (fabs(e - corr) - tcxoResidualPpm);
      else
        tcxoResidualPpm = 0.5;   // first samples: pessimistic until measured
      if (tcxoGlobalN == 0) tcxoGlobalPpm = e;
      else tcxoGlobalPpm += 0.01 * (e - tcxoGlobalPpm);
      tcxoGlobalN++;
      bool tempFresh = rtcTempUs != 0 && nowUs - rtcTempUs < 300000000LL;
      if (tempFresh) {
        int bin = (int)((rtcTempC - kTcxoTempMin) * 2.0f);
        if (bin >= 0 && bin < kTcxoBins) {
          if (tcxoBinN[bin] == 0) tcxoBinPpm[bin] = (float)e;
          else tcxoBinPpm[bin] += 0.05f * ((float)e - tcxoBinPpm[bin]);
          if (tcxoBinN[bin] < 65535) tcxoBinN[bin]++;
        }
      }
    }
  }

  // holdover frequency, never stale
  double corr;
  bool fromBin;
  if (tcxoRatioValid && tcxoCorrPpm(corr, fromBin)) {
    double fEst = tcxoApbPerSec * (1.0 + corr * 1e-6);
    tcxoHoldoverPpm = (fEst / 80000000.0 - 1.0) * 1e6;
    tcxoCorrFromBin = fromBin;
    tcxoHoldoverGood = true;
  } else {
    tcxoHoldoverGood = false;
  }

  /* serve average frequency since anchor */
  if (!holdover) {
    tcxoHoldStartUs = 0;
  } else if (tcxoHoldoverGood) {
    if (tcxoHoldStartUs == 0) {
      // integral starts at anchor
      tcxoHoldStartUs = (int64_t)lastPpsMonotonicUs;
      tcxoHoldLastUs = tcxoHoldStartUs;
      tcxoHoldPpmSecIntegral = 0;
    }
    tcxoHoldPpmSecIntegral +=
        tcxoHoldoverPpm * (double)(nowUs - tcxoHoldLastUs) / 1e6;
    tcxoHoldLastUs = nowUs;
    double elapsed = (double)(nowUs - tcxoHoldStartUs) / 1e6;
    tcxoAvgPpm = elapsed > 0.5 ? tcxoHoldPpmSecIntegral / elapsed
                               : (double)tcxoHoldoverPpm;
  }
}

bool IRAM_ATTR GpsDiscipline::getRxCapture(uint32_t& capTick, uint32_t& seq) const {
  if (rxCapChannel == nullptr) return false;
  uint32_t s1, s2;
  int tries = 0;
  for (;;) {
    if (++tries > 8) return false;    // bounded, unlike the original
    s1 = rxCapSeq;
    if (s1 & 1) continue;             // ISR mid-publish
    __sync_synchronize();
    capTick = rxCapTick;
    __sync_synchronize();
    s2 = rxCapSeq;
    if (s1 == s2) break;
  }
  seq = s1;
  return s1 != 0;
}

// Capture ticks resolve only +-2^31 ticks (+-26.8 s at 80 MHz) around the
// anchor, so BOTH the anchor's age and the computed delta must be bounded. Without
// that, holdover (which keeps isLocked() true for up to an hour while freezing the
// anchor) silently served receive timestamps wrong by multiples of 53.687 s.
static const int64_t kMaxAnchorAgeUs = 20000000;   // 20 s
static const double kMaxCaptureDeltaTicks = 1.6e9; // ~20 s at 80 MHz

bool IRAM_ATTR GpsDiscipline::captureToNtp(uint32_t capTick, uint32_t& sec1900,
                                 uint32_t& frac) const {
  if (!fitValid) return false;
  uint32_t aSeq1, aSeq2, aTick, aSec, aFrac;
  int64_t aMono;
  int tries = 0;
  for (;;) {
    aSeq1 = anchorSeq;
    if (!(aSeq1 & 1)) {                 // even => not mid-publish
      __sync_synchronize();
      aTick = anchorCapTick; aSec = anchorSec1900; aFrac = anchorFrac;
      aMono = anchorMonoUs;
      __sync_synchronize();
      aSeq2 = anchorSeq;
      if (aSeq1 == aSeq2) break;
    }
    if (++tries > 8) return false;      // bounded, unlike the original
  }
  if (aSeq1 == 0) return false;
  // Reject a stale anchor: in holdover no new anchor is published, and past
  // ~26.8 s the signed tick delta below wraps and lies by 53.687 s.
  if (esp_timer_get_time() - aMono > kMaxAnchorAgeUs) return false;
  // Signed difference handles the 32-bit counter wrap (~53.7 s period), so
  // any capture within +-26 s of the anchor resolves correctly.
  double dTicks = (double)(int32_t)(capTick - aTick);
  if (dTicks > kMaxCaptureDeltaTicks || dTicks < -kMaxCaptureDeltaTicks)
    return false;                                 // beyond unambiguous range
  double dSec = dTicks / fitTicksPerSec;          // GPS-rate seconds
  double frac0 = (double)aFrac / 4294967296.0 + dSec;
  int32_t whole = (int32_t)floor(frac0);
  double rem = frac0 - (double)whole;
  sec1900 = aSec + (uint32_t)whole;
  frac = (uint32_t)(rem * 4294967296.0);
  return true;
}

static bool parse_two(const char* s, int& v) {
  if (!isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[1])) return false;
  v = (s[0]-'0')*10 + (s[1]-'0');
  return true;
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

  outUnixSec = civil_to_unix(fullYear, MM, dd, hh, mm, ss);
  return true;
}


/* Split an NMEA sentence into field pointers, stopping at the checksum. */
static int nmea_fields(const char* line, int len, const char** f, int maxf) {
  int nf = 0;
  f[nf++] = line;
  for (int i = 0; i < len && nf < maxf; ++i) {
    if (line[i] == ',') f[nf++] = &line[i+1];
    if (line[i] == '*') break;
  }
  return nf;
}

/* atof() on a field that may be empty (",,") — returns 0 rather than garbage. */
static float nmea_f(const char* p) {
  if (!p || *p == ',' || *p == '*' || *p == 0) return 0.0f;
  return (float)atof(p);
}
static int nmea_i(const char* p) {
  if (!p || *p == ',' || *p == '*' || *p == 0) return 0;
  return atoi(p);
}


/* NMEA talker prefix -> constellation. GP GPS, GL GLONASS, GA Galileo,
 * GB/BD BeiDou, GQ QZSS. GN is the combined talker and carries no system of
 * its own, so per-SV sentences from it are attributed by SVID range. */
static uint8_t nmea_sys(char a, char b) {
  if (a == 'G' && b == 'P') return GNSS_GPS;
  if (a == 'G' && b == 'L') return GNSS_GLONASS;
  if (a == 'G' && b == 'A') return GNSS_GALILEO;
  if ((a == 'G' && b == 'B') || (a == 'B' && b == 'D')) return GNSS_BEIDOU;
  if (a == 'G' && b == 'Q') return GNSS_QZSS;
  return GNSS_GPS;
}

/* Normalise an NMEA SVID to a per-constellation id, and split SBAS out of the
 * GPS range (33-64 are augmentation satellites, not GPS). */
static void nmea_svid(uint8_t& sys, int& svid) {
  if (sys == GNSS_GPS && svid >= 33 && svid <= 64) { sys = GNSS_SBAS; return; }
  if (sys == GNSS_GLONASS && svid >= 65) { svid -= 64; return; }
}

/* ddmm.mmmm plus hemisphere -> signed degrees. */
static double nmea_latlon(const char* v, const char* hemi) {
  if (!v || *v == ',' || *v == '*' || *v == 0) return 0.0;
  double raw = atof(v);
  int deg = (int)(raw / 100.0);
  double min = raw - deg * 100.0;
  double d = deg + min / 60.0;
  if (hemi && (*hemi == 'S' || *hemi == 'W')) d = -d;
  return d;
}

void GpsDiscipline::satRecord(uint8_t sys, uint8_t svid, int cn0, int elev, int azim) {
  for (int i = 0; i < satBuildN; ++i) {
    if (satBuild[i].gnss == sys && satBuild[i].svid == svid) return;  /* dedupe */
  }
  if (satBuildN >= MAX_SATS) return;
  GpsSatellite& sv = satBuild[satBuildN++];
  sv.gnss = sys;
  sv.svid = svid;
  sv.cn0  = (uint8_t)(cn0 > 0 ? (cn0 > 255 ? 255 : cn0) : 0);
  sv.elev = (int8_t)(elev < -90 ? -90 : (elev > 90 ? 90 : elev));
  sv.azim = (uint16_t)(azim < 0 ? 0 : (azim > 359 ? 359 : azim));
  sv.used = false;
}

/* Called when RMC opens a new cycle: resolve GSA's used-list against the
 * satellites GSV reported, publish, and start the next sweep clean. */
void GpsDiscipline::satCommitCycle() {
  for (int i = 0; i < satBuildN; ++i) {
    for (int u = 0; u < usedN; ++u) {
      if (usedIds[u] == satBuild[i].svid && usedSys[u] == satBuild[i].gnss) {
        satBuild[i].used = true;
        break;
      }
    }
  }
  if (satBuildN > 0) {
    for (int i = 0; i < satBuildN; ++i) satPub[i] = satBuild[i];
    satPubN = satBuildN;
  }
  satBuildN = 0;
  usedN = 0;
}

int GpsDiscipline::getSatellites(GpsSatellite* out, int maxOut) const {
  int n = satPubN;
  if (n > maxOut) n = maxOut;
  for (int i = 0; i < n; ++i) out[i] = satPub[i];
  return n;
}

bool GpsDiscipline::parse_gga_line(const char* line, int len) {
  if (len < 6 || line[0] != '$') return false;
  if (strncmp(line+3, "GGA", 3) != 0) return false;
  if (!verify_nmea_checksum(line, len)) return false;
  const char* f[18];
  int nf = nmea_fields(line, len, f, 18);
  if (nf < 10) return false;
  qFixQuality = (uint8_t)nmea_i(f[6]);
  qLatDeg = nmea_latlon(f[2], f[3]);
  qLonDeg = nmea_latlon(f[4], f[5]);
  qSatsUsed   = (uint8_t)nmea_i(f[7]);
  qHdop       = nmea_f(f[8]);
  qAltitudeM  = nmea_f(f[9]);
  return true;
}

bool GpsDiscipline::parse_gsa_line(const char* line, int len) {
  if (len < 6 || line[0] != '$') return false;
  if (strncmp(line+3, "GSA", 3) != 0) return false;
  if (!verify_nmea_checksum(line, len)) return false;
  const char* f[22];
  int nf = nmea_fields(line, len, f, 22);
  if (nf < 18) return false;
  qFixMode = (uint8_t)nmea_i(f[2]);
  /* ts2phc-go reports the UBX encoding, where "no fix" is 0 rather than 1. */
  qFixType = (uint8_t)(qFixMode >= 2 ? qFixMode : 0);
  qPdop    = nmea_f(f[15]);
  qVdop    = nmea_f(f[17]);

  /* Fields 3..14 are the SVIDs used in the solution. u-blox emits one GSA per
   * constellation, so accumulate across them until RMC ends the cycle. */
  uint8_t sys = nmea_sys(line[1], line[2]);
  for (int i = 3; i <= 14 && i < nf; ++i) {
    int id = nmea_i(f[i]);
    if (id <= 0) continue;
    uint8_t sv = sys;
    nmea_svid(sv, id);
    if (usedN < MAX_SATS) { usedIds[usedN] = (uint8_t)id; usedSys[usedN] = sv; usedN++; }
  }
  return true;
}

bool GpsDiscipline::parse_gsv_line(const char* line, int len) {
  if (len < 6 || line[0] != '$') return false;
  if (strncmp(line+3, "GSV", 3) != 0) return false;
  if (!verify_nmea_checksum(line, len)) return false;
  const char* f[24];
  int nf = nmea_fields(line, len, f, 24);
  if (nf < 4) return false;

  /* Talker tells us the constellation: GP GPS, GL GLONASS, GA Galileo,
   * GB/BD BeiDou. Each constellation reports its own GSV run. */
  uint8_t talker = (uint8_t)((line[1] << 4) ^ line[2]);
  int total = nmea_i(f[1]);
  int msg   = nmea_i(f[2]);
  int inView = nmea_i(f[3]);
  if (msg == 1 || talker != gsvTalker) {
    gsvTalker = talker; gsvSeen = 0; gsvTracked = 0; gsvMax = 0; gsvCn0Sum = 0;
  }
  gsvSeen = (uint8_t)inView;

  /* Up to four satellites per sentence: id, elev, azim, C/N0. */
  uint8_t sysBase = nmea_sys(line[1], line[2]);
  for (int i = 4; i + 3 < nf; i += 4) {
    int cn0 = nmea_i(f[i+3]);
    int id = nmea_i(f[i]);
    if (id > 0) {
      uint8_t sv = sysBase;
      nmea_svid(sv, id);
      satRecord(sv, (uint8_t)id, cn0, nmea_i(f[i+1]), nmea_i(f[i+2]));
    }
    if (cn0 > 0) {
      gsvTracked++;
      gsvCn0Sum = (uint16_t)(gsvCn0Sum + cn0);
      if (cn0 > gsvMax) gsvMax = (uint8_t)cn0;
    }
  }

  if (total > 0 && msg >= total) {
    /* Run complete: publish this constellation's totals. */
    if (line[1] == 'G' && line[2] == 'L')      qSatsGlonass = gsvSeen;
    else if (line[1] == 'G' && line[2] == 'A') qSatsGalileo = gsvSeen;
    else if (line[2] == 'B' || line[1] == 'B') qSatsBeidou  = gsvSeen;
    else                                        qSatsGps     = gsvSeen;
    qSatsTracked = gsvTracked;
    qCn0Max = gsvMax;
    qCn0Mean = gsvTracked ? (uint8_t)(gsvCn0Sum / gsvTracked) : 0;
  }
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
        line[pos] = 0;
        if (GpsDiscipline::parse_rmc_line(line, pos, unixSec)) {
          self->lastNmeaUnixSec = unixSec;
          self->lastNmeaUpdateUs = esp_timer_get_time();
          self->qTimeValid = true;
          /* RMC opens a cycle; GSA/GSV for the previous one are complete. */
          self->satCommitCycle();
        } else {
          /* Quality sentences. Cheap, and off the time path entirely. */
          self->parse_gga_line(line, pos);
          self->parse_gsa_line(line, pos);
          self->parse_gsv_line(line, pos);
        }
      }
      pos = 0;
    } else {
      if (pos < maxLine-1) line[pos++] = (char)ch;
      else pos = 0; // reset on overflow
    }
  }
}

bool IRAM_ATTR GpsDiscipline::getLastPps(uint32_t& sec1900, uint32_t& frac) const {
  if (lastPpsSec1900 == 0) return false;
  sec1900 = lastPpsSec1900;
  frac = lastPpsFrac;
  return true;
}

void GpsDiscipline::handle_pps_deferred() {
  // Seqlock consume: retry while the ISR is mid-publish (odd) or republished
  // under us. Bounded retries so a pathological pulse storm cannot spin here.
  uint64_t ppsEdgeCapture = 0;
  uint32_t capValue = 0;
  uint32_t seq;
  int tries = 0;
  for (;;) {
    if (++tries > 8) return;          // give up; next call retries
    seq = ppsSeq;
    if (seq & 1) continue;            // ISR mid-publish
    if (seq == ppsSeqSeen) return;    // nothing new
    __sync_synchronize();
    capValue = ppsCapValue;
    ppsEdgeCapture = ppsEdgeUs;
    __sync_synchronize();
    if (ppsSeq == seq) break;         // snapshot was consistent
  }
  ppsSeqSeen = seq;

  uint64_t nowUs = esp_timer_get_time();
  /*
   * How long after the hardware capture this handler actually ran. Exported
   * because rms_offset gets blamed on this latency, and the claim is testable:
   * `offset` below is computed from fitResidualTicks / fitTicksPerSec, both of
   * which come only from MCPWM capture ticks and NMEA second labels, so no term
   * in it can depend on this number. Measuring it makes that checkable instead
   * of arguable.
   */
  {
    double latUs = (double)(int64_t)(nowUs - ppsEdgeCapture);
    if (latUs >= 0 && latUs < 1e6) {
      statPpsHandleLatUs += 0.1 * (latUs - statPpsHandleLatUs);
      if (latUs > statPpsHandleLatMaxUs) statPpsHandleLatMaxUs = latUs;
    }
  }
  uint64_t ageUs = nowUs - lastNmeaUpdateUs;
  bool wasLocked = gpsLock;

  if (ageUs < 1500000ULL && lastNmeaUnixSec > 0) {
    time_t ppsSec = lastNmeaUnixSec + 1;

    // --- PPS/NMEA mispair guard ---
    // ppsSec assumes the freshest RMC sentence describes the *previous*
    // second. If the receiver ever emits a sentence late, this PPS can pair
    // with a sentence one second stale and we'd serve time exactly 1s off —
    // confidently, at stratum 1. Once disciplined, the running clock is
    // accurate to microseconds, so any candidate that disagrees by more than
    // 0.5s is a mispair, not a real step. Skip it; holdover covers the gap.
    // If the disagreement persists it's genuine (someone stepped the clock) —
    // accept so we converge instead of free-running forever.
    if (gpsLock) {
      struct timeval curTv;
      gettimeofday(&curTv, nullptr);
      double sysAtEdgeSec = (double)curTv.tv_sec + (double)curTv.tv_usec / 1e6
                          - (double)(int64_t)(esp_timer_get_time() - ppsEdgeCapture) / 1e6;
      double pairDiff = (double)ppsSec - sysAtEdgeSec;
      if (fabs(pairDiff) > 0.5) {
        statMispairCount++;
        mispairStreak++;
        if (mispairStreak < 3) {
          ESP_LOGW(TAG, "PPS/NMEA mispair suspected (%+.3fs step) — pulse skipped (%d/3)",
                   pairDiff, mispairStreak);
          prevPpsCapValue = capValue;   // keep interval tracking continuous
          prevPpsMonotonicUs = ppsEdgeCapture;
          lastAppliedTotalCorrUs = 0;
          return;
        }
        ESP_LOGE(TAG, "%+.3fs step persisted %d pulses — accepting as genuine", pairDiff, mispairStreak);
        // The ring's (second -> tick) pairs straddle the step and would fit a
        // wildly wrong slope; start the model over from the new timescale.
        fitReset();
        tickInit = false;
      }
      mispairStreak = 0;
    }

    // --- PPS outlier rejection gate ---
    // Detect missed or spurious PPS by checking interval sanity.
    // Interval is computed from MCPWM hardware capture ticks (80 MHz)
    // for precise measurement free of ISR latency jitter.
    // Signed 32-bit subtraction handles single counter wraps correctly.
    bool outlier = false;
    double offset = 0;
    if (prevPpsEdgeForOffset != 0) {
      double ppsIntervalUs = (double)(int32_t)(capValue - prevPpsCapValue) / 80.0;
      if (ppsIntervalUs < 900000.0 || ppsIntervalUs > 1100000.0) {
        outlier = true;
        ppsRejectCount++;
        ESP_LOGW(TAG, "PPS outlier rejected: interval=%.0fus (expected ~1000000us)", ppsIntervalUs);
      }
    }

    // A gap longer than the counter's unambiguous range makes the tick delta
    // below alias, so restart the tick timeline and the fit rather than
    // injecting a sample that is wrong by a multiple of 2^32 ticks.
    if (prevPpsMonotonicUs != 0 &&
        (int64_t)(ppsEdgeCapture - prevPpsMonotonicUs) > 20000000LL) {
      ESP_LOGW(TAG, "PPS gap %.1fs exceeds unambiguous range — restarting fit",
               (double)(int64_t)(ppsEdgeCapture - prevPpsMonotonicUs) / 1e6);
      fitReset();
      tickInit = false;
    }

    // --- Phase/frequency model update ---
    // Unwrap the 32-bit capture counter (wraps every ~53.7 s at 80 MHz) onto
    // a monotonic timeline, then refit. The fit's residual for this pulse is
    // the phase error, measured directly rather than reconstructed from the
    // previous interval and the correction we happened to apply last time.
    if (!outlier) {
      if (!tickInit) {
        tickInit = true;
        tickExt = 0;
        tickLastRaw = capValue;
      } else {
        tickExt += (int32_t)(capValue - tickLastRaw);
        tickLastRaw = capValue;
      }
      fitPush((uint32_t)ppsSec, tickExt);
      fitValid = fitSolve();
      tcxoPpsSample(capValue, (uint32_t)ppsSec);
      if (fitValid) {
        offset = fitResidualTicks / fitTicksPerSec;  // seconds, signed
        statFrequencyPpm = (fitTicksPerSec / 80000000.0 - 1.0) * 1e6;
        /*
         * Frequency ramp rate. This is what the endpoint residual of a
         * least-squares line actually responds to: for a fit window T under a
         * constant frequency ramp a, the endpoint residual is a*T^2/12. With
         * T = 240 s that turns a drift of a few hundredths of a ppm per hour —
         * i.e. ordinary room-temperature change — into a residual of hundreds
         * of nanoseconds. Sampled across kDriftLag pulses because the change in
         * slope from one pulse to the next is pure fit noise.
         */
        if (++slopeLagCount >= kDriftLag) {
          if (slopeLagged != 0.0) {
            double dFrac = (fitTicksPerSec - slopeLagged) / 80000000.0;
            statFreqDriftPerSec = dFrac / (double)kDriftLag;
          }
          slopeLagged = fitTicksPerSec;
          slopeLagCount = 0;
        }
        // The fit IS the smoothed estimate; feed dispersion directly.
        if (filteredFrequencyPpm == 0 && statPpsCount > kFitMinSamples) {
          filteredFrequencyPpm = statFrequencyPpm;
        } else {
          filteredFrequencyPpm += kFreqEwmaAlpha * (statFrequencyPpm - filteredFrequencyPpm);
        }
      }
    }

    // The fit residual is the pulse's deviation from the local oscillator's
    // own trend line — a measurement-quality figure. It is NOT the system
    // clock's phase error: settimeofday() cannot move a hardware capture, so
    // feeding it into a servo was open loop. Worse, a linear fit's endpoint
    // residual under a frequency ramp is a*T^2/12, so ordinary thermal drift
    // alone produced a fixed tens-of-microseconds "error" that drove the
    // integrator into its clamp. Report it; do not act on it. Phase is set
    // absolutely from ppsSec below on every pulse, which is the real loop.
    if (!outlier && fitValid) statLastOffsetSec = offset;
    const double proportionalUs = 0;

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

    // Calibration plus GPS flywheel: the served anchor sits at the TCXO-
    // smoothed pulse position, not the raw pulse.
    double adjSec = (double)Config::getPpsCalibrationUs() * 1e-6
                  + (tcxoFwValid ? tcxoFlywheelNs * 1e-9 : 0.0);
    double adjWhole = floor(adjSec);
    // rounding may carry a second
    uint64_t adjFrac64 = (uint64_t)llround((adjSec - adjWhole) * 4294967296.0);
    int64_t adjustedPpsSec = (int64_t)ppsSec + (int64_t)adjWhole
                           + (int64_t)(adjFrac64 >> 32);
    lastPpsSec1900 = unix_to_ntp_seconds((time_t)adjustedPpsSec);
    lastPpsFrac = (uint32_t)adjFrac64;
    // Anchor pair for NTP timestamp extrapolation — must only advance together
    // with lastPpsSec1900/lastPpsFrac (a holdover pulse must not move it).
    lastPpsMonotonicUs = ppsEdgeCapture;
    // Publish the capture-tick anchor for the shared timebase (seqlock).
    anchorSeq = anchorSeq + 1;
    __sync_synchronize();
    anchorCapTick = capValue;
    anchorSec1900 = lastPpsSec1900;
    anchorFrac = lastPpsFrac;
    anchorMonoUs = (int64_t)ppsEdgeCapture;
    __sync_synchronize();
    anchorSeq = anchorSeq + 1;

    if (holdover) {
      ESP_LOGI(TAG, "GPS holdover ended after %.0fs — re-disciplined",
               (double)((int64_t)ppsEdgeCapture - lastGoodPpsUs) / 1e6);
      holdover = false;
    }
    lastGoodPpsUs = (int64_t)ppsEdgeCapture;
    lockExpiredLogged = false;

    gpsLock = true;
    if (!wasLocked) {
      ESP_LOGI(TAG, "GPS locked - PPS disciplining active (latency: %" PRId64 "us)", elapsedUs);
    }

    // RMS offset: only include clean samples that actually carry a measurement
    // (offset stays 0 when the fit is invalid, and averaging that in drives the
    // reported dispersion toward its floor while phase is unmeasured).
    if (!outlier && fitValid) {
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
      double intervalUs = (double)(int32_t)(capValue - prevPpsCapValue) / 80.0;
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
    prevPpsCapValue = capValue;

    // Frequency now comes from the fit above (never reset, so holdover starts
    // from the best estimate the window holds). The old boot-anchored and
    // 300-sample-reset estimators are gone.

    // After warmup, reset the noise accumulators so boot transients don't
    // persist in the reported statistics. The fit ring is deliberately NOT
    // cleared: its samples are already GPS-referenced and valid.
    static const uint32_t kWarmup = 20;
    if (statPpsCount == kWarmup) {
      statRmsOffsetSec = 0;
      filteredRmsOffsetSec = 0;
      ppsIntervalMeanUs = 0;
      ppsJitterVarUs2 = 0;
      prevPpsMonotonicUs = 0;
      statPpsJitterSec = 0;
      clockCorrectionUs = 0;
    }
  } else {
    // PPS with stale/absent NMEA: leave the system clock alone. It is
    // disciplined to the measured frequency error and coasts far better than
    // the old behavior here (truncating tv_usec could inject ~1s of error in
    // one call). The NTP anchor stays frozen at the last good pulse;
    // computeNtpTimestamp extrapolates with frequency feedforward, and loop()
    // declares holdover / expires the lock as the error bound grows.
    prevPpsEdgeForOffset = 0;
    prevPpsCapValue = 0;
    lastAppliedTotalCorrUs = 0;
    mispairStreak = 0;
    // Pulses keep arriving with no valid ToD, so the tick timeline keeps
    // advancing while nothing is pushed to the ring. Restart both so the
    // first re-disciplined pulse cannot alias across the gap.
    fitReset();
    tickInit = false;
    (void)wasLocked;
  }
}

void GpsDiscipline::getStats(GpsStats& out) const {
  out.lastOffsetSec = statLastOffsetSec;
  out.rmsOffsetSec = statRmsOffsetSec;
  out.ppsHandleLatUs = statPpsHandleLatUs;
  out.ppsHandleLatMaxUs = statPpsHandleLatMaxUs;
  out.freqDriftPpmPerHour = statFreqDriftPerSec * 1e6 * 3600.0;
  {
    /* a*T^2/12 with T = the fit window actually in use. */
    double T = (double)fitCount;
    double a = statFreqDriftPerSec < 0 ? -statFreqDriftPerSec : statFreqDriftPerSec;
    out.residualPredictedSec = a * T * T / 12.0;
  }
  out.frequencyPpm = statFrequencyPpm;
  out.ppsJitterSec = statPpsJitterSec;
  out.ppsCount = statPpsCount;
  out.ppsRejectCount = ppsRejectCount;
  out.nmeaMispairCount = statMispairCount;
  out.holdover = (gpsLock && holdover);

  out.fixType     = qFixType;
  out.timeValid   = qTimeValid && qFixType >= 2;
  out.latitudeDeg = qLatDeg;
  out.longitudeDeg = qLonDeg;
  out.fixQuality  = qFixQuality;
  out.fixMode     = qFixMode;
  out.satsUsed    = qSatsUsed;
  out.satsGps     = qSatsGps;
  out.satsGlonass = qSatsGlonass;
  out.satsGalileo = qSatsGalileo;
  out.satsBeidou  = qSatsBeidou;
  out.satsVisible = (uint8_t)(qSatsGps + qSatsGlonass + qSatsGalileo + qSatsBeidou);
  out.satsTracked = qSatsTracked;
  out.cn0Max      = qCn0Max;
  out.cn0Mean     = qCn0Mean;
  out.pdop        = qPdop;
  out.hdop        = qHdop;
  out.vdop        = qVdop;
  out.altitudeM   = qAltitudeM;
  uint64_t nowUs = esp_timer_get_time();
  out.nmeaAgeMs = lastNmeaUpdateUs ? (uint32_t)((nowUs - lastNmeaUpdateUs) / 1000) : 0;
  out.fitValid      = fitValid;
  out.fitSamples    = (uint32_t)fitCount;
  out.fitTicksPerSec = fitValid ? fitTicksPerSec : 0.0;
  out.tcxoValid     = tcxoRatioValid;
  out.tcxoErrPpm    = tcxoLiveErrPpm;
  out.tcxoResidualPpm = tcxoResidualPpm;
  out.tcxoSamples   = tcxoGlobalN;
  out.rtcTempC      = rtcTempC;
  out.tcxoPpsDevNs  = tcxoPpsDevNs;
  out.tcxoPpsRmsNs  = tcxoPpsRmsNs;
  out.tcxoFlywheelNs = tcxoFlywheelNs;
  out.tcxoFlywheelActive = tcxoFwValid;
}

double GpsDiscipline::getFrequencyPpm() const {
  if (holdover && tcxoHoldStartUs != 0) return tcxoAvgPpm;
  return statFrequencyPpm;
}

// versioned NVS blob layout
struct TcxoNvsState {
  uint32_t magic;
  float binPpm[GpsDiscipline::kTcxoBins];
  uint16_t binN[GpsDiscipline::kTcxoBins];
  float globalPpm;
  uint32_t globalN;
};
static const uint32_t kTcxoStateMagic = 0x54435832;  // layout version
static const char* kTcxoNvsNs = "tcxo";
static const char* kTcxoNvsKey = "state";

void GpsDiscipline::tcxoRestoreFromNvs() {
  nvs_handle_t h;
  if (nvs_open(kTcxoNvsNs, NVS_READONLY, &h) != ESP_OK) return;
  TcxoNvsState* st = (TcxoNvsState*)malloc(sizeof(TcxoNvsState));
  if (!st) { nvs_close(h); return; }
  size_t len = sizeof(*st);
  bool ok = nvs_get_blob(h, kTcxoNvsKey, st, &len) == ESP_OK &&
            len == sizeof(*st) && st->magic == kTcxoStateMagic;
  nvs_close(h);
  // reject implausible blob
  if (ok && !(st->globalPpm > -50.0f && st->globalPpm < 50.0f)) ok = false;
  for (int i = 0; ok && i < kTcxoBins; ++i)
    if (st->binN[i] && !(st->binPpm[i] > -50.0f && st->binPpm[i] < 50.0f)) ok = false;
  if (ok) {
    memcpy(tcxoBinPpm, st->binPpm, sizeof(tcxoBinPpm));
    memcpy(tcxoBinN, st->binN, sizeof(tcxoBinN));
    tcxoGlobalPpm = st->globalPpm;
    tcxoGlobalN = st->globalN;
    // pessimistic seed after restore
    tcxoResidualPpm = 0.5;
    ESP_LOGI(TAG, "TCXO tempcomp restored (%u samples)", (unsigned)st->globalN);
  }
  free(st);
}

bool GpsDiscipline::tcxoPersistToNvs() {
  TcxoNvsState* st = (TcxoNvsState*)malloc(sizeof(TcxoNvsState));
  if (!st) return false;
  st->magic = kTcxoStateMagic;
  memcpy(st->binPpm, (const void*)tcxoBinPpm, sizeof(st->binPpm));
  memcpy(st->binN, (const void*)tcxoBinN, sizeof(st->binN));
  st->globalPpm = (float)tcxoGlobalPpm;
  st->globalN = tcxoGlobalN;
  nvs_handle_t h;
  bool ok = false;
  if (nvs_open(kTcxoNvsNs, NVS_READWRITE, &h) == ESP_OK) {
    ok = nvs_set_blob(h, kTcxoNvsKey, st, sizeof(*st)) == ESP_OK &&
         nvs_commit(h) == ESP_OK;
    nvs_close(h);
  }
  free(st);
  return ok;
}

double GpsDiscipline::getRootDispersion() const {
  if (!gpsLock) return 1.0;
  // Grow from the last *disciplined* pulse (not merely the last PPS edge), so
  // holdover — GPS pulsing without valid NMEA included — is reported honestly.
  double sinceGoodSec = (double)(esp_timer_get_time() - lastGoodPpsUs) / 1e6;

  // NTP 16.16 fixed point truncates below 1/65536 (~15.25us) to 0, so a 16us
  // floor keeps dispersion from advertising as exactly 0.000000.
  if (sinceGoodSec <= (double)kFreshPpsUs / 1e6)
    return fabs(filteredRmsOffsetSec) + kDisciplinedDriftPpm * 1e-6 * sinceGoodSec + 16e-6;

  /*
   * Holdover: grow at the rate the frequency ESTIMATE goes wrong, not at the
   * crystal's raw offset. Using the raw offset here claimed ~27.7ppm of drift
   * against a measured 0.075ppm/h, which tripped the 10ms ceiling after 360s
   * and made kHoldoverMaxUs unreachable dead code -- the device gave up after
   * six minutes of a holdover budgeted for an hour, while its real error at
   * that point was about 2us.
   */
  /* TCXO holdover: linear growth */
  if (tcxoHoldoverGood) {
    // unlearned temperature: claim 10x worse
    double floorPpm = tcxoCorrFromBin ? kTcxoHoldoverFloorPpm
                                      : 10.0 * kTcxoHoldoverFloorPpm;
    double rPpm = fmax(tcxoResidualPpm, floorPpm);
    return fabs(filteredRmsOffsetSec) + rPpm * 1e-6 * sinceGoodSec + 16e-6;
  }

  // A linear fit over kFitWin lags a ramp by half its window; the EWMA on top
  // adds ~1/alpha samples. Derived here so retuning either stays consistent.
  const double lagSec = kFitWin / 2.0 + 1.0 / kFreqEwmaAlpha;
  double driftPpmPerSec = fmax(fabs(statFreqDriftPerSec) * 1e6,
                               kHoldoverDriftFloorPpmPerHour / 3600.0);
  double freqErrPpm = driftPpmPerSec * lagSec;
  double growSec = freqErrPpm * 1e-6 * sinceGoodSec
                 + 0.5 * driftPpmPerSec * 1e-6 * sinceGoodSec * sinceGoodSec;
  return fabs(filteredRmsOffsetSec) + growSec + 16e-6;
}

bool GpsDiscipline::isLocked() const {
  if (!gpsLock) return false;
  int64_t sinceGoodUs = esp_timer_get_time() - lastGoodPpsUs;
  if (sinceGoodUs < kFreshPpsUs) return true;
  // Holdover: keep claiming sync while the predicted error stays bounded.
  // TCXO earns longer budget
  if (sinceGoodUs > (tcxoHoldoverGood ? kHoldoverMaxTcxoUs : kHoldoverMaxUs))
    return false;
  return getRootDispersion() < kHoldoverMaxDispersionSec;
}

double GpsDiscipline::getRootDelay() const {
  return 15e-6;
}

void GpsDiscipline::loop() {
  handle_pps_deferred();
  tcxoUpdate();

  if (!gpsLock) return;
  int64_t sinceGoodUs = esp_timer_get_time() - lastGoodPpsUs;
  if (!holdover && sinceGoodUs > kFreshPpsUs) {
    // Covers both stale-NMEA (PPS still pulsing) and PPS-dead outages.
    holdover = true;
    // The fit's anchor is frozen from here on, so hardware RX timestamping must
    // stand down; served time falls back to the esp_timer extrapolation path.
    fitValid = false;
    ESP_LOGW(TAG, "GPS holdover: no discipline for %.1fs — %s (%.3f ppm)",
             (double)sinceGoodUs / 1e6,
             tcxoHoldoverGood ? "tracking DS3231 TCXO" : "coasting on oscillator",
             tcxoHoldoverGood ? (double)tcxoHoldoverPpm : filteredFrequencyPpm);
  }
  if (holdover && !lockExpiredLogged && !isLocked()) {
    lockExpiredLogged = true;
    ESP_LOGE(TAG, "GPS holdover expired — now serving unsynchronized (stratum 16)");
  }
}


