#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include <sys/time.h>
#include <math.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/mcpwm_cap.h"

/* One tracked satellite, as reported by GSV (signal) and GSA (used in fix).
 * Mirrors the per-SV series ts2phc-go exports, so the fleet's dashboards can
 * use one query for every clock. */
struct GpsSatellite {
    uint8_t gnss;    // GnssSys below
    uint8_t svid;    // per-constellation id (GLONASS already de-offset by 64)
    uint8_t cn0;     // dB-Hz; 0 = visible but not tracked
    int8_t  elev;    // degrees above horizon
    uint16_t azim;   // degrees true
    bool used;       // in the navigation solution
};

enum GnssSys : uint8_t { GNSS_GPS = 0, GNSS_SBAS, GNSS_GLONASS, GNSS_GALILEO,
                         GNSS_BEIDOU, GNSS_QZSS, GNSS_COUNT };

/*
 * Wake-on-PPS plumbing, used by the discipline task in app_main. Mirrors the
 * ntp_register_task/ntp_wait_for_packet pair already proven on the NTP task.
 */
void gps_register_task(void* handle);
bool gps_wait_for_pps(uint32_t timeout_ms);

struct GpsStats {
    double lastOffsetSec;     // last PPS-vs-system offset before correction
    double rmsOffsetSec;      // exponentially-weighted RMS
    double frequencyPpm;      // estimated local oscillator freq error
    double ppsJitterSec;      // inter-PPS interval jitter (sigma)
    uint32_t ppsCount;        // total PPS edges received
    uint32_t ppsRejectCount;  // PPS pulses rejected as outliers
    uint32_t nmeaMispairCount;// PPS pulses skipped as PPS/NMEA second mispairs
    bool holdover;            // coasting on the oscillator (GPS lost, still credible)

    /* --- Receiver quality, parsed from GGA/GSA/GSV ---------------------
     * Advisory only: none of this feeds the time path. It exists so a
     * degrading sky view is visible before it becomes a clock problem. */
    uint8_t fixType;          // ts2phc-go convention: 0 none, 2 = 2D, 3 = 3D
    bool timeValid;           // RMC status 'A' and a fix present
    double latitudeDeg;       // signed; 0 until first fix
    double longitudeDeg;
    uint8_t fixQuality;       // GGA: 0 none, 1 GPS, 2 DGPS, 4 RTK fix, 5 RTK float
    uint8_t fixMode;          // GSA: 1 none, 2 = 2D, 3 = 3D
    uint8_t satsUsed;         // GGA: satellites in the position solution
    uint8_t satsVisible;      // GSV: satellites in view, all constellations
    uint8_t satsGps;          // in view, per constellation
    uint8_t satsGlonass;
    uint8_t satsGalileo;
    uint8_t satsBeidou;
    uint8_t satsTracked;      // satellites reporting a non-zero C/N0
    uint8_t cn0Max;           // dB-Hz, strongest tracked signal
    uint8_t cn0Mean;          // dB-Hz, mean over satellites with non-zero C/N0
    float pdop;               // GSA dilution of precision
    float hdop;
    float vdop;
    float altitudeM;          // GGA altitude, metres above MSL
    uint32_t nmeaAgeMs;       // age of the newest valid RMC fix

    /* --- Diagnosis of what rmsOffsetSec actually responds to -----------
     * rmsOffsetSec is the endpoint residual of the capture fit, not a clock
     * phase error. Two things are needed to interpret it, and neither was
     * exported before: how late the handler ran (which is the mechanism it is
     * routinely blamed on) and how fast the oscillator's frequency is ramping
     * (which is the mechanism it actually measures). */
    double ppsHandleLatUs;        // PPS capture -> handler entry, EWMA
    double ppsHandleLatMaxUs;     // worst seen
    double freqDriftPpmPerHour;   // d(fractional frequency)/dt
    double residualPredictedSec;  // |a|*T^2/12 from that drift and the window

    /* --- Capture fit (the term that actually sets served accuracy) ----- */
    bool fitValid;
    uint32_t fitSamples;      // points in the current least-squares window
    double fitTicksPerSec;    // fitted slope; nominal 80 MHz

    /* --- DS3231 TCXO reference (optional; all zero when not fitted) ----- */
    bool tcxoValid;           // 32k flowing, ratio solved
    double tcxoErrPpm;        // TCXO vs GPS, locked
    double tcxoResidualPpm;   // holdover dispersion growth rate
    uint32_t tcxoSamples;     // tempcomp samples learned
    float rtcTempC;           // DS3231 die temperature

    /* PPS vs TCXO, oscillator-free */
    double tcxoPpsDevNs;      // newest pulse vs trend
    double tcxoPpsRmsNs;      // EW-RMS of that deviation
    double tcxoFlywheelNs;    // served-anchor correction
    bool tcxoFlywheelActive;  // PPS filtered through TCXO
};

class GpsDiscipline {
public:
  GpsDiscipline();
  esp_err_t begin(int uartPort, int baud, int txPin, int rxPin, int ppsGpio);
  bool isLocked() const;   // true while disciplined OR in credible holdover
  // Get last PPS in NTP epoch seconds and fractional 32-bit
  bool getLastPps(uint32_t& sec1900, uint32_t& frac) const;
  uint64_t getLastPpsMonotonicUs() const { return lastPpsMonotonicUs; }
  // holdover: live TCXO-referenced frequency
  double getFrequencyPpm() const;
  uint64_t getLastNmeaUpdateUs() const { return lastNmeaUpdateUs; }
  void getStats(GpsStats& out) const;
  /* Copy the published satellite table. Returns the count written. */
  int getSatellites(GpsSatellite* out, int maxOut) const;
  /* NMEA quality sentences. Parsed on the UART task; see GpsStats above. */
  bool parse_gga_line(const char* line, int len);
  bool parse_gsa_line(const char* line, int len);
  bool parse_gsv_line(const char* line, int len);

  // --- Shared hardware timebase -----------------------------------------
  // The MCPWM capture timer that latches PPS also latches the W5500 INTn
  // (packet-arrival) line on a second channel, so both live on one 80 MHz
  // counter and the fit converts either to UTC with no esp_timer hop and no
  // ISR latency in the path.
  esp_err_t beginRxCapture(int intGpio);
  // DS3231 32k as holdover reference
  esp_err_t beginTcxoCapture(int gpio);
  // die temp from I2C owner
  void setRtcTemp(float tempC);

  // learned tempcomp survives reboots
  static const int kTcxoBins = 250;  // -40..85C, 0.5C bins
  void tcxoRestoreFromNvs();
  bool tcxoPersistToNvs();
  uint32_t tcxoSampleCount() const { return tcxoGlobalN; }
  // Latest hardware-latched packet arrival; seq lets the caller detect a new
  // one. Returns false if RX capture is not configured.
  bool getRxCapture(uint32_t& capTick, uint32_t& seq) const;
  // Convert a raw capture tick to NTP epoch, via the PPS fit. Ticks are
  // unwrapped relative to the newest PPS anchor, so the argument must be
  // within ~±26 s of it. Returns false when the fit is not yet valid.
  bool captureToNtp(uint32_t capTick, uint32_t& sec1900, uint32_t& frac) const;
  double getTicksPerSec() const { return fitValid ? fitTicksPerSec : 80000000.0; }
  double getRootDispersion() const;
  double getRootDelay() const;
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
  volatile bool holdover;          // GPS lost but oscillator prediction still credible
  volatile int64_t lastGoodPpsUs;  // monotonic time of last full discipline (PPS + fresh NMEA)
  bool lockExpiredLogged;          // one-shot log gate for holdover expiry
  uint32_t statMispairCount;       // PPS/NMEA mispairs skipped
  uint8_t mispairStreak;           // consecutive mispairs (persistent step accepted)
  volatile uint32_t lastPpsSec1900;
  volatile uint32_t lastPpsFrac;
  volatile uint64_t lastPpsMonotonicUs;
  volatile time_t lastNmeaUnixSec;
  volatile uint64_t lastNmeaUpdateUs;
  // ISR->task handoff. The ISR publishes a snapshot then bumps ppsSeq; the
  // task reads ppsSeq, copies, re-reads ppsSeq and retries if it moved
  // (seqlock). The previous ppsPending flag let the task read a half-updated
  // snapshot if an edge landed mid-copy — 1 Hz made it improbable, not
  // impossible, and a torn capture value is a whole-second error.
  volatile uint32_t ppsSeq;
  volatile uint64_t ppsEdgeUs;
  volatile uint32_t ppsCapValue;
  uint32_t ppsSeqSeen;
  uint32_t prevPpsCapValue;

  // --- Phase/frequency model -------------------------------------------
  // The capture timer counts at 80 MHz and wraps every ~53.7 s, so ticks are
  // unwrapped into a monotonic 64-bit timeline. A least-squares fit of
  // (GPS second -> capture tick) over the window below yields BOTH the
  // oscillator frequency (slope) and the phase error of the newest pulse
  // (its residual), replacing the old scheme where phase was reconstructed
  // from interval deviation plus the last applied correction — bookkeeping
  // that biased phase whenever a correction was skipped, and a frequency
  // estimator that threw away all history every 300 samples.
  static const int kFitWin = 240;      // ~4 min of 1 Hz pulses
  static const int kFitMinSamples = 16;
  struct FitSample { uint32_t sec; int64_t tick; };
  FitSample fitRing[kFitWin];
  int fitHead;
  int fitCount;
  int64_t tickExt;            // unwrapped capture tick
  uint32_t tickLastRaw;
  bool tickInit;
  double fitTicksPerSec;      // slope: capture ticks per GPS second
  double fitResidualTicks;    // newest pulse's deviation from the fit
  bool fitValid;

  /* Receiver-quality state, written by the UART task only. */
  volatile uint8_t qFixQuality = 0, qFixMode = 0, qSatsUsed = 0;
  volatile uint8_t qSatsGps = 0, qSatsGlonass = 0, qSatsGalileo = 0, qSatsBeidou = 0;
  volatile uint8_t qSatsTracked = 0, qCn0Max = 0, qCn0Mean = 0;
  volatile float qPdop = 0, qHdop = 0, qVdop = 0, qAltitudeM = 0;
  volatile uint8_t qFixType = 0;
  volatile bool qTimeValid = false;
  volatile double qLatDeg = 0, qLonDeg = 0;
  /* GSV spans several sentences per constellation; accumulate then commit. */
  uint8_t gsvTalker = 0, gsvSeen = 0, gsvTracked = 0, gsvMax = 0;
  uint16_t gsvCn0Sum = 0;

  /*
   * Satellite table. GSV and GSA arrive as a burst once per NMEA cycle, so
   * fill a build buffer and commit it to the published copy when RMC opens the
   * next cycle. Readers therefore always see one whole consistent sweep.
   */
  static const int MAX_SATS = 40;
  GpsSatellite satBuild[MAX_SATS] = {};
  int satBuildN = 0;
  GpsSatellite satPub[MAX_SATS] = {};
  volatile int satPubN = 0;
  uint8_t usedIds[MAX_SATS] = {};
  uint8_t usedSys[MAX_SATS] = {};
  int usedN = 0;
  void satCommitCycle();
  void satRecord(uint8_t sys, uint8_t svid, int cn0, int elev, int azim);

  void fitPush(uint32_t gpsSec, int64_t tick);
  void fitReset();
  bool fitSolve();

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
  double clockCorrectionUs;      // integral servo correction applied to settimeofday
  uint64_t prevPpsEdgeForOffset;  // previous PPS edge for analytical offset
  double lastAppliedTotalCorrUs;  // total correction applied at previous settimeofday
  /* Diagnosis of the fit-residual metric (see GpsStats). */
  double statPpsHandleLatUs;
  double statPpsHandleLatMaxUs;
  double statFreqDriftPerSec;     // fractional frequency change per second
  double slopeLagged;             // fitTicksPerSec kDriftLag pulses ago
  uint32_t slopeLagCount;
  static const uint32_t kDriftLag = 60;   // 1 min: long enough to beat fit noise

  double filteredFrequencyPpm;   // EWMA-smoothed frequency for dispersion calc
  double filteredRmsOffsetSec;   // outlier-immune RMS for dispersion calc

  // MCPWM capture handles
  mcpwm_cap_timer_handle_t capTimer;
  mcpwm_cap_channel_handle_t capChannel;
  mcpwm_cap_channel_handle_t rxCapChannel;   // W5500 INTn, packet arrival
  volatile uint32_t rxCapTick;
  volatile uint32_t rxCapSeq;

  // --- DS3231 TCXO reference (optional) ----------------------------------
  // ISR side, seqlock published
  mcpwm_cap_channel_handle_t tcxoCapChannel;
  volatile uint32_t tcxoSeq;
  volatile int64_t tcxoTickExt;
  volatile uint32_t tcxoCount;
  volatile uint32_t tcxoRawPub;      // newest 32k capture tick
  uint32_t tcxoLastRaw;              // ISR-only
  bool tcxoTickInit;                 // ISR-only
  // 1 Hz snapshot ring
  static const int kTcxoRing = 17;
  struct TcxoSnap { int64_t tick; uint32_t count; };
  TcxoSnap tcxoRing[kTcxoRing];
  int tcxoRingHead, tcxoRingN;
  int64_t tcxoLastSnapUs;
  // cycles per event, runtime calibrated
  double tcxoCyclesPerEvent;
  double tcxoApbPerSec;              // APB ticks per TCXO-second
  bool tcxoRatioValid;
  volatile float rtcTempC;
  volatile int64_t rtcTempUs;        // when it was last fed
  // learned error per temp bin
  float tcxoBinPpm[kTcxoBins];
  uint16_t tcxoBinN[kTcxoBins];
  double tcxoGlobalPpm;
  uint32_t tcxoGlobalN;
  double tcxoLiveErrPpm;             // newest measured error while locked
  double tcxoResidualPpm;            // EWMA |error - correction|
  volatile double tcxoHoldoverPpm;   // instantaneous crystal-vs-TCXO estimate
  volatile bool tcxoHoldoverGood;
  volatile bool tcxoCorrFromBin;     // correction from populated bin
  // average frequency since anchor
  int64_t tcxoHoldStartUs;           // 0 = not integrating
  int64_t tcxoHoldLastUs;
  double tcxoHoldPpmSecIntegral;     // ppm seconds since anchor
  volatile double tcxoAvgPpm;        // integral / elapsed
  // PPS-vs-TCXO metric state
  bool tcxoPpsPrevValid;
  uint32_t tcxoPpsPrevCnt;
  int32_t tcxoPpsPrevSub;
  uint32_t tcxoPpsPrevSec;
  bool tcxoPpsFreqInit;
  double tcxoPpsFreqNsPerS;          // detrend EWMA, ns/s
  double tcxoPpsDevNs;
  double tcxoPpsRmsNs;
  // GPS filter: TCXO carries phase
  double tcxoFlywheelNs;             // smoothed minus raw pulse
  uint32_t tcxoFwSettle;
  volatile bool tcxoFwValid;
  void tcxoPpsSample(uint32_t ppsCapTick, uint32_t gpsSec);
  void tcxoUpdate();                 // 1 Hz, from loop()
  bool tcxoCorrPpm(double& out, bool& fromBin) const;
  static bool IRAM_ATTR tcxo_capture_callback(mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t* edata, void* ctx);

  // Anchor tying the newest disciplined pulse to both timescales, published
  // together so a reader always gets a consistent pair.
  volatile uint32_t anchorSeq;
  volatile uint32_t anchorCapTick;   // raw capture tick of that pulse
  volatile uint32_t anchorSec1900;   // NTP second it represents
  volatile uint32_t anchorFrac;      // calibration offset within that second
  volatile int64_t anchorMonoUs;     // esp_timer at that pulse, for staleness

  static bool IRAM_ATTR rx_capture_callback(mcpwm_cap_channel_handle_t ch, const mcpwm_capture_event_data_t* edata, void* ctx);

  static void uart_task(void* arg);
  static bool parse_rmc_line(const char* line, int len, time_t& outUnixSec);
  static int from_hex(char c);
  static bool verify_nmea_checksum(const char* line, int len);
  static bool IRAM_ATTR pps_capture_callback(mcpwm_cap_channel_handle_t cap_channel, const mcpwm_capture_event_data_t* edata, void* user_ctx);
  void handle_pps_deferred();
};


