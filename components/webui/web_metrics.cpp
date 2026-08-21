// SPDX-License-Identifier: Unlicense

#include "web_internal.h"
#include "config.h"
#include "config_store.h"
#include "gps.h"
#include "ntp_server.h"
#include "ntp_prof.h"
#include "w5500_eth.h"
#include "wifi_sta.h"
#include "w5k_tcp_wrapper.h"
#include "w5500_drv.h"
#include "w5500_dhcp.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <stdlib.h>
#include <fcntl.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"


extern volatile uint32_t g_mainLoopBeats;
extern uint32_t g_bootCount;
extern volatile int g_rtcPresent;
extern volatile float g_rtcOffsetSec;

void WebServer::sendMetrics() {
  GpsStats gs = {};
  if (gps) gps->getStats(gs);
  bool locked = (gps && gps->isLocked());
  uint32_t reqCount = ntp ? ntp->getRequestCount() : 0;
  uint32_t rxIrqCount = ntp ? ntp->getRxIrqCount() : 0;
  uint32_t primeSkips = ntp ? ntp->getPrimeSkips() : 0;
  uint32_t capRejects = ntp ? ntp->getCapRejects() : 0;
  uint32_t lateOk = ntp ? ntp->getLateStampOk() : 0;
  uint32_t ilServed = ntp ? ntp->getInterleavedServed() : 0;
  uint32_t lateFb = ntp ? ntp->getLateStampFallbacks() : 0;
  int lastStageRc = ntp ? ntp->getLastStageRc() : 0;
  int lastWrDelta = ntp ? ntp->getLastWrDelta() : 0;
  uint32_t turnN = ntp ? ntp->getTurnSamples() : 0;
  double turnUs = ntp ? ntp->getTurnUs() : 0.0;
  double txCorrUs = ntp ? ntp->getTxCorrectionUs() : 0.0;
  double uptimeSec = (double)esp_timer_get_time() / 1e6;
  int stratum = locked ? 1 : 16;
  double rootDispersion = gps ? gps->getRootDispersion() : 1.0;

  // Diagnostics (read-only; not on the timekeeping path).
  int resetReason     = (int)esp_reset_reason();
  uint32_t freeHeap   = (uint32_t)esp_get_free_heap_size();
  uint32_t minFreeHeap= (uint32_t)esp_get_minimum_free_heap_size();
  uint32_t loopBeats  = g_mainLoopBeats;
  uint32_t bootCount  = g_bootCount;
  int ethLink         = eth ? (eth->isLinkUp() ? 1 : 0) : -1;
  int w5500Ver        = eth ? (int)eth->readVersion() : -1;
  uint32_t chipResets = eth ? eth->getChipResetCount() : 0;

  char* resp = g_resp;
  char* body = resp + hdrReserve;
  int blen = snprintf(body, sizeof(g_resp) - hdrReserve,
    "# HELP ntp_clock_offset_seconds Last measured clock offset\n"
    "# TYPE ntp_clock_offset_seconds gauge\n"
    "ntp_clock_offset_seconds %.9f\n"
    "# HELP ntp_rms_offset_seconds EW-RMS of the capture-fit endpoint residual. NOT a clock phase error: it is how far the newest PPS deviates from the oscillator's own 240 s trend line, so it responds to frequency RAMP as a*T^2/12. See ntp_gps_freq_drift_ppm_per_hour and ntp_gps_residual_predicted_seconds.\n"
    "# TYPE ntp_rms_offset_seconds gauge\n"
    "ntp_rms_offset_seconds %.9f\n"
    "# HELP ntp_frequency_ppm Estimated frequency error\n"
    "# TYPE ntp_frequency_ppm gauge\n"
    "ntp_frequency_ppm %.6f\n"
    "# HELP ntp_pps_jitter_seconds PPS pulse jitter\n"
    "# TYPE ntp_pps_jitter_seconds gauge\n"
    "ntp_pps_jitter_seconds %.9f\n"
    "# HELP ntp_root_dispersion_seconds Estimated root dispersion\n"
    "# TYPE ntp_root_dispersion_seconds gauge\n"
    "ntp_root_dispersion_seconds %.6f\n"
    "# HELP ntp_gps_lock GPS lock status\n"
    "# TYPE ntp_gps_lock gauge\n"
    "ntp_gps_lock %d\n"
    "# HELP ntp_stratum NTP stratum\n"
    "# TYPE ntp_stratum gauge\n"
    "ntp_stratum %d\n"
    "# HELP ntp_uptime_seconds Seconds since boot\n"
    "# TYPE ntp_uptime_seconds gauge\n"
    "ntp_uptime_seconds %.1f\n"
    "# HELP ntp_requests_total Total NTP requests served\n"
    "# TYPE ntp_requests_total counter\n"
    "ntp_requests_total %" PRIu32 "\n"
    "# HELP ntp_pps_count Total PPS edges received\n"
    "# TYPE ntp_pps_count counter\n"
    "ntp_pps_count %" PRIu32 "\n"
    "# HELP ntp_pps_rejects_total PPS pulses rejected as outliers\n"
    "# TYPE ntp_pps_rejects_total counter\n"
    "ntp_pps_rejects_total %" PRIu32 "\n"
    "# HELP ntp_nmea_mispair_total PPS pulses skipped as suspected PPS/NMEA second mispairs\n"
    "# TYPE ntp_nmea_mispair_total counter\n"
    "ntp_nmea_mispair_total %" PRIu32 "\n"
    "# HELP ntp_gps_holdover 1 when coasting on the oscillator (GPS lost, sync still credible)\n"
    "# TYPE ntp_gps_holdover gauge\n"
    "ntp_gps_holdover %d\n"
    "# HELP ntp_interleaved_served_total Replies answered in interleaved mode (RFC 9769), carrying the measured transmit time of the previous reply instead of a predicted one\n"
    "# TYPE ntp_interleaved_served_total counter\n"
    "ntp_interleaved_served_total %" PRIu32 "\n"
    "# HELP ntp_tx_late_stamp_total Replies sent with t3 patched in place before SEND\n"
    "# TYPE ntp_tx_late_stamp_total counter\n"
    "# HELP ntp_tx_late_stamp_fallback_total Replies that fell back to the library send\n"
    "# TYPE ntp_tx_late_stamp_fallback_total counter\n"
    "# HELP ntp_rx_irq_total W5500 RX interrupts captured (hardware arrival edges)\n"
    "# TYPE ntp_rx_irq_total counter\n"
    "ntp_tx_late_stamp_total %" PRIu32 "\n"
    "ntp_tx_late_stamp_fallback_total %" PRIu32 "\n"
    "ntp_tx_stage_last_rc %d\n"
    "ntp_tx_wr_delta %d\n"
    "ntp_tx_turnaround_us %.2f\n"
    "ntp_tx_turnaround_samples %" PRIu32 "\n"
    "ntp_rx_irq_total %" PRIu32 "\n"
    "# HELP ntp_tx_correction_us Self-calibrated transmit-path correction added to t3\n"
    "# TYPE ntp_tx_correction_us gauge\n"
    "ntp_tx_correction_us %.1f\n"
    "# HELP ntp_reset_reason esp_reset_reason() of the last boot (1=POWERON,4=SW,7=TASK_WDT,8=INT_WDT,9=BROWNOUT)\n"
    "# TYPE ntp_reset_reason gauge\n"
    "ntp_reset_reason %d\n"
    "# HELP ntp_boot_count Boots since flash (persisted in NVS; a jump = auto-recovery)\n"
    "# TYPE ntp_boot_count counter\n"
    "ntp_boot_count %" PRIu32 "\n"
    "# HELP ntp_main_loop_beats Core-0 main-loop iterations (liveness heartbeat)\n"
    "# TYPE ntp_main_loop_beats counter\n"
    "ntp_main_loop_beats %" PRIu32 "\n"
    "# HELP ntp_free_heap_bytes Current free heap\n"
    "# TYPE ntp_free_heap_bytes gauge\n"
    "ntp_free_heap_bytes %" PRIu32 "\n"
    "# HELP ntp_min_free_heap_bytes Lowest free heap since boot\n"
    "# TYPE ntp_min_free_heap_bytes gauge\n"
    "ntp_min_free_heap_bytes %" PRIu32 "\n"
    "# HELP ntp_eth_link_up W5500 link health (1=up,0=down,-1=n/a)\n"
    "# TYPE ntp_eth_link_up gauge\n"
    "ntp_eth_link_up %d\n"
    "# HELP ntp_w5500_version W5500 VERSIONR (4=healthy; other=wedged SPI; -1=n/a)\n"
    "# TYPE ntp_w5500_version gauge\n"
    "ntp_w5500_version %d\n"
    "# HELP ntp_w5500_chip_resets_total W5500 register-loss events recovered in place (chip reset without a device reboot)\n"
    "# TYPE ntp_w5500_chip_resets_total counter\n"
    "ntp_w5500_chip_resets_total %" PRIu32 "\n"
    "# HELP ntp_w5500_mgmt_spin_timeouts_total Bounded management-path register waits that expired (chip stopped acknowledging commands)\n"
    "# TYPE ntp_w5500_mgmt_spin_timeouts_total counter\n"
    "ntp_w5500_mgmt_spin_timeouts_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_lease_seconds Lease duration granted by the DHCP server\n"
    "# TYPE ntp_dhcp_lease_seconds gauge\n"
    "ntp_dhcp_lease_seconds %" PRIu32 "\n"
    "# HELP ntp_dhcp_acks_total DHCP ACKs bound (acquisitions and renewals)\n"
    "# TYPE ntp_dhcp_acks_total counter\n"
    "ntp_dhcp_acks_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_naks_total DHCP NAKs received\n"
    "# TYPE ntp_dhcp_naks_total counter\n"
    "ntp_dhcp_naks_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_timeouts_total DHCP per-phase resend/expiry events\n"
    "# TYPE ntp_dhcp_timeouts_total counter\n"
    "ntp_dhcp_timeouts_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_renews_total Unicast renewal REQUESTs sent at T1\n"
    "# TYPE ntp_dhcp_renews_total counter\n"
    "ntp_dhcp_renews_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_reclaims_total Restarts that reclaimed the cached lease instead of rediscovering\n"
    "# TYPE ntp_dhcp_reclaims_total counter\n"
    "ntp_dhcp_reclaims_total %" PRIu32 "\n"
    "# HELP ntp_gps_fix_quality GGA fix quality (0 none, 1 GPS, 2 DGPS, 4 RTK fix, 5 RTK float)\n"
    "# TYPE ntp_gps_fix_quality gauge\n"
    "ntp_gps_fix_quality %d\n"
    "# HELP gps_fix_type GNSS fix type (0=none,2=2D,3=3D)\n"
    "# TYPE gps_fix_type gauge\n"
    "gps_fix_type %d\n"
    "# HELP gps_satellites_used Satellites in the position solution\n"
    "# TYPE gps_satellites_used gauge\n"
    "gps_satellites_used %d\n"
    "# HELP ntp_gps_satellites_visible Satellites in view across all constellations\n"
    "# TYPE ntp_gps_satellites_visible gauge\n"
    "ntp_gps_satellites_visible %d\n"
    "# HELP ntp_gps_satellites_in_view Satellites in view, by constellation\n"
    "# TYPE ntp_gps_satellites_in_view gauge\n"
    "ntp_gps_satellites_in_view{constellation=\"gps\"} %d\n"
    "ntp_gps_satellites_in_view{constellation=\"glonass\"} %d\n"
    "ntp_gps_satellites_in_view{constellation=\"galileo\"} %d\n"
    "ntp_gps_satellites_in_view{constellation=\"beidou\"} %d\n"
    "# HELP ntp_gps_satellites_tracked Satellites reporting a non-zero C/N0\n"
    "# TYPE ntp_gps_satellites_tracked gauge\n"
    "ntp_gps_satellites_tracked %d\n"
    "# HELP ntp_gps_cn0_max_db Strongest tracked signal, dB-Hz\n"
    "# TYPE ntp_gps_cn0_max_db gauge\n"
    "ntp_gps_cn0_max_db %d\n"
    "# HELP ntp_gps_cn0_mean_db Mean C/N0 over tracked satellites, dB-Hz\n"
    "# TYPE ntp_gps_cn0_mean_db gauge\n"
    "ntp_gps_cn0_mean_db %d\n"
    "# HELP gps_time_valid RMC reports a valid fix\n"
    "# TYPE gps_time_valid gauge\n"
    "gps_time_valid %d\n"
    "# HELP gps_latitude_degrees\n"
    "# TYPE gps_latitude_degrees gauge\n"
    "gps_latitude_degrees %.7f\n"
    "# HELP gps_longitude_degrees\n"
    "# TYPE gps_longitude_degrees gauge\n"
    "gps_longitude_degrees %.7f\n"
    "# HELP gps_pdop Position dilution of precision\n"
    "# TYPE gps_pdop gauge\n"
    "gps_pdop %.2f\n"
    "# HELP gps_hdop Horizontal dilution of precision\n"
    "# TYPE gps_hdop gauge\n"
    "gps_hdop %.2f\n"
    "# HELP gps_vdop Vertical dilution of precision\n"
    "# TYPE gps_vdop gauge\n"
    "gps_vdop %.2f\n"
    "# HELP gps_altitude_msl_meters GGA altitude above mean sea level\n"
    "# TYPE gps_altitude_msl_meters gauge\n"
    "gps_altitude_msl_meters %.1f\n"
    "# HELP ntp_gps_nmea_age_seconds Age of the newest valid RMC fix\n"
    "# TYPE ntp_gps_nmea_age_seconds gauge\n"
    "ntp_gps_nmea_age_seconds %.3f\n"
    "# HELP ntp_arp_prime_skips_total Replies that reused the socket's resolved peer\n"
    "# TYPE ntp_arp_prime_skips_total counter\n"
    "ntp_arp_prime_skips_total %" PRIu32 "\n"
    "# HELP ntp_capture_rejects_total Arrival captures rejected as mis-paired\n"
    "# TYPE ntp_capture_rejects_total counter\n"
    "ntp_capture_rejects_total %" PRIu32 "\n"
    "# HELP ts2phc_offset_ns PPS offset against the capture clock, nanoseconds\n"
    "# TYPE ts2phc_offset_ns gauge\n"
    "ts2phc_offset_ns{clock=\"mcpwm0\"} %.1f\n"
    "# HELP ts2phc_freq_ppb Capture-clock frequency error, parts per billion\n"
    "# TYPE ts2phc_freq_ppb gauge\n"
    "ts2phc_freq_ppb{clock=\"mcpwm0\"} %.1f\n"
    "# HELP ntp_gps_fit_valid Capture-to-GPS-second least-squares fit is solved\n"
    "# TYPE ntp_gps_fit_valid gauge\n"
    "ntp_gps_fit_valid %d\n"
    "# HELP ntp_gps_fit_samples Points in the current fit window\n"
    "# TYPE ntp_gps_fit_samples gauge\n"
    "ntp_gps_fit_samples %" PRIu32 "\n"
    "# HELP ntp_gps_fit_ticks_per_second Fitted capture-timer rate (nominal 80e6); 0 while unsolved\n"
    "# TYPE ntp_gps_fit_ticks_per_second gauge\n"
    "ntp_gps_fit_ticks_per_second %.3f\n"
    "# HELP ntp_pps_handle_latency_us PPS hardware capture to discipline-handler entry\n"
    "# TYPE ntp_pps_handle_latency_us gauge\n"
    "ntp_pps_handle_latency_us %.1f\n"
    "# HELP ntp_pps_handle_latency_max_us Worst PPS capture-to-handler latency seen\n"
    "# TYPE ntp_pps_handle_latency_max_us gauge\n"
    "ntp_pps_handle_latency_max_us %.1f\n"
    "# HELP ntp_gps_freq_drift_ppm_per_hour Rate of change of the fitted oscillator frequency\n"
    "# TYPE ntp_gps_freq_drift_ppm_per_hour gauge\n"
    "ntp_gps_freq_drift_ppm_per_hour %.4f\n"
    "# HELP ntp_gps_residual_predicted_seconds Fit endpoint residual predicted from that drift alone (a*T^2/12); compare against ntp_rms_offset_seconds\n"
    "# TYPE ntp_gps_residual_predicted_seconds gauge\n"
    "ntp_gps_residual_predicted_seconds %.9f\n",
    gs.lastOffsetSec,
    gs.rmsOffsetSec,
    gs.frequencyPpm,
    gs.ppsJitterSec,
    rootDispersion,
    locked ? 1 : 0,
    stratum,
    uptimeSec,
    reqCount,
    gs.ppsCount,
    gs.ppsRejectCount,
    gs.nmeaMispairCount,
    gs.holdover ? 1 : 0,
    ilServed,
    lateOk,
    lateFb,
    lastStageRc,
    lastWrDelta,
    turnUs,
    turnN,
    rxIrqCount,
    txCorrUs,
    resetReason,
    bootCount,
    loopBeats,
    freeHeap,
    minFreeHeap,
    ethLink,
    w5500Ver,
    chipResets,
    w5500_mgmt_spin_timeouts(),
    w5500_dhcp_lease_seconds(),
    w5500_dhcp_acks_total(),
    w5500_dhcp_naks_total(),
    w5500_dhcp_timeouts_total(),
    w5500_dhcp_renews_total(),
    w5500_dhcp_reclaims_total(),

    gs.fixQuality,
    gs.fixType,
    gs.satsUsed,
    gs.satsVisible,
    gs.satsGps,
    gs.satsGlonass,
    gs.satsGalileo,
    gs.satsBeidou,
    gs.satsTracked,
    gs.cn0Max,
    gs.cn0Mean,
    gs.timeValid ? 1 : 0,
    gs.latitudeDeg,
    gs.longitudeDeg,
    gs.pdop,
    gs.hdop,
    gs.vdop,
    gs.altitudeM,
    gs.nmeaAgeMs / 1000.0,
    primeSkips,
    capRejects,
    gs.lastOffsetSec * 1e9,
    gs.fitValid ? (gs.fitTicksPerSec / 80000000.0 - 1.0) * 1e9 : 0.0,
    gs.fitValid ? 1 : 0,
    gs.fitSamples,
    gs.fitTicksPerSec,
    gs.ppsHandleLatUs,
    gs.ppsHandleLatMaxUs,
    gs.freqDriftPpmPerHour,
    gs.residualPredictedSec);

  /* DS3231/TCXO metrics */
  if (blen > 0 && blen < (int)(sizeof(g_resp) - hdrReserve) - 1600) {
    int cap = (int)(sizeof(g_resp) - hdrReserve);
    blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
      "# HELP ntp_rtc_present DS3231 fitted and answering (-1 not configured, 0 not answering)\n"
      "# TYPE ntp_rtc_present gauge\n"
      "ntp_rtc_present %d\n"
      "# HELP ntp_rtc_temp_celsius DS3231 die temperature (its own tempcomp input)\n"
      "# TYPE ntp_rtc_temp_celsius gauge\n"
      "ntp_rtc_temp_celsius %.2f\n"
      "# HELP ntp_rtc_offset_seconds Battery-backed RTC minus system time, whole seconds (the RTC is boundary-aligned at write-back, so sub-second drift reads 0)\n"
      "# TYPE ntp_rtc_offset_seconds gauge\n"
      "ntp_rtc_offset_seconds %.3f\n"
      "# HELP ntp_tcxo_valid 32kHz captures flowing and the APB-vs-TCXO ratio solved\n"
      "# TYPE ntp_tcxo_valid gauge\n"
      "ntp_tcxo_valid %d\n"
      "# HELP ntp_tcxo_err_ppm Measured DS3231 TCXO error against GPS (newest, locked only)\n"
      "# TYPE ntp_tcxo_err_ppm gauge\n"
      "ntp_tcxo_err_ppm %.4f\n"
      "# HELP ntp_tcxo_residual_ppm EWMA |error - learned correction|; TCXO-holdover dispersion grows at this rate\n"
      "# TYPE ntp_tcxo_residual_ppm gauge\n"
      "ntp_tcxo_residual_ppm %.4f\n"
      "# HELP ntp_tcxo_samples Locked seconds folded into the learned tempcomp\n"
      "# TYPE ntp_tcxo_samples counter\n"
      "ntp_tcxo_samples %" PRIu32 "\n"
      "# HELP ntp_pps_vs_tcxo_ns Newest PPS deviation from the TCXO tick stream, frequency-detrended. No local oscillator in the loop: this is the pulse itself plus 12.5 ns capture quantisation.\n"
      "# TYPE ntp_pps_vs_tcxo_ns gauge\n"
      "ntp_pps_vs_tcxo_ns %.1f\n"
      "# HELP ntp_pps_vs_tcxo_rms_ns EW-RMS of ntp_pps_vs_tcxo_ns; compare against ntp_rms_offset_seconds, which mixes in crystal thermal ramp\n"
      "# TYPE ntp_pps_vs_tcxo_rms_ns gauge\n"
      "ntp_pps_vs_tcxo_rms_ns %.1f\n"
      "# HELP ntp_pps_flywheel_ns Served-anchor correction: TCXO-smoothed pulse position minus the raw pulse\n"
      "# TYPE ntp_pps_flywheel_ns gauge\n"
      "ntp_pps_flywheel_ns %.1f\n"
      "# HELP ntp_pps_flywheel_active 1 when served time rides the smoothed pulse (~30 s averaging) instead of each raw pulse\n"
      "# TYPE ntp_pps_flywheel_active gauge\n"
      "ntp_pps_flywheel_active %d\n",
      g_rtcPresent,
      gs.rtcTempC,
      (double)g_rtcOffsetSec,
      gs.tcxoValid ? 1 : 0,
      gs.tcxoErrPpm,
      gs.tcxoResidualPpm,
      gs.tcxoSamples,
      gs.tcxoPpsDevNs,
      gs.tcxoPpsRmsNs,
      gs.tcxoFlywheelNs,
      gs.tcxoFlywheelActive ? 1 : 0);
  }

  /*
   * Reply-path attribution. Labelled series rather than one metric per span so
   * a single query renders the whole budget, and so adding a checkpoint does
   * not change the metric namespace.
   */
  if (blen > 0) {
    int cap = (int)(sizeof(g_resp) - hdrReserve);
    if (blen > cap) blen = cap;   /* restore invariant after overflow */
    blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
      "# HELP ntp_path_span_us Honest software-timed reply-path spans (EWMA)\n"
      "# TYPE ntp_path_span_us gauge\n");
    for (int i = 0; i < NTP_PROF_COUNT && blen < cap - 128; ++i)
      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "ntp_path_span_us{span=\"%s\"} %.3f\n",
        ntp_prof_name(i), ntp_prof_ewma_us(i));
    blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
      "# HELP ntp_path_span_max_us Worst observed value of each span\n"
      "# TYPE ntp_path_span_max_us gauge\n");
    for (int i = 0; i < NTP_PROF_COUNT && blen < cap - 128; ++i)
      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "ntp_path_span_max_us{span=\"%s\"} %.3f\n",
        ntp_prof_name(i), ntp_prof_max_us(i));
    blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
      "# HELP ntp_path_span_min_us Best observed value of each span (the floor)\n"
      "# TYPE ntp_path_span_min_us gauge\n");
    for (int i = 0; i < NTP_PROF_COUNT && blen < cap - 128; ++i)
      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "ntp_path_span_min_us{span=\"%s\"} %.3f\n",
        ntp_prof_name(i), ntp_prof_min_us(i));
    blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
      "# HELP ntp_int_to_send_us Arrival edge to SEND accepted (the honest turnaround)\n"
      "# TYPE ntp_int_to_send_us gauge\n"
      "ntp_int_to_send_us %.3f\n"
      "# HELP ntp_path_samples Packets folded into the span EWMAs\n"
      "# TYPE ntp_path_samples counter\n"
      "ntp_path_samples %" PRIu32 "\n"
      "# HELP ntp_spi_txns_per_reply SPI transactions per loop() body, ARP prime INCLUDED\n"
      "# TYPE ntp_spi_txns_per_reply gauge\n"
      "ntp_spi_txns_per_reply %.2f\n"
      "# HELP ntp_spi_txns_per_reply_serve SPI transactions for the reply itself, ARP prime excluded\n"
      "# TYPE ntp_spi_txns_per_reply_serve gauge\n"
      "ntp_spi_txns_per_reply_serve %.2f\n"
      "# HELP ntp_spi_txns_per_prime SPI transactions spent inside w5k_arp_prime (mostly Sn_IR polls, so this RISES as SPI gets faster)\n"
      "# TYPE ntp_spi_txns_per_prime gauge\n"
      "ntp_spi_txns_per_prime %.2f\n"
      "# HELP ntp_spi_bytes_per_reply Bytes clocked per loop() body, headers included\n"
      "# TYPE ntp_spi_bytes_per_reply gauge\n"
      "ntp_spi_bytes_per_reply %.2f\n"
      "# HELP ntp_spi_bytes_per_reply_serve Bytes clocked for the reply itself, ARP prime excluded\n"
      "# TYPE ntp_spi_bytes_per_reply_serve gauge\n"
      "ntp_spi_bytes_per_reply_serve %.2f\n"
      "# HELP ntp_spi_selects_per_reply wizchip_select() calls per served reply\n"
      "# TYPE ntp_spi_selects_per_reply gauge\n"
      "ntp_spi_selects_per_reply %.2f\n"
      "# HELP ntp_spi_reap_polls_per_reply Sn_IR reads spent spinning on SENDOK\n"
      "# TYPE ntp_spi_reap_polls_per_reply gauge\n"
      "ntp_spi_reap_polls_per_reply %.2f\n"
      "# HELP ntp_spi_prime_polls_per_reply Sn_IR reads spent in w5k_arp_prime\n"
      "# TYPE ntp_spi_prime_polls_per_reply gauge\n"
      "ntp_spi_prime_polls_per_reply %.2f\n"
      "# HELP ntp_arp_primes_total ARP primes actually issued (paid before stamping t3)\n"
      "# TYPE ntp_arp_primes_total counter\n"
      "ntp_arp_primes_total %" PRIu32 "\n",
      ntp_prof_ewma_us(NTP_PROF_INT_TO_SEND),
      ntp_prof_samples(),
      ntp_prof_txns(), ntp_prof_txns_serve(), ntp_prof_prime_txns(),
      ntp_prof_bytes(), ntp_prof_bytes_serve(), ntp_prof_sels(),
      ntp_prof_reap_polls(), ntp_prof_prime_polls(), ntp_prof_primes());

    /* Cumulative histogram of int_to_send, so the bimodality is visible and a
     * percentile can be read off directly rather than inferred from an EWMA. */
    blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
      "# HELP ntp_int_to_send_us_bucket Cumulative histogram of arrival-edge to SEND\n"
      "# TYPE ntp_int_to_send_us_bucket histogram\n");
    uint32_t cum = 0;
    for (int i = 0; i < NTP_PROF_BUCKETS && blen < cap - 128; ++i) {
      cum += ntp_prof_bucket(i);
      uint32_t e = ntp_prof_bucket_edge_us(i);
      if (e)
        blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
          "ntp_int_to_send_us_bucket{le=\"%" PRIu32 "\"} %" PRIu32 "\n", e, cum);
      else
        blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
          "ntp_int_to_send_us_bucket{le=\"+Inf\"} %" PRIu32 "\n", cum);
    }
  }

  /*
   * Per-satellite series, matching ts2phc-go's label scheme so one dashboard
   * query covers every clock in the fleet. Appended rather than folded into the
   * snprintf above because the count varies with the sky.
   */
  if (blen > 0) {
    static const char* const kGnss[GNSS_COUNT] =
        { "gps", "sbas", "glonass", "galileo", "beidou", "qzss" };
    GpsSatellite sats[40];
    int nsat = gps ? gps->getSatellites(sats, 40) : 0;
    int cap = (int)(sizeof(g_resp) - hdrReserve);
    if (nsat > 0 && blen < cap - 1) {
      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "# HELP gps_satellite_cno_dbhz C/N0 per satellite\n"
        "# TYPE gps_satellite_cno_dbhz gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
          "gps_satellite_cno_dbhz{gnss=\"%s\",svid=\"%u\"} %u\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, (unsigned)sats[i].cn0);

      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "# HELP gps_satellite_elevation_degrees\n"
        "# TYPE gps_satellite_elevation_degrees gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
          "gps_satellite_elevation_degrees{gnss=\"%s\",svid=\"%u\"} %d\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, (int)sats[i].elev);

      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "# HELP gps_satellite_azimuth_degrees\n"
        "# TYPE gps_satellite_azimuth_degrees gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
          "gps_satellite_azimuth_degrees{gnss=\"%s\",svid=\"%u\"} %u\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, (unsigned)sats[i].azim);

      blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
        "# HELP gps_satellite_used 1 if SV used in nav solution\n"
        "# TYPE gps_satellite_used gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, blen < cap ? cap - blen : 0,
          "gps_satellite_used{gnss=\"%s\",svid=\"%u\"} %d\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, sats[i].used ? 1 : 0);
    }
  }

  if (blen < 0 || blen >= (int)(sizeof(g_resp) - hdrReserve)) {
    blen = (int)(sizeof(g_resp) - hdrReserve) - 1;
    body[blen] = '\0';
  }

  char hdr[128];
  int hlen = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/plain; version=0.0.4; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Content-Length: %d\r\n"
    "\r\n",
    blen);

  memmove(resp + hlen, body, blen);
  memcpy(resp, hdr, hlen);
  sendAll(resp, hlen + blen);
  closeConn();
}
