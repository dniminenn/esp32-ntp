// SPDX-License-Identifier: MIT-0

#include "ntp_stats.h"
#include "config.h"
#include "gps.h"
#include "ntp_server.h"
#include "w5500_eth.h"
#include "wifi_sta.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "w5k_tcp_wrapper.h"

static const char* TAG = "NTP_STATS";

static const uint8_t STATS_SOCKET = 2;

NtpStats::NtpStats() : sock(-1), port(0), gps(nullptr), ntp(nullptr),
                       eth(nullptr), wifi(nullptr), useWifi(false),
                       listening(false), disconnecting(false), startupLogged(false),
                       listen_sock(-1), client_sock(-1) {}

esp_err_t NtpStats::begin(int port_, GpsDiscipline* gps_, NtpServer* ntp_, W5500Eth* eth_, WifiSta* wifi_) {
  port = port_;
  gps = gps_;
  ntp = ntp_;
  eth = eth_;
  wifi = wifi_;
  useWifi = (wifi_ != nullptr);
  if (useWifi) {
    sock = -1;
    ESP_LOGI(TAG, "Stats HTTP server configured for port %d (WiFi), waiting for IP", port);
  } else {
    sock = STATS_SOCKET;
    ESP_LOGI(TAG, "Stats HTTP server configured for port %d (sn=%d), waiting for IP", port, sock);
  }
  return ESP_OK;
}

bool NtpStats::tryStartListener() {
  uint32_t ipVal = 0;
  if (useWifi) {
    if (!wifi || !wifi->getIpAddr(ipVal) || ipVal == 0) return false;
    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) return false;
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(listen_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
      close(listen_sock);
      listen_sock = -1;
      return false;
    }
    if (listen(listen_sock, 1) != 0) {
      close(listen_sock);
      listen_sock = -1;
      return false;
    }
    int flags = fcntl(listen_sock, F_GETFL, 0);
    if (flags >= 0) fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "Stats HTTP server listening on %lu.%lu.%lu.%lu:%d (WiFi)",
             (ipVal >> 24) & 0xff, (ipVal >> 16) & 0xff, (ipVal >> 8) & 0xff, ipVal & 0xff, port);
    startupLogged = true;
    listening = true;
    return true;
  }
  if (!eth) return false;
  bool gotIp = eth->getIpAddr(ipVal);
  if (!gotIp || ipVal == 0) {
    static int suppressCount = 0;
    if (suppressCount++ % 100 == 0)
      ESP_LOGW(TAG, "tryStartListener: no IP yet (gotIp=%d ipVal=0x%08" PRIx32 ")", gotIp, ipVal);
    return false;
  }
  uint8_t ip[4] = {
    (uint8_t)(ipVal >> 24), (uint8_t)(ipVal >> 16),
    (uint8_t)(ipVal >> 8),  (uint8_t)(ipVal)
  };
  int rc = w5k_tcp_listen((uint8_t)sock, (uint16_t)port);
  if (rc != 0) {
    ESP_LOGW(TAG, "TCP listen failed on s=%d port=%d rc=%d (will retry)", sock, port, rc);
    return false;
  }
  ESP_LOGI(TAG, "Stats HTTP server listening on %d.%d.%d.%d:%d (sn=%d)",
           ip[0], ip[1], ip[2], ip[3], port, sock);
  startupLogged = true;
  listening = true;
  return true;
}

void NtpStats::loop() {
  if (useWifi) {
    if (!listening && listen_sock < 0) {
      tryStartListener();
      return;
    }
    if (listen_sock < 0) return;
    if (client_sock < 0) {
      struct sockaddr_in from = {};
      socklen_t fromlen = sizeof(from);
      client_sock = accept(listen_sock, (struct sockaddr*)&from, &fromlen);
      return;
    }
    handleConnection();
    return;
  }

  if (sock < 0) return;
  if (!listening && !disconnecting) {
    tryStartListener();
    return;
  }

  uint8_t status = w5k_tcp_status((uint8_t)sock);
  static uint8_t lastStatus = 255;
  if (status != lastStatus) {
    ESP_LOGI(TAG, "Socket status changed: %d -> %d", lastStatus, status);
    lastStatus = status;
  }

  if (disconnecting) {
    if (status == W5K_SOCK_CLOSED) {
      disconnecting = false;
      listening = false;
      tryStartListener();
    } else if (status == W5K_SOCK_ESTABLISHED || status == W5K_SOCK_CLOSE_WAIT) {
      w5k_tcp_disconnect((uint8_t)sock);
    }
    return;
  }

  switch (status) {
    case W5K_SOCK_ESTABLISHED:
      handleConnection();
      break;
    case W5K_SOCK_CLOSE_WAIT:
      w5k_tcp_disconnect((uint8_t)sock);
      disconnecting = true;
      break;
    case W5K_SOCK_CLOSED:
      ESP_LOGW(TAG, "Socket %d unexpectedly closed, re-listening", sock);
      listening = false;
      disconnecting = false;
      tryStartListener();
      break;
    case W5K_SOCK_LISTEN:
      break;
    default:
      break;
  }
}


void NtpStats::handleConnection() {
  uint8_t reqBuf[256];
  int32_t n;
  if (useWifi) {
    n = recv(client_sock, reqBuf, sizeof(reqBuf) - 1, 0);
    if (n <= 0) {
      close(client_sock);
      client_sock = -1;
      return;
    }
  } else {
    n = w5k_tcp_recv((uint8_t)sock, reqBuf, sizeof(reqBuf) - 1);
    if (n <= 0) return;
  }
  reqBuf[n] = '\0';

  bool isMetrics = (strncmp((const char*)reqBuf, "GET /metrics", 12) == 0);
  const char* resp404 =
    "HTTP/1.1 404 Not Found\r\n"
    "Content-Type: text/plain\r\n"
    "Connection: close\r\n"
    "Content-Length: 9\r\n"
    "\r\n"
    "Not Found";
  if (!isMetrics) {
    if (useWifi) {
      send(client_sock, resp404, strlen(resp404), 0);
      close(client_sock);
      client_sock = -1;
    } else {
      w5k_tcp_send((uint8_t)sock, (const uint8_t*)resp404, (uint16_t)strlen(resp404));
      w5k_tcp_disconnect((uint8_t)sock);
      disconnecting = true;
    }
    return;
  }

  GpsStats gs = {};
  if (gps) gps->getStats(gs);
  bool locked = (gps && gps->isLocked());
  uint32_t reqCount = ntp ? ntp->getRequestCount() : 0;
  double uptimeSec = (double)esp_timer_get_time() / 1e6;
  int stratum = locked ? 1 : 16;
  double rootDispersion = gps ? gps->getRootDispersion() : 1.0;

  static char resp[1792];
  static const int hdrReserve = 128;
  char* body = resp + hdrReserve;
  int blen = snprintf(body, sizeof(resp) - hdrReserve,
    "# HELP ntp_clock_offset_seconds Last measured clock offset\n"
    "# TYPE ntp_clock_offset_seconds gauge\n"
    "ntp_clock_offset_seconds %.9f\n"
    "# HELP ntp_rms_offset_seconds Exponentially weighted RMS offset\n"
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
    "ntp_pps_rejects_total %" PRIu32 "\n",
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
    gs.ppsRejectCount
  );

  if (blen < 0 || blen >= (int)(sizeof(resp) - hdrReserve)) {
    blen = (int)(sizeof(resp) - hdrReserve) - 1;
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
  int totalLen = hlen + blen;

  if (useWifi) {
    send(client_sock, resp, totalLen, 0);
    close(client_sock);
    client_sock = -1;
  } else {
    w5k_tcp_send((uint8_t)sock, (const uint8_t*)resp, (uint16_t)totalLen);
    w5k_tcp_disconnect((uint8_t)sock);
    disconnecting = true;
  }
}
