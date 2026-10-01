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


static const char* TAG = "WEBUI";

static const uint8_t STATS_SOCKET = 2;

char g_req[3072];
char g_resp[26624];
const int hdrReserve = 128;

static bool wifi_ip(WifiSta* w, uint32_t& ip) {
#if CONFIG_SOC_WIFI_SUPPORTED
  return w && w->getIpAddr(ip);
#else
  (void)w; (void)ip;
  return false;
#endif
}

static const char* ci_find(const char* hay, const char* needle) {
  size_t n = strlen(needle);
  for (const char* p = hay; *p; ++p)
    if (strncasecmp(p, needle, n) == 0) return p;
  return nullptr;
}

// Liveness/boot diagnostics published by app_main (read-only here).
extern volatile uint32_t g_mainLoopBeats;
extern uint32_t g_bootCount;


WebServer::WebServer() : sock(-1), port(0), gps(nullptr), ntp(nullptr),
                       eth(nullptr), wifi(nullptr), useWifi(false),
                       listening(false), disconnecting(false), startupLogged(false),
                       listen_sock(-1), client_sock(-1),
                       reqLen(0), hdrEnd(-1), contentLen(0), reqStartUs(0) {}

esp_err_t WebServer::begin(int port_, GpsDiscipline* gps_, NtpServer* ntp_, W5500Eth* eth_, WifiSta* wifi_) {
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

bool WebServer::tryStartListener() {
  uint32_t ipVal = 0;
  if (useWifi) {
    if (!wifi_ip(wifi, ipVal) || ipVal == 0) return false;
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
    if (flags < 0 || fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK) < 0) {
      close(listen_sock);
      listen_sock = -1;
      return false;
    }
    ipVal = ntohl(ipVal);
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
  resetRequest();
  return true;
}

void WebServer::loop() {
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
      if (client_sock >= 0) {
        // Accepted sockets do not inherit O_NONBLOCK. Otherwise pumpRequest()
        // waits for more data after the HTTP request and starves the watchdog.
        int flags = fcntl(client_sock, F_GETFL, 0);
        if (flags < 0 || fcntl(client_sock, F_SETFL, flags | O_NONBLOCK) < 0) {
          closeConn();
          return;
        }
        resetRequest();
      }
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


void WebServer::resetRequest() {
  reqLen = 0;
  g_req[0] = '\0';
  hdrEnd = -1;
  contentLen = 0;
  reqStartUs = 0;
}

void WebServer::closeConn() {
  if (useWifi) {
    if (client_sock >= 0) close(client_sock);
    client_sock = -1;
  } else {
    w5k_tcp_disconnect((uint8_t)sock);
    disconnecting = true;
  }
  resetRequest();
}

void WebServer::sendAll(const char* data, int len) {
  if (useWifi) {
    int off = 0;
    const int64_t deadline = esp_timer_get_time() + 2000000;
    while (off < len && esp_timer_get_time() < deadline) {
      int r = send(client_sock, data + off, len - off, 0);
      if (r > 0) off += r;
      else if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) vTaskDelay(1);
      else break;
    }
    return;
  }
  // The W5500 socket TX buffer is 2KB; send in <=1KB chunks so a response
  // larger than the buffer chains cleanly through the driver's
  // one-chunk-in-flight send.
  /*
   * w5k_tcp_send() returns 0 (busy) — not an error — whenever the PREVIOUS
   * chunk has not reached SENDOK yet or the TX buffer lacks room. Treating
   * that as fatal silently truncated every response past the first chunk or
   * two, which is why the body length varied with load while Content-Length
   * stayed correct. Retry on 0, bail only on a negative return, and bound
   * the whole thing in time so a dead peer cannot wedge the housekeeping
   * slot.
   */
  int off = 0;
  const int64_t deadline = esp_timer_get_time() + 2000000;   /* 2 s */
  while (off < len && esp_timer_get_time() < deadline) {
    int chunk = len - off;
    if (chunk > 1024) chunk = 1024;
    int32_t r = w5k_tcp_send((uint8_t)sock, (const uint8_t*)data + off, (uint16_t)chunk);
    if (r > 0) off += r;
    else if (r < 0) break;     // socket closed/errored
    else vTaskDelay(1);        // SOCK_BUSY (waiting for prior chunk's SENDOK) --
                                // yield instead of spinning, so a slow SENDOK
                                // can no longer starve other tasks/the idle task
                                // for the whole 2s deadline
  }
}

void WebServer::sendStatus(const char* status, const char* ctype, const char* body) {
  char hdr[192];
  int blen = (int)strlen(body);
  int hlen = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 %s\r\n"
    "Content-Type: %s\r\n"
    "Connection: close\r\n"
    "%s"
    "Content-Length: %d\r\n"
    "\r\n",
    status, ctype,
    strncmp(status, "401", 3) == 0
      ? "WWW-Authenticate: Basic realm=\"esp32-ntp\"\r\n" : "",
    blen);
  sendAll(hdr, hlen);
  sendAll(body, blen);
  closeConn();
}

bool WebServer::pumpRequest() {
  if (reqStartUs == 0) reqStartUs = esp_timer_get_time();

  for (;;) {
    if (reqLen >= (int)sizeof(g_req) - 1) break;
    int32_t r;
    if (useWifi) {
      r = recv(client_sock, g_req + reqLen, sizeof(g_req) - 1 - reqLen, 0);
      if (r == 0) { closeConn(); return false; }            // peer closed
      if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        closeConn();
        return false;
      }
    } else {
      r = w5k_tcp_recv((uint8_t)sock, (uint8_t*)(g_req + reqLen),
                       (uint16_t)(sizeof(g_req) - 1 - reqLen));
      if (r < 0) { closeConn(); return false; }
      if (r == 0) break;
    }
    reqLen += r;
    g_req[reqLen] = '\0';
  }

  if (hdrEnd < 0) {
    char* p = strstr((char*)g_req, "\r\n\r\n");
    if (p) {
      hdrEnd = (int)(p - (char*)g_req) + 4;
      for (char* q = (char*)g_req; q < p; ++q) {
        if (strncasecmp(q, "Content-Length:", 15) == 0) {
          contentLen = atoi(q + 15);
          if (contentLen < 0) contentLen = 0;
          break;
        }
      }
    }
  }

  if (hdrEnd >= 0 && reqLen >= hdrEnd + contentLen) return true;

  if (esp_timer_get_time() - reqStartUs > 5000000) closeConn();
  return false;
}

bool WebServer::authorized(const char* req) {
  const char* want = cfg_str(CFG_UI_PASS);
  if (want[0] == '\0') return true;    // no password set: open, and the page says so

  const char* h = ci_find(req, "Authorization: Basic ");
  if (!h) return false;
  h += 21;

  static const char* B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  char dec[96];
  int dlen = 0, bits = 0, acc = 0;
  for (; *h && *h != '\r' && *h != '\n' && *h != ' '; ++h) {
    if (*h == '=') break;
    const char* p = strchr(B64, *h);
    if (!p) return false;
    acc = (acc << 6) | (int)(p - B64);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (dlen < (int)sizeof(dec) - 1) dec[dlen++] = (char)((acc >> bits) & 0xFF);
    }
  }
  dec[dlen] = '\0';

  const char* colon = strchr(dec, ':');
  if (!colon) return false;
  return strcmp(colon + 1, want) == 0;
}

void WebServer::handleConnection() {
  if (!pumpRequest()) return;

  const char* req = (const char*)g_req;
  char* body = g_req + hdrEnd;

  bool isGet  = strncmp(req, "GET ",  4) == 0;
  bool isPost = strncmp(req, "POST ", 5) == 0;
  const char* path = req + (isPost ? 5 : 4);

  auto pathIs = [&](const char* p) {
    size_t n = strlen(p);
    return strncmp(path, p, n) == 0 &&
           (path[n] == ' ' || path[n] == '?' || path[n] == '\0');
  };

  if (isGet && pathIs("/metrics")) {
    sendMetrics();
    return;
  }

  if (cfg_locked() && (pathIs("/") || pathIs("/config") || pathIs("/factory-reset"))) {
    sendStatus("403 Forbidden", "text/plain",
      "Settings are locked on this device. The lock is one way: erase the NVS "
      "partition over USB to undo it. Metrics remain available at /metrics.");
    return;
  }

  if (isGet && (pathIs("/") || pathIs("/config"))) {
    if (!authorized(req)) {
      sendStatus("401 Unauthorized", "text/plain", "Authentication required");
      return;
    }
    sendConfigPage(nullptr);
    return;
  }

  if (isPost && pathIs("/config")) {
    if (!authorized(req)) {
      sendStatus("401 Unauthorized", "text/plain", "Authentication required");
      return;
    }
    handleConfigPost(body);
    return;
  }

  if (isPost && pathIs("/factory-reset")) {
    if (!authorized(req)) {
      sendStatus("401 Unauthorized", "text/plain", "Authentication required");
      return;
    }
    cfg_factory_reset();
    w5500_dhcp_forget();
    sendStatus("200 OK", "text/plain",
               "Settings erased. Rebooting into build-time defaults.");
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
    return;
  }

  sendStatus("404 Not Found", "text/plain", "Not Found");
}

