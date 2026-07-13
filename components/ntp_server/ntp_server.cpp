// SPDX-License-Identifier: MIT-0

#include "ntp_server.h"
#include "config.h"
#include "gps.h"
#include <string.h>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_log.h"
#include <fcntl.h>
#include "lwip/sockets.h"
#include "w5k_udp_wrapper.h"
#include "driver/gpio.h"

static const char* TAG = "NTP_SRV";

// W5500 RX interrupt capture: the INTn pin (active-low) asserts on packet
// arrival. The GPIO ISR latches a monotonic timestamp at the edge — the same
// "capture in hardware, not in a poll" trick used for PPS — so t2 reflects true
// arrival instead of when the poll loop happened to read the socket.
static volatile uint64_t s_rxCaptureUs = 0;
static volatile uint32_t s_rxIrqCount = 0;

// Self-calibrating transmit correction (µs). t3 must be written into the packet
// before the SPI send completes, so the wire egress is always ~one send-duration
// later than t3. We measure each send's duration and feed an EWMA back as a
// pre-correction added to the next packet's t3, so t3 tracks true departure.
// Seeded near the measured steady-state (~700µs for a 48-byte blocking W5500
// send at 20MHz) so the very first replies after boot aren't biased while the
// EWMA converges.
static double s_txCorrectionUs = 700.0;

static void IRAM_ATTR w5500_rx_isr(void* arg) {
  s_rxCaptureUs = (uint64_t)esp_timer_get_time();
  s_rxIrqCount++;
}

static void wr32(uint8_t* p, int idx, uint32_t v) {
  p[idx+0] = (uint8_t)(v >> 24);
  p[idx+1] = (uint8_t)(v >> 16);
  p[idx+2] = (uint8_t)(v >> 8);
  p[idx+3] = (uint8_t)(v);
}

static void wr_ntp_ts(uint8_t* p, int idx, uint32_t sec, uint32_t frac) {
  wr32(p, idx, sec);
  wr32(p, idx + 4, frac);
}

NtpServer::NtpServer() : sock(-1), port(0), gps(nullptr), requestCount(0), lastRxIrqConsumed(0), useWifi(false) {
}

esp_err_t NtpServer::begin(int port_, GpsDiscipline* gps_) {
  port = port_;
  gps = gps_;
  useWifi = Config::getNetworkWifi();

  if (useWifi) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      ESP_LOGE(TAG, "WiFi UDP socket create failed");
      return ESP_FAIL;
    }
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
      ESP_LOGE(TAG, "WiFi UDP bind failed on port %d", port);
      close(sock);
      sock = -1;
      return ESP_FAIL;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags >= 0) fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "NTP server listening on port %d (WiFi)", port);
    return ESP_OK;
  }

  sock = 1;
  if (w5k_udp_open((uint8_t)sock, (uint16_t)port) != 0) {
    ESP_LOGE(TAG, "W5500 UDP socket open failed on s=%d port=%d", sock, port);
    sock = -1;
    return ESP_FAIL;
  }
  w5k_set_nonblock((uint8_t)sock);
  setupRxInterrupt();
  ESP_LOGI(TAG, "NTP server listening on port %d (Wiznet sn=%d)", port, sock);
  return ESP_OK;
}

uint32_t NtpServer::getRxIrqCount() const { return s_rxIrqCount; }
double NtpServer::getTxCorrectionUs() const { return s_txCorrectionUs; }

void NtpServer::setupRxInterrupt() {
  int pin = Config::getW5500IntPin();
  if (pin < 0) {
    ESP_LOGW(TAG, "No W5500 INT pin configured; RX timestamping stays poll-based");
    return;
  }
  // GPIO34-39 are input-only with no internal pulls; the W5500 INTn line needs
  // an external pull-up (present on the module). Trigger on the falling edge.
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << pin);
  io.mode = GPIO_MODE_INPUT;
  io.intr_type = GPIO_INTR_NEGEDGE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io);

  // gpio_install_isr_service is idempotent across the app; ignore "already installed".
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
    return;
  }
  gpio_isr_handler_add((gpio_num_t)pin, w5500_rx_isr, nullptr);

  // Enable the socket's RECV interrupt so INTn actually asserts on arrival.
  w5k_enable_rx_irq((uint8_t)sock);
  ESP_LOGI(TAG, "W5500 RX interrupt capture armed on GPIO%d (sn=%d)", pin, sock);
}

void NtpServer::computeNtpTimestamp(uint64_t monoUs, bool locked, uint32_t& sec1900, uint32_t& frac) {
  if (locked && gps) {
    uint64_t lastPpsUs = gps->getLastPpsMonotonicUs();
    uint32_t ppsSec1900, ppsFrac;
    if (gps->getLastPps(ppsSec1900, ppsFrac)) {
      uint64_t rawDelta = monoUs - lastPpsUs;
      int64_t freqCorr = (int64_t)(-gps->getFrequencyPpm() * 1e-6 * (double)rawDelta);
      uint64_t correctedDelta = rawDelta + freqCorr;
      sec1900 = ppsSec1900 + (uint32_t)(correctedDelta / 1000000ULL);
      frac = (uint32_t)(((correctedDelta % 1000000ULL) << 32) / 1000000ULL);
      return;
    }
  }
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  sec1900 = (uint32_t)((uint64_t)tv.tv_sec + 2208988800ULL);
  frac = 0;
}

void NtpServer::loop() {
  if (sock < 0) return;

  static uint32_t windowSec = 0;
  static int windowCount = 0;

  uint8_t req[48];
  uint8_t from_ip[4];
  uint16_t from_port = 0;
  struct sockaddr_in wifi_from = {};
  socklen_t wifi_fromlen = sizeof(wifi_from);
  int32_t n;
  uint64_t t2_us;
  if (useWifi) {
    n = recvfrom(sock, req, sizeof(req), 0, (struct sockaddr*)&wifi_from, &wifi_fromlen);
    // lwIP has already buffered the datagram; stamp after recvfrom returns.
    t2_us = esp_timer_get_time();
    if (n > 0) {
      uint32_t a = ntohl(wifi_from.sin_addr.s_addr);
      from_ip[0] = (a >> 24) & 0xff;
      from_ip[1] = (a >> 16) & 0xff;
      from_ip[2] = (a >> 8) & 0xff;
      from_ip[3] = a & 0xff;
      from_port = ntohs(wifi_from.sin_port);
    }
  } else {
    // W5500: a full NTP request is 48 bytes plus the 8-byte PACKET-INFO header
    // the chip prepends in the RX buffer. Detect arrival with a cheap RSR read
    // and stamp t2 *before* clocking the payload out over SPI, so the SPI read
    // duration no longer inflates the receive timestamp.
    if (w5k_rx_ready((uint8_t)sock) < 48 + 8) return;
    // Prefer the hardware-captured arrival edge (INTn ISR). If a fresh IRQ has
    // fired since the last packet we handled, use its latched timestamp — this
    // removes the poll-loop quantization from t2. Otherwise (e.g. a second
    // datagram queued behind one INTn assertion) fall back to the poll instant.
    uint32_t irqc = s_rxIrqCount;
    if (irqc != lastRxIrqConsumed) {
      t2_us = s_rxCaptureUs;
      lastRxIrqConsumed = irqc;
    } else {
      t2_us = esp_timer_get_time();
    }
    n = w5k_recvfrom((uint8_t)sock, req, sizeof(req), from_ip, &from_port);
    // Ack the RECV interrupt so INTn de-asserts and the next arrival re-fires.
    w5k_clear_rx_irq((uint8_t)sock);
  }

  if (n <= 0) return;
  if (n < 48) {
    ESP_LOGW(TAG, "Short NTP request (%d bytes) from %d.%d.%d.%d", n, from_ip[0], from_ip[1], from_ip[2], from_ip[3]);
    return;
  }

  // Only answer client requests (mode 3; mode 0 for ancient v3 hosts).
  // Replying to anything else — e.g. a spoofed mode-4 server response —
  // invites reflection loops between servers.
  uint8_t reqMode = req[0] & 0x07;
  if (reqMode != 3 && reqMode != 0) {
    ESP_LOGD(TAG, "Ignoring mode-%d packet from %d.%d.%d.%d", reqMode, from_ip[0], from_ip[1], from_ip[2], from_ip[3]);
    return;
  }

  bool locked = (gps && gps->isLocked());

  uint8_t rsp[48];
  memset(rsp, 0, sizeof(rsp));

  uint8_t clientVersion = (req[0] >> 3) & 0x7;
  if (clientVersion < 3) clientVersion = 3;
  if (clientVersion > 4) clientVersion = 4;

  uint8_t li = locked ? 0 : 3;
  rsp[0] = (li << 6) | (clientVersion << 3) | 4;
  rsp[1] = locked ? 1 : 16;
  rsp[2] = 6;
  rsp[3] = 0xEC;  // precision: 2^-20 (~1µs)

  auto to_fixed_16_16 = [](double secs) -> uint32_t {
    if (secs < 0) secs = 0;
    double x = secs * 65536.0;
    if (x > 4294967295.0) x = 4294967295.0;
    return (uint32_t)(x + 0.5);
  };
  double rootDelay = (locked && gps) ? gps->getRootDelay() : 1.0;
  double dispersion = (locked && gps) ? gps->getRootDispersion() : 1.0;
  wr32(rsp, 4, to_fixed_16_16(rootDelay));
  wr32(rsp, 8, to_fixed_16_16(dispersion));

  if (locked) {
    rsp[12] = 'P'; rsp[13] = 'P'; rsp[14] = 'S'; rsp[15] = 0;
  }

  uint32_t refSec1900 = 0, refFrac = 0;
  if (locked && gps && gps->getLastPps(refSec1900, refFrac)) {
    wr_ntp_ts(rsp, 16, refSec1900, refFrac);
  }

  // Originate timestamp (client's transmit time)
  memcpy(&rsp[24], &req[40], 8);

  // Receive timestamp (t2)
  uint32_t t2_sec, t2_frac;
  computeNtpTimestamp(t2_us, locked, t2_sec, t2_frac);
  wr_ntp_ts(rsp, 32, t2_sec, t2_frac);

  // KoD rate limiting
  uint32_t currentSec = t2_sec - 2208988800ULL;
  if (currentSec != windowSec) {
    windowSec = currentSec;
    windowCount = 0;
  }
  windowCount++;
  if (windowCount > 50) {
    rsp[1] = 0;
    rsp[12] = 'R'; rsp[13] = 'A'; rsp[14] = 'T'; rsp[15] = 'E';
  }

  if (useWifi) {
    // WiFi: stamp t3 as late as possible, then send
    uint32_t t3_sec, t3_frac;
    computeNtpTimestamp(esp_timer_get_time(), locked, t3_sec, t3_frac);
    wr_ntp_ts(rsp, 40, t3_sec, t3_frac);
    if (sendto(sock, rsp, 48, 0, (struct sockaddr*)&wifi_from, wifi_fromlen) == 48) {
      requestCount++;
      ESP_LOGD(TAG, "Replied to %d.%d.%d.%d:%u (stratum %d, LI=%d)",
               from_ip[0], from_ip[1], from_ip[2], from_ip[3], (unsigned)from_port, rsp[1], li);
    }
    return;
  }

  // W5500: prime ARP first. This is load-bearing, not just a timing nicety — the
  // chip's UDP send only completes (and advances Sn_TX_RD) once the destination
  // MAC is resolved. Skipping it leaves Sn_TX_RD stuck while Sn_TX_WR grows, so
  // every reply becomes an oversized/garbage packet. arp_prime blocks until the
  // ARP is warm, after which the real send departs promptly.
  // If ARP can't resolve (spoofed or unroutable source), drop the reply — the
  // real send would stall the same way and the answer would go nowhere anyway.
  if (w5k_arp_prime((uint8_t)sock, from_ip) != 0) {
    ESP_LOGW(TAG, "ARP unresolved for %d.%d.%d.%d — dropping reply",
             from_ip[0], from_ip[1], from_ip[2], from_ip[3]);
    return;
  }

  // Stamp t3, pre-corrected by the measured send duration so it reflects wire
  // egress rather than the pre-send instant (ARP is already warm here).
  uint32_t t3_sec, t3_frac;
  computeNtpTimestamp(esp_timer_get_time() + (uint64_t)(s_txCorrectionUs + 0.5), locked, t3_sec, t3_frac);
  wr_ntp_ts(rsp, 40, t3_sec, t3_frac);

  uint64_t txStart = esp_timer_get_time();
  int32_t sret = w5k_sendto((uint8_t)sock, rsp, sizeof(rsp), from_ip, from_port);
  uint64_t txEnd = esp_timer_get_time();
  if (sret == (int32_t)sizeof(rsp)) {
    requestCount++;
    // EWMA of actual send duration → next packet's t3 pre-correction.
    s_txCorrectionUs += 0.05 * ((double)(txEnd - txStart) - s_txCorrectionUs);
    ESP_LOGD(TAG, "Replied to %d.%d.%d.%d:%u (stratum %d, LI=%d)",
             from_ip[0], from_ip[1], from_ip[2], from_ip[3], (unsigned)from_port, rsp[1], li);
  } else {
    ESP_LOGW(TAG, "W5500 sendto failed");
  }
}
