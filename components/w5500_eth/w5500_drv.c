// SPDX-License-Identifier: Unlicense

#include "w5500_drv.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "W5500DRV";

static uint32_t s_mgmt_spin_timeouts = 0;
/* Bit per socket. Single-owner (every W5500 access funnels through the NTP
 * task, see app_main.cpp), so plain bytes suffice. */
static uint8_t s_nonblock_mask = 0;
static uint8_t s_sending_mask = 0;

uint32_t w5500_mgmt_spin_timeouts(void) { return s_mgmt_spin_timeouts; }

uint8_t w5500_rd8(uint32_t addrsel) {
  uint8_t v = 0;
  w5500_bus_rd(addrsel, &v, 1);
  return v;
}

void w5500_wr8(uint32_t addrsel, uint8_t v) {
  w5500_bus_wr(addrsel, &v, 1);
}

uint16_t w5500_rd16(uint32_t addrsel) {
  uint8_t b[2];
  w5500_bus_rd(addrsel, b, 2);
  return (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
}

void w5500_wr16(uint32_t addrsel, uint16_t v) {
  uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
  w5500_bus_wr(addrsel, b, 2);
}

/* Sn_CR self-clears on acceptance, normally within microseconds. */
static bool spin_cr_clear(uint8_t sn, int64_t budget_us) {
  const int64_t deadline = esp_timer_get_time() + budget_us;
  do {
    if (w5500_rd8(W5500_SREG(sn, W5500_SN_CR)) == 0) return true;
  } while (esp_timer_get_time() < deadline);
  s_mgmt_spin_timeouts++;
  return false;
}

static bool spin_sr(uint8_t sn, uint8_t want, int64_t budget_us) {
  const int64_t deadline = esp_timer_get_time() + budget_us;
  do {
    if (w5500_rd8(W5500_SREG(sn, W5500_SN_SR)) == want) return true;
  } while (esp_timer_get_time() < deadline);
  s_mgmt_spin_timeouts++;
  return false;
}

uint8_t w5500_version(void) {
  return w5500_rd8(W5500_CREG(W5500_VERSIONR));
}

int w5500_chip_init(void) {
  w5500_wr8(W5500_CREG(W5500_MR), W5500_MR_RST);
  const int64_t deadline = esp_timer_get_time() + 10000;
  while (w5500_rd8(W5500_CREG(W5500_MR)) & W5500_MR_RST) {
    if (esp_timer_get_time() >= deadline) {
      s_mgmt_spin_timeouts++;
      ESP_LOGE(TAG, "MR reset bit stuck — SPI link dead?");
      return -1;
    }
  }
  for (uint8_t sn = 0; sn < 8; ++sn) {
    w5500_wr8(W5500_SREG(sn, W5500_SN_TXBUF_SIZE), 2);
    w5500_wr8(W5500_SREG(sn, W5500_SN_RXBUF_SIZE), 2);
  }
  s_nonblock_mask = 0;
  s_sending_mask = 0;
  for (int i = 0; i < 3; ++i) {
    if (w5500_version() == 0x04) return 0;
  }
  return -1;
}

bool w5500_phy_link_up(void) {
  return (w5500_rd8(W5500_CREG(W5500_PHYCFGR)) & W5500_PHYCFGR_LNK) != 0;
}

void w5500_phy_autonego(void) {
  /* PHYCFGR reset is active-low: write opmode with RST held low, then release. */
  const uint8_t conf = W5500_PHYCFGR_OPMD | W5500_PHYCFGR_OPMDC_ALLA;
  w5500_wr8(W5500_CREG(W5500_PHYCFGR), conf);
  w5500_wr8(W5500_CREG(W5500_PHYCFGR), conf | W5500_PHYCFGR_RST);
}

void w5500_set_mac(const uint8_t mac[6]) { w5500_bus_wr(W5500_CREG(W5500_SHAR), mac, 6); }
void w5500_get_mac(uint8_t mac[6])       { w5500_bus_rd(W5500_CREG(W5500_SHAR), mac, 6); }

void w5500_set_ipconf(const uint8_t ip[4], const uint8_t gw[4], const uint8_t mask[4]) {
  w5500_bus_wr(W5500_CREG(W5500_GAR),  gw,   4);
  w5500_bus_wr(W5500_CREG(W5500_SUBR), mask, 4);
  w5500_bus_wr(W5500_CREG(W5500_SIPR), ip,   4);
}

void w5500_get_ip(uint8_t ip[4])     { w5500_bus_rd(W5500_CREG(W5500_SIPR), ip,   4); }
void w5500_get_gw(uint8_t gw[4])     { w5500_bus_rd(W5500_CREG(W5500_GAR),  gw,   4); }
void w5500_get_mask(uint8_t mask[4]) { w5500_bus_rd(W5500_CREG(W5500_SUBR), mask, 4); }

void w5500_sock_set_nonblock(uint8_t sn, bool nb) {
  if (nb) s_nonblock_mask |= (uint8_t)(1 << sn);
  else    s_nonblock_mask &= (uint8_t)~(1 << sn);
}

uint8_t w5500_sock_status(uint8_t sn) {
  return w5500_rd8(W5500_SREG(sn, W5500_SN_SR));
}

int w5500_sock_close(uint8_t sn) {
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_CLOSE);
  bool ok = spin_cr_clear(sn, 5000);
  w5500_wr8(W5500_SREG(sn, W5500_SN_IR), 0xFF);
  s_nonblock_mask &= (uint8_t)~(1 << sn);
  s_sending_mask  &= (uint8_t)~(1 << sn);
  ok = spin_sr(sn, W5500_SR_CLOSED, 5000) && ok;
  return ok ? 0 : -1;
}

static int sock_open(uint8_t sn, uint8_t mode, uint16_t port, uint8_t want_sr) {
  w5500_sock_close(sn);
  w5500_wr8(W5500_SREG(sn, W5500_SN_MR), mode);
  w5500_wr16(W5500_SREG(sn, W5500_SN_PORT), port);
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_OPEN);
  if (!spin_cr_clear(sn, 5000)) return -1;
  if (!spin_sr(sn, want_sr, 5000)) return -1;
  return 0;
}

int w5500_udp_open(uint8_t sn, uint16_t port) {
  return sock_open(sn, W5500_SN_MR_UDP, port, W5500_SR_UDP);
}

int32_t w5500_udp_sendto(uint8_t sn, const uint8_t* buf, uint16_t len,
                         const uint8_t ip[4], uint16_t port) {
  if (len == 0) return 0;
  if (len > W5500_SOCK_BUFSZ) len = W5500_SOCK_BUFSZ;

  uint8_t dst[6] = { ip[0], ip[1], ip[2], ip[3],
                     (uint8_t)(port >> 8), (uint8_t)port };
  w5500_bus_wr(W5500_SREG(sn, W5500_SN_DIPR), dst, 6);

  /* Unbounded in blocking mode: no TX room here means a wedged chip, which
   * must starve the task watchdog into a panic reboot (fail-loud). */
  for (;;) {
    if (w5500_sock_status(sn) != W5500_SR_UDP) return -1;
    if (w5500_rd16(W5500_SREG(sn, W5500_SN_TX_FSR)) >= len) break;
    if (s_nonblock_mask & (1 << sn)) return 0;
  }

  uint16_t wr = w5500_rd16(W5500_SREG(sn, W5500_SN_TX_WR));
  w5500_bus_wr(W5500_TXBUF(sn, wr), buf, len);
  w5500_wr16(W5500_SREG(sn, W5500_SN_TX_WR), (uint16_t)(wr + len));

  /* Clear stale completion flags so the wait below sees only THIS send. */
  w5500_wr8(W5500_SREG(sn, W5500_SN_IR),
            (uint8_t)(W5500_IR_SENDOK | W5500_IR_TIMEOUT));
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_SEND);
  if (!spin_cr_clear(sn, 5000)) return -1;

  const int64_t send_deadline = esp_timer_get_time() + 3000000;
  for (;;) {
    uint8_t ir = w5500_rd8(W5500_SREG(sn, W5500_SN_IR));
    if (ir & W5500_IR_SENDOK) {
      w5500_wr8(W5500_SREG(sn, W5500_SN_IR), W5500_IR_SENDOK);
      return (int32_t)len;
    }
    if (ir & W5500_IR_TIMEOUT) {
      w5500_wr8(W5500_SREG(sn, W5500_SN_IR), W5500_IR_TIMEOUT);
      return -2;
    }
    if (w5500_sock_status(sn) != W5500_SR_UDP) return -1;
    if (esp_timer_get_time() >= send_deadline) {
      s_mgmt_spin_timeouts++;
      return -1;
    }
  }
}

int32_t w5500_udp_recvfrom(uint8_t sn, uint8_t* buf, uint16_t len,
                           uint8_t from_ip[4], uint16_t* from_port) {
  uint16_t rsr = w5500_rd16(W5500_SREG(sn, W5500_SN_RX_RSR));
  if (rsr < 8) return 0;

  uint16_t rd = w5500_rd16(W5500_SREG(sn, W5500_SN_RX_RD));
  uint8_t hdr[8];
  w5500_bus_rd(W5500_RXBUF(sn, rd), hdr, 8);
  uint16_t plen = (uint16_t)(((uint16_t)hdr[6] << 8) | hdr[7]);
  if (from_ip)   memcpy(from_ip, hdr, 4);
  if (from_port) *from_port = (uint16_t)(((uint16_t)hdr[4] << 8) | hdr[5]);

  uint16_t copy = plen < len ? plen : len;
  if (copy) w5500_bus_rd(W5500_RXBUF(sn, (uint16_t)(rd + 8)), buf, copy);

  /* Advance past the whole datagram even when truncated; cap at RSR so a
   * corrupt header resynchronises the ring instead of desynchronising it. */
  uint16_t consumed = (uint16_t)(8 + plen);
  if (consumed > rsr) consumed = rsr;
  w5500_wr16(W5500_SREG(sn, W5500_SN_RX_RD), (uint16_t)(rd + consumed));
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_RECV);
  spin_cr_clear(sn, 5000);
  return (int32_t)copy;
}

int w5500_tcp_listen(uint8_t sn, uint16_t port) {
  if (sock_open(sn, W5500_SN_MR_TCP, port, W5500_SR_INIT) != 0) return -1;
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_LISTEN);
  if (!spin_cr_clear(sn, 5000) ||
      w5500_sock_status(sn) != W5500_SR_LISTEN) {
    w5500_sock_close(sn);
    return -2;
  }
  return 0;
}

int32_t w5500_tcp_recv(uint8_t sn, uint8_t* buf, uint16_t len) {
  uint16_t rsr = w5500_rd16(W5500_SREG(sn, W5500_SN_RX_RSR));
  if (rsr == 0) return 0;
  if (len > rsr) len = rsr;

  uint16_t rd = w5500_rd16(W5500_SREG(sn, W5500_SN_RX_RD));
  w5500_bus_rd(W5500_RXBUF(sn, rd), buf, len);
  w5500_wr16(W5500_SREG(sn, W5500_SN_RX_RD), (uint16_t)(rd + len));
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_RECV);
  spin_cr_clear(sn, 5000);
  return (int32_t)len;
}

int w5500_tcp_send_complete(uint8_t sn) {
  uint8_t sr = w5500_sock_status(sn);
  if (sr != W5500_SR_ESTABLISHED && sr != W5500_SR_CLOSE_WAIT) return -1;
  if (s_sending_mask & (1 << sn)) {
    uint8_t ir = w5500_rd8(W5500_SREG(sn, W5500_SN_IR));
    if (ir & W5500_IR_SENDOK) {
      w5500_wr8(W5500_SREG(sn, W5500_SN_IR), W5500_IR_SENDOK);
      s_sending_mask &= (uint8_t)~(1 << sn);
    } else if (ir & W5500_IR_TIMEOUT) {
      w5500_sock_close(sn);
      return -2;
    } else {
      return 0;
    }
  }
  return 1;
}

int32_t w5500_tcp_send(uint8_t sn, const uint8_t* buf, uint16_t len) {
  int ready = w5500_tcp_send_complete(sn);
  if (ready < 0) return ready;
  if (ready == 0) return 0;

  if (len > W5500_SOCK_BUFSZ) len = W5500_SOCK_BUFSZ;
  if (w5500_rd16(W5500_SREG(sn, W5500_SN_TX_FSR)) < len) return 0;

  uint16_t wr = w5500_rd16(W5500_SREG(sn, W5500_SN_TX_WR));
  w5500_bus_wr(W5500_TXBUF(sn, wr), buf, len);
  w5500_wr16(W5500_SREG(sn, W5500_SN_TX_WR), (uint16_t)(wr + len));
  w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_SEND);
  if (!spin_cr_clear(sn, 5000)) return -1;
  s_sending_mask |= (uint8_t)(1 << sn);
  return (int32_t)len;
}

int w5500_tcp_discon(uint8_t sn) {
  if (w5500_sock_status(sn) != W5500_SR_CLOSED) {
    w5500_wr8(W5500_SREG(sn, W5500_SN_CR), W5500_CR_DISCON);
    spin_cr_clear(sn, 5000);
  }
  return 0;
}
