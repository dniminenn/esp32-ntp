// SPDX-License-Identifier: MIT-0

#include "w5k_udp_wrapper.h"
#include "wizchip_conf.h"
#include "socket.h"

int w5k_udp_open(uint8_t socket_num, uint16_t port) {
  return socket(socket_num, Sn_MR_UDP, port, 0) == socket_num ? 0 : -1;
}

int w5k_close(uint8_t socket_num) {
  return close(socket_num);
}

int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len, uint8_t* from_ip, uint16_t* from_port) {
  return recvfrom(socket_num, buf, len, from_ip, from_port);
}

int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port) {
  return sendto(socket_num, (uint8_t*)buf, len, (uint8_t*)to_ip, to_port);
}

int w5k_set_nonblock(uint8_t socket_num) {
  uint8_t mode = SOCK_IO_NONBLOCK;
  return ctlsocket(socket_num, CS_SET_IOMODE, &mode);
}

int32_t w5k_rx_ready(uint8_t socket_num) {
  return getSn_RX_RSR(socket_num);
}

void w5k_enable_rx_irq(uint8_t socket_num) {
  // Route this socket's interrupts to the INTn pin, and enable only RECV.
  setSIMR(getSIMR() | (uint8_t)(1 << socket_num));
  setSn_IMR(socket_num, Sn_IR_RECV);
}

void w5k_clear_rx_irq(uint8_t socket_num) {
  setSn_IR(socket_num, Sn_IR_RECV);
}

int w5k_sendto_nb(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port) {
  setSn_DIPR(socket_num, (uint8_t*)to_ip);
  setSn_DPORT(socket_num, to_port);
  uint16_t maxsz = getSn_TxMAX(socket_num);
  if (len > maxsz) len = maxsz;
  if (getSn_TX_FSR(socket_num) < len) return -1;
  wiz_send_data(socket_num, (uint8_t*)buf, len);
  setSn_CR(socket_num, Sn_CR_SEND);
  while (getSn_CR(socket_num));
  return 0;
}

int w5k_sendto_poll(uint8_t socket_num) {
  uint8_t ir = getSn_IR(socket_num);
  if (ir & Sn_IR_SENDOK) {
    setSn_IR(socket_num, Sn_IR_SENDOK);
    return 1;
  }
  if (ir & Sn_IR_TIMEOUT) {
    setSn_IR(socket_num, Sn_IR_TIMEOUT);
    return -1;
  }
  return 0;
}

int w5k_arp_prime(uint8_t socket_num, const uint8_t* ip) {
  /* Send a 1-byte dummy to port 9 (discard protocol) to trigger
     W5500 ARP resolution.  Block until SENDOK so the caller knows
     the ARP cache is warm before stamping t3.

     A spoofed/unroutable source must not stall the main loop (which also
     services PPS disciplining and DHCP) for the chip's default ~1.8s ARP
     retry budget, so shrink RTR/RCR for the prime: 40ms/try x 2 tries
     ~= 80ms worst case, then restore.  A live LAN host answers ARP in
     well under 40ms. */
  uint16_t rtr = getRTR();
  uint8_t rcr = getRCR();
  setRTR(400);   /* 40ms per try (unit 100us) */
  setRCR(1);     /* 1 retry -> ~80ms to TIMEOUT */

  uint8_t dummy = 0;
  setSn_DIPR(socket_num, (uint8_t*)ip);
  setSn_DPORT(socket_num, 9);           /* RFC 863 discard */
  wiz_send_data(socket_num, &dummy, 1);
  setSn_CR(socket_num, Sn_CR_SEND);
  while (getSn_CR(socket_num));

  int ret = -1;
  for (int i = 0; i < 20000; i++) {     /* backstop; TIMEOUT IR fires first */
    uint8_t ir = getSn_IR(socket_num);
    if (ir & Sn_IR_SENDOK) {
      setSn_IR(socket_num, Sn_IR_SENDOK);
      ret = 0;
      break;
    }
    if (ir & Sn_IR_TIMEOUT) {
      setSn_IR(socket_num, Sn_IR_TIMEOUT);
      break;
    }
  }

  setRTR(rtr);
  setRCR(rcr);
  return ret;
}

