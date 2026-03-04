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

