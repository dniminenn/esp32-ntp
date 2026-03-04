// SPDX-License-Identifier: MIT-0

#include "w5k_tcp_wrapper.h"
#include "wizchip_conf.h"
#include "socket.h"

int w5k_tcp_listen(uint8_t socket_num, uint16_t port) {
  close(socket_num);
  if (socket(socket_num, Sn_MR_TCP, port, 0) != socket_num) {
    return -1;
  }
  if (listen(socket_num) != SOCK_OK) {
    close(socket_num);
    return -2;
  }
  return 0;
}

uint8_t w5k_tcp_status(uint8_t socket_num) {
  return getSn_SR(socket_num);
}

int32_t w5k_tcp_recv(uint8_t socket_num, uint8_t* buf, uint16_t len) {
  int32_t size = getSn_RX_RSR(socket_num);
  if (size <= 0) return 0;
  if (size > len) size = len;
  return recv(socket_num, buf, (uint16_t)size);
}

int32_t w5k_tcp_send(uint8_t socket_num, const uint8_t* buf, uint16_t len) {
  return send(socket_num, (uint8_t*)buf, len);
}

int w5k_tcp_disconnect(uint8_t socket_num) {
  if (getSn_SR(socket_num) != SOCK_CLOSED) {
    setSn_CR(socket_num, Sn_CR_DISCON);
    while (getSn_CR(socket_num));
  }
  return 0;
}

int w5k_tcp_close(uint8_t socket_num) {
  return close(socket_num);
}

