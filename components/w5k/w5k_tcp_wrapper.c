// SPDX-License-Identifier: Unlicense

#include "w5k_tcp_wrapper.h"
#include "w5500_drv.h"

int w5k_tcp_listen(uint8_t socket_num, uint16_t port) {
  return w5500_tcp_listen(socket_num, port);
}

uint8_t w5k_tcp_status(uint8_t socket_num) {
  return w5500_sock_status(socket_num);
}

int32_t w5k_tcp_recv(uint8_t socket_num, uint8_t* buf, uint16_t len) {
  return w5500_tcp_recv(socket_num, buf, len);
}

int32_t w5k_tcp_send(uint8_t socket_num, const uint8_t* buf, uint16_t len) {
  return w5500_tcp_send(socket_num, buf, len);
}

int w5k_tcp_send_complete(uint8_t socket_num) {
  return w5500_tcp_send_complete(socket_num);
}

int w5k_tcp_disconnect(uint8_t socket_num) {
  return w5500_tcp_discon(socket_num);
}
