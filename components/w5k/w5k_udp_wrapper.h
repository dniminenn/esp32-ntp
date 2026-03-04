#pragma once
// SPDX-License-Identifier: MIT-0
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int w5k_udp_open(uint8_t socket_num, uint16_t port);
int w5k_close(uint8_t socket_num);
int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len, uint8_t* from_ip, uint16_t* from_port);
int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port);
int w5k_set_nonblock(uint8_t socket_num);

// Non-blocking UDP send: initiate send (returns 0 on success, -1 on error)
int w5k_sendto_nb(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port);
// Poll for send completion: returns 1=done, 0=pending, -1=timeout
int w5k_sendto_poll(uint8_t socket_num);

// Prime ARP cache for destination IP by sending a 1-byte dummy.
// Blocks until ARP resolves or times out.  Returns 0=ok, -1=timeout.
int w5k_arp_prime(uint8_t socket_num, const uint8_t* ip);

#ifdef __cplusplus
}
#endif


