#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int w5k_udp_open(uint8_t socket_num, uint16_t port);
int w5k_close(uint8_t socket_num);
int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len, uint8_t* from_ip, uint16_t* from_port);
int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port);
int w5k_set_nonblock(uint8_t socket_num);

// Cheap "bytes waiting in RX buffer" check (reads Sn_RX_RSR only, no payload
// read). Lets the caller timestamp packet arrival before the SPI payload read.
int32_t w5k_rx_ready(uint8_t socket_num);

// Enable the W5500 RECV interrupt for this socket so the INTn pin asserts on
// packet arrival (drives a GPIO ISR for hardware-accurate RX timestamping).
void w5k_enable_rx_irq(uint8_t socket_num);
// Acknowledge/clear the RECV interrupt so INTn de-asserts and can fire again.
void w5k_clear_rx_irq(uint8_t socket_num);

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


