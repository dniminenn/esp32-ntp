#pragma once
// SPDX-License-Identifier: Unlicense
/*
 * First-party W5500 driver. addrsel packs an access as
 * (offset << 8) | (block << 3), the chip's own SPI header layout.
 *
 * Spin policy: hot-path waits are unbounded (fail-loud contract, see
 * w5500_eth.cpp); management waits are bounded and counted in
 * w5500_mgmt_spin_timeouts().
 */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SPI transport, implemented in w5500_eth.cpp. */
void w5k_xfer_rd(uint32_t addrsel, uint8_t* buf, uint16_t len);   /* one FIFO transaction, len+3 <= 64 */
void w5k_xfer_wr(uint32_t addrsel, const uint8_t* buf, uint16_t len);
void w5500_bus_rd(uint32_t addrsel, uint8_t* buf, uint16_t len);  /* any length */
void w5500_bus_wr(uint32_t addrsel, const uint8_t* buf, uint16_t len);

#define W5500_CREG(off)      ((uint32_t)(off) << 8)
#define W5500_SREG(sn, off)  (((uint32_t)(off) << 8) | ((uint32_t)(1u + 4u * (sn)) << 3))
#define W5500_TXBUF(sn, off) (((uint32_t)(off) << 8) | ((uint32_t)(2u + 4u * (sn)) << 3))
#define W5500_RXBUF(sn, off) (((uint32_t)(off) << 8) | ((uint32_t)(3u + 4u * (sn)) << 3))

/* common block */
#define W5500_MR        0x0000
#define W5500_GAR       0x0001   /* 0x01-0x04 */
#define W5500_SUBR      0x0005   /* 0x05-0x08 */
#define W5500_SHAR      0x0009   /* 0x09-0x0E */
#define W5500_SIPR      0x000F   /* 0x0F-0x12 */
#define W5500_SIMR      0x0018
#define W5500_RTR       0x0019   /* 0x19-0x1A, RCR at 0x1B */
#define W5500_RCR       0x001B
#define W5500_PHYCFGR   0x002E
#define W5500_VERSIONR  0x0039

/* socket block */
#define W5500_SN_MR         0x0000
#define W5500_SN_CR         0x0001
#define W5500_SN_IR         0x0002
#define W5500_SN_SR         0x0003
#define W5500_SN_PORT       0x0004   /* 0x04-0x05 */
#define W5500_SN_DIPR       0x000C   /* 0x0C-0x0F, port at 0x10-0x11: contiguous */
#define W5500_SN_DPORT      0x0010
#define W5500_SN_RXBUF_SIZE 0x001E
#define W5500_SN_TXBUF_SIZE 0x001F
#define W5500_SN_TX_FSR     0x0020   /* 0x20 FSR, 0x22 RD, 0x24 WR: contiguous */
#define W5500_SN_TX_RD      0x0022
#define W5500_SN_TX_WR      0x0024
#define W5500_SN_RX_RSR     0x0026
#define W5500_SN_RX_RD      0x0028
#define W5500_SN_IMR        0x002C

#define W5500_MR_RST        0x80

#define W5500_SN_MR_CLOSED  0x00
#define W5500_SN_MR_TCP     0x01
#define W5500_SN_MR_UDP     0x02

#define W5500_CR_OPEN       0x01
#define W5500_CR_LISTEN     0x02
#define W5500_CR_DISCON     0x08
#define W5500_CR_CLOSE      0x10
#define W5500_CR_SEND       0x20
#define W5500_CR_RECV       0x40

#define W5500_IR_CON        0x01
#define W5500_IR_DISCON     0x02
#define W5500_IR_RECV       0x04
#define W5500_IR_TIMEOUT    0x08
#define W5500_IR_SENDOK     0x10

#define W5500_SR_CLOSED       0x00
#define W5500_SR_INIT         0x13
#define W5500_SR_LISTEN       0x14
#define W5500_SR_ESTABLISHED  0x17
#define W5500_SR_CLOSE_WAIT   0x1C
#define W5500_SR_UDP          0x22

#define W5500_PHYCFGR_RST     0x80
#define W5500_PHYCFGR_OPMD    0x40
#define W5500_PHYCFGR_OPMDC_ALLA 0x38
#define W5500_PHYCFGR_LNK     0x01

#define W5500_SOCK_BUFSZ    2048   /* per socket, asserted at chip init */

uint8_t  w5500_rd8(uint32_t addrsel);
void     w5500_wr8(uint32_t addrsel, uint8_t v);
uint16_t w5500_rd16(uint32_t addrsel);   /* one 2-byte burst: atomic, no double-read loop */
void     w5500_wr16(uint32_t addrsel, uint16_t v);

int      w5500_chip_init(void);          /* soft reset + buffer sizes; -1 if chip absent */
uint8_t  w5500_version(void);
bool     w5500_phy_link_up(void);
void     w5500_phy_autonego(void);
void     w5500_set_mac(const uint8_t mac[6]);
void     w5500_get_mac(uint8_t mac[6]);
void     w5500_set_ipconf(const uint8_t ip[4], const uint8_t gw[4], const uint8_t mask[4]);
void     w5500_get_ip(uint8_t ip[4]);
void     w5500_get_gw(uint8_t gw[4]);
void     w5500_get_mask(uint8_t mask[4]);

void     w5500_sock_set_nonblock(uint8_t sn, bool nb);   /* affects TX free-space waits only */
uint8_t  w5500_sock_status(uint8_t sn);
int      w5500_sock_close(uint8_t sn);
int      w5500_udp_open(uint8_t sn, uint16_t port);
/* Returns len, 0 if non-blocking with no TX room, <0 on error/ARP timeout. */
int32_t  w5500_udp_sendto(uint8_t sn, const uint8_t* buf, uint16_t len,
                          const uint8_t ip[4], uint16_t port);
/* Returns payload bytes, 0 if nothing waiting. (NTP uses w5k_recvfrom instead.) */
int32_t  w5500_udp_recvfrom(uint8_t sn, uint8_t* buf, uint16_t len,
                            uint8_t from_ip[4], uint16_t* from_port);
int      w5500_tcp_listen(uint8_t sn, uint16_t port);   /* 0 ok, -1 open failed, -2 listen failed */
int32_t  w5500_tcp_recv(uint8_t sn, uint8_t* buf, uint16_t len);
/* Returns len once staged, 0 while the previous chunk is unacknowledged or
 * the TX buffer lacks room (never spins on the peer), <0 when the connection
 * is gone. webui is built around the 0-means-retry contract. */
int32_t  w5500_tcp_send(uint8_t sn, const uint8_t* buf, uint16_t len);
/* 1 when the last TCP chunk has SENDOK, 0 while pending, <0 on error. */
int      w5500_tcp_send_complete(uint8_t sn);
int      w5500_tcp_discon(uint8_t sn);

uint32_t w5500_mgmt_spin_timeouts(void);

#ifdef __cplusplus
}
#endif
