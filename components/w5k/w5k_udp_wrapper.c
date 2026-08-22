// SPDX-License-Identifier: Unlicense

#include "w5k_udp_wrapper.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "w5500_drv.h"
#include "w5500_fast.h"
#include <string.h>
#include <stddef.h>
#include "esp_attr.h"

/*
 * ---------------------------------------------------------------------------
 * Fast reply path.
 *
 * Everything below talks to the chip through w5k_xfer_rd/w5k_xfer_wr, which
 * cost exactly one SPI transaction per access. The driver path
 * (w5500_bus_rd/wr via w5500_drv.c) is used for DHCP, the stats TCP socket,
 * and socket setup, where latency is irrelevant.
 *
 * Why one transaction per access suffices (the vendored library spent many):
 *  - Sn_RX_RSR / Sn_TX_FSR were read in a do{}while(v != v1) loop because the
 *    library fetched each 16-bit register as two single-byte accesses, which
 *    can tear. A 2-byte burst inside one transaction is atomic on the wire, so
 *    one read suffices.
 *  - recvfrom() drained the 8-byte PACKET-INFO header and the payload in two
 *    calls, each of which re-read Sn_RX_RD, rewrote it, issued Sn_CR_RECV and
 *    polled Sn_CR. Both live in the same contiguous ring, so one burst reads
 *    the header and payload together and one RECV closes it out.
 *  - Sn_DIPR (0x0C-0x0F) and Sn_DPORT (0x10-0x11) are contiguous: one 6-byte
 *    write replaces two accesses.
 *  - Sn_TX_FSR (0x20), Sn_TX_RD (0x22) and Sn_TX_WR (0x24) are contiguous too:
 *    one 6-byte read replaces two double-read loops.
 *  - The TX/RX buffer size is a constant 2048 for this memsize configuration.
 *
 * The W5500 masks a socket buffer pointer to the allocated buffer size and
 * auto-increments across the wrap in VDM, so a burst that crosses the ring
 * boundary needs no splitting here either.
 * ---------------------------------------------------------------------------
 */

/* Attribution counters: how many register reads each spin actually costs. */
volatile uint32_t g_w5k_reap_polls  = 0;
volatile uint32_t g_w5k_prime_polls = 0;

int w5k_udp_open(uint8_t socket_num, uint16_t port) {
  return w5500_udp_open(socket_num, port);
}

/*
 * Drain one datagram in six transactions: Sn_RX_RD, one burst covering the
 * PACKET-INFO header and the payload together, the Sn_RX_RD advance, Sn_CR_RECV
 * and one Sn_CR acceptance poll. `rsr` is the byte count the caller already read
 * with w5k_rx_ready(), passed in rather than re-read.
 */
IRAM_ATTR int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len,
                     uint8_t* from_ip, uint16_t* from_port, uint16_t rsr) {
  /* One access must fit the SPI peripheral's 64-byte data buffer once the
   * 3-byte W5500 header is included, so the header-plus-payload burst is capped
   * at 58: the 8-byte PACKET-INFO header plus 50 payload bytes. A 48-byte NTP
   * request fits with room to spare; anything longer is truncated to 50 bytes
   * here, which still exceeds everything this server parses (the first 48). */
  uint8_t frame[58];
  if (rsr < 8) return 0;

  uint8_t rdb[2];
  w5k_xfer_rd(W5500_SREG(socket_num, W5500_SN_RX_RD), rdb, 2);
  uint16_t rd = (uint16_t)(((uint16_t)rdb[0] << 8) | rdb[1]);

  uint16_t want = rsr;
  if (len > (uint16_t)(sizeof(frame) - 8)) len = (uint16_t)(sizeof(frame) - 8);
  if (want > (uint16_t)(8 + len)) want = (uint16_t)(8 + len);
  w5k_xfer_rd(W5500_RXBUF(socket_num, rd), frame, want);

  uint16_t plen  = (uint16_t)(((uint16_t)frame[6] << 8) | frame[7]);
  uint16_t avail = (uint16_t)(want - 8);
  uint16_t copy  = plen < avail ? plen : avail;
  if (from_ip)   memcpy(from_ip, frame, 4);
  if (from_port) *from_port = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
  if (copy)      memcpy(buf, &frame[8], copy);

  /* Advance past the WHOLE datagram even when it did not fit the caller's
   * buffer, so the next read still lands on a PACKET-INFO header. A header
   * claiming more than the socket holds can only be corruption; resynchronise
   * on Sn_RX_RSR rather than desynchronising the ring. */
  uint16_t consumed = (uint16_t)(8 + plen);
  if (consumed > rsr) consumed = rsr;
  rd = (uint16_t)(rd + consumed);
  uint8_t wrb[2] = { (uint8_t)(rd >> 8), (uint8_t)rd };
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_RX_RD), wrb, 2);

  uint8_t cmd = W5500_CR_RECV;
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_CR), &cmd, 1);
  /* Sn_CR self-clears when the command is accepted. Wait for that, because a
   * later Sn_CR write (the SEND below) would otherwise be dropped. */
  for (int i = 0; i < 200; i++) {
    uint8_t cr = 0;
    w5k_xfer_rd(W5500_SREG(socket_num, W5500_SN_CR), &cr, 1);
    if (cr == 0) break;
  }
  return (int32_t)copy;
}

int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port) {
  return w5500_udp_sendto(socket_num, buf, len, to_ip, to_port);
}

int w5k_set_nonblock(uint8_t socket_num) {
  w5500_sock_set_nonblock(socket_num, true);
  return 0;
}

IRAM_ATTR int32_t w5k_rx_ready(uint8_t socket_num) {
  /* One 2-byte burst: atomic on the wire, so no double read is needed. */
  uint8_t b[2];
  w5k_xfer_rd(W5500_SREG(socket_num, W5500_SN_RX_RSR), b, 2);
  return (int32_t)(uint16_t)(((uint16_t)b[0] << 8) | b[1]);
}

void w5k_enable_rx_irq(uint8_t socket_num) {
  // Route this socket's interrupts to the INTn pin, and enable only RECV.
  uint8_t simr = w5500_rd8(W5500_CREG(W5500_SIMR));
  w5500_wr8(W5500_CREG(W5500_SIMR), (uint8_t)(simr | (1 << socket_num)));
  w5500_wr8(W5500_SREG(socket_num, W5500_SN_IMR), W5500_IR_RECV);
}

/*
 * Additionally assert INTn on send completion.
 *
 * The W5500 has NO timestamping hardware of any kind, but INTn is already
 * routed to an MCPWM capture channel for packet arrival, and SENDOK drives the
 * same pin. Enabling it means transmit completion is latched in hardware on the
 * same 80 MHz counter as arrival and as the GPS pulse — so the request-to-egress
 * turnaround becomes a hardware measurement instead of a software estimate.
 *
 * Note INTn is level-asserted: RECV must already be cleared before SEND, or
 * SENDOK produces no falling edge to latch. The caller checks that the capture
 * counter advanced exactly once and discards the sample otherwise.
 */
void w5k_enable_sendok_irq(uint8_t socket_num) {
  uint8_t simr = w5500_rd8(W5500_CREG(W5500_SIMR));
  w5500_wr8(W5500_CREG(W5500_SIMR), (uint8_t)(simr | (1 << socket_num)));
  w5500_wr8(W5500_SREG(socket_num, W5500_SN_IMR),
            (uint8_t)(W5500_IR_RECV | W5500_IR_SENDOK));
}

IRAM_ATTR void w5k_clear_rx_irq(uint8_t socket_num) {
  uint8_t v = W5500_IR_RECV;
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_IR), &v, 1);
}

/*
 * Late-stamped send, in two steps.
 *
 * The whole datagram is staged with ONE ring write and ONE Sn_TX_WR advance
 * (splitting the payload across two staged writes was tried and does not work
 * on this chip: the Sn_TX_WR advance from the first write does not survive to
 * the second, so the tail overwrites the head and SEND emits only 8 bytes).
 * The transmit timestamp is then patched in place by writing 8 bytes straight
 * to its offset in the socket's TX buffer. That write does not touch Sn_TX_WR
 * at all, so there is no pointer arithmetic to lose — and only those 8 bytes
 * plus the SEND command sit between stamping t3 and the packet leaving.
 */
IRAM_ATTR int w5k_send_stage(uint8_t socket_num, const uint8_t* buf, uint16_t len,
                   const uint8_t* to_ip, uint16_t to_port, uint16_t* stamp_off,
                   int* wr_delta) {
  /*
   * Stops short of Sn_CR_SEND so the caller can patch the transmit timestamp
   * in place — four transactions:
   *   1. Sn_DIPR+Sn_DPORT as one 6-byte write (the registers are contiguous)
   *   2. Sn_TX_FSR+Sn_TX_RD+Sn_TX_WR as one 6-byte read (likewise contiguous,
   *      and atomic within one transaction, so no double-read loop is needed)
   *   3. the frame into the TX ring at Sn_TX_WR
   *   4. the Sn_TX_WR advance
   *
   * Sn_TX_WR is written here rather than read back afterwards. Reading it back
   * immediately returns a stale value on this chip, and an earlier verification
   * gate built on that read broke the send path outright, so wr_delta is left
   * unreported.
   */
  if (len > W5500_SOCK_BUFSZ) len = W5500_SOCK_BUFSZ;

  uint8_t dst[6];
  dst[0] = to_ip[0]; dst[1] = to_ip[1]; dst[2] = to_ip[2]; dst[3] = to_ip[3];
  dst[4] = (uint8_t)(to_port >> 8); dst[5] = (uint8_t)to_port;
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_DIPR), dst, 6);

  uint8_t p[6];
  w5k_xfer_rd(W5500_SREG(socket_num, W5500_SN_TX_FSR), p, 6);
  uint16_t fsr = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
  uint16_t wr0 = (uint16_t)(((uint16_t)p[4] << 8) | p[5]);
  if (fsr < len) return -1;

  w5k_xfer_wr(W5500_TXBUF(socket_num, wr0), buf, len);
  uint16_t nwr = (uint16_t)(wr0 + len);
  uint8_t wrb[2] = { (uint8_t)(nwr >> 8), (uint8_t)nwr };
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_TX_WR), wrb, 2);

  if (stamp_off) *stamp_off = wr0;
  if (wr_delta) *wr_delta = -1;   /* not read back: see above */
  return 0;
}

/*
 * Patch the transmit timestamp in place, then fire.
 *
 * EXACTLY two transactions, back to back, with nothing between them: the 8-byte
 * write into the TX ring, then the Sn_CR_SEND byte. This is the whole point of
 * the split — it is the window between t3 being computed and the frame leaving,
 * and it must not grow. An Sn_CR acceptance poll after SEND is deliberately
 * omitted: it would only confirm the chip took the command, and the next Sn_CR
 * write is the following packet's RECV, milliseconds away.
 */
IRAM_ATTR int w5k_send_stamp_and_fire(uint8_t socket_num, uint16_t off,
                            const uint8_t* stamp, uint16_t len) {
  w5k_xfer_wr(W5500_TXBUF(socket_num, off), stamp, len);
  uint8_t cmd = W5500_CR_SEND;
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_CR), &cmd, 1);
  return 0;
}

/* Reap the completion. Call after timing has stopped. */
IRAM_ATTR int w5k_send_reap(uint8_t socket_num) {
  /* quiet bus during frame egress */
  esp_rom_delay_us(10);
  /* A fixed iteration count is a latency cliff: 20000 register reads is ~180 ms
   * of spinning. Bound it in time instead. */
  int64_t deadline = esp_timer_get_time() + 2000;
  while (esp_timer_get_time() < deadline) {
    esp_rom_delay_us(2);
    uint8_t ir = 0;
    g_w5k_reap_polls++;
    w5k_xfer_rd(W5500_SREG(socket_num, W5500_SN_IR), &ir, 1);
    if (ir & W5500_IR_SENDOK) {
      uint8_t c = W5500_IR_SENDOK;
      w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_IR), &c, 1);
      return 0;
    }
    if (ir & W5500_IR_TIMEOUT) {
      uint8_t c = W5500_IR_TIMEOUT;
      w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_IR), &c, 1);
      return -1;
    }
  }
  return -1;
}

uint32_t g_w5k_primes = 0;

int w5k_arp_prime(uint8_t socket_num, const uint8_t* ip) {
  /* Send a 1-byte dummy to port 9 (discard protocol) to trigger
     W5500 ARP resolution.  Block until SENDOK so the caller knows
     the ARP cache is warm before stamping t3.

     A spoofed/unroutable source must not stall the main loop (which also
     services PPS disciplining and DHCP) for the chip's default ~1.8s ARP
     retry budget, so shrink RTR/RCR for the prime: 40ms/try x 2 tries
     ~= 80ms worst case, then restore.  A live LAN host answers ARP in
     well under 40ms.

     On the fast accessors throughout: RTR (0x0019-0x001A) and RCR (0x001B) are
     contiguous in the common register block, so saving and restoring the retry
     budget is one 3-byte read and two 3-byte writes rather than six accesses.
     The staging is shared with the reply path. */
  uint8_t saved[3];
  w5k_xfer_rd(W5500_CREG(W5500_RTR), saved, 3);
  const uint8_t tight[3] = { 0x01, 0x90, 0x01 };  /* RTR=400 (40ms), RCR=1 */
  w5k_xfer_wr(W5500_CREG(W5500_RTR), tight, 3);

  /* Clear stale completion flags first: otherwise the poll below can see a
   * previous send's SENDOK and return before ARP has actually resolved. */
  uint8_t clr = (uint8_t)(W5500_IR_SENDOK | W5500_IR_TIMEOUT);
  w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_IR), &clr, 1);

  uint8_t dummy = 0;
  uint16_t off = 0;
  int ret = -1;
  if (w5k_send_stage(socket_num, &dummy, 1, ip, 9, &off, NULL) == 0) {
    uint8_t cmd = W5500_CR_SEND;
    w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_CR), &cmd, 1);
    /* Time-bounded rather than iteration-bounded: the chip's own TIMEOUT fires
     * first at ~80ms, this is only a backstop against a wedged interface. */
    const int64_t deadline = esp_timer_get_time() + 120000;
    while (esp_timer_get_time() < deadline) {
      uint8_t ir = 0;
      g_w5k_prime_polls++;
      w5k_xfer_rd(W5500_SREG(socket_num, W5500_SN_IR), &ir, 1);
      if (ir & W5500_IR_SENDOK) {
        uint8_t c = W5500_IR_SENDOK;
        w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_IR), &c, 1);
        ret = 0;
        break;
      }
      if (ir & W5500_IR_TIMEOUT) {
        uint8_t c = W5500_IR_TIMEOUT;
        w5k_xfer_wr(W5500_SREG(socket_num, W5500_SN_IR), &c, 1);
        break;
      }
    }
  }

  w5k_xfer_wr(W5500_CREG(W5500_RTR), saved, 3);
  g_w5k_primes++;
  return ret;
}
