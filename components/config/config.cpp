// SPDX-License-Identifier: Unlicense

#include "config.h"
#include "config_store.h"
#include "sdkconfig.h"
#include <time.h>

namespace Config {

void init(bool safeMode) { cfg_init(safeMode); }
bool isSafeMode() { return cfg_safe_mode(); }
bool isProvisioned() { return cfg_provisioned(); }

int getNtpServerPort() { return cfg_int(CFG_NTP_PORT); }
int getStatsPort() { return cfg_int(CFG_STATS_PORT); }

bool getUseDisplay() { return cfg_int(CFG_DISP_EN) != 0; }
bool getUsePreSyncGlyph() { return cfg_int(CFG_DISP_GLYPH) != 0; }
spi_host_device_t getSpiHost() { return (spi_host_device_t)cfg_int(CFG_DISP_HOST); }
int getCsPin() { return cfg_int(CFG_DISP_CS); }
int getMaxDevices() { return cfg_int(CFG_DISP_NDEV); }
int getSpiClockHz() { return cfg_int(CFG_DISP_HZ); }
int getSpiMosiPin() { return cfg_int(CFG_DISP_MOSI); }
int getSpiMisoPin() { return -1; }     // Display doesn't need MISO
int getSpiSclkPin() { return cfg_int(CFG_DISP_SCLK); }

const char* getTimezone() { return cfg_str(CFG_SYS_TZ); }
void applyTimezone() { setenv("TZ", cfg_str(CFG_SYS_TZ), 1); tzset(); }

int getGpsUartPort() { return cfg_int(CFG_GPS_UART); }
int getGpsUartBaud() { return cfg_int(CFG_GPS_BAUD); }
int getGpsUartTxPin() { return cfg_int(CFG_GPS_TX); }
int getGpsUartRxPin() { return cfg_int(CFG_GPS_RX); }
int getPpsGpioPin() { return cfg_int(CFG_PPS_GPIO); }
int64_t getPpsCalibrationUs() { return (int64_t)cfg_int(CFG_PPS_CAL); }

int getRtcSdaPin() { return cfg_int(CFG_RTC_SDA); }
int getRtcSclPin() { return cfg_int(CFG_RTC_SCL); }
int getRtc32kPin() { return cfg_int(CFG_RTC_32K); }

// Served-time calibration, microseconds, SUBTRACTED from both t2 and t3.
//
// t2 is latched when INTn asserts, which is after the W5500 has stored the
// entire frame, so it is systematically late by the chip's receive latency —
// and NTP's offset formula turns that straight into a positive bias. Measured
// against a GPS stratum-1 reference on the same switch (no path asymmetry):
// +15 us median before compensation. Subtracting it from both timestamps
// shifts the reported clock without touching the round-trip delay.
//
// RE-DERIVED after the reply path was cut from ~2.1 ms to ~0.4 ms. The old 15 us
// was measured when the transmit path still stamped t3 well before the frame
// actually left, and that residual lateness partly cancelled the subtraction.
// With t3 now landing within ~5 us of egress the subtraction stopped being
// cancelled and over-corrected: served offset sat at -6..-12 us median against
// the GPS reference across three 60-sample runs. 7 us re-centres it.
int getServeCalibrationUs() { return cfg_int(CFG_SERVE_CAL); }

spi_host_device_t getW5500SpiHost() { return (spi_host_device_t)cfg_int(CFG_W5_HOST); }
int getW5500CsPin() { return cfg_int(CFG_W5_CS); }
int getW5500MosiPin() { return cfg_int(CFG_W5_MOSI); }
int getW5500MisoPin() { return cfg_int(CFG_W5_MISO); }
int getW5500SclkPin() { return cfg_int(CFG_W5_SCLK); }
int getW5500IntPin() { return cfg_int(CFG_W5_INT); }
// SPI clock for the W5500. 20 MHz is the long-proven value on this wiring.
// CEILING: reads are full duplex (the header is clocked out while the payload
// comes in), and full-duplex transfers routed through the GPIO matrix — which
// these are, since the SPI pins are arbitrary config values rather than IO_MUX
// pins — top out at 26.6 MHz. Past that, READS corrupt silently, which surfaces
// as wrong Sn_TX_FSR/Sn_TX_WR values and therefore malformed packets rather
// than an obvious failure. Do not exceed 26 MHz without moving SPI to the
// IO_MUX pins.
// TRIED AND REVERTED: 80 MHz / 3 = 26.67 MHz. The divider is integral, so that
// is the only step above 20 MHz that exists (anything between rounds back down
// to 80/4). At 26.67 MHz this board does not come up at all — no NTP, no
// /metrics, no ICMP — so the ceiling above is optimistic for this wiring rather
// than merely marginal. It stays at 20 MHz. Byte clocking is only ~100 us of the
// reply path anyway, so the most this could ever have returned is ~25 us.
int getW5500SpiHz() { return cfg_int(CFG_W5_HZ); }
int getW5500RstPin() { return cfg_int(CFG_W5_RST); }

bool getNetworkWiznet() { return cfg_int(CFG_NET_MODE) == 0; }
bool getNetworkWifi() { return cfg_int(CFG_NET_MODE) == 1; }

const char* getWifiSsid() { return cfg_str(CFG_WIFI_SSID); }
const char* getWifiPassword() { return cfg_str(CFG_WIFI_PASS); }

bool getUseStaticIp() { return cfg_int(CFG_NET_DHCP) == 0; }
const char* getStaticIp() { return cfg_str(CFG_NET_IP); }
const char* getStaticGw() { return cfg_str(CFG_NET_GW); }
const char* getStaticNetmask() { return cfg_str(CFG_NET_MASK); }

}
