// SPDX-License-Identifier: MIT-0

#include "config.h"
#include "sdkconfig.h"
#include <time.h>

#ifndef CONFIG_APP_USE_PRESYNC_GLYPH
#define CONFIG_APP_USE_PRESYNC_GLYPH 0
#endif

namespace Config {

static const char* kNtpServer = "192.168.0.1";
static const int kNtpPort = 123;
static const int kNtpServerPort = 123; // for serving NTP

// Display on VSPI (SPI3_HOST)
static const spi_host_device_t kSpiHost = SPI3_HOST;  // VSPI for display
static const int kCsPin = 5;           // MAX7219 display CS pin
static const int kMaxDevices = 4;
static const int kSpiClockHz = 10 * 1000 * 1000;
static const int kSpiMosiPin = 23;     // VSPI MOSI
static const int kSpiMisoPin = -1;     // Display doesn't need MISO
static const int kSpiSclkPin = 18;     // VSPI SCLK

static const char* kTimezone = "AST4ADT,M3.2.0,M11.1.0";

// GPS / PPS
static const int kGpsUartPort = 2;   // UART2
static const int kGpsUartBaud = 9600;
static const int kGpsUartTxPin = 17; // not used by GPS NEO-6M
static const int kGpsUartRxPin = 16; // GPS TX -> ESP RX
static const int kPpsGpioPin = 19;   // PPS input (back to original pin!)
// PPS offset calibration (microseconds, can be positive or negative)
static const int64_t kPpsCalibrationUs = 0;  // MCPWM hardware capture — no ISR latency to compensate

// W5500 Ethernet on HSPI (SPI2_HOST) - separate bus from display
static const spi_host_device_t kW5500SpiHost = SPI2_HOST;  // HSPI for W5500
static const int kW5500CsPin = 25;     // W5500 CS pin
static const int kW5500MosiPin = 33;   // HSPI MOSI
static const int kW5500MisoPin = 35;   // HSPI MISO (GPIO 35 is input-only, perfect for MISO)
static const int kW5500SclkPin = 32;   // HSPI SCLK
static const int kW5500IntPin = 34;    // W5500 INT pin (GPIO 34 is input-only, perfect for INT)
static const int kW5500RstPin = 26;    // W5500 RST pin

const char* getNtpServer() { return kNtpServer; }
int getNtpPort() { return kNtpPort; }
int getNtpServerPort() { return kNtpServerPort; }

bool getUseDisplay() { return CONFIG_APP_USE_DISPLAY; }
bool getUsePreSyncGlyph() { return CONFIG_APP_USE_PRESYNC_GLYPH; }
spi_host_device_t getSpiHost() { return kSpiHost; }
int getCsPin() { return kCsPin; }
int getMaxDevices() { return kMaxDevices; }
int getSpiClockHz() { return kSpiClockHz; }
int getSpiMosiPin() { return kSpiMosiPin; }
int getSpiMisoPin() { return kSpiMisoPin; }
int getSpiSclkPin() { return kSpiSclkPin; }

const char* getTimezone() { return kTimezone; }
void applyTimezone() { setenv("TZ", kTimezone, 1); tzset(); }

int getGpsUartPort() { return kGpsUartPort; }
int getGpsUartBaud() { return kGpsUartBaud; }
int getGpsUartTxPin() { return kGpsUartTxPin; }
int getGpsUartRxPin() { return kGpsUartRxPin; }
int getPpsGpioPin() { return kPpsGpioPin; }
int64_t getPpsCalibrationUs() { return kPpsCalibrationUs; }

spi_host_device_t getW5500SpiHost() { return kW5500SpiHost; }
int getW5500CsPin() { return kW5500CsPin; }
int getW5500MosiPin() { return kW5500MosiPin; }
int getW5500MisoPin() { return kW5500MisoPin; }
int getW5500SclkPin() { return kW5500SclkPin; }
int getW5500IntPin() { return kW5500IntPin; }
int getW5500RstPin() { return kW5500RstPin; }

bool getNetworkWiznet() {
#ifdef CONFIG_APP_NETWORK_WIZNET
  return true;
#else
  return false;
#endif
}
bool getNetworkWifi() {
#ifdef CONFIG_APP_NETWORK_WIFI
  return true;
#else
  return false;
#endif
}

const char* getWifiSsid() {
#ifdef CONFIG_APP_NETWORK_WIFI
  return CONFIG_APP_WIFI_SSID;
#else
  return "";
#endif
}
const char* getWifiPassword() {
#ifdef CONFIG_APP_NETWORK_WIFI
  return CONFIG_APP_WIFI_PASSWORD;
#else
  return "";
#endif
}

bool getUseStaticIp() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return true;
#else
  return false;
#endif
}
const char* getStaticIp() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return CONFIG_APP_STATIC_IP_ADDR;
#else
  return "192.168.1.100";
#endif
}
const char* getStaticGw() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return CONFIG_APP_STATIC_GW;
#else
  return "192.168.1.1";
#endif
}
const char* getStaticNetmask() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return CONFIG_APP_STATIC_NETMASK;
#else
  return "255.255.255.0";
#endif
}

}