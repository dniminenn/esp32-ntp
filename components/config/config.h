#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include "driver/spi_master.h"

namespace Config {
// Load settings from NVS. safeMode serves build-time defaults and skips NVS
// entirely, so a committed setting can never make the device unrecoverable.
void init(bool safeMode);
bool isSafeMode();
bool isProvisioned();

int getNtpServerPort();
int getStatsPort();

bool getUseDisplay();
bool getUsePreSyncGlyph();
spi_host_device_t getSpiHost();
int getCsPin();
int getMaxDevices();
int getSpiClockHz();
int getSpiMosiPin();
int getSpiMisoPin();
int getSpiSclkPin();

const char* getTimezone();
void applyTimezone();

// GPS / PPS configuration
int getGpsUartPort();
int getGpsUartBaud();
int getGpsUartTxPin();
int getGpsUartRxPin();
int getPpsGpioPin();
int64_t getPpsCalibrationUs();

// DS3231 RTC (optional; -1 = not fitted)
int getRtcSdaPin();
int getRtcSclPin();
int getRtc32kPin();

// Network: WIZnet or WiFi
bool getNetworkWiznet();
bool getNetworkWifi();

// W5500 Ethernet configuration (when getNetworkWiznet())
spi_host_device_t getW5500SpiHost();
int getW5500CsPin();
int getW5500MosiPin();
int getW5500MisoPin();
int getW5500SclkPin();
int getW5500IntPin();
int getW5500SpiHz();
int getServeCalibrationUs();
int getW5500RstPin();

// WiFi STA (when getNetworkWifi())
const char* getWifiSsid();
const char* getWifiPassword();

// IP config: shared by Wiznet and WiFi when static
bool getUseStaticIp();
const char* getStaticIp();
const char* getStaticGw();
const char* getStaticNetmask();
}


