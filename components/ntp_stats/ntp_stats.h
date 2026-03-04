#pragma once
// SPDX-License-Identifier: MIT-0
#include <stdint.h>
#include "esp_err.h"

class GpsDiscipline;
class NtpServer;
class W5500Eth;
class WifiSta;

class NtpStats {
public:
  NtpStats();
  esp_err_t begin(int port, GpsDiscipline* gps, NtpServer* ntp, W5500Eth* eth, WifiSta* wifi);
  void loop();

private:
  int sock;
  int port;
  GpsDiscipline* gps;
  NtpServer* ntp;
  W5500Eth* eth;
  WifiSta* wifi;
  bool useWifi;
  bool listening;
  bool disconnecting;
  bool startupLogged;
  int listen_sock;
  int client_sock;
  void handleConnection();
  bool tryStartListener();
};
