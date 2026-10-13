#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include "esp_err.h"

class GpsDiscipline;
class NtpServer;
class W5500Eth;
class WifiSta;

class WebServer {
public:
  WebServer();
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

  // A request is accumulated across loop() calls rather than waited for, so a
  // POST body that spans packets never blocks the task that also serves NTP.
  int reqLen;
  int hdrEnd;
  int contentLen;
  int64_t reqStartUs;

  void handleConnection();
  bool tryStartListener();
  void resetRequest();
  bool pumpRequest();
  void sendAll(const char* data, int len);
  void sendStatus(const char* status, const char* ctype, const char* body);
  void closeConn();
  void sendMetrics();
  void sendConfigPage(const char* notice);
  void sendConfigCsv();
  void renderSection(char** pp, char* end, bool advanced);
  void renderField(char** pp, char* end, int i);
  void handleConfigPost(char* body);
  bool authorized(const char* req);
};
