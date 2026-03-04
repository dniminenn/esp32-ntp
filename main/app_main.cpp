// SPDX-License-Identifier: MIT-0
// main/app_main.cpp

#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "esp_pm.h"

#include "config.h"
#include "display.h"
#include "gps.h"
#include "ntp_server.h"
#include "ntp_stats.h"
#include "w5500_eth.h"
#include "wifi_sta.h"

static const char* TAG = "APP";

static Display* g_display = nullptr;
static GpsDiscipline* g_gps = nullptr;
static NtpServer* g_ntpServer = nullptr;
static NtpStats* g_ntpStats = nullptr;
static W5500Eth* g_ethernet = nullptr;
static WifiSta* g_wifi = nullptr;
static SemaphoreHandle_t g_displayMutex = nullptr;

extern "C" void app_main();

static void display_task(void* arg) {
  int lastDisplayedSecond = -1;
  unsigned long long lastCentiseconds = 0;
  bool wasSynced = false;

  ESP_LOGI(TAG, "Display task started");

  while (true) {
    if (xSemaphoreTake(g_displayMutex, pdMS_TO_TICKS(10))) {
      struct timeval tv;
      gettimeofday(&tv, nullptr);
      struct tm timeinfo;
      localtime_r(&tv.tv_sec, &timeinfo);
      
      unsigned long long centiseconds = (unsigned long long)(esp_timer_get_time() / 10000ULL);
      
      if (centiseconds != lastCentiseconds) {
        int currentSecond = timeinfo.tm_sec;
        bool synced = (g_gps && g_gps->isLocked());
        if (synced && !wasSynced) {
          g_display->clear();
          lastDisplayedSecond = -1;
        }
        
        if (!synced) {
          g_display->clear();
          g_display->drawPreSyncGlyph();
        } else {
          if (currentSecond != lastDisplayedSecond) {
            ESP_LOGD(TAG, "Update display: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            g_display->drawTime(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            lastDisplayedSecond = currentSecond;
          }
          g_display->drawTopRowFromCentiseconds(centiseconds);
        }
        g_display->push();
        lastCentiseconds = centiseconds;
        wasSynced = synced;
      }
      
      xSemaphoreGive(g_displayMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void app_main() {
  uart_wait_tx_done(static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM), pdMS_TO_TICKS(100));
  vTaskDelay(pdMS_TO_TICKS(500));
  
  uart_flush_input(static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM));
  
  printf("\n\n\n");
  printf("========================================\n");
  printf("       esp32-ntp v1.0\n");
  printf("========================================\n\n");
  
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set(TAG, ESP_LOG_INFO);
  esp_log_level_set("spi_master", ESP_LOG_WARN);

  ESP_LOGI(TAG, "=== Clock Starting ===");
  ESP_LOGI(TAG, "Initializing NVS...");
  ESP_ERROR_CHECK(nvs_flash_init());
  
  // Set a reasonable initial time to avoid massive NTP corrections
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  ESP_LOGI(TAG, "Initial system time: %ld seconds (epoch: %ld)", tv.tv_sec, tv.tv_sec);
  
  if (tv.tv_sec < 1577836800LL) {  // Before 2020-01-01
    tv.tv_sec = 1700000000LL;  // Set to 2023-11-15 as reasonable default
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    ESP_LOGI(TAG, "Set initial time to 2023-11-15 to avoid massive NTP corrections");
    
    // Verify the change
    gettimeofday(&tv, nullptr);
    ESP_LOGI(TAG, "New system time: %ld seconds", tv.tv_sec);
  } else {
    ESP_LOGI(TAG, "System time already reasonable: %ld seconds", tv.tv_sec);
  }
  
  // Initialize networking infrastructure first
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  
  esp_err_t err;

  if (Config::getNetworkWiznet()) {
    ESP_LOGI(TAG, "Initializing W5500 Ethernet...");
    g_ethernet = new W5500Eth();
    err = g_ethernet->begin(
      Config::getW5500SpiHost(),
      Config::getW5500MosiPin(),
      Config::getW5500MisoPin(),
      Config::getW5500SclkPin(),
      Config::getW5500CsPin(),
      Config::getW5500IntPin(),
      Config::getW5500RstPin(),
      20000000
    );
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "W5500 initialized, starting Ethernet...");
      g_ethernet->start(Config::getUseStaticIp(), Config::getStaticIp(),
                       Config::getStaticGw(), Config::getStaticNetmask());
    } else {
      ESP_LOGE(TAG, "W5500 initialization failed: %s", esp_err_to_name(err));
      delete g_ethernet;
      g_ethernet = nullptr;
    }
  } else if (Config::getNetworkWifi()) {
    ESP_LOGI(TAG, "Initializing WiFi STA...");
    g_wifi = new WifiSta();
    err = g_wifi->begin(Config::getWifiSsid(), Config::getWifiPassword(),
                        Config::getUseStaticIp(), Config::getStaticIp(),
                        Config::getStaticGw(), Config::getStaticNetmask());
    if (err == ESP_OK) {
      g_wifi->start();
      ESP_LOGI(TAG, "WiFi STA started, connecting...");
    } else {
      ESP_LOGE(TAG, "WiFi begin failed: %s", esp_err_to_name(err));
      delete g_wifi;
      g_wifi = nullptr;
    }
  }
  
  ESP_LOGI(TAG, "Applying timezone: %s", Config::getTimezone());
  Config::applyTimezone();

  ESP_LOGI(TAG, "Waiting for network connection...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  if (Config::getUseDisplay()) {
    ESP_LOGI(TAG, "Initializing display...");
    g_display = new Display(Config::getSpiHost(), Config::getCsPin(), Config::getMaxDevices(), Config::getSpiClockHz());
    err = g_display->begin();
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Display initialized successfully");
      g_display->setIntensity(1);
      g_displayMutex = xSemaphoreCreateMutex();
      ESP_LOGI(TAG, "Display mutex created");
    } else {
      ESP_LOGE(TAG, "Display init failed: %d", err);
      delete g_display;
      g_display = nullptr;
    }
  } else {
    ESP_LOGI(TAG, "Display disabled via config");
  }

  // GPS/PPS will discipline time; NTP client not required.

  // Start GPS/PPS disciplining
  g_gps = new GpsDiscipline();
  g_gps->begin(Config::getGpsUartPort(), Config::getGpsUartBaud(), Config::getGpsUartTxPin(), Config::getGpsUartRxPin(), Config::getPpsGpioPin());

  // Start NTP server (passes GPS reference for lock checking)
  g_ntpServer = new NtpServer();
  g_ntpServer->begin(Config::getNtpServerPort(), g_gps);

  g_ntpStats = new NtpStats();
  g_ntpStats->begin(8080, g_gps, g_ntpServer, g_ethernet, g_wifi);
  gettimeofday(&tv, nullptr);
  struct tm timeinfo;
  localtime_r(&tv.tv_sec, &timeinfo);
  
  ESP_LOGI(TAG, "Initial time: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  if (g_display) {
    vTaskDelay(pdMS_TO_TICKS(50));
    g_display->clear();
    g_display->drawPreSyncGlyph();
    g_display->push();

    ESP_LOGI(TAG, "Starting display task...");
    xTaskCreatePinnedToCore(display_task, "display_task", 4096, nullptr, 4, nullptr, 1);
  }

  ESP_LOGI(TAG, "Entering main loop...");
  unsigned long lastLog = 0;
  unsigned int loopCount = 0;
  bool mainDiag = true;
  while (true) {
    // Process GPS PPS events with highest priority
    if (g_gps) g_gps->loop();
    
    // Process NTP requests (time-critical)
    if (g_ntpServer) g_ntpServer->loop();
    
    if ((loopCount++ % 10) == 0) {
      if (g_ethernet) g_ethernet->loop();
      if (g_wifi) g_wifi->loop();
      if (g_ntpStats) g_ntpStats->loop();
      if (mainDiag) { mainDiag = false; }

      unsigned long now = esp_timer_get_time() / 1000000ULL;
      if (now - lastLog >= 60) {
        gettimeofday(&tv, nullptr);
        localtime_r(&tv.tv_sec, &timeinfo);
        ESP_LOGI(TAG, "Current time: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        lastLog = now;
      }
    }
    
    vTaskDelay(1);
  }
}