// SPDX-License-Identifier: Unlicense
// main/app_main.cpp

#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "driver/uart.h"
#include "esp_pm.h"

#include "config.h"
#include "config_store.h"
#include "display.h"
#include "ds3231.h"
#include "gps.h"
#include "esp_task_wdt.h"
#include "ntp_server.h"
#include "web_server.h"
#include "w5500_eth.h"
#include "wifi_sta.h"

static const char* TAG = "APP";

static Display* g_display = nullptr;
static GpsDiscipline* g_gps = nullptr;
static NtpServer* g_ntpServer = nullptr;
static WebServer* g_web = nullptr;
static W5500Eth* g_ethernet = nullptr;
static WifiSta* g_wifi = nullptr;
static Ds3231* g_rtc = nullptr;
static SemaphoreHandle_t g_displayMutex = nullptr;

// Diagnostics exposed over /metrics (read-only; never on the timekeeping path).
// g_mainLoopBeats advances every core-0 loop iteration — if it stalls between
// two HTTP scrapes the loop is wedged; g_bootCount (persisted in NVS) counts
// boots, so an auto-recovery (watchdog reboot / W5500 restart) is visible
// remotely without attaching a serial monitor.
volatile uint32_t g_mainLoopBeats = 0;
uint32_t g_bootCount = 0;
// -1/0/1: unconfigured/dead/ok
volatile int g_rtcPresent = -1;
volatile float g_rtcOffsetSec = 0;   // RTC minus system, seconds

extern "C" void app_main();

static void display_task(void* arg) {
  int lastDisplayedSecond = -1;
  unsigned long long lastCentiseconds = 0;
  bool wasSynced = false;
  bool glyphShown = false;

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
          if (!glyphShown) {   /* static image; redraw only on entering the unsynced state */
            g_display->clear();
            g_display->drawPreSyncGlyph();
            g_display->push();
            glyphShown = true;
          }
        } else {
          glyphShown = false;
          if (currentSecond != lastDisplayedSecond) {
            ESP_LOGD(TAG, "Update display: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            g_display->drawTime(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            lastDisplayedSecond = currentSecond;
          }
          g_display->drawTopRowFromCentiseconds(centiseconds);
          g_display->push();
        }
        lastCentiseconds = centiseconds;
        wasSynced = synced;
      }
      
      xSemaphoreGive(g_displayMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* --- Task layout ------------------------------------------------------ */
/* Discipline must outrank serving: a packet flood must never delay a pulse. */
#define PRIO_DISCIPLINE 10
#define PRIO_NTP         7
#define CORE_CLOCK       0
#define CORE_NET         1

static void discipline_task(void* arg) {
  esp_task_wdt_add(nullptr);
  gps_register_task(xTaskGetCurrentTaskHandle());
  for (;;) {
    g_mainLoopBeats = g_mainLoopBeats + 1;              /* liveness, exported over /metrics */
    /*
     * Wake on the PPS capture edge instead of polling for it. The 50 ms timeout
     * is a safety net, not the mechanism: it keeps the holdover checks in
     * GpsDiscipline::loop() running when PPS stops, which is precisely when they
     * matter. NMEA is not affected — it has its own gps_uart task.
     *
     * Note this makes g_mainLoopBeats advance ~20x/s instead of ~500x/s. It is a
     * liveness heartbeat, so what matters is that it still advances.
     */
    gps_wait_for_pps(50);
    if (g_gps) g_gps->loop();
    esp_task_wdt_reset();
  }
}

static void ntp_task(void* arg) {
  esp_task_wdt_add(nullptr);
  unsigned long lastLog = 0;
  unsigned long lastHouse = 0;
  for (;;) {
    /* wake on INTn; drain socket */
    ntp_wait_for_packet(50 /* ms */);
    while (g_ntpServer && g_ntpServer->loop()) {}

    /*
     * Housekeeping shares this task deliberately. Every W5500 access funnels
     * through shared static frame buffers with no locking anywhere in the
     * driver (w5500_drv.c relies on this), so a second task touching the chip
     * corrupts transfers in flight. Serving from here keeps a single owner
     * without a mutex the hot path would have to acquire per packet.
     */
    /* time-gated; idle SPI stays quiet */
    unsigned long nowMs = (unsigned long)(esp_timer_get_time() / 1000ULL);
    if (nowMs - lastHouse >= 100) {
      lastHouse = nowMs;
      if (g_ethernet) g_ethernet->loop();
#if CONFIG_SOC_WIFI_SUPPORTED
      if (g_wifi) g_wifi->loop();
#endif
      if (g_web) g_web->loop();

      unsigned long now = esp_timer_get_time() / 1000000ULL;
      if (now - lastLog >= 60) {
        struct timeval tv2; struct tm ti2;
        gettimeofday(&tv2, nullptr);
        localtime_r(&tv2.tv_sec, &ti2);
        ESP_LOGI(TAG, "Current time: %02d:%02d:%02d", ti2.tm_hour, ti2.tm_min, ti2.tm_sec);
        lastLog = now;
      }
    }
    esp_task_wdt_reset();
  }
}

/* RTC housekeeping, off serving task. */
static void rtc_task(void* arg) {
  unsigned long lastTempMs = 0, lastCheckMs = 0, lastWriteMs = 0, lastSaveMs = 0;
  uint32_t lastSavedN = g_gps ? g_gps->tcxoSampleCount() : 0;  /* count restored at boot */
  bool wroteOnce = false;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    unsigned long nowMs = (unsigned long)(esp_timer_get_time() / 1000ULL);

    /* temp feeds tempcomp learning */
    if (nowMs - lastTempMs >= 10000) {
      lastTempMs = nowMs;
      float t;
      if (g_rtc->readTempC(t)) {
        g_rtcPresent = 1;
        if (g_gps) g_gps->setRtcTemp(t);
      } else {
        g_rtcPresent = 0;
      }
    }

    /* persist hourly, flash-friendly */
    if (g_gps && nowMs - lastSaveMs >= 3600000UL) {
      lastSaveMs = nowMs;
      uint32_t n = g_gps->tcxoSampleCount();
      if (n - lastSavedN >= 300 && g_gps->tcxoPersistToNvs())
        lastSavedN = n;
    }

    /* refresh battery-backed seed */
    if (g_gps && nowMs - lastCheckMs >= 60000) {
      lastCheckMs = nowMs;
      GpsStats gs;
      g_gps->getStats(gs);
      if (g_gps->isLocked() && !gs.holdover) {
        struct timeval now;
        gettimeofday(&now, nullptr);
        /* aligned RTC: skip near boundary */
        if (now.tv_usec < 100000 || now.tv_usec > 900000) continue;
        time_t rt;
        bool haveRt = g_rtc->readTime(rt);
        if (haveRt)
          g_rtcOffsetSec = (float)(rt - now.tv_sec);
        bool stale = !haveRt || g_rtcOffsetSec > 0.5f || g_rtcOffsetSec < -0.5f;
        bool refresh = wroteOnce && (nowMs - lastWriteMs >= 86400000UL);
        if (!wroteOnce || stale || refresh) {
          /* align write to second boundary */
          gettimeofday(&now, nullptr);
          vTaskDelay(pdMS_TO_TICKS((1000000 - now.tv_usec) / 1000 + 1));
          gettimeofday(&now, nullptr);
          time_t w = now.tv_sec + (now.tv_usec >= 500000 ? 1 : 0);
          if (g_rtc->setTime(w) == ESP_OK) {
            wroteOnce = true;
            lastWriteMs = nowMs;
            g_rtcOffsetSec = 0;
            ESP_LOGI(TAG, "DS3231 set from GPS time");
          }
        }
      }
    }
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

  // Persisted boot counter — a jump between scrapes reveals an auto-recovery
  // (watchdog reboot or W5500 restart) without needing a serial console.
  {
    nvs_handle_t h;
    if (nvs_open("diag", NVS_READWRITE, &h) == ESP_OK) {
      nvs_get_u32(h, "boots", &g_bootCount);
      g_bootCount++;
      nvs_set_u32(h, "boots", g_bootCount);
      nvs_commit(h);
      nvs_close(h);
    }
    ESP_LOGI(TAG, "Boot count: %u, reset reason: %d", (unsigned)g_bootCount, (int)esp_reset_reason());
  }

  {
    uint8_t attempt = cfg_boot_begin();
    bool safeMode = attempt >= CFG_SAFE_MODE_FAILS;
    ESP_LOGI(TAG, "Start attempt %u since the last start that reached the network",
             (unsigned)attempt);
    Config::init(safeMode);
    if (safeMode) {
      ESP_LOGE(TAG, "SAFE MODE on start attempt %u: build-time defaults, "
                    "stored settings ignored until you save from the config page",
               (unsigned)attempt);
    } else if (cfg_locked()) {
      ESP_LOGW(TAG, "Settings are locked; the config page is disabled");
    } else if (Config::isProvisioned()) {
      ESP_LOGI(TAG, "Using stored settings from NVS");
    } else {
      ESP_LOGI(TAG, "No stored settings, using build-time defaults");
    }
  }

  // seed clock before network
  if (Config::getRtcSdaPin() >= 0 && Config::getRtcSclPin() >= 0) {
    g_rtc = new Ds3231();
    if (g_rtc->begin(Config::getRtcSdaPin(), Config::getRtcSclPin()) == ESP_OK) {
      g_rtcPresent = 1;
      g_rtc->enable32kOutput();
      float bootTempC;
      if (g_rtc->readTempC(bootTempC))
        ESP_LOGI(TAG, "DS3231 die temp %.2fC", bootTempC);
      time_t t;
      if (g_rtc->readTime(t) && t > 1735689600LL) {  // sanity: after 2025-01-01
        // midpoint of truncated second
        struct timeval seed = { .tv_sec = t, .tv_usec = 500000 };
        settimeofday(&seed, nullptr);
        ESP_LOGI(TAG, "System time seeded from DS3231");
      } else {
        ESP_LOGW(TAG, "DS3231 present but its time is not trustworthy (OSF set or never set)");
      }
    } else {
      g_rtcPresent = 0;
      delete g_rtc;
      g_rtc = nullptr;
    }
  }

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
      Config::getW5500SpiHz()  // 20 MHz default; W5500 spec allows far more
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
#if CONFIG_SOC_WIFI_SUPPORTED
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
#endif
  }
  
  ESP_LOGI(TAG, "Applying timezone: %s", Config::getTimezone());
  Config::applyTimezone();

  ESP_LOGI(TAG, "Waiting for network connection...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  {
    uint32_t ip = 0;
    bool up = (g_ethernet && g_ethernet->getIpAddr(ip));
#if CONFIG_SOC_WIFI_SUPPORTED
    up = up || (g_wifi && g_wifi->getIpAddr(ip));
#endif
    if (up) cfg_boot_healthy();
  }

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
  // Latch W5500 packet arrival on a second MCPWM capture channel, sharing the
  // 80 MHz counter that timestamps PPS. Must come after gps->begin() (which
  // creates the capture timer) and before the server starts using it. Wired
  // (not wireless) only — lwIP buffers datagrams, so there is no edge to latch.
  if (!Config::getNetworkWifi() && Config::getW5500IntPin() >= 0) {
    esp_err_t rxerr = g_gps->beginRxCapture(Config::getW5500IntPin());
    if (rxerr == ESP_OK)
      ESP_LOGI(TAG, "RX arrival capture armed on GPIO%d (shared PPS timebase)",
               Config::getW5500IntPin());
    else
      ESP_LOGW(TAG, "RX arrival capture unavailable (%s); t2 falls back to the "
                    "GPIO ISR stamp", esp_err_to_name(rxerr));
  }

  // 32k TCXO holdover reference
  if (g_rtc && Config::getRtc32kPin() >= 0) {
    // restore learned tempcomp
    g_gps->tcxoRestoreFromNvs();
    esp_err_t terr = g_gps->beginTcxoCapture(Config::getRtc32kPin());
    if (terr == ESP_OK)
      ESP_LOGI(TAG, "DS3231 32kHz capture armed on GPIO%d (shared PPS timebase)",
               Config::getRtc32kPin());
    else
      ESP_LOGW(TAG, "DS3231 32kHz capture unavailable (%s); holdover coasts on "
                    "the frozen estimate", esp_err_to_name(terr));
  }

  g_ntpServer = new NtpServer();
  g_ntpServer->begin(Config::getNtpServerPort(), g_gps);
  if (g_ethernet) {
    // A W5500 register loss closes the NTP socket along with everything else;
    // reopen it whenever the ethernet layer rebuilds the chip.
    g_ethernet->onChipReset([](void*) {
      if (g_ntpServer) g_ntpServer->reopenSocket();
    }, nullptr);
  }

  g_web = new WebServer();
  g_web->begin(Config::getStatsPort(), g_gps, g_ntpServer, g_ethernet, g_wifi);
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
  /*
   * Split the old single poll loop into prioritised tasks.
   *
   * The monolithic loop ended in vTaskDelay(1), so a packet waited up to a full
   * tick before being answered — that random latency was the dominant term in
   * served jitter, and it also meant an arrival capture could not be reliably
   * paired with the datagram that produced it. Worse, NTP load and GPS
   * disciplining shared one thread, so traffic could starve the clock.
   *
   * Now: discipline runs at the highest priority on its own core, the NTP
   * server blocks on a notification from the INTn ISR (so it wakes on arrival
   * rather than on a timer), and housekeeping runs lowest. Each task registers
   * with the Task Watchdog, which is already configured to panic — a wedged
   * discipline task now reboots the box instead of serving a frozen clock.
   */
  xTaskCreatePinnedToCore(discipline_task, "discipline", 4096, nullptr,
                          PRIO_DISCIPLINE, nullptr, CORE_CLOCK);
  xTaskCreatePinnedToCore(ntp_task, "ntp_srv", 4096, nullptr,
                          PRIO_NTP, nullptr, CORE_NET);
  if (g_rtc)
    xTaskCreatePinnedToCore(rtc_task, "rtc_hk", 3072, nullptr, 3, nullptr, CORE_NET);
  ESP_LOGI(TAG, "tasks started: discipline(prio %d core %d) ntp(prio %d core %d)",
           PRIO_DISCIPLINE, CORE_CLOCK, PRIO_NTP, CORE_NET);
  /* app_main returns; the tasks own the system from here. */
}
