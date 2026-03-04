// SPDX-License-Identifier: MIT-0

#include "w5500_eth.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static bool parse_ip4(const char* str, uint8_t* out) {
  unsigned a, b, c, d;
  if (sscanf(str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
  if (a > 255 || b > 255 || c > 255 || d > 255) return false;
  out[0] = (uint8_t)a;
  out[1] = (uint8_t)b;
  out[2] = (uint8_t)c;
  out[3] = (uint8_t)d;
  return true;
}

extern "C" {
#include "wizchip_conf.h"
#include "W5500/w5500.h"
#include "dhcp.h"
}

static const char* TAG = "W5500";

static spi_device_handle_t g_spi_handle = nullptr;
static int g_cs_pin = -1;
static int g_rst_pin = -1;

static const uint8_t DHCP_SOCKET_NUM = 0;
static uint8_t s_dhcp_buf[548];

static void wizchip_select(void) {
  if (g_spi_handle) {
    spi_device_acquire_bus(g_spi_handle, portMAX_DELAY);
  }
  if (g_cs_pin >= 0) {
    gpio_set_level((gpio_num_t)g_cs_pin, 0);
  }
}

static void wizchip_deselect(void) {
  if (g_cs_pin >= 0) {
    gpio_set_level((gpio_num_t)g_cs_pin, 1);
  }
  if (g_spi_handle) {
    spi_device_release_bus(g_spi_handle);
  }
}

static uint8_t wizchip_read(void) {
  uint8_t rx_data = 0;
  if (g_spi_handle) {
    spi_transaction_t t = {};
    t.length = 8;
    t.rxlength = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    spi_device_polling_transmit(g_spi_handle, &t);
    rx_data = t.rx_data[0];
  }
  return rx_data;
}

static void wizchip_write(uint8_t wb) {
  if (g_spi_handle) {
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_data[0] = wb;
    t.flags = SPI_TRANS_USE_TXDATA;
    spi_device_polling_transmit(g_spi_handle, &t);
  }
}

static void wizchip_readburst(uint8_t* pBuf, uint16_t len) {
  if (g_spi_handle && pBuf && len > 0) {
    spi_transaction_t t = {};
    t.length = len * 8;
    t.rxlength = len * 8;
    t.rx_buffer = pBuf;
    spi_device_polling_transmit(g_spi_handle, &t);
  }
}

static void wizchip_writeburst(uint8_t* pBuf, uint16_t len) {
  if (g_spi_handle && pBuf && len > 0) {
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = pBuf;
    spi_device_polling_transmit(g_spi_handle, &t);
  }
}

W5500Eth::W5500Eth()
  : eth_netif(nullptr), linkUp(false), intPin(-1), rstPin(-1) {
}

W5500Eth::~W5500Eth() {
  stop();
}

esp_err_t W5500Eth::begin(spi_host_device_t spiHost, int mosiPin, int misoPin, int sclkPin, int csPin, int intPin_, int rstPin_, int clockHz) {
  ESP_LOGI(TAG, "Initializing W5500 Ethernet...");
  
  intPin = intPin_;
  rstPin = rstPin_;
  g_rst_pin = rstPin_;
  g_cs_pin = csPin;
  
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << csPin);
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)csPin, 1);
  
  if (rstPin >= 0) {
    io_conf.pin_bit_mask = (1ULL << rstPin);
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)rstPin, 1);
  }
  
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = mosiPin;
  buscfg.miso_io_num = misoPin;
  buscfg.sclk_io_num = sclkPin;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 4092;
  
  esp_err_t ret = spi_bus_initialize(spiHost, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    return ret;
  }
  
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 20 * 1000 * 1000;
  devcfg.mode = 0;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 7;
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  
  ret = spi_bus_add_device(spiHost, &devcfg, &g_spi_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    spi_bus_free(spiHost);
    return ret;
  }
  
  if (rstPin >= 0) {
    ESP_LOGI(TAG, "Resetting W5500...");
    gpio_set_level((gpio_num_t)rstPin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level((gpio_num_t)rstPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
  } else {
    ESP_LOGW(TAG, "No reset pin configured - recommend connecting RST pin");
  }
  
  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
  reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
  reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);
  
  uint8_t memsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
  if (wizchip_init(memsize, memsize) < 0) {
    ESP_LOGE(TAG, "W5500 initialization failed");
    spi_bus_remove_device(g_spi_handle);
    spi_bus_free(spiHost);
    g_spi_handle = nullptr;
    return ESP_FAIL;
  }
  
  uint8_t version = 0;
  for (int i = 0; i < 3; ++i) {
    version = getVERSIONR();
    if (version == 0x04) break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  ESP_LOGI(TAG, "W5500 version: 0x%02x (expected 0x04)", version);
  
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  mac[0] = 0x02;
  
  wiz_NetInfo netinfo = {};
  memcpy(netinfo.mac, mac, 6);
  netinfo.dhcp = NETINFO_DHCP;
  wizchip_setnetinfo(&netinfo);
  
  wiz_PhyConf phyconf = {};
  phyconf.by = PHY_CONFBY_SW;
  phyconf.mode = PHY_MODE_AUTONEGO;
  phyconf.speed = PHY_SPEED_100;
  phyconf.duplex = PHY_DUPLEX_FULL;
  wizphy_setphyconf(&phyconf);
  
  uint8_t phycfgr = getPHYCFGR();
  ESP_LOGI(TAG, "PHYCFGR: 0x%02x (Link:%d Speed:%d Duplex:%d)", 
           phycfgr, 
           (phycfgr & 0x01) ? 1 : 0,
           (phycfgr & 0x02) ? 100 : 10,
           (phycfgr & 0x04) ? 1 : 0);
  
  ESP_LOGI(TAG, "W5500 initialized successfully");
  return ESP_OK;
}

esp_err_t W5500Eth::start(bool use_static_ip,
                         const char* static_ip,
                         const char* static_gw,
                         const char* static_netmask) {
  ESP_LOGI(TAG, "Starting Ethernet...");

  int retry = 0;
  while (retry < 50) {
    if (wizphy_getphylink() == PHY_LINK_ON) {
      linkUp = true;
      ESP_LOGI(TAG, "Ethernet link is up");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    retry++;
  }

  if (!linkUp) {
    ESP_LOGW(TAG, "No Ethernet link detected");
    return ESP_ERR_TIMEOUT;
  }

  wiz_NetInfo netinfo = {};
  getSHAR(netinfo.mac);

  if (use_static_ip && static_ip && static_gw && static_netmask) {
    uint8_t ip[4], gw[4], sn[4];
    if (parse_ip4(static_ip, ip) && parse_ip4(static_gw, gw) && parse_ip4(static_netmask, sn)) {
      memcpy(netinfo.ip, ip, 4);
      memcpy(netinfo.gw, gw, 4);
      memcpy(netinfo.sn, sn, 4);
      netinfo.dns[0] = netinfo.dns[1] = netinfo.dns[2] = netinfo.dns[3] = 0;
      netinfo.dhcp = NETINFO_STATIC;
      wizchip_setnetinfo(&netinfo);
      wizchip_getnetinfo(&netinfo);
      ESP_LOGI(TAG, "Static IP: %d.%d.%d.%d  GW: %d.%d.%d.%d  SN: %d.%d.%d.%d",
               netinfo.ip[0], netinfo.ip[1], netinfo.ip[2], netinfo.ip[3],
               netinfo.gw[0], netinfo.gw[1], netinfo.gw[2], netinfo.gw[3],
               netinfo.sn[0], netinfo.sn[1], netinfo.sn[2], netinfo.sn[3]);
    } else {
      ESP_LOGE(TAG, "Invalid static IP/gw/netmask, falling back to DHCP");
      use_static_ip = false;
    }
  }

  if (!use_static_ip) {
    ESP_LOGI(TAG, "Starting DHCP on W5500...");
    DHCP_init(DHCP_SOCKET_NUM, s_dhcp_buf);
    reg_dhcp_cbfunc(nullptr, nullptr, nullptr);

    uint32_t elapsedMs = 0;
    uint32_t tickAccumMs = 0;
    uint8_t dhcpState = DHCP_RUNNING;
    const uint32_t timeoutMs = 30000;

    while (elapsedMs < timeoutMs) {
      vTaskDelay(pdMS_TO_TICKS(100));
      elapsedMs += 100;
      tickAccumMs += 100;
      if (tickAccumMs >= 1000) {
        DHCP_time_handler();
        tickAccumMs -= 1000;
      }

      dhcpState = DHCP_run();
      if (dhcpState == DHCP_IP_ASSIGN ||
          dhcpState == DHCP_IP_CHANGED ||
          dhcpState == DHCP_IP_LEASED) {
        break;
      }
      if (dhcpState == DHCP_FAILED ||
          dhcpState == DHCP_STOPPED) {
        break;
      }
    }

    if (dhcpState == DHCP_IP_ASSIGN ||
        dhcpState == DHCP_IP_CHANGED ||
        dhcpState == DHCP_IP_LEASED) {
      wizchip_getnetinfo(&netinfo);
      ESP_LOGI(TAG, "DHCP acquired IP: %d.%d.%d.%d  GW: %d.%d.%d.%d  SN: %d.%d.%d.%d",
               netinfo.ip[0], netinfo.ip[1], netinfo.ip[2], netinfo.ip[3],
               netinfo.gw[0], netinfo.gw[1], netinfo.gw[2], netinfo.gw[3],
               netinfo.sn[0], netinfo.sn[1], netinfo.sn[2], netinfo.sn[3]);
    } else {
      ESP_LOGW(TAG, "DHCP failed (state=%d), falling back to static IP", dhcpState);
      uint8_t ip[4] = {192, 168, 69, 2};
      uint8_t sn[4] = {255, 255, 255, 0};
      uint8_t gw[4] = {192, 168, 69, 1};
      uint8_t dns[4] = {0, 0, 0, 0};
      memcpy(netinfo.ip, ip, 4);
      memcpy(netinfo.sn, sn, 4);
      memcpy(netinfo.gw, gw, 4);
      memcpy(netinfo.dns, dns, 4);
      netinfo.dhcp = NETINFO_STATIC;
      wizchip_setnetinfo(&netinfo);
      wizchip_getnetinfo(&netinfo);
    }
  }

  ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
           netinfo.mac[0], netinfo.mac[1], netinfo.mac[2],
           netinfo.mac[3], netinfo.mac[4], netinfo.mac[5]);

  return ESP_OK;
}

esp_err_t W5500Eth::stop() {
  if (g_spi_handle) {
    spi_bus_remove_device(g_spi_handle);
    g_spi_handle = nullptr;
  }
  linkUp = false;
  return ESP_OK;
}

void W5500Eth::loop() {
  bool newLinkUp = (wizphy_getphylink() == PHY_LINK_ON);
  if (newLinkUp != linkUp) {
    linkUp = newLinkUp;
    ESP_LOGI(TAG, "Ethernet link %s", linkUp ? "up" : "down");
  }
}

bool W5500Eth::isLinkUp() const {
  return linkUp;
}

esp_err_t W5500Eth::getMacAddr(uint8_t mac[6]) const {
  getSHAR(mac);
  return ESP_OK;
}

bool W5500Eth::getIpAddr(uint32_t& ip) const {
  uint8_t ip_arr[4];
  getSIPR(ip_arr);
  ip = (ip_arr[0] << 24) | (ip_arr[1] << 16) | (ip_arr[2] << 8) | ip_arr[3];
  return ip != 0;
}
