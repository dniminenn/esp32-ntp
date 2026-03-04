idf_component_register(
  SRCS "app_main.cpp"
  INCLUDE_DIRS "."
  REQUIRES config display gps ntp_server ntp_stats w5500_eth wifi_sta esp_event esp_timer nvs_flash esp_netif esp_driver_uart
)


