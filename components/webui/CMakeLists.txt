idf_component_register(
    SRCS "web_server.cpp" "web_metrics.cpp" "web_config.cpp" "web_log.cpp"
    INCLUDE_DIRS "."
    REQUIRES config gps ntp_server w5500_eth w5k wifi_sta lwip
)
