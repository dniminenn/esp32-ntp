idf_component_register(
    SRCS "ntp_stats.cpp"
    INCLUDE_DIRS "."
    REQUIRES config gps ntp_server w5500_eth w5k wifi_sta lwip
)
