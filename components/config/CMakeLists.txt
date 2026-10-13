idf_component_register(SRCS "config.cpp" "config_store.c" "config_csv.c" INCLUDE_DIRS "." REQUIRES esp_driver_spi nvs_flash)
