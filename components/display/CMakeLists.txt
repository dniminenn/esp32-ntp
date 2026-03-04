idf_component_register(SRCS "display.cpp" "max7219_chain.cpp" INCLUDE_DIRS "." REQUIRES config esp_driver_spi PRIV_REQUIRES esp_driver_gpio)


