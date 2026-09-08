idf_component_register(SRCS "matrix_display.cpp" "max7219_chain.cpp" "ssd1306_display.cpp" INCLUDE_DIRS "." REQUIRES config esp_driver_spi esp_driver_i2c i2c_bus PRIV_REQUIRES esp_driver_gpio)
