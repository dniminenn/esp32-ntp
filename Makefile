SHELL := /usr/bin/bash

# SPDX-License-Identifier: Unlicense

# Adjust if your ESP-IDF lives elsewhere (or export IDF_PATH)
IDF_PATH ?= $(HOME)/esp/esp-idf-v6.0.2
IDF_EXPORT := . $(IDF_PATH)/export.sh &&

# sdkconfig is generated from sdkconfig.defaults by idf.py set-target. TARGET defaults to
# whatever the current sdkconfig was generated for, so `make build TARGET=esp32s3` switches
# and a bare `make build` stays put.
CUR_TARGET := $(shell sed -n 's/^CONFIG_IDF_TARGET="\(.*\)"$$/\1/p' sdkconfig 2>/dev/null)
TARGET ?= $(if $(CUR_TARGET),$(CUR_TARGET),esp32)
PORT ?= $(firstword $(wildcard /dev/ttyUSB0 /dev/ttyACM0) /dev/ttyUSB0)
BAUD ?= 921600
JOBS ?= $(shell nproc)

.PHONY: all target build flash monitor flash-monitor erase clean fullclean size menuconfig doctor

all: build

target:
	@if [ "$(CUR_TARGET)" != "$(TARGET)" ]; then $(IDF_EXPORT) idf.py set-target $(TARGET); fi

build: target
	$(IDF_EXPORT) IDF_CCACHE_ENABLE=1 idf.py build

flash: target
	$(IDF_EXPORT) IDF_CCACHE_ENABLE=1 idf.py -p $(PORT) -b $(BAUD) flash

monitor:
	$(IDF_EXPORT) idf.py -p $(PORT) monitor

flash-monitor: target
	$(IDF_EXPORT) IDF_CCACHE_ENABLE=1 idf.py -p $(PORT) -b $(BAUD) flash monitor

erase:
	$(IDF_EXPORT) idf.py -p $(PORT) erase-flash

size: target
	$(IDF_EXPORT) idf.py size

menuconfig: target
	$(IDF_EXPORT) idf.py menuconfig

clean:
	$(IDF_EXPORT) idf.py clean

fullclean:
	rm -rf build

doctor:
	$(IDF_EXPORT) idf.py doctor


