SHELL := /usr/bin/bash

# SPDX-License-Identifier: MIT-0

# Adjust if your ESP-IDF lives elsewhere
IDF_PATH ?= /home/jr/esp/esp-idf
IDF_EXPORT := . $(IDF_PATH)/export.sh &&

# Common vars
IDF_TARGET ?= esp32
PORT ?= /dev/ttyUSB0
BAUD ?= 921600
JOBS ?= $(shell nproc)

.PHONY: all build flash monitor flash-monitor erase clean fullclean size menuconfig doctor

all: build

build:
	$(IDF_EXPORT) IDF_CCACHE_ENABLE=1 idf.py build

flash:
	$(IDF_EXPORT) IDF_CCACHE_ENABLE=1 idf.py -p $(PORT) -b $(BAUD) flash

monitor:
	$(IDF_EXPORT) idf.py -p $(PORT) monitor

flash-monitor:
	$(IDF_EXPORT) IDF_CCACHE_ENABLE=1 idf.py -p $(PORT) -b $(BAUD) flash monitor

erase:
	$(IDF_EXPORT) idf.py -p $(PORT) erase-flash

size:
	$(IDF_EXPORT) idf.py size

menuconfig:
	$(IDF_EXPORT) idf.py menuconfig

clean:
	$(IDF_EXPORT) idf.py clean

fullclean:
	rm -rf build

doctor:
	$(IDF_EXPORT) idf.py doctor


