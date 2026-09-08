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

# The IDF version this tree was last configured with, and the one at IDF_PATH.
IDF_WANT := $(shell sed -n 's/^    version: \([0-9.]*\)$$/\1/p' dependencies.lock)
IDF_HAVE := $(shell sed -n 's/^set(IDF_VERSION_[A-Z]* \([0-9]*\))$$/\1/p' $(IDF_PATH)/tools/cmake/version.cmake 2>/dev/null | paste -sd.)
DOCKER_IMAGE ?= espressif/idf:v$(IDF_WANT)

.PHONY: all target build docker-build flash monitor flash-monitor erase clean fullclean size menuconfig doctor

all: build

target:
	@if [ -n "$(IDF_HAVE)" ] && [ "$(IDF_HAVE)" != "$(IDF_WANT)" ]; then echo "warning: IDF $(IDF_HAVE) at $(IDF_PATH), this tree is built against $(IDF_WANT)"; fi
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

# Same toolchain CI uses, no local IDF needed. Always a clean build.
docker-build:
	docker run --rm -u $(shell id -u):$(shell id -g) -e HOME=/tmp -v $(CURDIR):/project -w /project $(DOCKER_IMAGE) idf.py set-target $(TARGET) build


