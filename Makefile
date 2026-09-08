SHELL := /usr/bin/bash

# SPDX-License-Identifier: Unlicense

# Adjust if your ESP-IDF lives elsewhere (or export IDF_PATH)
IDF_PATH ?= $(HOME)/esp/esp-idf-v6.0.2
IDF_EXPORT := . $(IDF_PATH)/export.sh &&

# sdkconfig is generated from sdkconfig.defaults by idf.py set-target. TARGET defaults to
# whatever the current sdkconfig was generated for, so `make build TARGET=esp32s3` switches
# and a bare `make build` stays put.
CUR_TARGET := $(shell sed -n 's/\r$$//; s/^CONFIG_IDF_TARGET="\(.*\)"$$/\1/p' sdkconfig 2>/dev/null)
TARGET ?= $(if $(CUR_TARGET),$(CUR_TARGET),esp32)
PORT ?= $(firstword $(wildcard /dev/ttyUSB0 /dev/ttyACM0) /dev/ttyUSB0)
BAUD ?= 921600
JOBS ?= $(shell nproc)

# The IDF version this tree was last configured with, and the one at IDF_PATH.
IDF_WANT := $(shell awk '/^  idf:/{f=1} f && /^    version:/{print $$2; exit}' dependencies.lock 2>/dev/null)
IDF_HAVE := $(shell sed -n 's/^set(IDF_VERSION_[A-Z]* \([0-9]*\))$$/\1/p' $(IDF_PATH)/tools/cmake/version.cmake 2>/dev/null | paste -sd.)
DOCKER_IMAGE ?= espressif/idf:v$(IDF_WANT)

# Settings image for a board that cannot reach its own settings page yet. See settings.example.csv.
SETTINGS ?= settings.csv
NVS_OFFSET := $(shell awk -F, '/^nvs,/{gsub(/ /,"");print $$4}' partitions.csv)
NVS_SIZE := $(shell awk -F, '/^nvs,/{gsub(/ /,"");print $$5}' partitions.csv)

.PHONY: all target build docker-build nvs nvs-erase flash monitor flash-monitor erase clean fullclean size menuconfig doctor

all: build

target:
	@test -n "$(IDF_WANT)" || { echo "run make from the project root"; exit 1; }
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

nvs:
	@test -f "$(SETTINGS)" || { echo "$(SETTINGS) not found, start from settings.example.csv"; exit 1; }
	@mkdir -p build
	$(IDF_EXPORT) python -m esp_idf_nvs_partition_gen generate "$(SETTINGS)" build/settings.bin $(NVS_SIZE)
	$(IDF_EXPORT) python -m esptool -p $(PORT) -b $(BAUD) write-flash $(NVS_OFFSET) build/settings.bin

nvs-erase:
	$(IDF_EXPORT) python -m esptool -p $(PORT) -b $(BAUD) erase-region $(NVS_OFFSET) $(NVS_SIZE)

# Same toolchain CI uses, no local IDF needed. Always a clean build.
docker-build:
	@test -n "$(IDF_WANT)" || { echo "run make from the project root"; exit 1; }
	docker run --rm -u $(shell id -u):$(shell id -g) -e HOME=/tmp -e IDF_GIT_SAFE_DIR='*' \
	  -v "$(CURDIR):/project" -w /project $(DOCKER_IMAGE) idf.py set-target $(TARGET) build


