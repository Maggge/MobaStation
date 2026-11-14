# Development helpers for MobaStation (arduino-cli)
#
#   make setup      install the AVR core, the libraries and clone DCC/MobaBus next to this repo
#   make build      compile the sketch
#   make upload     compile and flash it to the board
#   make monitor    interactive serial console (exit with Ctrl+C)
#   make log        print the serial output for 20 seconds (resets the board)
#   make z21test    run the Z21 LAN selftest against the station
#   make test       host tests on the PC (g++, no board needed)
#
# Every variable can be overridden: make upload SKETCH=examples/MobaStationEth PORT=/dev/ttyUSB1
# The settings of the sketch (WiFi, IP, MobaBus, ...) are in MobaStationConfig.h.

SKETCH  ?= examples/MobaStationWiFi
PORT    ?= /dev/ttyUSB0
FQBN    ?= arduino:avr:mega
BAUD    ?= 115200
Z21_IP  ?= 192.168.30.111
SECONDS ?= 20

# the DCC and MobaBus libraries are used from their git checkouts next to this repo,
# changes there are compiled in directly (no copy into ~/Arduino/libraries needed)
DCC_DIR     ?= ../DCC
MOBABUS_DIR ?= ../MobaBus
GITHUB      ?= https://github.com/Maggge

ARDUINO_CLI ?= arduino-cli
LIBS = --library . --library $(DCC_DIR) --library $(MOBABUS_DIR)
BUILD_DIR = build/$(notdir $(SKETCH))

.PHONY: help setup build upload monitor log z21test test clean

help:
	@sed -n '3,13p' Makefile | sed 's/^# \{0,1\}//'

setup:
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install arduino:avr@1.8.8
	$(ARDUINO_CLI) lib install Ethernet Servo mcp_can "Adafruit PWM Servo Driver Library"
	@[ -d $(DCC_DIR) ] || git clone $(GITHUB)/DCC.git $(DCC_DIR)
	@[ -d $(MOBABUS_DIR) ] || git clone $(GITHUB)/MobaBus.git $(MOBABUS_DIR)
	@[ -f MobaStationConfig.h ] || { cp MobaStationConfig.example.h MobaStationConfig.h; \
		echo "MobaStationConfig.h created, fill in your settings"; }

build:
	$(ARDUINO_CLI) compile --fqbn $(FQBN) $(LIBS) --warnings default \
		--output-dir $(BUILD_DIR) $(SKETCH)

upload: build
	$(ARDUINO_CLI) upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_DIR) $(SKETCH)

monitor:
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=$(BAUD)

log:
	python3 extras/serial_log.py $(PORT) $(SECONDS) $(BAUD)

z21test:
	python3 extras/z21test.py $(Z21_IP) selftest

# host tests: protocol (MobaStation.cpp + XpressNetMaster.cpp), WiFi driver, Ethernet adapter
TEST_DIR   = extras/test
TEST_FLAGS = -std=gnu++17 -DARDUINO=10819 -Wall -Wextra -Wno-unused-parameter -I$(TEST_DIR)/stub -I.

test:
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/ms_test $(TEST_DIR)/ms_test.cpp XpressNetMaster.cpp
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/wifi_test $(TEST_DIR)/wifi_test.cpp WiFi_ESP_AT.cpp
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/eth_test $(TEST_DIR)/eth_test.cpp Eth_Adapter.cpp
	$(TEST_DIR)/ms_test
	$(TEST_DIR)/wifi_test
	$(TEST_DIR)/eth_test

clean:
	rm -rf build $(TEST_DIR)/ms_test $(TEST_DIR)/wifi_test $(TEST_DIR)/eth_test
