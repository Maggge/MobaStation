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
# ESP8266 with the MobaStation firmware (extras/esp/MobaStationESP, Mega sketch examples/MobaStationESP):
#   make esp-setup     install the ESP8266 core and the web server libraries
#   make esp-build     compile the ESP firmware
#   DIP switches 5-7 ON (USB <-> ESP flash mode), others OFF, then unplug and plug in the board:
#   make esp-info      chip and flash size
#   make esp-backup    save the whole flash of the ESP (eg. the AT firmware) to extras/esp/backup
#   make esp-upload    flash the firmware
#   make esp-restore BACKUP=extras/esp/backup/<file>.bin   write a backup back
#   then DIP switches 1-4 ON, 5-8 OFF and power the board again.
#   make esp-ota       later updates over WiFi (ESP_IP, default Z21_IP)
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

.PHONY: help setup build upload monitor log z21test test clean esp-setup esp-build esp-info esp-backup esp-upload esp-restore esp-ota

help:
	@sed -n '3,/^$$/p' Makefile | sed 's/^# \{0,1\}//'

setup:
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install arduino:avr@1.8.8
	$(ARDUINO_CLI) lib install Ethernet Servo mcp_can "Adafruit PWM Servo Driver Library"
	@[ -d $(DCC_DIR) ] || git clone $(GITHUB)/DCC.git $(DCC_DIR)
	@[ -d $(MOBABUS_DIR) ] || git clone $(GITHUB)/MobaBus.git $(MOBABUS_DIR)
	@[ -f MobaStationConfig.h ] || { cp MobaStationConfig.example.h MobaStationConfig.h; \
		echo "MobaStationConfig.h created, fill in your settings"; }

# extra compiler flags, eg. make upload EXTRA_FLAGS=-DMOBASTATION_LOOP_STATS
EXTRA_FLAGS ?=

build:
	$(ARDUINO_CLI) compile --fqbn $(FQBN) $(LIBS) --warnings default \
		--build-property "build.extra_flags=$(EXTRA_FLAGS)" \
		--output-dir $(BUILD_DIR) $(SKETCH)

upload: build
	$(ARDUINO_CLI) upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_DIR) $(SKETCH)

monitor:
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=$(BAUD)

# make log RESET=noreset listens without restarting the board
RESET ?=
log:
	python3 extras/serial_log.py $(PORT) $(SECONDS) $(BAUD) $(RESET)

z21test:
	python3 extras/z21test.py $(Z21_IP) selftest

# host tests: protocol (MobaStation.cpp + src/XpressNetMaster.cpp), WiFi driver, Ethernet adapter,
# ESP link codec and the Mega side of the ESP link
TEST_DIR   = extras/test
TEST_FLAGS = -std=gnu++17 -DARDUINO=10819 -Wall -Wextra -Wno-unused-parameter -I$(TEST_DIR)/stub -Isrc

test:
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/ms_test $(TEST_DIR)/ms_test.cpp src/XpressNetMaster.cpp
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/wifi_test $(TEST_DIR)/wifi_test.cpp src/WiFi_ESP_AT.cpp
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/eth_test $(TEST_DIR)/eth_test.cpp src/Eth_Adapter.cpp
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/esplink_test $(TEST_DIR)/esplink_test.cpp
	$(CXX) $(TEST_FLAGS) -o $(TEST_DIR)/esp_link_mega_test $(TEST_DIR)/esp_link_mega_test.cpp src/ESP_Link.cpp src/XpressNetMaster.cpp
	$(TEST_DIR)/ms_test
	$(TEST_DIR)/wifi_test
	$(TEST_DIR)/eth_test
	$(TEST_DIR)/esplink_test
	$(TEST_DIR)/esp_link_mega_test

clean:
	rm -rf build $(TEST_DIR)/ms_test $(TEST_DIR)/wifi_test $(TEST_DIR)/eth_test $(TEST_DIR)/esplink_test $(TEST_DIR)/esp_link_mega_test

#---------------------------------------------------------------- ESP8266 firmware
ESP_URL        = https://arduino.esp8266.com/stable/package_esp8266com_index.json
ESP_FQBN      ?= esp8266:esp8266:generic:eesz=4M1M
ESP_SKETCH     = extras/esp/MobaStationESP
ESP_BUILD      = build/MobaStationESP-esp
ESP_BIN        = $(ESP_BUILD)/MobaStationESP.ino.bin
ESP_IP        ?= $(Z21_IP)
ESPTOOL       ?= esptool
ESP_BAUD      ?= 460800
ESP_BACKUP_DIR = extras/esp/backup
#the board has no reset line to the ESP: DIP 7 holds it in the boot loader after power on
ESPTOOL_RUN    = $(ESPTOOL) --port $(PORT) --baud $(ESP_BAUD) --before no-reset --after no-reset

esp-setup:
	$(ARDUINO_CLI) core update-index --additional-urls $(ESP_URL)
	$(ARDUINO_CLI) core install esp8266:esp8266@3.1.2 --additional-urls $(ESP_URL)
	$(ARDUINO_CLI) lib install "ESP Async WebServer" "ESP Async TCP" ArduinoJson

#only src/esplink is added: src/ has a WiFi.h and WiFiUdp.h of its own that would hide the ESP core ones
esp-build:
	$(ARDUINO_CLI) compile --fqbn $(ESP_FQBN) --build-property "compiler.cpp.extra_flags=-I$(CURDIR)/src/esplink" \
		--warnings default --output-dir $(ESP_BUILD) $(ESP_SKETCH)

esp-info:
	$(ESPTOOL_RUN) flash-id

esp-backup:
	mkdir -p $(ESP_BACKUP_DIR)
	$(ESPTOOL_RUN) read-flash 0 ALL $(ESP_BACKUP_DIR)/esp-flash-$(shell date +%Y%m%d-%H%M%S).bin

esp-upload: esp-build
	$(ESPTOOL_RUN) write-flash 0x0 $(ESP_BIN)

esp-restore:
	@[ -n "$(BACKUP)" ] || { echo "make esp-restore BACKUP=extras/esp/backup/<file>.bin"; exit 1; }
	$(ESPTOOL_RUN) write-flash 0x0 $(BACKUP)

esp-ota: esp-build
	curl -f -F "firmware=@$(ESP_BIN)" http://$(ESP_IP)/update
