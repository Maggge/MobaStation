/*
  MobaStation with the MobaStation firmware on the ESP8266 (extras/esp/MobaStationESP)

  The ESP does WiFi, UDP and a web interface (status, network, settings, MobaBus modules
  and their CVs), the Mega exchanges Z21 packets with it in short frames instead of AT
  commands. Mega + WiFi R3 board: DIP switches 1-4 ON, serial selector on RXD3/TXD3.

  The WiFi settings of MobaStationConfig.h (WIFI_SSID, WIFI_PWD, WIFI_MODE, NETWORK_*,
  WIFI_AP_*) are sent to the ESP as defaults. Settings saved in the web interface win.
  Without router the ESP opens the access point WIFI_AP_SSID, web interface at NETWORK_IP.
*/
#include <Arduino.h>

#include <MobaStation.h>

#include "ESP_Link.h"

ESP_Link wifi(&WIFI_SERIAL, WIFI_SSID, WIFI_PWD, WIFI_MODE, WIFI_AP_SSID, WIFI_AP_PWD);

#if MOBABUS_ENABLED
MobaBus mobaBus(MOBABUS_EEPROM, MOBABUS_PROG_PIN, MOBABUS_LED_PIN);
MobaBus_CAN can(MOBABUS_CAN_CS, MOBABUS_CAN_SPEED, MOBABUS_CAN_CLOCK, MOBABUS_CAN_INT);
#endif

#if XPRESSNET_ENABLED
XpressNetMasterClass XpressNet;
#endif


void setup() {
  Serial.begin(115200);

  #if MOBABUS_ENABLED
  //must be called before attaching interfaces and modules
  mobaBus.begin(true);
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);
  #endif

  MobaStation::attachEthInterface(&wifi);

  #if XPRESSNET_ENABLED
  MobaStation::attachXpressNet(&XpressNet, XPRESSNET_CONTROL_PIN);
  #endif

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);
}

void loop() {
  MobaStation::loop();
}
