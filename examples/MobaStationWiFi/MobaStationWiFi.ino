/*
  MobaStation with an ESP8266 (AT firmware 1.7)

  Arduino Mega with an ESP-01 on Serial3, or a Mega + WiFi R3 board
  (DIP switches 1-4 ON, serial selector on RXD3/TXD3).

  All settings are in MobaStationConfig.h in the library folder, copy
  MobaStationConfig.example.h and fill in at least WIFI_SSID and WIFI_PWD:
    - WiFi:      WIFI_SSID, WIFI_PWD, WIFI_MODE, WIFI_SERIAL, WIFI_AP_PWD
    - network:   NETWORK_DHCP, NETWORK_IP, NETWORK_SUBNET, NETWORK_GATEWAY
    - hardware:  MOTOR_SHIELD_TYPE, DRIVE_ON_PROG, MOBABUS_*, XPRESSNET_*
*/
#include <Arduino.h>

#include <MobaStation.h>

#include "ESP_AT_Wifi.h"

ESP_AT_Wifi wifi(&WIFI_SERIAL, WIFI_SSID, WIFI_PWD, WIFI_MODE);

#if MOBABUS_ENABLED
//the MobaStation config uses the EEPROM addresses 0-75 and the client
//broadcast flags 3000-3511, MOBABUS_EEPROM must keep clear of those
MobaBus mobaBus(MOBABUS_EEPROM, MOBABUS_PROG_PIN, MOBABUS_LED_PIN);
MobaBus_CAN can(MOBABUS_CAN_CS, MOBABUS_CAN_SPEED, MOBABUS_CAN_CLOCK, MOBABUS_CAN_INT);
#endif

#if XPRESSNET_ENABLED
//MAX485 on Serial1 (TX1/RX1), direction pin XPRESSNET_CONTROL_PIN
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
