/*
  MobaStation: Arduino Mega + Arduino Motor Shield + ESP8266 with the MobaStation firmware

  Flash it as it is, there is nothing to configure. The ESP (extras/esp/MobaStationESP, on the
  Mega + WiFi R3 board or a separate one at Serial3) opens the access point "MobaStation"
  (password "MobaStation"), the web interface at http://192.168.0.111 sets up the router
  and everything else. See the README for the wiring.

  Standard pins (defaults in MobaStation.h):
    ESP8266          Serial3 (RX3 = 15, TX3 = 14), WiFi R3 board: DIP switches 1-4 ON
    Motor Shield     main track channel A, programming track channel B
    MCP2515 (CAN)    CS 53, INT 21, SPI 50-52, MobaBus is off when no module answers
    MAX485           Serial1 (TX1 = 18, RX1 = 19), direction pin 22, XpressNet throttles
    power button     pin 24 to GND, status LED pin 25, track voltage divider at A15
*/
#include <Arduino.h>

#include <MobaStation.h>

#include "ESP_Link.h"

ESP_Link wifi(&WIFI_SERIAL);

MobaBus mobaBus(MOBABUS_EEPROM, MOBABUS_PROG_PIN, MOBABUS_LED_PIN);
MobaBus_CAN can(MOBABUS_CAN_CS, MOBABUS_CAN_SPEED, MOBABUS_CAN_CLOCK, MOBABUS_CAN_INT);

XpressNetMasterClass XpressNet;


void setup() {
  Serial.begin(115200);

  //must be called before attaching interfaces and modules
  mobaBus.begin(true);
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);

  MobaStation::attachEthInterface(&wifi);

  MobaStation::attachXpressNet(&XpressNet, XPRESSNET_CONTROL_PIN);

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);
}

void loop() {
  MobaStation::loop();
}
