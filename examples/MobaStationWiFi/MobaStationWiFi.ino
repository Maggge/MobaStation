#include <Arduino.h>

#include <MobaStation.h>

#include "ESP_AT_Wifi.h"

#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
#define DRIVE_ON_PROG true

//XpressNet MAX485 direction control pin, the data lines are on Serial1 (TX1/RX1)
//#define XPRESSNET_CONTROL_PIN 22

const char *ssid = "ssid";
const char *pwd = "password";

IPAddress ip(192, 168, 0, 111);
IPAddress subnetMask(255, 255, 255, 0);

//ESP-01 connected to Serial3, or the ESP of the Arduino Mega WiFi R3
ESP_AT_Wifi wifi(&Serial3, ssid, pwd);

//EEPROM storage for the MobaBus modules, the MobaStation config uses the addresses
//0-75 and the client broadcast flags 3000-3511, so keep clear of those
MobaBus mobaBus(1000);
MobaBus_CAN can(53, CAN_125KBPS, MCP_8MHZ, 21);

//XpressNetMasterClass XpressNet;


void setup() {
  Serial.begin(115200);

  //must be called before attaching interfaces and modules
  mobaBus.begin(true);
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);

  MobaStation::attachEthInterface(&wifi, ip, Z21_STANDARD_PORT, subnetMask);

  //MobaStation::attachXpressNet(&XpressNet, XPRESSNET_CONTROL_PIN);

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);

}

void loop() {
  MobaStation::loop();

}
