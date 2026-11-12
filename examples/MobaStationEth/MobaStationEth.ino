#include <Arduino.h>

#include <MobaStation.h>

#include "Eth_Adapter.h"

#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
#define DRIVE_ON_PROG true

//XpressNet MAX485 direction control pin, the data lines are on Serial1 (TX1/RX1)
#define XPRESSNET_CONTROL_PIN 22

IPAddress ip(192, 168, 0, 111);
IPAddress subnetMask(255, 255, 255, 0);
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

Eth_Adapter eth(49, mac);

//EEPROM storage for the MobaBus modules, the MobaStation config uses the addresses
//0-75 and the client broadcast flags 3000-3511, so keep clear of those
MobaBus mobaBus(1000);
MobaBus_CAN can(53, CAN_125KBPS, MCP_8MHZ, 21);

XpressNetMasterClass XpressNet;


void setup() {
  Serial.begin(115200);

  //must be called before attaching interfaces and modules
  mobaBus.begin(true);
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);

  MobaStation::attachEthInterface(&eth, ip, Z21_STANDARD_PORT, subnetMask);

  MobaStation::attachXpressNet(&XpressNet, XPRESSNET_CONTROL_PIN);

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);

}

void loop() {
  MobaStation::loop();

}
