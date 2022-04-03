#include <Arduino.h>

#include <MobaStation.h>

#include "Eth_Adapter.h"

#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
#define DRIVE_ON_PROG true

IPAddress ip(192, 168, 0, 111);
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

Eth_Adapter eth(49, mac);

MobaBus mobaBus;
MobaBus_CAN can(53, CAN_125KBPS, MCP_8MHZ, 21);

//XpressNetMasterClass XpressNet;


void setup() {
  Serial.begin(115200);
  
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);
  MobaStation::attachEthInterface(&eth, ip, Z21_STANDARD_PORT);

  //MobaStation::attachXpressNet(&XpressNet, 22);

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);

}

void loop() {
  MobaStation::loop();

}