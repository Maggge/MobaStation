#include <Arduino.h>

#include <MobaStation.h>

#include "Eth_Adapter.h"

const char * ip = "192.168.0.111";
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

Eth_Adapter eth(49, mac);

MobaBus mobaBus;
MobaBus_CAN can(53, CAN_125KBPS, MCP_8MHZ, 21);

#ifdef XPRESS_NET
XpressNetMasterClass XpressNet;
#endif

void setup() {
  Serial.begin(115200);
  
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);

  #ifdef XPRESS_NET
  MobaStation::attachXpressNet(&XpressNet, 22);
  #endif

  MobaStation::begin(&eth, ip, DRIVE_ON_PROG);

  

}

void loop() {
  MobaStation::loop();

}