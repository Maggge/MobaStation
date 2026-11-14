/*
  MobaStation with a W5x00 Ethernet module

  All settings are in MobaStationConfig.h in the library folder, copy
  MobaStationConfig.example.h:
    - Ethernet:  ETH_CS_PIN, ETH_MAC, ETH_DHCP_TIMEOUT
    - network:   NETWORK_DHCP, NETWORK_IP, NETWORK_SUBNET, NETWORK_GATEWAY
    - hardware:  MOTOR_SHIELD_TYPE, DRIVE_ON_PROG, MOBABUS_*, XPRESSNET_*
*/
#include <Arduino.h>

#include <MobaStation.h>

#include "Eth_Adapter.h"

byte mac[] = { ETH_MAC };

Eth_Adapter eth(ETH_CS_PIN, mac);

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

  MobaStation::attachEthInterface(&eth);

  #if XPRESSNET_ENABLED
  MobaStation::attachXpressNet(&XpressNet, XPRESSNET_CONTROL_PIN);
  #endif

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);

}

void loop() {
  MobaStation::loop();

}
