#include <Arduino.h>

#include <MobaStation.h>

#include "ESP_AT_Wifi.h"

#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
#define DRIVE_ON_PROG true

const char *ssid ="ssid";
const char * pwd = "password";
IPAddress ip(192, 168, 0, 111);


ESP_AT_Wifi wifi(&Serial3, ssid, pwd);

MobaBus mobaBus;
MobaBus_CAN can(53, CAN_125KBPS, MCP_8MHZ, 21);

//XpressNetMasterClass XpressNet;


void setup() {
  Serial.begin(115200);
  
  mobaBus.attachInterface(&can);
  MobaStation::attachMobaBus(&mobaBus);

  MobaStation::attachEthInterface(&wifi, ip, Z21_STANDARD_PORT);

  //MobaStation::attachXpressNet(&XpressNet, 22);

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);

  

}

void loop() {
  MobaStation::loop();

}