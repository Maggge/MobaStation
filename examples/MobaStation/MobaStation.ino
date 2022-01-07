#include <Arduino.h>

#include <MobaStation.h>

const char *name = "MobaStation";
const char *ssid ="ssid";
const char * pwd = "password";
const char * ip = "192.168.0.111";


ESP_AT_Wifi wifi(&Serial3, name, ssid, pwd);

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

  MobaStation::begin(&wifi, ip, DRIVE_ON_PROG);

  

}

void loop() {
  MobaStation::loop();

}