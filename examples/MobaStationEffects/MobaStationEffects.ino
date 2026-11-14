/*
  MobaStation with light effects on its own pins

  Like MobaStationWiFi, plus MobaBus modules that run on the Mega itself. The commands of
  the station (Z21 app, multiMAUS) reach them directly, without a CAN bus:
    house with TV       accessory 1001  permanent light 30, rooms 31, 32, TV 44
    candle              accessory 1010  pin 45
    level crossing      accessory 1020  pins 33/34 flashing alternately
  Direction 1 (green/straight) = on, 0 = off.

  Dimmed lights (TV, candle) need PWM pins: 2, 5, 6, 7, 44, 45, 46. Pins 11-13 belong to
  the DCC signal (Timer1). One pin drives at most 20 mA, use a ULN2803 for more LEDs.

  All other settings are in MobaStationConfig.h (see examples/MobaStationWiFi).
*/
#include <Arduino.h>

#include <MobaStation.h>
#include <MobaBus_House.h>
#include <MobaBus_Flicker.h>
#include <MobaBus_Blink.h>

#include "ESP_AT_Wifi.h"

ESP_AT_Wifi wifi(&WIFI_SERIAL, WIFI_SSID, WIFI_PWD, WIFI_MODE);

//MobaBus runs also without CAN module, it delivers the commands of the station to the
//modules on the pins of the Mega. The EEPROM keeps their on/off state over a restart
MobaBus mobaBus(MOBABUS_EEPROM, MOBABUS_PROG_PIN, MOBABUS_LED_PIN);
#if MOBABUS_ENABLED
MobaBus_CAN can(MOBABUS_CAN_CS, MOBABUS_CAN_SPEED, MOBABUS_CAN_CLOCK, MOBABUS_CAN_INT);
#endif

#if XPRESSNET_ENABLED
XpressNetMasterClass XpressNet;
#endif

MobaBus_House house(MobaBus_LightOutput(30));                                   //permanent light
MobaBus_Flicker candle(MobaBus_LightOutput(45), FLICKER_CANDLE);
MobaBus_Blink crossing(MobaBus_LightOutput(33), MobaBus_LightOutput(34), 1000);


void setup() {
  Serial.begin(115200);

  //must be called before attaching interfaces and modules
  mobaBus.begin(true);
  #if MOBABUS_ENABLED
  mobaBus.attachInterface(&can);
  #endif

  house.addRoom(MobaBus_LightOutput(31));
  house.addRoom(MobaBus_LightOutput(32));
  house.addRoom(MobaBus_LightOutput(44), ROOM_TV);
  house.setAddress(1001);
  mobaBus.attachModule(&house);

  candle.setAddress(1010);
  mobaBus.attachModule(&candle);

  crossing.setAddress(1020);
  mobaBus.attachModule(&crossing);

  MobaStation::attachMobaBus(&mobaBus);
  //turnouts up to 1000 are DCC decoders, from 1001 on MobaBus (CAN and the modules above)
  MobaStation::setTurnoutMode(Combined, 1000);

  MobaStation::attachEthInterface(&wifi);

  #if XPRESSNET_ENABLED
  MobaStation::attachXpressNet(&XpressNet, XPRESSNET_CONTROL_PIN);
  #endif

  MobaStation::begin(MOTOR_SHIELD_TYPE, DRIVE_ON_PROG);
}

void loop() {
  MobaStation::loop();
}
