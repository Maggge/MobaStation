#pragma once
#include "Arduino.h"
typedef __FlashStringHelper FSH;
class MotorDriver {};
class DCC { public:
  static void begin(const FSH*, MotorDriver*, MotorDriver*); static void loop(); static void setProgTrackSyncMain(bool);
  static void setAccessory(int, byte, bool); static void setThrottle(uint16_t, uint8_t, bool);
  static void setThrottle(uint16_t, uint8_t, uint8_t, bool); static uint8_t getThrottleSpeed(int);
  static bool getThrottleDirection(int); static uint8_t getSpeedSteps(int); static unsigned long getFns(int);
  static void setFn(int, int, bool); static void readCV(int, void(*)(int16_t)); static void writeCVByte(int, uint8_t, void(*)(int16_t));
  static void writeCVByteMain(int,int,uint8_t); static void writeCVBitMain(int,int,uint8_t,bool); };
