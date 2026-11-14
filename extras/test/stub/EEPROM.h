#pragma once
#include <stdint.h>
struct EEPROMClass { uint8_t read(int){return 0;} void update(int,uint8_t){} uint16_t length(){return 4096;} };
extern EEPROMClass EEPROM;
