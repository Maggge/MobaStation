#pragma once
#include <stdint.h>
#include <string.h>
//4 KB like the Mega, erased = 0xFF
struct EEPROMClass { uint8_t mem[4096]; EEPROMClass(){ memset(mem, 0xFF, sizeof(mem)); }
  uint8_t read(int a){ return mem[a & 4095]; } void update(int a, uint8_t v){ mem[a & 4095] = v; } void write(int a, uint8_t v){ mem[a & 4095] = v; }
  uint16_t length(){ return 4096; } };
extern EEPROMClass EEPROM;
