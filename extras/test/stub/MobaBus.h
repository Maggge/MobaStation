#pragma once
#include <stdint.h>
enum { ACCESSORIE, FEEDBACK }; enum { INFO, SET, GET };
struct MobaBus_Packet { struct { uint8_t type, cmd; uint16_t address; } meta; uint8_t len; uint8_t data[8]; };
class MobaBus { public: bool loop(MobaBus_Packet*){ return false; } void sendPkg(int, uint16_t, int, uint8_t, uint8_t*){} };
