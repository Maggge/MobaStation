#pragma once
#include <stdint.h>
#include <deque>
#include <vector>
//MobaBus for the station test: sent packets are recorded, rx is returned by loop()
enum MobaDevType { ALL = 0, BOOSTER = 1, ACCESSORIE = 2, FEEDBACK = 3 };
enum MobaCmd { GET = 1, SET = 2, INFO = 3, CONFIG_WRITE = 4, CONFIG_READ = 5, CONFIG_INFO = 6, DISCOVER = 7, ANNOUNCE = 8 };
enum MobaCvStatus { CV_OK = 0, CV_UNKNOWN = 1, CV_READONLY = 2, CV_INVALID = 3 };
struct MobaBus_Meta { MobaDevType type; uint16_t address; uint8_t port; MobaCmd cmd; };
struct MobaBus_Packet { MobaBus_Meta meta; char busId = -1; uint8_t len = 0; uint8_t data[8]; };
class MobaBus { public:
  std::deque<MobaBus_Packet> rx; std::vector<MobaBus_Packet> sent;
  bool loop(MobaBus_Packet *p){ if(rx.empty()) return false; *p = rx.front(); rx.pop_front(); return true; }
  uint8_t sendPkg(MobaDevType t, uint16_t a, MobaCmd c, uint8_t len, uint8_t *d){
    MobaBus_Packet p; p.meta = {t, a, 0, c}; p.len = len; for(int i = 0; i < len; i++) p.data[i] = d[i]; sent.push_back(p); return 0; }
  int discovers = 0; void discover(){ discovers++; }
  void push(MobaDevType t, uint16_t a, MobaCmd c, std::vector<uint8_t> d){
    MobaBus_Packet p; p.meta = {t, a, 0, c}; p.len = d.size(); for(size_t i = 0; i < d.size(); i++) p.data[i] = d[i]; rx.push_back(p); } };
