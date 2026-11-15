/*
  Host test of ESP_Link (Mega side of the ESP firmware link): make test
  The test plays the ESP: it encodes frames into the serial input and decodes what
  the Mega writes. MobaStation.cpp is included to reach the station state.
*/
#include "MobaStation.cpp"
#include "ESP_Link.h"
#include <vector>
#include <string>
uint32_t fakeMillis = 1000; HardwareSerial Serial; EEPROMClass EEPROM;
DCCWaveform DCCWaveform::mainTrack, DCCWaveform::progTrack;
static POWERMODE mainMode = POWERMODE::OFF;
int DCCWaveform::getCurrentmA(){ return this == &mainTrack ? 120 : 0; }
void DCCWaveform::setPowerMode(POWERMODE p){ if(this == &mainTrack) mainMode = p; }
POWERMODE DCCWaveform::getPowerMode(){ return this == &mainTrack ? mainMode : POWERMODE::OFF; }
void DCC::begin(const FSH*, MotorDriver*, MotorDriver*){} void DCC::loop(){} void DCC::setProgTrackSyncMain(bool){}
std::vector<int> accessories;
void DCC::setAccessory(int a, byte n, bool act){ accessories.push_back(a * 10 + n); }
void DCC::setThrottle(uint16_t, uint8_t, bool){} void DCC::setThrottle(uint16_t, uint8_t, uint8_t, bool){}
uint8_t DCC::getThrottleSpeed(int){ return 0; } bool DCC::getThrottleDirection(int){ return 0; }
uint8_t DCC::getSpeedSteps(int){ return 0; } unsigned long DCC::getFns(int){ return 0; } void DCC::setFn(int, int, bool){}
void DCC::readCV(int, void(*)(int16_t)){} void DCC::writeCVByte(int, uint8_t, void(*)(int16_t)){}
void DCC::writeCVByteMain(int,int,uint8_t){} void DCC::writeCVBitMain(int,int,uint8_t,bool){}

int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } }while(0)

struct Frame { uint8_t type; std::vector<uint8_t> d; };
static std::vector<Frame> written(HardwareSerial &s){       //frames the Mega wrote, the output is consumed
  std::vector<Frame> f; EspLinkCodec c;
  for(unsigned char b: s.out) if(c.feed(b)) f.push_back({c.type, std::vector<uint8_t>(c.data(), c.data() + c.len)});
  s.out.clear(); return f;
}
static void espSends(HardwareSerial &s, uint8_t type, std::vector<uint8_t> d){
  uint8_t out[ESP_LINK_MAX_FRAME]; uint8_t n = EspLinkCodec::encode(type, d.data(), d.size(), out);
  s.push(std::string((char*)out, n));
}
static const Frame *find(const std::vector<Frame> &f, uint8_t type, int key = -1){
  for(auto &x: f) if(x.type == type && (key < 0 || (!x.d.empty() && x.d[0] == key))) return &x;
  return nullptr;
}

int main(){
  HardwareSerial esp;
  MobaBus mbus;
  ESP_Link link(&esp, "home", "secret", Hybrid_MODE, "MobaStation", "appass12");
  MobaStation::attachMobaBus(&mbus);
  MobaStation::attachEthInterface(&link);
  MobaStation::begin(F("x"), nullptr, nullptr, false);

  // after begin: HELLO and the network defaults of the sketch
  auto f = written(esp);
  CHECK(find(f, ESPLINK_HELLO) && find(f, ESPLINK_HELLO)->d[1] == ESPLINK_ROLE_MEGA, "HELLO of the Mega");
  auto ssid = find(f, ESPLINK_NET_DEFAULT, ESPLINK_NETDEF_SSID);
  CHECK(ssid && std::string(ssid->d.begin() + 1, ssid->d.end()) == "home", "ssid default");
  auto ip = find(f, ESPLINK_NET_DEFAULT, ESPLINK_NETDEF_IP);
  CHECK(ip && ip->d[1] == NETWORK_DHCP && ip->d[2] == IPAddress(NETWORK_IP)[0] && ip->d[13] == 1, "ip defaults (gateway .1)");
  CHECK(find(f, ESPLINK_NET_DEFAULT, ESPLINK_NETDEF_MODE)->d[1] == Hybrid_MODE && find(f, ESPLINK_NET_DEFAULT, ESPLINK_NETDEF_DONE), "mode and done");
  CHECK(!link.linkUp(), "link down until the ESP answers");

  CHECK(find(f, ESPLINK_HELLO)->d[2] == 1, "start HELLO asks for an answer");
  // the ESP boots: HELLO with request -> HELLO without request + defaults again
  espSends(esp, ESPLINK_HELLO, {ESP_LINK_VERSION, ESPLINK_ROLE_ESP, 1}); MobaStation::loop();
  f = written(esp);
  CHECK(link.linkUp() && find(f, ESPLINK_HELLO) && find(f, ESPLINK_HELLO)->d[2] == 0 && find(f, ESPLINK_NET_DEFAULT, ESPLINK_NETDEF_DONE),
        "ESP found, answer without request, defaults sent again");
  // the answer of the ESP to our start HELLO is not answered again
  espSends(esp, ESPLINK_HELLO, {ESP_LINK_VERSION, ESPLINK_ROLE_ESP, 0}); MobaStation::loop();
  f = written(esp);
  CHECK(!find(f, ESPLINK_HELLO), "no HELLO ping-pong");

  // Z21 over the link: serial number request -> answer as UDP_TX to the same client and port
  espSends(esp, ESPLINK_UDP_RX, {192, 168, 30, 20, 0x50, 0xC3, 0x04, 0x00, 0x10, 0x00}); MobaStation::loop();
  f = written(esp);
  bool serial = false;
  for(auto &x: f) if(x.type == ESPLINK_UDP_TX && x.d[0] == 192 && x.d[3] == 20 && espLinkGet16(&x.d[4]) == 50000 && x.d[8] == 0x10) serial = true;
  CHECK(serial, "Z21 answer goes back over the link to port 50000");

  // status every second
  fakeMillis += 1100; MobaStation::loop(); f = written(esp);
  auto st = find(f, ESPLINK_STATUS);
  CHECK(st && st->d.size() == ESPLINK_STATUS_SIZE && espLinkGet16(&st->d[ESPLINK_STATUS_MAIN_MA]) == 120 && st->d[ESPLINK_STATUS_CLIENTS] == 1,
        "status with current and client count");
  CHECK(find(f, ESPLINK_HEARTBEAT), "heartbeat");

  // settings: read, write (stored), invalid value
  espSends(esp, ESPLINK_SETTING_SET, {SETTING_DCC_RANGE, 0xE8, 0x03, 0, 0}); MobaStation::loop();   //1000
  f = written(esp); auto set = find(f, ESPLINK_SETTING, SETTING_DCC_RANGE);
  CHECK(set && espLinkGet32(&set->d[1]) == 1000 && set->d[5] == 0, "DCC range set to 1000");
  espSends(esp, ESPLINK_SETTING_SET, {SETTING_TURNOUT_MODE, 7, 0, 0, 0}); MobaStation::loop();
  f = written(esp); set = find(f, ESPLINK_SETTING, SETTING_TURNOUT_MODE);
  CHECK(set && set->d[5] != 0 && espLinkGet32(&set->d[1]) == Combined, "invalid turnout mode refused");
  espSends(esp, ESPLINK_SETTING_SET, {SETTING_POWER_ON_START, 1, 0, 0, 0}); MobaStation::loop(); written(esp);
  CHECK(EEPROM.read(CONFIG_STORAGE + 16) == 0xA7 && EEPROM.read(CONFIG_STORAGE + 19) == 0xE8, "settings stored in the EEPROM");

  // CV of a MobaBus module (range 1000: 1020 is MobaBus) and of a DCC turnout
  mbus.sent.clear();
  espSends(esp, ESPLINK_CV_READ, {0xFC, 0x03, 33, 0}); MobaStation::loop();     //1020, CV33
  CHECK(mbus.sent.size() == 1 && mbus.sent[0].meta.cmd == CONFIG_READ && mbus.sent[0].meta.address == 1020, "CV read to MobaBus");
  mbus.push(ACCESSORIE, 1020, CONFIG_INFO, {0, 33, 50, CV_OK}); MobaStation::loop();
  f = written(esp); auto cv = find(f, ESPLINK_CV_RESULT);
  CHECK(cv && espLinkGet16(&cv->d[0]) == 1020 && espLinkGet16(&cv->d[2]) == 33 && cv->d[4] == 50 && cv->d[5] == CV_OK, "CV result to the ESP");
  espSends(esp, ESPLINK_CV_READ, {5, 0, 1, 0}); MobaStation::loop();
  f = written(esp); cv = find(f, ESPLINK_CV_RESULT);
  CHECK(cv && cv->d[5] == 254, "DCC turnout: status 254");
  // feedback modules have their own addresses: flag in bit 15, always MobaBus
  mbus.sent.clear();
  espSends(esp, ESPLINK_CV_WRITE, {3, 0x80, 33, 0, 0x81}); MobaStation::loop();   //feedback 3, CV33 = 0x81
  CHECK(mbus.sent.size() == 1 && mbus.sent[0].meta.type == FEEDBACK && mbus.sent[0].meta.address == 3 &&
        mbus.sent[0].meta.cmd == CONFIG_WRITE && mbus.sent[0].data[2] == 0x81, "CV write to a feedback module");
  mbus.push(FEEDBACK, 3, CONFIG_INFO, {0, 33, 0x81, CV_OK}); MobaStation::loop();
  f = written(esp); cv = find(f, ESPLINK_CV_RESULT);
  CHECK(cv && espLinkGet16(&cv->d[0]) == (3 | ESPLINK_CV_FEEDBACK) && cv->d[4] == 0x81 && cv->d[5] == CV_OK, "feedback result keeps the flag");
  mbus.push(ACCESSORIE, 3, CONFIG_INFO, {0, 33, 9, CV_OK}); MobaStation::loop();
  f = written(esp); cv = find(f, ESPLINK_CV_RESULT);
  CHECK(cv && espLinkGet16(&cv->d[0]) == 3, "accessory 3 without the flag");

  // discover and module list
  espSends(esp, ESPLINK_DISCOVER, {}); MobaStation::loop();
  CHECK(mbus.discovers == 1, "discover forwarded");
  mbus.push(ACCESSORIE, 1001, ANNOUNCE, {5, 1, 0, 2}); MobaStation::loop();
  f = written(esp); auto m = find(f, ESPLINK_MODULE);
  CHECK(m && m->d[0] == ACCESSORIE && espLinkGet16(&m->d[1]) == 1001 && m->d[3] == 5 && m->d.size() == 10 && espLinkGet16(&m->d[7]) == 0,
        "module found (old module without board id)");
  mbus.push(ACCESSORIE, 1002, ANNOUNCE, {8, 1, 3, 2, 0x3A, 0x7F, 2}); MobaStation::loop();
  f = written(esp); m = find(f, ESPLINK_MODULE);
  CHECK(m && espLinkGet16(&m->d[7]) == 0x3A7F && m->d[9] == 2 && m->d[5] == 3, "board id and flags of the module");

  // universal boards: requests go to the bus, answers to the ESP
  mbus.sent.clear();
  espSends(esp, ESPLINK_BOARD_WRITE, {0x3A, 0x7F, 0, 16, 2}); MobaStation::loop();
  CHECK(mbus.sent.size() == 1 && mbus.sent[0].meta.cmd == BOARD_WRITE && mbus.sent[0].meta.type == ALL && mbus.sent[0].len == 5 &&
        mbus.sent[0].data[0] == 0x3A && mbus.sent[0].data[4] == 2, "BOARD_WRITE on the bus");
  espSends(esp, ESPLINK_BOARD_READ, {0x3A, 0x7F, 1, 3}); MobaStation::loop();
  f = written(esp); auto br = find(f, ESPLINK_BOARD_RESULT);
  CHECK(br && br->d.size() == 5 && br->d[4] == 254 && mbus.sent.size() == 1, "read without count is refused");
  mbus.push(ALL, 0, BOARD_INFO, {0x3A, 0x7F, 0, 16, 0, 2}); MobaStation::loop();
  f = written(esp); br = find(f, ESPLINK_BOARD_RESULT);
  CHECK(br && br->d.size() == 6 && br->d[0] == 0x3A && br->d[5] == 2, "BOARD_INFO to the ESP");
  mbus.push(ALL, 0, BOARD_ANNOUNCE, {0x3A, 0x7F, 1, 4, 5, 16, 46}); MobaStation::loop();
  f = written(esp); auto bf = find(f, ESPLINK_BOARD);
  CHECK(bf && bf->d.size() == 7 && bf->d[6] == 46, "BOARD_ANNOUNCE to the ESP");

  // commands
  espSends(esp, ESPLINK_COMMAND, {ESPLINK_CMD_POWER_ON}); MobaStation::loop();
  CHECK(mainMode == POWERMODE::ON, "power on from the web interface");
  accessories.clear();
  espSends(esp, ESPLINK_COMMAND, {ESPLINK_CMD_TURNOUT, 5, 0, 1}); MobaStation::loop();
  CHECK(accessories.size() == 1 && accessories[0] == 20, "turnout 5 switched (decoder 2 port 0)");
  mbus.sent.clear();
  espSends(esp, ESPLINK_COMMAND, {ESPLINK_CMD_TURNOUT, 0xFD, 0x03, 1}); MobaStation::loop();
  CHECK(mbus.sent.size() == 2 && mbus.sent[0].meta.address == 1021 && mbus.sent[0].data[0] == 1, "MobaBus turnout 1021 switched");

  // HELLO of a restarting ESP right after its boot messages (no 0x00 in between from the ESP side
  // before the leading delimiter was added, the frame was lost in the overflow)
  { std::string boot(200, '\x55'); esp.push(boot); esp.push(std::string(1, '\0'));
    espSends(esp, ESPLINK_HELLO, {ESP_LINK_VERSION, ESPLINK_ROLE_ESP, 1}); MobaStation::loop();
    f = written(esp);
    CHECK(find(f, ESPLINK_NET_DEFAULT, ESPLINK_NETDEF_DONE), "HELLO after boot garbage answered with the defaults"); }
  CHECK(esp.out.empty(), "output consumed");

  // noise on the line does not stop the link
  esp.push(std::string("\x12\x34\x00garbage\x00", 12));
  espSends(esp, ESPLINK_HEARTBEAT, {}); MobaStation::loop();
  CHECK(link.linkUp() && link.linkErrors() >= 1, "noise counted, link stays up (%d errors)", link.linkErrors());
  fakeMillis += ESP_LINK_TIMEOUT + 100; MobaStation::loop();
  CHECK(!link.linkUp(), "link down without frames from the ESP");

  // settings survive a restart of the station, the reset command forgets them
  MobaStation::setTurnoutMode(Combined, 2047);
  MobaStation::begin(F("x"), nullptr, nullptr, false);
  int32_t v; MobaStation::getSetting(SETTING_DCC_RANGE, v);
  CHECK(v == 1000 && mainMode == POWERMODE::ON, "stored settings win after begin() (range %d, power on start)", (int)v);

  printf("%s (%d failures)\n", fails ? "FAILED" : "all ESP link (Mega) tests passed", fails);
  return fails != 0;
}
