/*
  Host test of MobaStation (runs on the PC, no board needed): make test

  MobaStation.cpp is included (not linked) so the static speed step functions can be
  tested too. The DCC library, the network interfaces and the Arduino functions are
  replaced by stubs in extras/test/stub, the stubs record what the station does.
*/
#include "MobaStation.cpp"
#include <vector>
#include <string>
uint32_t fakeMillis = 1000; HardwareSerial Serial; EEPROMClass EEPROM;
DCCWaveform DCCWaveform::mainTrack, DCCWaveform::progTrack;
static POWERMODE mainMode = POWERMODE::OFF;
int DCCWaveform::getCurrentmA(){ return 0; }
void DCCWaveform::setPowerMode(POWERMODE p){ if(this == &mainTrack) mainMode = p; }
POWERMODE DCCWaveform::getPowerMode(){ return this == &mainTrack ? mainMode : POWERMODE::OFF; }
// --- DCC stub, records the calls ---
std::vector<std::string> dccLog;
static void (*cvCb)(int16_t) = nullptr;
static uint8_t spd = 0; static bool dirv = false; static uint8_t steps = 0;
void DCC::begin(const FSH*, MotorDriver*, MotorDriver*){} void DCC::loop(){} void DCC::setProgTrackSyncMain(bool){}
void DCC::setAccessory(int a, byte n, bool act){ dccLog.push_back("acc " + std::to_string(a) + " " + std::to_string(n) + " " + std::to_string(act)); }
void DCC::setThrottle(uint16_t c, uint8_t s, bool d){ dccLog.push_back("thr " + std::to_string(c) + " " + std::to_string(s) + " " + std::to_string(d)); }
void DCC::setThrottle(uint16_t c, uint8_t s, uint8_t st, bool d){ spd = s; dirv = d; steps = st; dccLog.push_back("thr4 " + std::to_string(c) + " " + std::to_string(s) + " " + std::to_string(st) + " " + std::to_string(d)); }
uint8_t DCC::getThrottleSpeed(int){ return spd; } bool DCC::getThrottleDirection(int){ return dirv; }
uint8_t DCC::getSpeedSteps(int){ return steps; } unsigned long DCC::getFns(int){ return 0; }
void DCC::setFn(int, int, bool){}
void DCC::readCV(int cv, void(*cb)(int16_t)){ cvCb = cb; dccLog.push_back("readCV " + std::to_string(cv)); }
void DCC::writeCVByte(int cv, uint8_t v, void(*cb)(int16_t)){ cvCb = cb; dccLog.push_back("writeCV " + std::to_string(cv)); }
void DCC::writeCVByteMain(int a,int cv,uint8_t v){ dccLog.push_back("pom " + std::to_string(a) + " " + std::to_string(cv) + " " + std::to_string(v)); }
void DCC::writeCVBitMain(int a,int cv,uint8_t b,bool v){ dccLog.push_back("pombit " + std::to_string(a) + " " + std::to_string(cv) + " " + std::to_string(b) + " " + std::to_string(v)); }
// --- fake interface ---
struct Sent { IPAddress ip; uint16_t port; std::vector<uint8_t> d; };
struct FakeIf : Eth_Interface {
  std::vector<Sent> sent; std::vector<UdpPacket> rx;
  NetworkConfig cfg; bool begin(uint16_t, const NetworkConfig &n){ cfg = n; return true; }
  bool send(IPAddress c, uint16_t p, uint8_t *data){ sent.push_back({c, p, std::vector<uint8_t>(data, data + data[0])}); return true; }
  bool receivePacket(UdpPacket *pkg){ if(rx.empty()) return false; *pkg = rx.front(); rx.erase(rx.begin()); return true; }
};
int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } }while(0)
static UdpPacket pkt(IPAddress ip, uint16_t port, std::vector<uint8_t> d){
  UdpPacket p; p.ip = ip; p.port = port; p.dataLength = d.size(); memset(p.data, 0xEE, sizeof(p.data)); memcpy(p.data, d.data(), d.size()); return p;
}
static std::vector<uint8_t> x(std::vector<uint8_t> body){   // LAN_X record with XOR
  std::vector<uint8_t> r = { (uint8_t)(body.size() + 5), 0, 0x40, 0 }; uint8_t xr = 0;
  for(auto b: body){ r.push_back(b); xr ^= b; } r.push_back(xr); return r;
}
static int count(FakeIf &f, IPAddress ip, uint8_t xh, uint8_t db0){
  int n = 0; for(auto &s: f.sent) if(s.ip == ip && s.d.size() > 5 && s.d[2] == 0x40 && s.d[4] == xh && s.d[5] == db0) n++; return n;
}
static uint8_t code28ToByte(uint8_t code){ return (code >> 1) | ((code & 1) << 4); }
static void speedTests(){

  bool dir;
  // 128: identity incl. direction
  for(int v = 0; v < 256; v++){
    uint8_t s = speedToDcc128(v, 128, dir);
    CHECK(s == (v & 0x7F) && dir == ((v & 0x80) != 0), "128 to %d", v);
    CHECK(speedFromDcc128(s, dir, 128) == v, "128 back %d", v);
  }
  // 14: all 16 values in both directions round trip, step 14 -> full speed
  int last = -1;
  for(int n = 0; n < 16; n++) for(int d = 0; d < 2; d++){
    uint8_t b = n | (d ? 0x80 : 0);
    uint8_t s = speedToDcc128(b, 14, dir);
    CHECK(dir == d, "14 dir %02x", b);
    CHECK(speedFromDcc128(s, dir, 14) == b, "14 roundtrip %02x -> %d -> %02x", b, s, speedFromDcc128(s, dir, 14));
    if(d == 0){ CHECK((int)s > last || n <= 1, "14 monotonic n=%d", n); last = s; }
  }
  CHECK(speedToDcc128(0x8F, 14, dir) == 127, "14 max");
  CHECK(speedToDcc128(0x82, 14, dir) == 2, "14 step 1");
  CHECK(speedToDcc128(0x01, 14, dir) == 1, "14 estop");
  // 28: codes 4..31 round trip exactly, stop/estop map to 0/1
  last = -1;
  for(int code = 0; code < 32; code++) for(int d = 0; d < 2; d++){
    uint8_t b = code28ToByte(code) | (d ? 0x80 : 0);
    uint8_t s = speedToDcc128(b, 28, dir);
    CHECK(dir == d, "28 dir code %d byte %02x", code, b);
    uint8_t back = speedFromDcc128(s, dir, 28);
    if(code >= 4) CHECK(back == b, "28 roundtrip code %d %02x -> %d -> %02x", code, b, s, back);
    else CHECK(s == (code <= 1 ? 0 : 1), "28 stop/estop code %d -> %d", code, s);
    if(d == 0 && code >= 4){ CHECK((int)s > last, "28 monotonic code %d", code); last = s; }
  }
  CHECK(speedToDcc128(0x9F, 28, dir) == 127 && dir, "28 max forward");
  CHECK(speedToDcc128(0x12, 28, dir) == 6 && !dir, "28 step 2 reverse (old code: forward)");
  // every 128 speed maps to a valid 28/14 code and back to the nearest step
  for(int s = 2; s < 128; s++){
    uint8_t b = speedFromDcc128(s, true, 28);
    uint8_t code = ((b & 0x0F) << 1) | ((b >> 4) & 1);
    CHECK(code >= 4 && code <= 31, "128->28 range s=%d code=%d", s, code);
    uint8_t n = speedFromDcc128(s, true, 14) & 0x0F;
    CHECK(n >= 2 && n <= 15, "128->14 range s=%d n=%d", s, n);
  }
}

int main(){
  speedTests();
  FakeIf eth, wifi;
  XpressNetMasterClass xnet;
  MobaStation::attachEthInterface(&eth);   // settings from the config
  MobaStation::attachEthInterface(&wifi, IPAddress(10,0,0,5), 21105, IPAddress(255,0,0,0));
  CHECK(eth.cfg.dhcp == (bool)NETWORK_DHCP && eth.cfg.ip == IPAddress(NETWORK_IP) && eth.cfg.subnet == IPAddress(NETWORK_SUBNET) && eth.cfg.gateway == IPAddress(NETWORK_GATEWAY), "config network settings");
  CHECK(!wifi.cfg.dhcp && wifi.cfg.ip == IPAddress(10,0,0,5) && wifi.cfg.subnet == IPAddress(255,0,0,0), "explicit network settings");
  NetworkConfig g{false, IPAddress(10,1,2,3), IPAddress(255,255,0,0), IPAddress()};
  CHECK(gatewayOf(g) == IPAddress(10,1,0,1), "default gateway is the first address of the subnet");
  MobaStation::attachXpressNet(&xnet, 22);
  MobaStation::begin(F("x"), nullptr, nullptr, false);
  IPAddress A(192,168,0,20), B(192,168,0,30);

  // 1) two records in one datagram: GET_SERIAL + SET_BROADCASTFLAGS(all), client A on wifi with port 50000
  std::vector<uint8_t> d = {0x04,0,0x10,0, 0x08,0,0x50,0, 0x01,0,0,0};
  wifi.rx.push_back(pkt(A, 50000, d));
  MobaStation::loop();
  CHECK(eth.sent.empty(), "A is a wifi client, nothing may go out over ethernet (%zu)", eth.sent.size());
  bool serial = false; for(auto &s: wifi.sent) if(s.d[2] == 0x10) serial = true;
  CHECK(serial, "serial number answered (first record)");
  CHECK(wifi.sent.size() >= 2 && wifi.sent.back().port == 50000, "answer to the source port");
  // power is off after begin(): broadcast flags answer must say off (0x61 0x00), not railState
  CHECK(count(wifi, A, 0x61, 0x00) >= 1, "track power off reported");

  // 2) power on from Z21 B (ethernet): every client gets exactly one power on
  eth.rx.push_back(pkt(B, 21105, {0x08,0,0x50,0, 0x01,0,0,0}));
  MobaStation::loop();
  eth.sent.clear(); wifi.sent.clear();
  eth.rx.push_back(pkt(B, 21105, x({0x21, 0x81})));
  MobaStation::loop();
  CHECK(count(wifi, A, 0x61, 0x01) == 1, "A got power on once (%d)", count(wifi, A, 0x61, 0x01));
  CHECK(count(eth, B, 0x61, 0x01) == 1, "B got power on once (%d)", count(eth, B, 0x61, 0x01));
  CHECK(count(eth, A, 0x61, 0x01) == 0, "A not over ethernet");

  // 3) set broadcast flags while power on -> 0x61 0x01
  wifi.sent.clear();
  wifi.rx.push_back(pkt(A, 50000, {0x08,0,0x50,0, 0x01,0,0,0}));
  MobaStation::loop();
  CHECK(count(wifi, A, 0x61, 0x01) == 1, "broadcast flags answer says power on");

  // 4) turnout 5 (FAdr 4) position 1 activate -> decoder 2 port 0 output 1, info answer ZZ=10
  dccLog.clear(); eth.sent.clear();
  eth.rx.push_back(pkt(B, 21105, x({0x53, 0x00, 0x04, 0x89}))); MobaStation::loop();
  eth.rx.push_back(pkt(B, 21105, x({0x53, 0x00, 0x04, 0x81}))); MobaStation::loop();   // deactivate
  CHECK(dccLog.size() == 1 && dccLog[0] == "acc 2 0 1", "DCC accessory call [%s] (%zu)", dccLog.empty() ? "" : dccLog[0].c_str(), dccLog.size());
  eth.sent.clear();
  eth.rx.push_back(pkt(B, 21105, x({0x43, 0x00, 0x04}))); MobaStation::loop();
  bool info = false; for(auto &s: eth.sent) if(s.d[4] == 0x43 && s.d[6] == 0x04 && s.d[7] == 0x02) info = true;
  CHECK(info, "turnout info answered with position 1");
  eth.sent.clear();
  eth.rx.push_back(pkt(B, 21105, x({0x43, 0x00, 0x07}))); MobaStation::loop();
  info = false; for(auto &s: eth.sent) if(s.d[4] == 0x43 && s.d[6] == 0x07 && s.d[7] == 0x00) info = true;
  CHECK(info, "unknown turnout answered with 00");

  // 5) Z21 drive 28 steps, step 28 forward -> 127, loco info reports 28 step format back
  dccLog.clear(); eth.sent.clear();
  eth.rx.push_back(pkt(B, 21105, x({0xE4, 0x12, 0x00, 0x03, 0x9F}))); MobaStation::loop();
  CHECK(!dccLog.empty() && dccLog[0] == "thr4 3 127 28 1", "drive [%s]", dccLog.empty() ? "" : dccLog[0].c_str());
  bool li = false; for(auto &s: eth.sent) if(s.d[4] == 0xEF && s.d[7] == (0x02 | 0x08) && s.d[8] == 0x9F) li = true;
  CHECK(li, "loco info in 28 step format");

  // 6) emergency stop: broadcast LAN_X_BC_STOPPED to all, track power stays on
  eth.sent.clear(); wifi.sent.clear(); dccLog.clear();
  eth.rx.push_back(pkt(B, 21105, x({0x80}))); MobaStation::loop();
  CHECK(count(eth, B, 0x81, 0x00) == 1 && count(wifi, A, 0x81, 0x00) == 1, "stopped to both clients");
  CHECK(mainMode == POWERMODE::ON, "track power still on");
  CHECK(!dccLog.empty() && dccLog[0] == "thr 0 1 1", "dcc emergency stop");

  // 7) CV: Z21 read running, XpressNet read is refused, result goes to the Z21 client
  dccLog.clear(); eth.sent.clear();
  eth.rx.push_back(pkt(B, 21105, x({0x23, 0x11, 0x00, 0x07}))); MobaStation::loop();
  CHECK(!dccLog.empty() && dccLog[0] == "readCV 8", "readCV 8");
  notifyXNetDirectReadCV(28);
  CHECK(dccLog.size() == 1, "xnet read refused while busy");
  cvCb(42);
  bool cvr = false; for(auto &s: eth.sent) if(s.d[4] == 0x64 && s.d[5] == 0x14 && s.d[7] == 7 && s.d[8] == 42) cvr = true;
  CHECK(cvr, "cv result to Z21");
  notifyXNetDirectReadCV(28);
  CHECK(dccLog.size() == 2 && dccLog[1] == "readCV 29", "xnet read starts when free");

  // 7b) POM of a loco: Z21 and XpressNet count the CV from 0, the library from 1
  dccLog.clear();
  eth.rx.push_back(pkt(B, 21105, x({0xE6, 0x30, 0xC0 | (1234 >> 8), 1234 & 0xFF, 0xEC, 3, 25}))); MobaStation::loop();
  CHECK(dccLog.size() == 1 && dccLog[0] == "pom 1234 4 25", "Z21 POM byte CV4 (%s)", dccLog.empty() ? "-" : dccLog[0].c_str());
  dccLog.clear();
  eth.rx.push_back(pkt(B, 21105, x({0xE6, 0x30, 0, 3, 0xE8, 28, 0x08 | 5}))); MobaStation::loop();
  CHECK(dccLog.size() == 1 && dccLog[0] == "pombit 3 29 5 1", "Z21 POM bit CV29 (%s)", dccLog.empty() ? "-" : dccLog[0].c_str());
  dccLog.clear();
  notifyXNetPOMwriteByte(3, 2, 7);
  CHECK(dccLog.size() == 1 && dccLog[0] == "pom 3 3 7", "XpressNet POM byte CV3 (%s)", dccLog.empty() ? "-" : dccLog[0].c_str());

  // 8) truncated / corrupt record: nothing happens, no stale bytes
  eth.sent.clear(); dccLog.clear();
  eth.rx.push_back(pkt(B, 21105, {0x0A,0,0x40,0, 0xE4,0x13})); MobaStation::loop();
  CHECK(dccLog.empty(), "truncated record ignored");
  eth.rx.push_back(pkt(B, 21105, {0x02,0,0x40,0})); MobaStation::loop();
  CHECK(dccLog.empty(), "length < 4 ignored");

  // 9) short circuit is reported and cleared
  eth.sent.clear();
  mainMode = POWERMODE::OVERLOAD; MobaStation::loop();
  CHECK(count(eth, B, 0x61, 0x08) == 1, "short circuit broadcast");
  mainMode = POWERMODE::ON; MobaStation::loop();
  CHECK(count(eth, B, 0x61, 0x01) == 1, "power on after the short");

  // 10) xpressnet power off from a throttle reaches the LAN once
  eth.sent.clear();
  notifyXNetPower(csTrackVoltageOff);
  CHECK(count(eth, B, 0x61, 0x00) == 1, "xnet power off once (%d)", count(eth, B, 0x61, 0x00));
  CHECK(mainMode == POWERMODE::OFF, "track off");

  // 11) POM of accessories on MobaBus (Z21 spec 6.9 - 6.11)
  MobaBus mbus; MobaStation::attachMobaBus(&mbus); MobaStation::setTurnoutMode(Combined, 1000);
  auto pom = [&](uint16_t turnout, uint8_t option, uint16_t cvNum, uint8_t value){
    uint16_t a = turnout - 1, dec = a / 4 + 1, c = cvNum - 1;
    uint16_t w = ((dec & 0x1FF) << 4) | 0x08 | ((a % 4) << 1);     //C = 1, DDD = port * 2
    eth.rx.push_back(pkt(B, 21105, x({0xE6, 0x31, (uint8_t)(w >> 8), (uint8_t)w, (uint8_t)(option | (c >> 8)), (uint8_t)c, value})));
    MobaStation::loop();
  };
  auto cvResult = [&](uint16_t cvNum, uint8_t value){
    for(auto &e: eth.sent) if(e.d[4] == 0x64 && e.d[5] == 0x14 && ((e.d[6] << 8) | e.d[7]) == cvNum - 1 && e.d[8] == value) return true;
    return false; };
  mbus.sent.clear(); eth.sent.clear();
  pom(1001, 0xE4, 33, 0);
  CHECK(mbus.sent.size() == 1 && mbus.sent[0].meta.cmd == CONFIG_READ && mbus.sent[0].meta.address == 1001 && mbus.sent[0].data[1] == 33,
        "POM read becomes CONFIG_READ for turnout 1001");
  mbus.push(ACCESSORIE, 1001, CONFIG_INFO, {0, 33, 77, CV_OK}); MobaStation::loop();
  CHECK(cvResult(33, 77), "CONFIG_INFO answered as LAN_X_CV_RESULT");
  mbus.sent.clear();
  pom(1003, 0xEC, 34, 120);
  CHECK(mbus.sent.size() == 1 && mbus.sent[0].meta.cmd == CONFIG_WRITE && mbus.sent[0].meta.address == 1003 && mbus.sent[0].data[2] == 120,
        "POM write to output pair 2 = turnout 1003 (%d)", mbus.sent.empty() ? -1 : mbus.sent[0].meta.address);
  mbus.sent.clear();
  pom(1001, 0xE8, 33, 0x08 | 2);         //set bit 2
  mbus.push(ACCESSORIE, 1001, CONFIG_INFO, {0, 33, 0x01, CV_OK}); MobaStation::loop();
  CHECK(mbus.sent.size() == 2 && mbus.sent[1].meta.cmd == CONFIG_WRITE && mbus.sent[1].data[2] == 0x05, "bit write = read, set bit, write");
  eth.sent.clear();
  pom(1001, 0xE4, 35, 0); fakeMillis += 1200; MobaStation::loop();
  CHECK(count(eth, B, 0x61, 0x13) == 1, "no answer within 1 s: NACK");
  eth.sent.clear(); mbus.sent.clear();
  pom(5, 0xE4, 1, 0);
  CHECK(mbus.sent.empty() && count(eth, B, 0x61, 0x13) == 1, "DCC accessory: NACK, nothing on MobaBus");
  mbus.push(ACCESSORIE, 1001, CONFIG_INFO, {0, 1, 9, CV_OK}); MobaStation::loop();
  CHECK(count(eth, B, 0x64, 0x14) == 0, "CONFIG_INFO without request is ignored");

  printf("%s (%d failures)\n", fails ? "FAILED" : "all MobaStation protocol and speed step tests passed", fails);
  return fails != 0;
}
