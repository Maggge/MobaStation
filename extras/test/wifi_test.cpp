/*
  Host test of the ESP AT WiFi driver (WiFi_ESP_AT.cpp) against a simulated module:
  send queue, CIPSEND prompt, timeouts, reconnect and accesspoint fallback. make test
*/
#define private public
#include "WiFi_ESP_AT.h"
#undef private
uint32_t fakeMillis = 1000;
HardwareSerial Serial;
int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } }while(0)
static bool has(const std::string &s, const char *x){ return s.find(x) != std::string::npos; }
static void queue(WiFiClass &w, uint8_t len, uint8_t fill){
  WIFI_PACKET p; p.ip = IPAddress(192,168,0,20); p.port = 21105; p.linkId = 0; p.len = len;
  for(int i = 0; i < len; i++) p.data[i] = fill;
  CHECK(w.sendPkg(&p), "sendPkg");
}
static void fresh(WiFiClass &w, Stream &s, WL_STATE st, bool echo){
  w.wifi = &s; w.state = st; w.echo = echo; w.clearSendQueue(); s.out.clear(); s.in.clear();
}
int main(){
  Stream s; WiFiClass w;

  // init switches the echo off
  s.push("\r\nOK\r\n"); s.push("ATE0\r\r\n\r\nOK\r\n"); s.push("\r\nOK\r\n"); s.push("\r\nOK\r\n");
  w.init(&s);
  CHECK(has(s.out, "ATE0\r\n") && !w.echo, "ATE0 sent and echo off");

  // A: accesspoint mode sends (regression of the uncommitted change)
  fresh(w, s, WL_AP_LISTENING, false);
  queue(w, 7, 0x0A);
  w.loop();
  CHECK(has(s.out, "AT+CIPSEND=0,7,\"192.168.0.20\",21105"), "AP: CIPSEND issued, out=[%s]", s.out.c_str());
  s.push("\r\nOK\r\n> "); w.loop();
  CHECK(has(s.out, std::string(7, '\x0A').c_str()), "AP: payload written");
  s.push("\r\nRecv 7 bytes\r\n\r\nSEND OK\r\n"); w.loop();
  CHECK(!w.pendingCipsend && !w.waitForSend && w.sendIdx == w.tail, "AP: queue empty after SEND OK");

  // B: timeout after the prompt, late SEND OK must not finish the next packet
  fresh(w, s, WL_CONNECTED, false);
  queue(w, 7, 'a'); queue(w, 9, 'b');
  w.loop(); s.push("> "); w.loop();
  CHECK(has(s.out, "aaaaaaa"), "B: P1 written");
  fakeMillis += 250; s.out.clear(); w.loop();
  CHECK(has(s.out, "AT+CIPSEND=0,9,"), "B: P2 started after timeout, not skipped, out=[%s]", s.out.c_str());
  CHECK(w.sendIdx == 1 && w.tail == 2, "B: index advanced once (sendIdx=%d tail=%d)", w.sendIdx, w.tail);
  s.push("\r\nSEND OK\r\n"); w.loop();
  CHECK(w.pendingCipsend && w.sendIdx == 1, "B: late SEND OK ignored");
  s.push("> "); w.loop();
  CHECK(has(s.out, "bbbbbbbbb"), "B: P2 written");
  s.push("\r\nRecv 9 bytes\r\n\r\nSEND OK\r\n"); w.loop();
  CHECK(!w.pendingCipsend && !w.waitForSend && w.sendIdx == w.tail, "B: queue empty");

  // C: timeout without any prompt drops exactly one packet
  fresh(w, s, WL_CONNECTED, false);
  queue(w, 7, 'c'); w.loop(); fakeMillis += 250; s.out.clear(); w.loop();
  CHECK(!w.waitForSend && w.sendIdx == w.tail && s.out.empty(), "C: dropped, no stale resend (sendIdx=%d tail=%d out=[%s])", w.sendIdx, w.tail, s.out.c_str());
  fakeMillis += 250; w.loop();
  CHECK(s.out.empty(), "C: nothing sent afterwards");

  // D: echo on, echoed payload contains \n and '+', parser stays in sync
  fresh(w, s, WL_CONNECTED, true);
  queue(w, 10, '\n'); w.loop(); s.push("> ");
  s.push(std::string(10, '\n'));
  s.push("\r\nRecv 10 bytes\r\n\r\nSEND OK\r\n"); w.loop();
  CHECK(!w.pendingCipsend && !w.waitForSend, "D: echo skipped, SEND OK seen");

  // E: busy answer without a pending send must not create one
  fresh(w, s, WL_CONNECTED, false);
  s.push("busy p...\r\n"); w.loop(); fakeMillis += 250; w.loop();
  CHECK(!w.pendingCipsend && w.sendIdx == 0 && w.tail == 0, "E: busy ignored");

  // F: station down -> queue cleared, nothing sent
  fresh(w, s, WL_CONNECTION_LOST, false);
  queue(w, 7, 'f'); w.loop();
  CHECK(!w.waitForSend && s.out.empty(), "F: cleared while down");

  // G: receiving still works next to sending
  fresh(w, s, WL_CONNECTED, false);
  { const char ipd[] = "+IPD,0,4,192.168.0.20,21105:\x04\x00\x10\x00"; s.push(std::string(ipd, sizeof(ipd) - 1)); } w.loop();
  WIFI_PACKET p; CHECK(w.getPacket(0, &p) && p.len == 4 && p.data[2] == 0x10 && p.ip[3] == 20, "G: packet received");

  // H: hybrid fallback, non blocking, and back to the station
  {
    WiFiClass h; Stream m; h.wifi = &m; h.state = WL_CONNECTED; h.echo = false;
    h.stationMode = true; h.linkId = 0; h.linkIdOccupied[0] = true; h.linkPort = 21105;
    strcpy(h.ssid, "home"); strcpy(h.pwd, "secret");
    h.enableFallbackAP("MobaStation", "pass1234", IPAddress(192,168,0,111));
    auto step = [&](const char *expect, const char *answer, const char *what){
      //the state machine does one step per loop, a command may need a few loops
      uint32_t t = fakeMillis; m.out.clear();
      for(int i = 0; i < 3 && m.out.empty(); i++) h.loop();
      CHECK(fakeMillis == t, "H %s: loop blocked", what);
      CHECK(has(m.out, expect), "H %s: expected [%s] got [%s] state=%d", what, expect, m.out.c_str(), h.reconnectState);
      m.push(answer); h.loop();
    };
    m.push("WIFI DISCONNECT\r\n"); h.loop();
    CHECK(h.state == WL_CONNECTION_LOST && h.reconnectState == RECONNECT_LINK, "H: lost");
    for(int round = 0; round < 3; round++){
      for(int j = 0; j < 3; j++) step("AT+CWJAP_CUR=\"home\",\"secret\"", "WIFI DISCONNECT\r\n+CWJAP:3\r\n\r\nFAIL\r\n", "join");
      if(round < 2){
        m.out.clear(); h.loop();
        CHECK(h.reconnectState == RECONNECT_WAIT && m.out.empty(), "H: waits after round %d (state %d)", round, h.reconnectState);
        fakeMillis += WIFI_RECONNECT_INTERVAL;
      }
    }
    step("AT+CWMODE_CUR=3", "\r\nOK\r\n", "ap mode");
    step("AT+CWSAP_CUR=\"MobaStation\",\"pass1234\",1,4,8,0", "\r\nOK\r\n", "ap config");
    step("AT+CIPAP_CUR=\"192.168.0.111\",\"192.168.0.111\",\"255.255.255.0\"", "\r\nOK\r\n", "ap ip");
    step("AT+CWDHCPS_CUR=1,3,\"192.168.0.10\",\"192.168.0.25\"", "\r\nOK\r\n", "ap dhcp");
    CHECK(h.apActive && h.canSend(), "H: accesspoint active and sending allowed");
    step("AT+CIPMUX=1", "link is builded\r\n\r\nERROR\r\n", "cipmux (already set)");
    step("AT+CIPDINFO=1", "\r\nOK\r\n", "dinfo");
    step("AT+CIPSTART=0,\"UDP\",\"0.0.0.0\",21105,21105,2", "ALREADY CONNECTED\r\n\r\nERROR\r\n", "open");
    CHECK(h.reconnectState == RECONNECT_WAIT, "H: keeps searching the station (state %d)", h.reconnectState);
    // AP clients are served while the station is down
    queue(h, 7, 'h'); m.out.clear(); h.loop();
    CHECK(has(m.out, "AT+CIPSEND=0,7"), "H: sends to AP clients");
    m.push("> \r\nRecv 7 bytes\r\n\r\nSEND OK\r\n"); h.loop();
    // the fallback interval is used, the 5s one would disturb the accesspoint
    fakeMillis += WIFI_RECONNECT_INTERVAL; m.out.clear(); h.loop(); h.loop();
    CHECK(m.out.empty(), "H: no join after 5s while the accesspoint runs");
    fakeMillis += WIFI_FALLBACK_RECONNECT_INTERVAL; h.loop();
    step("AT+CWJAP_CUR=\"home\"", "WIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n", "join ok");
    step("AT+CWMODE_CUR=1", "\r\nOK\r\n", "sta only");
    CHECK(!h.apActive && h.state == WL_CONNECTED, "H: accesspoint off, station up");
    step("AT+CWHOSTNAME", "\r\nOK\r\n", "hostname");
    step("AT+CIPMUX=1", "\r\nOK\r\n", "cipmux");
    step("AT+CIPDINFO=1", "\r\nOK\r\n", "dinfo");
    step("AT+CIPSTART=0", "\r\nOK\r\n", "open");
    CHECK(h.reconnectState == RECONNECT_IDLE && h.failedRounds == 0, "H: reconnect done (state %d)", h.reconnectState);
  }

  // J: static ip with gateway and netmask, also in the reconnect; DHCP; ip query
  fresh(w, s, WL_CONNECTED, false);
  s.push("\r\nOK\r\n");
  CHECK(w.setIP(IPAddress(192,168,0,111), IPAddress(192,168,0,1), IPAddress(255,255,255,0)), "J: setIP");
  CHECK(has(s.out, "AT+CIPSTA_CUR=\"192.168.0.111\",\"192.168.0.1\",\"255.255.255.0\"\r\n"), "J: full static ip [%s]", s.out.c_str());
  s.out.clear(); w.reconnectSetIp();
  CHECK(has(s.out, "AT+CIPSTA_CUR=\"192.168.0.111\",\"192.168.0.1\",\"255.255.255.0\"\r\n"), "J: reconnect sets the same [%s]", s.out.c_str());
  w.cmdExpected = false;
  s.out.clear(); s.push("\r\nOK\r\n");
  CHECK(w.setIP("192.168.0.112"), "J: setIP string");
  s.out.clear(); w.reconnectSetIp();
  CHECK(has(s.out, "AT+CIPSTA_CUR=\"192.168.0.112\"\r\n"), "J: string ip without old gateway [%s]", s.out.c_str());
  w.cmdExpected = false;
  s.out.clear(); s.push("\r\nOK\r\n");
  CHECK(w.enableDHCP() && !w.staticIpSet && has(s.out, "AT+CWDHCP_CUR=1,1\r\n"), "J: dhcp");
  w.reconnectState = RECONNECT_IP; w.reconnectSetIp();
  CHECK(w.reconnectState == RECONNECT_HOSTNAME, "J: dhcp skips the static ip in the reconnect");
  s.out.clear(); s.push("+CIFSR:APIP,\"192.168.4.1\"\r\n+CIFSR:STAIP,\"192.168.0.57\"\r\n+CIFSR:STAMAC,\"aa:bb\"\r\n\r\nOK\r\n");
  CHECK(w.localIP() == IPAddress(192,168,0,57) && s.available() == 0, "J: localIP");

  // I: millis overflow does not fire the send timeout early
  fresh(w, s, WL_CONNECTED, false);
  fakeMillis = 0xFFFFFFF0u; queue(w, 7, 'i'); w.loop();
  fakeMillis += 0x20; w.loop();   // 32 ms later, millis wrapped
  CHECK(w.pendingCipsend && w.sendIdx == 0, "I: no timeout after the millis overflow");

  printf("%s (%d failures)\n", fails ? "FAILED" : "all wifi tests passed", fails);
  return fails != 0;
}
