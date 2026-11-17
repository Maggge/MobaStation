/*
  MobaStation ESP firmware

  Runs on the ESP8266 of a Mega + WiFi R3 board or on a separate ESP8266 (Wemos D1 mini, NodeMCU,
  ESP-01) at Serial3 of the Mega: WiFi, the UDP port of the Z21 protocol and a web interface. The Mega
  (examples/MobaStationESP) stays the command station, both talk over the serial line in
  frames of EspLinkProtocol.h.

  The serial port is the link to the Mega: no debug output on Serial, messages go to the
  web interface.

  Build: make esp-build (Makefile of MobaStation), flash: make esp-upload (DIP 5-7 ON),
  updates later over WiFi: web interface (System), make esp-ota, or the network port
  "MobaStation" of the Arduino IDE (ArduinoOTA, also reachable as MobaStation.local).
*/
//ArduinoJson first: AsyncJson.h only enables its JSON handler when it is already known
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <Updater.h>
#include <ArduinoOTA.h>

#include "EspLinkProtocol.h"    //src/esplink of MobaStation, added to the include path by the Makefile
#include "NetConfig.h"
#include "WebPage.h"
//make esp-build generates the gzip compressed page (extras/esp/gzip_page.py), the Arduino IDE serves the plain one
#if __has_include(<WebPageGz.h>)
#include <WebPageGz.h>
#endif

#define FIRMWARE_VERSION "0.6.0"
#define MAX_MODULES_LIST 32
//the Z21 data of one datagram, longer packets are cut (the Mega handles 64 bytes)
#define UDP_MAX 64

NetConfig cfg;              //active settings: /net.json or the defaults of NetConfig.h
bool megaAnswered = false;  //the Mega answered our HELLO

WiFiUDP udp;
DNSServer dns;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

EspLinkCodec link;
uint32_t lastLinkRx = 0;
uint32_t lastBeat = 0;
uint32_t lastNetStatus = 0;
bool megaSeen = false;
uint8_t megaStatus[ESPLINK_STATUS_SIZE];
bool haveStatus = false;
uint32_t udpToMega = 0, udpFromMega = 0;

struct ModuleInfo { uint8_t type; uint16_t address; uint8_t typeId, channels, slot, version; uint16_t board; uint8_t flags; };
ModuleInfo modules[MAX_MODULES_LIST];
uint8_t moduleCount = 0;
//universal boards (BOARD_ANNOUNCE)
#define MAX_BOARDS_LIST 8
struct BoardInfo { uint16_t id; uint8_t version, built, configured, slots, pool, flags; };
BoardInfo boards[MAX_BOARDS_LIST];
uint8_t boardCount = 0;

bool apOn = false;
uint32_t staLostSince = 0;
uint32_t staUpSince = 0;
uint32_t restartAt = 0;
uint16_t udpPort = 0;

//the web pages drive locos and turnouts as one Z21 client with this pseudo address: its packets
//go to the Mega like UDP packets, the answers are taken out of UDP_TX instead of being sent
const uint8_t WEB_CLIENT_IP[4] = {127, 0, 0, 1};
#define WEB_CLIENT_PORT 1
//the Mega forgets a client after 40 s without packets
#define WEB_CLIENT_KEEPALIVE 20000
uint32_t lastWebSubscribe = 0;
bool webSubscribed = false;

#define ITEMS_FILE "/items.json"
#define ITEMS_MAX_SIZE 4096

//---------------------------------------------------------------- link to the Mega

//The Mega buffers 64 received bytes and its loop can stand about 5 ms (EEPROM writes, CAN).
//At 500000 baud a burst would fill that in 1.3 ms, so the frames to the Mega are paced:
//at most LINK_TX_BURST bytes at once, then LINK_TX_RATE bytes per ms (24 + 5 * 8 = 64).
#define LINK_TX_RATE 8
#define LINK_TX_BURST 24
#define LINK_TX_QUEUE 1024
uint8_t txQueue[LINK_TX_QUEUE];
uint16_t txHead = 0, txCount = 0;
uint32_t txCredit = LINK_TX_BURST * 1000UL;     //bytes * 1000
uint32_t txAt = 0;
uint16_t txDropped = 0;

void linkTxPump(){
    uint32_t now = micros();
    txCredit += (now - txAt) * LINK_TX_RATE;
    if(txCredit > LINK_TX_BURST * 1000UL) txCredit = LINK_TX_BURST * 1000UL;
    txAt = now;
    while(txCount > 0 && txCredit >= 1000 && Serial.availableForWrite() > 0){
        Serial.write(txQueue[txHead]);
        txHead = (txHead + 1) % LINK_TX_QUEUE;
        txCount--;
        txCredit -= 1000;
    }
}

void sendFrame(uint8_t type, const uint8_t *data, uint8_t len){
    uint8_t out[ESP_LINK_MAX_FRAME];
    uint8_t n = EspLinkCodec::encode(type, data, len, out);
    if(n == 0) return;
    if(txCount + n + 1 > LINK_TX_QUEUE){
        txDropped++;
        return;
    }
    //a leading 0x00 ends what the Mega received before, eg. our boot messages
    uint16_t t = (txHead + txCount) % LINK_TX_QUEUE;
    txQueue[t] = 0;
    for(uint8_t i = 0; i < n; i++) txQueue[(t + 1 + i) % LINK_TX_QUEUE] = out[i];
    txCount += n + 1;
    linkTxPump();
}

void sendHello(bool request){
    uint8_t d[3] = {ESP_LINK_VERSION, ESPLINK_ROLE_ESP, (uint8_t)(request ? 1 : 0)};
    sendFrame(ESPLINK_HELLO, d, 3);
}

bool megaLinkUp(){
    return megaSeen && millis() - lastLinkRx < ESP_LINK_TIMEOUT;
}

void wsSend(JsonDocument &d){
    if(ws.count() == 0) return;
    String s;
    serializeJson(d, s);
    ws.textAll(s);
}

void wsLog(const String &text){
    JsonDocument d;
    d["t"] = "log";
    d["text"] = text;
    wsSend(d);
}

String textOf(const uint8_t *d, uint8_t len){
    String s;
    for(uint8_t i = 0; i < len; i++) s += (char)d[i];
    return s;
}

void pushStatus(){
    JsonDocument d;
    d["t"] = "status";
    d["link"] = megaLinkUp();
    if(haveStatus){
        const uint8_t *s = megaStatus;
        d["rail"] = s[ESPLINK_STATUS_RAIL_STATE];
        d["main"] = (int16_t)espLinkGet16(s + ESPLINK_STATUS_MAIN_MA);
        d["prog"] = (int16_t)espLinkGet16(s + ESPLINK_STATUS_PROG_MA);
        d["volt"] = espLinkGet16(s + ESPLINK_STATUS_VOLTAGE_MV);
        d["clients"] = s[ESPLINK_STATUS_CLIENTS];
        d["loop"] = espLinkGet16(s + ESPLINK_STATUS_LOOP_US);
        d["ram"] = espLinkGet16(s + ESPLINK_STATUS_FREE_RAM);
        d["uptime"] = espLinkGet32(s + ESPLINK_STATUS_UPTIME_S);
        d["megaErr"] = espLinkGet16(s + ESPLINK_STATUS_LINK_ERRORS);
    }
    d["espErr"] = link.errors;
    d["udpIn"] = udpToMega;
    d["udpOut"] = udpFromMega;
    wsSend(d);
}

void pushModule(const ModuleInfo &m){
    JsonDocument d;
    d["t"] = "module";
    d["type"] = m.type;
    d["addr"] = m.address;
    d["typeId"] = m.typeId;
    d["channels"] = m.channels;
    d["slot"] = m.slot;
    d["version"] = m.version;
    d["board"] = m.board;
    d["flags"] = m.flags;
    wsSend(d);
}

//board frames carry the data of the MobaBus packets: high byte first
uint16_t be16(const uint8_t *d){ return ((uint16_t)d[0] << 8) | d[1]; }

void pushBoard(const BoardInfo &b){
    JsonDocument d;
    d["t"] = "board";
    d["board"] = b.id;
    d["version"] = b.version;
    d["built"] = b.built;
    d["configured"] = b.configured;
    d["slots"] = b.slots;
    d["pool"] = b.pool;
    d["flags"] = b.flags;
    wsSend(d);
}

//---------------------------------------------------------------- Z21 client of the web pages

void webZ21Send(const uint8_t *p, uint8_t n){
    if(n > UDP_MAX) return;
    uint8_t d[6 + UDP_MAX];
    memcpy(d, WEB_CLIENT_IP, 4);
    espLinkPut16(d + 4, WEB_CLIENT_PORT);
    memcpy(d + 6, p, n);
    sendFrame(ESPLINK_UDP_RX, d, 6 + n);
}

void webSubscribe(){
    //LAN_SET_BROADCASTFLAGS 0x00000003: driving and switching info, feedback (R-Bus),
    //keeps the client alive
    const uint8_t p[8] = {8, 0, 0x50, 0, 0x03, 0, 0, 0};
    webZ21Send(p, 8);
    lastWebSubscribe = millis();
    webSubscribed = true;
}

//LAN_X packet from the X bytes of a page: header 0x40 and XOR are added here
void webZ21X(JsonArrayConst x){
    uint8_t n = x.size();
    if(n == 0 || n > 16) return;
    if(!webSubscribed) webSubscribe();
    uint8_t p[16 + 5];
    p[0] = n + 5;
    p[1] = 0;
    p[2] = 0x40;
    p[3] = 0;
    uint8_t xr = 0;
    for(uint8_t i = 0; i < n; i++){
        p[4 + i] = x[i].as<uint8_t>();
        xr ^= p[4 + i];
    }
    p[4 + n] = xr;
    webZ21Send(p, n + 5);
}

//answers of the Mega to the web client: loco and turnout info go to the pages as JSON
void handleWebZ21(const uint8_t *p, uint8_t n){
    while(n >= 4){
        uint8_t len = p[0];
        if(len < 4 || len > n || p[1] != 0) return;
        if(p[2] == 0x40 && p[3] == 0 && len >= 6){
            const uint8_t *x = p + 4;
            uint8_t xn = len - 5;       //without the XOR byte
            if(x[0] == 0xEF && xn >= 6){
                //LAN_X_LOCO_INFO
                JsonDocument j;
                j["t"] = "loco";
                j["addr"] = ((x[1] & 0x3F) << 8) | x[2];
                j["steps"] = x[3] & 0x07;
                j["busy"] = (x[3] & 0x08) != 0;
                j["speed"] = x[4];
                uint32_t fn = ((x[5] >> 4) & 0x01) | ((uint32_t)(x[5] & 0x0F) << 1);
                if(xn >= 7) fn |= (uint32_t)x[6] << 5;
                if(xn >= 8) fn |= (uint32_t)x[7] << 13;
                if(xn >= 9) fn |= (uint32_t)x[8] << 21;
                j["fn"] = fn;
                wsSend(j);
            }
            else if(x[0] == 0x43 && xn >= 4){
                //LAN_X_TURNOUT_INFO: 1 = output 0, 2 = output 1, 0 = unknown
                JsonDocument j;
                j["t"] = "turnout";
                j["addr"] = (((x[1] & 0x3F) << 8) | x[2]) + 1;
                j["state"] = x[3] & 0x03;
                wsSend(j);
            }
            else if(x[0] == 0x64 && x[1] == 0x14 && xn >= 5){
                //LAN_X_CV_RESULT of the programming track, the CV counts from 0
                JsonDocument j;
                j["t"] = "prog";
                j["cv"] = (((x[2] & 0x3F) << 8) | x[3]) + 1;
                j["value"] = x[4];
                wsSend(j);
            }
            else if(x[0] == 0x61 && (x[1] == 0x12 || x[1] == 0x13) && xn >= 2){
                //LAN_X_CV_NACK_SC (short circuit) or LAN_X_CV_NACK (no answer of the decoder)
                JsonDocument j;
                j["t"] = "prog";
                j["error"] = x[1] == 0x12 ? "short" : "nack";
                wsSend(j);
            }
        }
        else if(p[2] == 0x80 && p[3] == 0 && len >= 15){
            //LAN_RMBUS_DATACHANGED: group, 10 feedback modules with 8 inputs each
            JsonDocument j;
            j["t"] = "rbus";
            j["group"] = p[4];
            JsonArray a = j["data"].to<JsonArray>();
            for(uint8_t i = 0; i < 10; i++) a.add(p[5 + i]);
            wsSend(j);
        }
        p += len;
        n -= len;
    }
}

void webZ21Loop(){
    if(!webSubscribed) return;
    if(ws.count() == 0){
        webSubscribed = false;          //the Mega drops the client by itself
        return;
    }
    if(millis() - lastWebSubscribe >= WEB_CLIENT_KEEPALIVE) webSubscribe();
}

//---------------------------------------------------------------- frames of the Mega

void handleFrame(){
    const uint8_t *d = link.data();
    uint8_t len = link.len;
    lastLinkRx = millis();
    switch(link.type){
        case ESPLINK_HELLO:
            if(len >= 2 && d[1] == ESPLINK_ROLE_MEGA){
                if(!megaSeen) wsLog(F("Zentrale verbunden"));
                megaSeen = true;
                megaAnswered = true;
                if(len >= 3 && d[2]){
                    sendHello(false);
                    //the Mega restarted and lost its clients: subscribe again
                    if(webSubscribed) webSubscribe();
                }
            }
            break;
        case ESPLINK_HEARTBEAT:
            megaSeen = true;
            break;
        case ESPLINK_UDP_TX:
            if(len >= 6 && memcmp(d, WEB_CLIENT_IP, 4) == 0){
                handleWebZ21(d + 6, len - 6);
            }
            else if(len >= 6 && udpPort){
                udp.beginPacket(IPAddress(d[0], d[1], d[2], d[3]), espLinkGet16(d + 4));
                udp.write(d + 6, len - 6);
                udp.endPacket();
                udpFromMega++;
            }
            break;
        case ESPLINK_STATUS:
            if(len >= ESPLINK_STATUS_SIZE){
                memcpy(megaStatus, d, ESPLINK_STATUS_SIZE);
                haveStatus = true;
                pushStatus();
            }
            break;
        case ESPLINK_SETTING:
            if(len >= 6){
                JsonDocument j;
                j["t"] = "setting";
                j["key"] = d[0];
                j["value"] = (int32_t)espLinkGet32(d + 1);
                j["ok"] = d[5] == 0;
                wsSend(j);
            }
            break;
        case ESPLINK_CV_RESULT:
            if(len >= 6){
                JsonDocument j;
                j["t"] = "cv";
                uint16_t a = espLinkGet16(d);
                j["addr"] = a & ~ESPLINK_CV_FEEDBACK;
                if(a & ESPLINK_CV_FEEDBACK) j["fb"] = true;
                j["cv"] = espLinkGet16(d + 2);
                j["value"] = d[4];
                j["status"] = d[5];
                wsSend(j);
            }
            break;
        case ESPLINK_MODULE:
            if(len >= 7){
                ModuleInfo m = {d[0], espLinkGet16(d + 1), d[3], d[4], d[5], d[6], 0, 0};
                if(len >= 10){
                    m.board = espLinkGet16(d + 7);
                    m.flags = d[9];
                }
                uint8_t i = 0;
                //a module is known by its board and slot (older modules: slot, type id and address)
                while(i < moduleCount && !(m.board ? modules[i].board == m.board && modules[i].slot == m.slot :
                       modules[i].slot == m.slot && modules[i].typeId == m.typeId && modules[i].address == m.address)) i++;
                if(i == moduleCount && moduleCount < MAX_MODULES_LIST) moduleCount++;
                if(i < MAX_MODULES_LIST) modules[i] = m;
                pushModule(m);
            }
            break;
        case ESPLINK_BOARD:
            if(len >= 7){
                BoardInfo b = {be16(d), d[2], d[3], d[4], d[5], d[6], (uint8_t)(len >= 8 ? d[7] : 0)};
                uint8_t i = 0;
                while(i < boardCount && boards[i].id != b.id) i++;
                if(i == boardCount && boardCount < MAX_BOARDS_LIST) boardCount++;
                if(i < MAX_BOARDS_LIST) boards[i] = b;
                pushBoard(b);
            }
            break;
        case ESPLINK_BOARD_RESULT:
            if(len >= 5){
                JsonDocument j;
                j["t"] = "breg";
                j["board"] = be16(d);
                j["reg"] = be16(d + 2);
                j["status"] = d[4];
                JsonArray v = j["values"].to<JsonArray>();
                for(uint8_t i = 5; i < len; i++) v.add(d[i]);
                wsSend(j);
            }
            break;
        case ESPLINK_LOG:
            wsLog(textOf(d, len));
            break;
    }
}

void linkLoop(){
    linkTxPump();
    while(Serial.available()){
        if(link.feed(Serial.read())) handleFrame();
    }
    uint32_t now = millis();
    if(now - lastBeat >= 1000){
        lastBeat = now;
        sendFrame(ESPLINK_HEARTBEAT, NULL, 0);
        //the Mega may have missed our start (the HELLO right after the boot messages):
        //ask again until it answered, its heartbeats alone are not enough
        if(!megaAnswered) sendHello(true);
        if(!megaLinkUp()) pushStatus();
    }
}

//---------------------------------------------------------------- WiFi

uint8_t netMode(){
    if(WiFi.status() == WL_CONNECTED) return ESPLINK_NET_STATION;
    if(apOn) return ESPLINK_NET_AP;
    return ESPLINK_NET_DOWN;
}

IPAddress netIp(){
    if(WiFi.status() == WL_CONNECTED) return WiFi.localIP();
    if(apOn) return WiFi.softAPIP();
    return IPAddress(0, 0, 0, 0);
}

void sendNetStatus(){
    IPAddress ip = netIp();
    uint8_t d[6] = {netMode(), ip[0], ip[1], ip[2], ip[3], (uint8_t)(int8_t)WiFi.RSSI()};
    sendFrame(ESPLINK_NET_STATUS, d, 6);

    JsonDocument j;
    j["t"] = "net";
    uint8_t m = netMode();
    j["mode"] = m == ESPLINK_NET_STATION ? (apOn ? "station+ap" : "station") : m == ESPLINK_NET_AP ? "ap" : "down";
    j["ip"] = ip.toString();
    j["rssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
    j["ssid"] = cfg.ssid;
    j["apClients"] = apOn ? WiFi.softAPgetStationNum() : 0;
    j["saved"] = cfg.saved;
    j["apPwdDefault"] = cfg.apPwd == AP_DEFAULT_PWD;
    j["heap"] = ESP.getFreeHeap();
    j["version"] = FIRMWARE_VERSION;
    wsSend(j);
}

void startAP(){
    if(apOn) return;
    //hybrid: AP and station, the station keeps searching the router
    WiFi.mode(cfg.mode == NET_MODE_AP || cfg.ssid.length() == 0 ? WIFI_AP : WIFI_AP_STA);
    WiFi.softAPConfig(cfg.ip, cfg.ip, cfg.subnet);
    //WPA2 needs at least 8 characters, a shorter password would fail
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPwd.length() >= 8 ? cfg.apPwd.c_str() : NULL);
    dns.start(53, "*", cfg.ip);         //captive portal: every name points to the web interface
    apOn = true;
    wsLog("Access Point " + cfg.apSsid + " gestartet");
}

void stopAP(){
    if(!apOn) return;
    dns.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apOn = false;
    wsLog(F("Access Point beendet, Router erreichbar"));
}

void startWiFi(){
    WiFi.persistent(false);         //the settings are in /net.json, not in the flash of the SDK
    WiFi.setSleepMode(WIFI_NONE_SLEEP); //no modem sleep: answers the web interface and Z21 apps without delay
    if(apOn){
        dns.stop();
        WiFi.softAPdisconnect(true);
        apOn = false;
    }
    WiFi.disconnect();
    WiFi.hostname("MobaStation");
    staLostSince = millis();
    staUpSince = 0;
    if(cfg.mode == NET_MODE_AP || cfg.ssid.length() == 0){
        startAP();
    }
    else{
        WiFi.mode(WIFI_STA);
        if(cfg.dhcp) WiFi.config(0U, 0U, 0U);
        else WiFi.config(cfg.ip, cfg.gateway, cfg.subnet);
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.ssid.c_str(), cfg.pwd.c_str());
    }
    if(udpPort != cfg.port){
        udp.stop();
        udp.begin(cfg.port);
        udpPort = cfg.port;
    }
}

void wifiLoop(){
    uint32_t now = millis();
    bool sta = WiFi.status() == WL_CONNECTED;
    if(sta && staUpSince == 0) staUpSince = now;
    if(!sta) staUpSince = 0;
    if(cfg.mode == NET_MODE_HYBRID && cfg.ssid.length() > 0){
        if(sta){
            staLostSince = 0;
            //router back: the access point goes, unless someone still uses it
            if(apOn && now - staUpSince > 10000 && WiFi.softAPgetStationNum() == 0) stopAP();
        }
        else{
            if(staLostSince == 0) staLostSince = now;
            if(!apOn && now - staLostSince > 20000) startAP();
        }
    }
    if(apOn) dns.processNextRequest();
    if(now - lastNetStatus >= 2000){
        lastNetStatus = now;
        sendNetStatus();
    }
}

//---------------------------------------------------------------- UDP (Z21)

void udpLoop(){
    for(uint8_t k = 0; k < 4; k++){
        int n = udp.parsePacket();
        if(n <= 0) return;
        uint8_t d[6 + UDP_MAX];
        IPAddress ip = udp.remoteIP();
        for(uint8_t i = 0; i < 4; i++) d[i] = ip[i];
        espLinkPut16(d + 4, udp.remotePort());
        int r = udp.read(d + 6, UDP_MAX);
        udp.flush();
        if(r > 0){
            sendFrame(ESPLINK_UDP_RX, d, 6 + r);
            udpToMega++;
        }
    }
}

//---------------------------------------------------------------- web interface

void wsCommand(JsonVariantConst j){
    const char *c = j["c"] | "";
    if(!strcmp(c, "power")){
        uint8_t d = j["on"].as<bool>() ? ESPLINK_CMD_POWER_ON : ESPLINK_CMD_POWER_OFF;
        sendFrame(ESPLINK_COMMAND, &d, 1);
    }
    else if(!strcmp(c, "resetSettings")){
        uint8_t d = ESPLINK_CMD_RESET_SETTINGS;
        sendFrame(ESPLINK_COMMAND, &d, 1);
    }
    else if(!strcmp(c, "stop")){
        uint8_t d = ESPLINK_CMD_EMERGENCY_STOP;
        sendFrame(ESPLINK_COMMAND, &d, 1);
    }
    else if(!strcmp(c, "turnout")){
        uint8_t d[4] = {ESPLINK_CMD_TURNOUT, 0, 0, (uint8_t)(j["dir"].as<int>() ? 1 : 0)};
        espLinkPut16(d + 1, j["addr"].as<int>());
        sendFrame(ESPLINK_COMMAND, d, 4);
    }
    else if(!strcmp(c, "settings")){
        for(uint8_t k = 1; k < SETTING_COUNT; k++) sendFrame(ESPLINK_SETTING_GET, &k, 1);
    }
    else if(!strcmp(c, "set")){
        uint8_t d[5] = {(uint8_t)j["key"].as<int>()};
        espLinkPut32(d + 1, (uint32_t)j["value"].as<int32_t>());
        sendFrame(ESPLINK_SETTING_SET, d, 5);
    }
    else if(!strcmp(c, "discover")){
        moduleCount = 0;
        boardCount = 0;
        sendFrame(ESPLINK_DISCOVER, NULL, 0);
    }
    else if(!strcmp(c, "breg")){
        //register of a universal board: "count" reads, "values" writes (register 0 = 1 takes the configuration)
        uint8_t d[8];
        int board = j["board"].as<int>(), reg = j["reg"].as<int>();
        d[0] = board >> 8; d[1] = board; d[2] = reg >> 8; d[3] = reg;
        JsonArrayConst v = j["values"].as<JsonArrayConst>();
        if(!v.isNull()){
            uint8_t n = 0;
            for(JsonVariantConst x: v){
                if(n == 4) break;
                d[4 + n++] = x.as<int>();
            }
            if(n > 0) sendFrame(ESPLINK_BOARD_WRITE, d, 4 + n);
        }
        else{
            int count = j["count"] | 1;
            d[4] = count < 1 ? 1 : count > 3 ? 3 : count;
            sendFrame(ESPLINK_BOARD_READ, d, 5);
        }
    }
    else if(!strcmp(c, "rbus")){
        //LAN_RMBUS_GETDATA of a group, the answer comes as "rbus"
        const uint8_t p[5] = {5, 0, 0x81, 0, (uint8_t)j["group"].as<int>()};
        webZ21Send(p, 5);
    }
    else if(!strcmp(c, "x")){
        //Z21 X command of the control and programming pages (loco, function, info, CVs)
        webZ21X(j["d"].as<JsonArrayConst>());
    }
    else if(!strcmp(c, "cv")){
        uint8_t d[5];
        //fb: feedback module, it has its own addresses
        espLinkPut16(d, (j["addr"].as<int>() & 0x3FFF) | (j["fb"].as<bool>() ? ESPLINK_CV_FEEDBACK : 0));
        espLinkPut16(d + 2, j["cv"].as<int>());
        if(j["value"].is<int>()){
            d[4] = j["value"].as<int>();
            sendFrame(ESPLINK_CV_WRITE, d, 5);
        }
        else{
            sendFrame(ESPLINK_CV_READ, d, 4);
        }
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
    if(type == WS_EVT_CONNECT){
        //a new page gets the current state at once
        sendNetStatus();
        pushStatus();
        for(uint8_t i = 0; i < boardCount; i++) pushBoard(boards[i]);
        for(uint8_t i = 0; i < moduleCount; i++) pushModule(modules[i]);
        webSubscribe();
    }
    else if(type == WS_EVT_DATA){
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT){
            JsonDocument j;
            if(!deserializeJson(j, data, len)) wsCommand(j.as<JsonVariantConst>());
        }
    }
}

//the request goes to the web interface itself, not to a foreign name of the captive portal DNS
bool ownHost(const String &host){
    String h = host;
    int colon = h.indexOf(':');
    if(colon >= 0) h.remove(colon);
    h.toLowerCase();
    return h.length() == 0 || h == cfg.ip.toString() || h == WiFi.localIP().toString()
        || h == "mobastation.local" || h == "mobastation";
}

//the page: gzip compressed with an ETag, a reload only asks whether it changed
void sendPage(AsyncWebServerRequest *r){
#ifdef WEB_PAGE_ETAG
    if(r->hasHeader("If-None-Match") && r->header("If-None-Match") == WEB_PAGE_ETAG){
        AsyncWebServerResponse *res = r->beginResponse(304);
        res->addHeader("ETag", WEB_PAGE_ETAG);
        r->send(res);
        return;
    }
    AsyncWebServerResponse *res = r->beginResponse(200, "text/html; charset=utf-8", WEB_PAGE_GZ, sizeof(WEB_PAGE_GZ));
    res->addHeader("Content-Encoding", "gzip");
    res->addHeader("ETag", WEB_PAGE_ETAG);
    res->addHeader("Cache-Control", "no-cache");
    r->send(res);
#else
    r->send(200, "text/html; charset=utf-8", WEB_PAGE);
#endif
}

void setupWeb(){
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, sendPage);
    //browsers ask for it on every page, the page has none
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(204); });

    server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *r){
        JsonDocument d;
        configToJson(cfg, d["active"].to<JsonObject>(), false);
        String s;
        serializeJson(d, s);
        r->send(200, "application/json", s);
    });

    AsyncCallbackJsonWebHandler *save = new AsyncCallbackJsonWebHandler("/api/config",
        [](AsyncWebServerRequest *r, JsonVariant &json){
            NetConfig c = cfg;
            configFromJson(c, json.as<JsonVariantConst>());
            if(c.mode != NET_MODE_AP && c.ssid.length() == 0 && c.mode == NET_MODE_STATION){
                r->send(400, "text/plain", "Ohne WLAN-Name ist nur Access Point oder Hybrid möglich");
                return;
            }
            if(c.apPwd.length() > 0 && c.apPwd.length() < 8){
                r->send(400, "text/plain", "Das Access-Point-Passwort braucht mindestens 8 Zeichen");
                return;
            }
            if(!saveConfig(c)){
                r->send(500, "text/plain", "Speichern fehlgeschlagen");
                return;
            }
            r->send(200, "text/plain", "Gespeichert, das WLAN startet neu");
            restartAt = millis() + 1000;
        });
    server.addHandler(save);

    //locos, turnouts and accessories of the control page
    server.on("/api/items", HTTP_GET, [](AsyncWebServerRequest *r){
        if(LittleFS.exists(ITEMS_FILE)) r->send(LittleFS, ITEMS_FILE, "application/json");
        else r->send(200, "application/json", "{\"locos\":[],\"items\":[]}");
    });

    AsyncCallbackJsonWebHandler *items = new AsyncCallbackJsonWebHandler("/api/items",
        [](AsyncWebServerRequest *r, JsonVariant &json){
            if(!json["locos"].is<JsonArray>() || !json["items"].is<JsonArray>()){
                r->send(400, "text/plain", "Ungültige Liste");
                return;
            }
            File f = LittleFS.open(ITEMS_FILE, "w");
            if(!f){
                r->send(500, "text/plain", "Speichern fehlgeschlagen");
                return;
            }
            serializeJson(json, f);
            f.close();
            r->send(200, "text/plain", "Gespeichert");
        });
    items->setMaxContentLength(ITEMS_MAX_SIZE);
    server.addHandler(items);

    server.on("/api/config/reset", HTTP_POST, [](AsyncWebServerRequest *r){
        clearConfig();
        r->send(200, "text/plain", "WLAN-Einstellungen gelöscht, Neustart als Access Point");
        restartAt = millis() + 1000;
    });

    server.on("/api/restart", HTTP_POST, [](AsyncWebServerRequest *r){
        r->send(200, "text/plain", "Neustart");
        restartAt = millis() + 500;
    });

    //firmware update of the ESP: curl -F "firmware=@MobaStationESP.ino.bin" http://IP/update
    server.on("/update", HTTP_POST,
        [](AsyncWebServerRequest *r){
            bool ok = !Update.hasError();
            r->send(ok ? 200 : 500, "text/plain", ok ? "Update OK, Neustart" : "Update fehlgeschlagen");
            if(ok) restartAt = millis() + 1000;
        },
        [](AsyncWebServerRequest *r, const String &filename, size_t index, uint8_t *data, size_t len, bool final){
            if(index == 0){
                Update.runAsync(true);
                if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) return;
            }
            if(!Update.hasError()) Update.write(data, len);
            if(final) Update.end(true);
        });

    server.onNotFound([](AsyncWebServerRequest *r){
        //captive portal: the DNS server points every name at us. A short redirect for foreign
        //hosts (the connectivity checks of phones and laptops: /generate_204, /hotspot-detect.html,
        ///connecttest.txt, ...) makes them show the login page with the web interface
        if(apOn && !ownHost(r->host())){
            AsyncWebServerResponse *res = r->beginResponse(302);
            res->addHeader("Location", "http://" + cfg.ip.toString() + "/");
            res->addHeader("Cache-Control", "no-store");
            r->send(res);
        }
        else r->send(404, "text/plain", "Nicht gefunden");
    });

    server.begin();
}

//---------------------------------------------------------------- start

void setup(){
    //20 ms of data at 500000 baud while WiFi keeps the loop busy
    Serial.setRxBufferSize(2048);
    Serial.begin(ESP_LINK_BAUD);
    Serial.setDebugOutput(false);

    if(!LittleFS.begin()){
        LittleFS.format();
        LittleFS.begin();
    }
    loadConfig(cfg);

    sendHello(true);
    startWiFi();
    setupWeb();

    //network upload from the Arduino IDE (port "MobaStation"), also gives the name MobaStation.local
    ArduinoOTA.setHostname("MobaStation");
    ArduinoOTA.onStart([](){ wsLog(F("Firmware-Update über das Netzwerk startet")); });
    ArduinoOTA.onError([](ota_error_t e){ wsLog("Firmware-Update fehlgeschlagen (" + String(e) + ")"); });
    ArduinoOTA.begin();
}

void loop(){
    ArduinoOTA.handle();
    linkLoop();
    udpLoop();
    wifiLoop();
    webZ21Loop();
    ws.cleanupClients();
    if(restartAt && (int32_t)(millis() - restartAt) >= 0){
        ESP.restart();
    }
}
