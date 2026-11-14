/**
  MobaStation Wifi Esp AT

  © 2021, Markus Mair. All rights reserved.

  This file is part of the MobaStation Project

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __WIFI_ESP_AT__
#define __WIFI_ESP_AT__

#include <Arduino.h>
#include <stdarg.h>
#include "WiFi.h"

//the WiFi files are compiled without MobaStation.h, so they pick up the
//user configuration themselves (WIFI_* settings)
#include "MobaStationUserConfig.h"

typedef __FlashStringHelper FSH;
#define GETFLASH(addr) pgm_read_byte_near(addr)

#ifndef WIFI_CONNECT_TIMEOUT
#define WIFI_CONNECT_TIMEOUT 16000
#endif

//time between two wifi reconnect attempts
#ifndef WIFI_RECONNECT_INTERVAL
#define WIFI_RECONNECT_INTERVAL 5000
#endif

//how long a single AT+CWJAP of a reconnect may take
#ifndef WIFI_RECONNECT_JOIN_TIMEOUT
#define WIFI_RECONNECT_JOIN_TIMEOUT 12000
#endif

//hybrid mode: after this many failed reconnect rounds (3 joins each) the own
//accesspoint is started next to the station
#ifndef WIFI_FALLBACK_ROUNDS
#define WIFI_FALLBACK_ROUNDS 3
#endif

//while the fallback accesspoint runs the station is searched less often,
//every join attempt makes the accesspoint unreachable for a moment
#ifndef WIFI_FALLBACK_RECONNECT_INTERVAL
#define WIFI_FALLBACK_RECONNECT_INTERVAL 60000
#endif

//the debug build is close to the ram limit of the mega, so the send queue is
//smaller there, 20 packets would need 1420 bytes
#ifndef SEND_BUF_SIZE
  #ifdef DEBUG
  #define SEND_BUF_SIZE 12
  #else
  #define SEND_BUF_SIZE 20
  #endif
#endif

#define SEND_TIMEOUT 200

//usable client links when CIPMUX=1. link 4 is reserved by the AT firmware for the TCP server
#define WIFI_LINKS 4

//max length of an ssid and a password in the AT firmware
#define WIFI_SSID_MAX_LEN 32
#define WIFI_PWD_MAX_LEN 64

//size of the buffer used to parse unsolicited result codes and AT responses
#define WIFI_LINE_SIZE 24

//AT commands that are sent by the reconnect state machine
#define WIFI_CMD_NONE 0
#define WIFI_CMD_JOIN 1
#define WIFI_CMD_IP 2
#define WIFI_CMD_HOSTNAME 3
#define WIFI_CMD_OPEN 4
#define WIFI_CMD_SERVER 5
#define WIFI_CMD_CIPMUX 6
#define WIFI_CMD_CIPDINFO 7
#define WIFI_CMD_AP_MODE 8
#define WIFI_CMD_AP_CONFIG 9
#define WIFI_CMD_AP_IP 10
#define WIFI_CMD_AP_DHCP 11
#define WIFI_CMD_STA_ONLY 12

class WiFiClass{
private:
    Stream *wifi = NULL;

    WL_STATE state = WL_NO_MODULE;

    WiFiMode activeMode = Station_Mode;


    WIFI_PACKET recBuffer;
    RECEIVE_STATE receiveState = STANDBY;
    bool pendingReceive = false;
    uint8_t readDataIdx = 0;
    //bytes of the current payload that do not fit into recBuffer and must be discarded
    uint16_t dropBytes = 0;

    WIFI_PACKET sendBuffer[SEND_BUF_SIZE];
    uint8_t sendIdx = 0;
    uint8_t tail = 0;
    bool waitForSend = false;
    bool pendingCipsend = false;
    bool promptSeen = false;        //the '>' of the pending CIPSEND arrived, the data is written
    uint32_t sendTime = 0;

    bool oldCmd = false;
    //the module echoes everything we write (ATE1), also the payload after the CIPSEND prompt
    bool echo = true;

    bool linkIdOccupied[WIFI_LINKS] = {false};

    //---------------- wifi reconnect ----------------
    RECONNECT_STATE reconnectState = RECONNECT_IDLE;
    uint32_t reconnectTime = 0;     //time of the last state change
    uint8_t joinRetries = 0;

    char ssid[WIFI_SSID_MAX_LEN + 1] = "";
    char pwd[WIFI_PWD_MAX_LEN + 1] = "";

    bool stationMode = false;       //a station connection must be kept alive
    bool staticIpSet = false;
    uint8_t staticIp[4] = {0};
    uint8_t staticGateway[4] = {0};    //0.0.0.0 = only the ip is set
    uint8_t staticSubnet[4] = {0};
    //prints "ip" or "ip","gateway","netmask" of the static address
    void printStaticIp();
    char linkHost[16] = "0.0.0.0";  //host of the link that has to be reopened
    uint16_t linkPort = 0;
    uint8_t linkId = 255;
    uint16_t tcpServerPort = 0;     //restart the tcp server after a reconnect

    //---------------- accesspoint ----------------
    bool apActive = false;          //an own accesspoint runs (SoftAP or hybrid fallback)
    bool fallbackAP = false;        //hybrid mode: start the accesspoint when the station is lost
    uint8_t failedRounds = 0;       //reconnect rounds without success
    char apSsid[WIFI_SSID_MAX_LEN + 1] = "";
    char apPwd[WIFI_PWD_MAX_LEN + 1] = "";
    uint8_t apIp[4] = {192, 168, 0, 111};
    void storeAP(const char *ssid, const char *pwd, IPAddress ip);
    //prints one of the accesspoint commands without the leading "AT+"
    void printApCmd(uint8_t what);

    //response of the AT command that was started by the reconnect state machine
    uint8_t cmdCmd = WIFI_CMD_NONE;
    bool cmdExpected = false;
    uint32_t cmdStart = 0;
    uint32_t cmdTimeout = 0;
    char line[WIFI_LINE_SIZE];
    uint8_t lineIdx = 0;
    uint8_t matchIdx = 0;
    uint16_t parseValue = 0;        //current number while parsing

    //parse a text line that was received between the AT commands
    void handleLine();
    void linkClosed(uint8_t id);
    bool lineContains(const char *line, const char *str);
    bool cmdActive();
    //called when the response of an AT command of the reconnect is known
    void cmdResult(bool success);
    void cmdFail();

    void reconnectStateMachine(uint32_t now);
    void reconnectNext();
    void reconnectJoin();
    void reconnectSetIp();
    void reconnectHostname();
    void reconnectCipmux();
    void reconnectDinfo();
    void reconnectStartLink();
    void reconnectApStep(RECONNECT_STATE st, uint8_t what);

    void sendCmd(const __FlashStringHelper *cmd, uint8_t what, uint32_t timeout);
    //wifi connection lost, all links are gone with it
    void markWifiLost();
    void clearSendQueue();

    bool checkForAnswer( const unsigned int timeout, const FSH * waitfor);
    //waits for the end of AT+CWJAP and prints why it failed (+CWJAP:<code>)
    bool waitJoin(uint32_t timeout);
    bool checkForOk(const unsigned int timeout);

    //Return the linkID, 255 if not successfull
    uint8_t getFreeLinkID();

    //packets can be sent as station and as accesspoint
    bool canSend();
    void sendNext();
    void txData();
    //the pending packet is done (sent, failed or timed out), continue with the next one
    void finishSend();
public:
    bool init(HardwareSerial *ser, unsigned long baud);
    bool init(Stream *stream);

    bool setIP(IPAddress ip);
    //static address with gateway and netmask, kept for the reconnect
    bool setIP(IPAddress ip, IPAddress gateway, IPAddress subnet);
    //the station gets its address from the router, a static address is forgotten
    bool enableDHCP();
    //address of the station, asks the module (blocking, max ~3s), 0.0.0.0 if unknown
    IPAddress localIP();
    bool setIP(const char *ip);
    bool setHostname(const char *hostname);
    bool connectToAP(const char * ssid, const char * pwd);
    //blocking, keepStation = true runs the accesspoint next to the station (CWMODE 3),
    //the accesspoint uses ip as its own address
    bool startAP(const char * ssid, const char * pwd, IPAddress ip, bool keepStation = false);
    //hybrid mode: the reconnect starts this accesspoint on its own, never blocking,
    //after WIFI_FALLBACK_ROUNDS failed rounds and switches it off when the station is back
    void enableFallbackAP(const char * ssid, const char * pwd, IPAddress ip);
    bool isAPActive();

    WiFiMode getMode();
    WL_STATE getStatus();
    //true while the station connection is down and a reconnect is running
    RECONNECT_STATE getReconnectState();
    //number of reconnect attempts of the current wifi outage
    uint8_t getJoinRetries();
    //true when the module lost the connection to the access point,
    //the reconnect runs inside loop() and never blocks
    bool needsReconnect();
    //start a reconnect now instead of waiting for WIFI_RECONNECT_INTERVAL
    void startReconnect();

    //Return the linkID, 255 if not successfull
    uint8_t connect(CONN_TYPE type, const char *host, uint16_t port);
    bool disconnect(uint8_t linkID);
    uint8_t startTcpServer(uint16_t port);
    bool stopTcpServer();

    bool packetAvailable(uint8_t linkId);

    bool getPacket(uint8_t linkId, WIFI_PACKET *pkg);

    bool sendPkg(WIFI_PACKET *pkg);

    void loop();

};

extern WiFiClass WiFi;

#endif