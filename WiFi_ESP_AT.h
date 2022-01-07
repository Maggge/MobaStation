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

typedef __FlashStringHelper FSH;
#define GETFLASH(addr) pgm_read_byte_near(addr)

#define WIFI_CONNECT_TIMEOUT 16000

#define SEND_BUF_SIZE 20

#define SEND_TIMEOUT 200

class WiFiClass{
private:
    Stream *wifi;

    WL_STATE state;

    WIFI_PACKET recBuffer;
    RECEIVE_STATE receiveState;
    bool pendingReceive;
    uint8_t readDataIdx = 0;

    WIFI_PACKET sendBuffer[SEND_BUF_SIZE];
    uint8_t sendIdx;
    uint8_t tail;
    bool waitForSend = false;
    bool pendingCipsend;
    uint32_t sendTime;

    bool oldCmd = false;

    bool linkIdOccupied[4];

    bool checkForAnswer( const unsigned int timeout, const FSH * waitfor);
    bool checkForOk(const unsigned int timeout);

    //Return the linkID, 255 if not successfull
    uint8_t getFreeLinkID();

    void sendNext();
    void txData();
public:
    bool init(HardwareSerial *ser, unsigned long baud);
    bool init(Stream *stream);

    bool setIP(IPAddress ip);
    bool setIP(const char *ip);
    bool setHostname(const char *hostname);
    bool connectToAP(const char * ssid, const char * pwd);
    bool startAP(const char * ssid, const char * pwd);

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