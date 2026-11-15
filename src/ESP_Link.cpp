/**
  MobaStation ESP Link

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

#include "ESP_Link.h"

ESP_Link::ESP_Link(HardwareSerial *serial){
    ser = serial;
}

bool ESP_Link::begin(uint16_t port){
    this->port = port;
    ser->begin(ESP_LINK_BAUD);
    MobaStation::setManagement(this);
    uint8_t hello[3] = {ESP_LINK_VERSION, ESPLINK_ROLE_MEGA, 1};
    sendFrame(ESPLINK_HELLO, hello, 3);
    Serial.println(F("ESP link started, waiting for the ESP"));
    return true;
}

bool ESP_Link::linkUp(){
    return espSeen && millis() - lastRx < ESP_LINK_TIMEOUT;
}

uint16_t ESP_Link::linkErrors(){
    return rx.errors;
}

void ESP_Link::sendFrame(uint8_t type, const uint8_t *data, uint8_t len){
    uint8_t out[ESP_LINK_MAX_FRAME];
    uint8_t n = EspLinkCodec::encode(type, data, len, out);
    if(n == 0) return;
    //a leading 0x00 ends whatever the other side received before (boot messages of the ESP)
    ser->write((uint8_t)0);
    ser->write(out, n);
}

void ESP_Link::sendText(uint8_t type, uint8_t key, const char *text){
    uint8_t d[ESP_LINK_MAX_PAYLOAD - 1];
    uint8_t n = 0;
    d[n++] = key;
    while(text != NULL && *text && n < sizeof(d)) d[n++] = *text++;
    sendFrame(type, d, n);
}

void ESP_Link::sendStatus(){
    uint8_t s[ESPLINK_STATUS_SIZE];
    s[ESPLINK_STATUS_RAIL_STATE] = MobaStation::getRailState();
    espLinkPut16(s + ESPLINK_STATUS_MAIN_MA, MobaStation::getMainCurrent());
    espLinkPut16(s + ESPLINK_STATUS_PROG_MA, MobaStation::getProgCurrent());
    espLinkPut16(s + ESPLINK_STATUS_VOLTAGE_MV, MobaStation::getVoltage());
    s[ESPLINK_STATUS_CLIENTS] = MobaStation::getClientCount();
    espLinkPut16(s + ESPLINK_STATUS_LOOP_US, MobaStation::getLoopTime());
    espLinkPut16(s + ESPLINK_STATUS_FREE_RAM, MobaStation::getFreeRam());
    espLinkPut32(s + ESPLINK_STATUS_UPTIME_S, millis() / 1000);
    espLinkPut16(s + ESPLINK_STATUS_LINK_ERRORS, rx.errors);
    sendFrame(ESPLINK_STATUS, s, ESPLINK_STATUS_SIZE);
}

void ESP_Link::sendSetting(uint8_t key, uint8_t status){
    uint8_t d[6];
    int32_t value = 0;
    if(!MobaStation::getSetting(key, value) && status == 0) status = 1;
    d[0] = key;
    espLinkPut32(d + 1, (uint32_t)value);
    d[5] = status;
    sendFrame(ESPLINK_SETTING, d, 6);
}

bool ESP_Link::send(IPAddress client, uint16_t dstPort, uint8_t *data){
    uint8_t d[ESP_LINK_MAX_PAYLOAD - 1];
    uint8_t len = data[0];
    if((size_t)len + 6 > sizeof(d)) return false;
    for(uint8_t i = 0; i < 4; i++) d[i] = client[i];
    espLinkPut16(d + 4, dstPort);
    for(uint8_t i = 0; i < len; i++) d[6 + i] = data[i];
    sendFrame(ESPLINK_UDP_TX, d, len + 6);
    return true;
}

bool ESP_Link::receivePacket(UdpPacket *pkg){
    uint32_t now = millis();
    if(now - lastBeat >= 1000){
        lastBeat = now;
        sendFrame(ESPLINK_HEARTBEAT, NULL, 0);
        sendStatus();
    }
    while(ser->available()){
        if(rx.feed(ser->read())){
            lastRx = millis();
            //a Z21 packet goes to the station at once, the rest of the bytes waits for the next call
            if(handleFrame(pkg)) return true;
        }
    }
    return false;
}

bool ESP_Link::handleFrame(UdpPacket *pkg){
    const uint8_t *d = rx.data();
    uint8_t len = rx.len;
    switch(rx.type){
        case ESPLINK_HELLO:
            if(len >= 2 && d[1] == ESPLINK_ROLE_ESP){
                if(!espSeen) Serial.println(F("ESP link: ESP found"));
                espSeen = true;
                if(len >= 3 && d[2]){
                    //the ESP started: answer, then it stops asking
                    uint8_t hello[3] = {ESP_LINK_VERSION, ESPLINK_ROLE_MEGA, 0};
                    sendFrame(ESPLINK_HELLO, hello, 3);
                }
            }
            return false;
        case ESPLINK_HEARTBEAT:
            espSeen = true;
            return false;
        case ESPLINK_UDP_RX:{
            if(len < 6) return false;
            uint16_t n = len - 6;
            if(n > Z21_MAX_UDP_SIZE) n = Z21_MAX_UDP_SIZE;
            pkg->ip = IPAddress(d[0], d[1], d[2], d[3]);
            pkg->port = espLinkGet16(d + 4);
            pkg->dataLength = n;
            for(uint16_t i = 0; i < n; i++) pkg->data[i] = d[6 + i];
            return true;
        }
        case ESPLINK_NET_STATUS:
            if(len >= 5 && (d[0] != netMode || memcmp(netIp, d + 1, 4) != 0)){
                //the address the Z21 apps need, also without web interface
                netMode = d[0];
                memcpy(netIp, d + 1, 4);
                Serial.print(netMode == ESPLINK_NET_STATION ? F("WiFi station, IP ") :
                             netMode == ESPLINK_NET_AP ? F("WiFi access point, IP ") : F("WiFi down "));
                Serial.println(IPAddress(netIp[0], netIp[1], netIp[2], netIp[3]));
            }
            return false;
        case ESPLINK_SETTING_GET:
            if(len >= 1) sendSetting(d[0], 0);
            return false;
        case ESPLINK_SETTING_SET:
            if(len >= 5){
                bool ok = MobaStation::setSetting(d[0], (int32_t)espLinkGet32(d + 1));
                sendSetting(d[0], ok ? 0 : 1);
            }
            return false;
        case ESPLINK_CV_READ:
        case ESPLINK_CV_WRITE:{
            if(len < 4) return false;
            uint16_t addr = espLinkGet16(d);
            uint8_t type = addr & ESPLINK_CV_FEEDBACK ? FEEDBACK : ACCESSORIE;
            addr &= ~ESPLINK_CV_FEEDBACK;
            uint16_t cv = espLinkGet16(d + 2);
            bool write = rx.type == ESPLINK_CV_WRITE;
            uint8_t value = write && len >= 5 ? d[4] : 0;
            if(!MobaStation::requestCv(type, addr, cv, write, value)){
                cvResult(type, addr, cv, value, 254);     //no MobaBus module
            }
            return false;
        }
        case ESPLINK_DISCOVER:
            MobaStation::discoverModules();
            return false;
        case ESPLINK_BOARD_READ:
        case ESPLINK_BOARD_WRITE:
            if(!MobaStation::boardRequest(rx.type == ESPLINK_BOARD_WRITE, d, len) && len >= 4){
                uint8_t r[5] = {d[0], d[1], d[2], d[3], 254};      //no MobaBus
                sendFrame(ESPLINK_BOARD_RESULT, r, 5);
            }
            return false;
        case ESPLINK_COMMAND:
            if(len < 1) return false;
            switch(d[0]){
                case ESPLINK_CMD_POWER_OFF: MobaStation::setRailPower(false); break;
                case ESPLINK_CMD_POWER_ON: MobaStation::setRailPower(true); break;
                case ESPLINK_CMD_EMERGENCY_STOP: MobaStation::stopAll(); break;
                case ESPLINK_CMD_RESET_SETTINGS: MobaStation::resetSettings(); break;
                case ESPLINK_CMD_TURNOUT:
                    if(len >= 4){
                        MobaStation::setTurnout(espLinkGet16(d + 1), d[3], true);
                        MobaStation::setTurnout(espLinkGet16(d + 1), d[3], false);
                    }
                    break;
            }
            return false;
    }
    return false;
}

void ESP_Link::cvResult(uint8_t type, uint16_t address, uint16_t cv, uint8_t value, uint8_t status){
    uint8_t d[6];
    espLinkPut16(d, address | (type == FEEDBACK ? ESPLINK_CV_FEEDBACK : 0));
    espLinkPut16(d + 2, cv);
    d[4] = value;
    d[5] = status;
    sendFrame(ESPLINK_CV_RESULT, d, 6);
}

void ESP_Link::moduleFound(uint8_t type, uint16_t address, uint8_t typeId, uint8_t channels, uint8_t slot, uint8_t version,
                           uint16_t board, uint8_t flags){
    uint8_t d[10] = {type, 0, 0, typeId, channels, slot, version, 0, 0, flags};
    espLinkPut16(d + 1, address);
    espLinkPut16(d + 7, board);
    sendFrame(ESPLINK_MODULE, d, 10);
}

void ESP_Link::boardFound(const uint8_t *data, uint8_t len){
    sendFrame(ESPLINK_BOARD, data, len);
}

void ESP_Link::boardResult(const uint8_t *data, uint8_t len){
    sendFrame(ESPLINK_BOARD_RESULT, data, len);
}
