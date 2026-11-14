/**
  MobaStation ESP link protocol

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

/*
  Serial link between the Arduino Mega (command station) and the ESP8266 with the
  MobaStation firmware (WiFi, UDP, web interface). Used by both sides, so it only
  depends on <stdint.h>: the ESP build adds this folder to its include path.

  Frame:  0x00 COBS( type, data..., crc16 low, crc16 high ) 0x00
  - the leading 0x00 ends bytes received before (boot messages of the ESP), so the first
    frame after a start is not lost; empty frames are ignored
  - 0x00 ends a frame, a receiver that lost bytes (boot messages of the ESP, overflow)
    finds the next frame at the next 0x00
  - CRC-16/CCITT-FALSE over type and data, broken frames are dropped
  - numbers are little endian and written byte by byte: AVR and ESP lay out structs
    differently
*/

#ifndef __ESP_LINK_PROTOCOL__
#define __ESP_LINK_PROTOCOL__

#include <stdint.h>

#define ESP_LINK_VERSION 1

//the Mega buffers only 64 received bytes and its loop can stand a few ms (EEPROM
//writes, CAN), 115200 baud gives it 5.5 ms. Both sides must use the same value
#ifndef ESP_LINK_BAUD
#define ESP_LINK_BAUD 115200
#endif

//type + data, enough for a Z21 datagram of 64 bytes with ip and port
#define ESP_LINK_MAX_PAYLOAD 80
//COBS adds 1 byte (payload < 254), plus crc and the 0x00 delimiter
#define ESP_LINK_MAX_FRAME (ESP_LINK_MAX_PAYLOAD + 2 + 1 + 1)

//a side that hears nothing for this time considers the link lost
#define ESP_LINK_TIMEOUT 3000

enum EspLinkType : uint8_t {
    //both: version, role (ESPLINK_ROLE_*), request (1 = please answer with HELLO, sent after
    //a start; the answer has 0, so the two sides do not answer each other forever)
    ESPLINK_HELLO        = 0x01,
    ESPLINK_HEARTBEAT    = 0x02,    //both, every second, no data
    ESPLINK_UDP_RX       = 0x10,    //ESP -> Mega: ip[4], port u16, Z21 data
    ESPLINK_UDP_TX       = 0x11,    //Mega -> ESP: ip[4], port u16, Z21 data
    ESPLINK_STATUS       = 0x20,    //Mega -> ESP every second, see ESPLINK_STATUS_* offsets
    ESPLINK_NET_STATUS   = 0x21,    //ESP -> Mega: mode (ESPLINK_NET_*), ip[4], rssi i8
    //Mega -> ESP after HELLO: key (ESPLINK_NETDEF_*), value. Settings of MobaStationConfig.h,
    //the ESP uses them as long as nothing was saved in its web interface
    ESPLINK_NET_DEFAULT  = 0x22,
    ESPLINK_SETTING_GET  = 0x30,    //ESP -> Mega: key (EspLinkSetting)
    ESPLINK_SETTING_SET  = 0x31,    //ESP -> Mega: key, value i32
    ESPLINK_SETTING      = 0x32,    //Mega -> ESP: key, value i32, status (0 = ok)
    ESPLINK_CV_READ      = 0x40,    //ESP -> Mega: address u16, cv u16 (CV1 = 1)
    ESPLINK_CV_WRITE     = 0x41,    //ESP -> Mega: address u16, cv u16, value
    ESPLINK_CV_RESULT    = 0x42,    //Mega -> ESP: address u16, cv u16, value, status (MobaCvStatus, 255 = timeout)
    ESPLINK_DISCOVER     = 0x48,    //ESP -> Mega: ask all MobaBus modules
    ESPLINK_MODULE       = 0x49,    //Mega -> ESP: type, address u16, type id, channels, slot, version
    ESPLINK_COMMAND      = 0x50,    //ESP -> Mega: command (EspLinkCommand), arguments
    ESPLINK_LOG          = 0x60,    //Mega -> ESP: text without line end
};

enum EspLinkRole : uint8_t {
    ESPLINK_ROLE_MEGA = 0,
    ESPLINK_ROLE_ESP  = 1,
};

enum EspLinkNetDefault : uint8_t {
    ESPLINK_NETDEF_SSID     = 1,    //text
    ESPLINK_NETDEF_PWD      = 2,    //text
    ESPLINK_NETDEF_AP_SSID  = 3,    //text
    ESPLINK_NETDEF_AP_PWD   = 4,    //text
    ESPLINK_NETDEF_IP       = 5,    //dhcp u8, ip[4], subnet[4], gateway[4]
    ESPLINK_NETDEF_MODE     = 6,    //0 station, 1 access point, 2 hybrid (access point as fallback)
    ESPLINK_NETDEF_PORT     = 7,    //u16 Z21 port
    ESPLINK_NETDEF_DONE     = 8,    //all defaults sent
};

enum EspLinkNetMode : uint8_t {
    ESPLINK_NET_DOWN    = 0,
    ESPLINK_NET_STATION = 1,    //connected to the router
    ESPLINK_NET_AP      = 2,    //own access point
};

//runtime settings of the command station, stored in the EEPROM of the Mega
enum EspLinkSetting : uint8_t {
    SETTING_TURNOUT_MODE    = 1,    //0 MobaBus only, 1 DCC only, 2 combined
    SETTING_DCC_RANGE       = 2,    //combined: turnouts up to this address are DCC
    SETTING_VOLTAGE_MULTI   = 3,    //voltage multiplier * 1000
    SETTING_POWER_ON_START  = 4,    //1 = track power on after the start
    SETTING_COUNT           = 5,
};

enum EspLinkCommand : uint8_t {
    ESPLINK_CMD_POWER_OFF       = 0,
    ESPLINK_CMD_POWER_ON        = 1,
    ESPLINK_CMD_EMERGENCY_STOP  = 2,
    ESPLINK_CMD_TURNOUT         = 3,    //address u16, direction
    ESPLINK_CMD_RESET_SETTINGS  = 4,    //forget the stored settings, MobaStationConfig.h after the next start
};

//data of ESPLINK_STATUS
#define ESPLINK_STATUS_RAIL_STATE   0   //u8 central state (csNormal, csTrackVoltageOff, ...)
#define ESPLINK_STATUS_MAIN_MA      1   //i16
#define ESPLINK_STATUS_PROG_MA      3   //i16
#define ESPLINK_STATUS_VOLTAGE_MV   5   //u16
#define ESPLINK_STATUS_CLIENTS      7   //u8 Z21 clients
#define ESPLINK_STATUS_LOOP_US      8   //u16 mean loop time
#define ESPLINK_STATUS_FREE_RAM     10  //u16 lowest free RAM since the start
#define ESPLINK_STATUS_UPTIME_S     12  //u32
#define ESPLINK_STATUS_LINK_ERRORS  16  //u16 broken frames received by the Mega
#define ESPLINK_STATUS_SIZE         18

static inline void espLinkPut16(uint8_t *p, uint16_t v){ p[0] = v; p[1] = v >> 8; }
static inline void espLinkPut32(uint8_t *p, uint32_t v){ p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static inline uint16_t espLinkGet16(const uint8_t *p){ return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t espLinkGet32(const uint8_t *p){
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint16_t espLinkCrc(uint16_t crc, uint8_t b){
    crc ^= (uint16_t)b << 8;
    for(uint8_t i = 0; i < 8; i++){
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

/**
 * Builds and parses frames. One object per direction is enough: encode() is static,
 * feed() collects the bytes of the receiver.
 */
class EspLinkCodec{
private:
    uint8_t buf[ESP_LINK_MAX_FRAME];
    uint8_t idx = 0;
    bool overflow = false;
public:
    //valid after feed() returned true
    uint8_t type = 0;
    uint8_t len = 0;            //data bytes without type and crc
    const uint8_t *data(){ return buf + 1; }
    uint16_t errors = 0;        //broken or too long frames

    /**
     * @param out at least ESP_LINK_MAX_FRAME bytes
     * @return length of the frame including the 0x00 delimiter, 0 if data is too long
     */
    static uint8_t encode(uint8_t type, const uint8_t *data, uint8_t len, uint8_t *out){
        if(len + 1 > ESP_LINK_MAX_PAYLOAD) return 0;
        uint16_t crc = espLinkCrc(0xFFFF, type);
        for(uint8_t i = 0; i < len; i++) crc = espLinkCrc(crc, data[i]);
        //COBS: every 0x00 becomes the distance to the next 0x00
        uint8_t codeIdx = 0, o = 1, code = 1;
        uint8_t total = len + 3;
        for(uint8_t i = 0; i < total; i++){
            uint8_t b = i == 0 ? type : i <= len ? data[i - 1] : (i == len + 1 ? (uint8_t)crc : (uint8_t)(crc >> 8));
            if(b == 0){
                out[codeIdx] = code;
                codeIdx = o++;
                code = 1;
            }
            else{
                out[o++] = b;
                code++;
            }
        }
        out[codeIdx] = code;
        out[o++] = 0;
        return o;
    }

    /**
     * @return true when b completed a valid frame
     */
    bool feed(uint8_t b){
        if(b != 0){
            if(idx < sizeof(buf)) buf[idx++] = b;
            else overflow = true;
            return false;
        }
        uint8_t n = idx;
        idx = 0;
        if(overflow){
            overflow = false;
            errors++;
            return false;
        }
        if(n == 0) return false;            //empty frame between two delimiters
        //COBS decode in place, the decoded data is one byte shorter
        uint8_t o = 0, i = 0;
        while(i < n){
            uint8_t code = buf[i++];
            if(code == 0 || i + code - 1 > n){ errors++; return false; }
            for(uint8_t k = 1; k < code; k++) buf[o++] = buf[i++];
            if(code < 0xFF && i < n) buf[o++] = 0;
        }
        if(o < 3){ errors++; return false; }
        uint16_t crc = 0xFFFF;
        for(uint8_t k = 0; k < o - 2; k++) crc = espLinkCrc(crc, buf[k]);
        if(crc != espLinkGet16(buf + o - 2)){ errors++; return false; }
        type = buf[0];
        len = o - 3;
        return true;
    }
};

#endif
