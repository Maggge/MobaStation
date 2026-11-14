/**
  MobaStation

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

#ifndef __WIFI__
#define __WIFI__

#include <Arduino.h>
#include <IPAddress.h>

//#define DEBUG

enum WiFiMode{
    Station_Mode,
    SoftAP_Mode,
    Hybrid_MODE,
};

enum WL_STATE{
    WL_IDLE_STATUS = 0,
    WL_CONNECTED,
    WL_CONNECT_FAILED,
    WL_CONNECTION_LOST,
    WL_DISCONNECTED,
    WL_AP_LISTENING,
    WL_AP_CONNECTED,
    WL_AP_FAILED,
    WL_NO_MODULE = 255,
};

enum RECEIVE_STATE{
    STANDBY,
    IPD,            //the char after "+", decides between +IPD and +WIFI DISCONNECT
    LINK_ID,
    LENGTH,
    IP,
    PORT,
    DATA,
    BUFFER,
    SKIPTOEND,
    URC_LINE,       //unsolicited result code or AT response, read as text line
    SKIPDATA,       //echo of the bytes written after the CIPSEND prompt
};

//states of the non blocking wifi reconnect
enum RECONNECT_STATE{
    RECONNECT_IDLE = 0,     //no reconnect needed
    RECONNECT_LINK,         //the udp/tcp link must be opened again
    RECONNECT_JOIN,         //AT+CWJAP is running
    RECONNECT_IP,           //the static ip is set
    RECONNECT_HOSTNAME,     //the hostname is set
    RECONNECT_OPEN,         //the link is opened
    RECONNECT_CIPMUX,       //AT+CIPMUX=1
    RECONNECT_DINFO,        //AT+CIPDINFO=1
    RECONNECT_WAIT,         //wifi is down, waiting for the next attempt
    //hybrid mode: accesspoint next to the station while the station is not reachable
    RECONNECT_AP_MODE,      //AT+CWMODE_CUR=3
    RECONNECT_AP_CONFIG,    //AT+CWSAP_CUR
    RECONNECT_AP_IP,        //AT+CIPAP_CUR
    RECONNECT_AP_DHCP,      //AT+CWDHCPS_CUR
    RECONNECT_STA_ONLY,     //station is back, AT+CWMODE_CUR=1 switches the accesspoint off
};

enum CONN_TYPE{
    UDP,
    TCP,
    SSL,
};

#define PKG_SIZE 64

struct WIFI_PACKET{
    IPAddress ip;
    uint8_t linkId;
    uint16_t port;
    uint16_t len;
    uint8_t data[PKG_SIZE];
};
#endif