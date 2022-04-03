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
    IPD,
    LINK_ID,
    LENGTH,
    IP,
    PORT,
    DATA,
    BUFFER,
    SKIPTOEND,
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