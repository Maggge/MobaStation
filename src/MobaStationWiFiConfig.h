/**
  MobaStation WiFi settings

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

//WiFi settings of the example sketches, used by ESP_AT_Wifi (AT firmware) and
//ESP_Link (MobaStation firmware on the ESP). Set them in MobaStationConfig.h.
#ifndef __MOBASTATION_WIFI_CONFIG__
#define __MOBASTATION_WIFI_CONFIG__

#include "MobaStationUserConfig.h"
#include "WiFi.h"           //WiFiMode

//WiFi of the example sketch
//accesspoint (router) the station connects to, empty = the one stored in the ESP
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PWD
#define WIFI_PWD ""
#endif
//Station_Mode, SoftAP_Mode or Hybrid_MODE (station, own accesspoint as fallback)
#ifndef WIFI_MODE
#define WIFI_MODE Hybrid_MODE
#endif
//serial port of the ESP8266, Serial3 on the Mega WiFi R3 boards
#ifndef WIFI_SERIAL
#define WIFI_SERIAL Serial3
#endif

//accesspoint of the SoftAP mode and of the hybrid fallback,
//set your own password in MobaStationConfig.h, everybody knows this one
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "MobaStation"
#endif
#ifndef WIFI_AP_PWD
#define WIFI_AP_PWD "MobaStation"
#endif

#endif
