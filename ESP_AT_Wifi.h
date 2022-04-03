/**
  MobaStation ESP Interface

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

#ifndef __ESP_AT__
#define __ESP_AT__

#include <Arduino.h>
#include "Eth_Interface.h"
#include "WiFiUdp.h"

#define STANDARD_AP_SSID "MobaStation"
#define STANDARD_AP_PWD "MobaStation"

class ESP_AT_Wifi : public Eth_Interface {
private:
    HardwareSerial *ser;

    WiFiUDP Udp;

    WiFiMode _mode;

    const char * _ssid;
    const char * _pwd;
    uint16_t _port;
    
public:
    ESP_AT_Wifi(HardwareSerial *serial, const char *ssid, const char *pwd, WiFiMode mode = Hybrid_MODE);

    bool begin(uint16_t port, IPAddress ip, IPAddress subnetMask);
    bool send(IPAddress client, uint8_t *data);
    bool receivePacket(UdpPacket *pkg);

    bool setIP(IPAddress ip);
    bool setIP(const char *ip);
    bool connectToAP(const char * ssid, const char * pwd);
    bool startAP(const char * ssid, const char * pwd);
    
};

#endif