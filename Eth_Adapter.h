/**
  MobaStation Ethernet Adapter

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

#ifndef __ETH_ADAPTER__
#define __ETH_ADAPTER__

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "Eth_Interface.h"


class Eth_Adapter : public Eth_Interface{
private:
    EthernetUDP Udp;
    char packetBuffer[Z21_MAX_UDP_SIZE];  // buffer to hold incoming packet

    uint8_t cs;
    byte *mac;

    IPAddress ip;

    uint16_t port;


public:
    /**
     * @brief Construct a new Eth_Adapter object
     * 
     * @param cs 
     * @param mac 
     * @param backup interface, maybe Wifi. If the link is down on startup the backup Interface will be initialized
     */
    Eth_Adapter(uint8_t cs, byte * mac);

    bool begin(uint16_t port, IPAddress ip, IPAddress subnetMask);
    bool send(IPAddress client , uint8_t *data);
    bool receivePacket(UdpPacket *pkg);    

};


#endif