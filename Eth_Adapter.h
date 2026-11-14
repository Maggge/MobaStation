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
//compiled without MobaStation.h, ETH_DHCP_TIMEOUT may come from the user configuration
#include "MobaStationUserConfig.h"

//W5x00 of the example sketch: chip select pin and MAC address (must be unique in the network)
#ifndef ETH_CS_PIN
#define ETH_CS_PIN 49
#endif
#ifndef ETH_MAC
#define ETH_MAC 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
#endif

//how long the DHCP request may take before the static address is used,
//it blocks: at the start and when the cable is plugged in again
#ifndef ETH_DHCP_TIMEOUT
#define ETH_DHCP_TIMEOUT 10000
#endif

class Eth_Adapter : public Eth_Interface{
private:
    EthernetUDP Udp;

    uint8_t cs;
    byte *mac;

    NetworkConfig net;
    IPAddress gateway;
    bool dhcpActive = false;    //the address came from DHCP, the lease must be renewed

    uint16_t port;

    bool linkOK = false;
    uint32_t lastLinkCheck = 0;

    //bring the W5x00 and the UDP socket back up after the link was lost
    bool initStack();

public:
    /**
     * @brief Construct a new Eth_Adapter object
     * 
     * @param cs chip select pin of the W5x00
     * @param mac MAC address, must be unique in the network
     */
    Eth_Adapter(uint8_t cs, byte * mac);

    bool begin(uint16_t port, const NetworkConfig &net);
    bool send(IPAddress client, uint16_t dstPort, uint8_t *data);
    bool receivePacket(UdpPacket *pkg);    

};


#endif