/**
  MobaStation Ethernet Interface

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

#ifndef __Eth_Interface__
#define __Eth_Interface__

#include <Arduino.h>
#include "MobaStation_Protocol.h"

struct NetworkConfig{
    bool dhcp;          //get the address from the router
    IPAddress ip;       //static address, with dhcp the fallback and the address of an own accesspoint
    IPAddress subnet;
    IPAddress gateway;  //0.0.0.0 = first address of the subnet
};

//the configured gateway, or the first address of the subnet when none is set
//(the Z21 LAN protocol needs no gateway, but the network stacks want one)
inline IPAddress gatewayOf(const NetworkConfig &net){
    if(!(net.gateway == IPAddress())) return net.gateway;
    return IPAddress(net.ip[0] & net.subnet[0], net.ip[1] & net.subnet[1],
                     net.ip[2] & net.subnet[2], (net.ip[3] & net.subnet[3]) | 1);
}

class Eth_Interface{
public:
    virtual ~Eth_Interface() {}

    virtual bool begin(uint16_t port, const NetworkConfig &net) = 0;
    //data is a complete Z21 record, data[0] is its length
    virtual bool send(IPAddress client, uint16_t port, uint8_t *data) = 0;
    //fills ip, port, dataLength and data, the caller sets iface
    virtual bool receivePacket(UdpPacket *pkg) = 0;
};

#endif