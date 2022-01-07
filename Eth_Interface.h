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

class Eth_Interface{
public:
    virtual bool begin(uint16_t port, const char *ip = NULL);
    virtual bool send(IPAddress client , uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR);
    virtual bool receivePacket(UdpPacket *pkg);     
};

#endif