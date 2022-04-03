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

#include "Eth_Adapter.h"

//#define DEBUG

Eth_Adapter::Eth_Adapter(uint8_t cs, byte * mac){
    this->cs = cs;
    this->mac = mac;
}

bool Eth_Adapter::begin(uint16_t port,  IPAddress ip, IPAddress subnetMask){
    this->port = port;
    this->ip = ip;
    Ethernet.init(cs);

    Ethernet.begin(mac, this->ip);
    Ethernet.setSubnetMask(subnetMask);

    if(Ethernet.hardwareStatus() == EthernetNoHardware){
        #ifdef DEBUG
        Serial.println("Ethernet shield was not found.");
        #endif
        return false;
    }
    
    if(Ethernet.linkStatus() == LinkOFF){
        #ifdef DEBUG
        Serial.println("Ethernet cable is not connected.");
        #endif
    }

    Udp.begin(port);
    #ifdef DEBUG
    Serial.print("Ethernet connected IP: ");
    Serial.println(Ethernet.localIP());
    #endif
    return true;
}

bool Eth_Adapter::send(IPAddress client , uint8_t *data){
    #ifdef DEBUG
    Serial.print("Send Packet to ");
    Serial.println(client);
    #endif
    //--------------------------------------------   
    if(!Udp.beginPacket(client, port)){
        #ifdef DEBUG
        Serial.println("Send Packet failed");
        #endif
        return false;
    }
    Udp.write(data, data[0]);
    return Udp.endPacket();
}

bool Eth_Adapter::receivePacket(UdpPacket *pkg){
    int pkgSize = Udp.parsePacket();
    if(pkgSize) {
        pkg->ip = Udp.remoteIP();
        pkg->dataLength = pkgSize;
        Udp.read(pkg->data, Z21_MAX_UDP_SIZE);
        return true;
    }
    return false;
}