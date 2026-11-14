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

#define LINK_CHECK_INTERVAL 2000  //check the link state every 2 seconds

Eth_Adapter::Eth_Adapter(uint8_t cs, byte * mac){
    this->cs = cs;
    this->mac = mac;
}

bool Eth_Adapter::begin(uint16_t port,  IPAddress ip, IPAddress subnetMask){
    this->port = port;
    this->ip = ip;
    this->subnetMask = subnetMask;
    //no gateway/DNS needed for the Z21 LAN protocol, use the first address of the subnet
    this->gateway = IPAddress(ip[0], ip[1], ip[2], 1);

    Ethernet.init(cs);

    return initStack();
}

bool Eth_Adapter::initStack(){
    //Attention: the order of the arguments is mac, ip, dns, gateway, subnetMask
    Ethernet.begin(mac, ip, gateway, gateway, subnetMask);

    if(Ethernet.hardwareStatus() == EthernetNoHardware){
        #ifdef DEBUG
        Serial.println("Ethernet shield was not found.");
        #endif
        return false;
    }

    linkOK = Ethernet.linkStatus() == LinkON;
    if(!linkOK){
        #ifdef DEBUG
        Serial.println("Ethernet cable is not connected.");
        #endif
    }

    Udp.stop();
    Udp.begin(port);
    #ifdef DEBUG
    Serial.print("Ethernet connected IP: ");
    Serial.println(Ethernet.localIP());
    #endif
    return true;
}

bool Eth_Adapter::send(IPAddress client, uint16_t dstPort, uint8_t *data){
    #ifdef DEBUG
    Serial.print("Send Packet to ");
    Serial.println(client);
    #endif
    //--------------------------------------------
    if(!Udp.beginPacket(client, dstPort)){
        #ifdef DEBUG
        Serial.println("Send Packet failed");
        #endif
        return false;
    }
    Udp.write(data, data[0]);
    return Udp.endPacket();
}

bool Eth_Adapter::receivePacket(UdpPacket *pkg){
    //recover the connection if the cable was unplugged or the router was restarted
    uint32_t now = millis();
    if(now - lastLinkCheck > LINK_CHECK_INTERVAL){
        lastLinkCheck = now;
        bool link = Ethernet.linkStatus() == LinkON;
        if(!link){
            if(linkOK){
                linkOK = false;
                #ifdef DEBUG
                Serial.println("Ethernet link lost.");
                #endif
            }
        }
        else if(!linkOK){
            #ifdef DEBUG
            Serial.println("Ethernet link back up, reinitialize.");
            #endif
            initStack();
        }
    }

    int pkgSize = Udp.parsePacket();
    if(pkgSize) {
        if(pkgSize > Z21_MAX_UDP_SIZE){
            #ifdef DEBUG
            Serial.println("Received packet too long, truncated");
            #endif
            pkgSize = Z21_MAX_UDP_SIZE;
        }
        pkg->ip = Udp.remoteIP();
        pkg->port = Udp.remotePort();
        pkg->dataLength = pkgSize;
        Udp.read(pkg->data, pkgSize);
        return true;
    }
    return false;
}