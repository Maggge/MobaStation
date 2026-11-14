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

bool Eth_Adapter::begin(uint16_t port, const NetworkConfig &net){
    this->port = port;
    this->net = net;
    gateway = gatewayOf(net);

    Ethernet.init(cs);

    return initStack();
}

bool Eth_Adapter::initStack(){
    dhcpActive = false;
    if(net.dhcp){
        Serial.print(F("Ethernet DHCP..."));
        //returns 0 without hardware, without link or when no server answers
        if(Ethernet.begin(mac, ETH_DHCP_TIMEOUT, 4000) == 1){
            dhcpActive = true;
            Serial.println(F(" OK"));
        }
        else{
            Serial.println(F(" failed, using the static address"));
        }
    }
    if(!dhcpActive){
        //Attention: the order of the arguments is mac, ip, dns, gateway, subnetMask
        Ethernet.begin(mac, net.ip, gateway, gateway, net.subnet);
    }

    if(Ethernet.hardwareStatus() == EthernetNoHardware){
        Serial.println(F("Ethernet shield was not found."));
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
    //always printed, with DHCP this is the address the Z21 clients need
    Serial.print(F("Ethernet IP: "));
    Serial.println(Ethernet.localIP());
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

    if(dhcpActive){
        //renews the lease in time, only talks to the server when it is due
        Ethernet.maintain();
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