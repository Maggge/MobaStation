/**
  MobaStation Wifi Udp

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

#include "WiFiUdp.h"

uint8_t WiFiUDP::begin(uint16_t port){
    return beginMulticast(IPAddress(0,0,0,0), port);
}

uint8_t WiFiUDP::beginMulticast(IPAddress ip, uint16_t port){  
    char host[16];
    sprintf(host, "%d.%d.%d.%d\0", ip[0],ip[1],ip[2],ip[3]);
    link_ID = WiFi.connect(CONN_TYPE::UDP, host, port);
    #ifdef DEBUG
    Serial.print("Link id: ");
    Serial.println(link_ID);
    #endif
    if(link_ID == 255) return 0;
    return 1;
}

void WiFiUDP::stop(){
    if(link_ID == 255) return;
    WiFi.disconnect(link_ID);
}

int WiFiUDP::parsePacket(){
    if(WiFi.packetAvailable(link_ID)){
        recPending = WiFi.getPacket(link_ID, &pkgBuffer);
        recIdx = 0;
        if(recPending){
            return pkgBuffer.len;
        }
    }
    return 0;
}

int WiFiUDP::available(){
    if(!recPending) return 0;
    return pkgBuffer.len - recIdx;
}

int WiFiUDP::read(){
    if(!recPending) return 0;
    int d = pkgBuffer.data[recIdx];
    recIdx++;
    if(recIdx == pkgBuffer.len){
        recPending = false;
    }
    return d;
}

int WiFiUDP::read(unsigned char* buffer, size_t len){
    if(!recPending) return 0;
    size_t l = (len < pkgBuffer.len) ? len : pkgBuffer.len;
    for(size_t i = 0; i < l; i++){
        buffer[i] = read();
    }
    return l;
}

int WiFiUDP::read(char* buffer, size_t len){
    if(!recPending) return 0;
    size_t l = (len < pkgBuffer.len) ? len : pkgBuffer.len;
    for(size_t i = 0; i < l; i++){
        buffer[i] = read();
    }
    return l;
}

int WiFiUDP::peek(){
    if(!recPending) return 0;
    return pkgBuffer.data[recIdx];
}

void WiFiUDP::flush(){
    recPending = false;
}

IPAddress WiFiUDP::remoteIP(){
    if(!recPending) return IPAddress();
    return pkgBuffer.ip;
}

uint16_t WiFiUDP::remotePort(){
    if(!recPending) return 0;
    return pkgBuffer.port;
}

int WiFiUDP::beginPacket(IPAddress ip, uint16_t port){
    if(pkgBegin) return 0;
    sendBuffer.ip = ip;
    sendBuffer.port = port;
    sendBuffer.len = 0;
    pkgBegin = true;
    return 1;
}

int WiFiUDP::beginPacket(const char *host, uint16_t port){
    if(pkgBegin) return 0;
    sendBuffer.ip.fromString(host);
    sendBuffer.port = port;
    sendBuffer.len = 0;
    pkgBegin = true;
    return 1;
}

int WiFiUDP::endPacket(){
    if(!pkgBegin) return 0;
    sendBuffer.linkId = link_ID;
    pkgBegin = false;
    return WiFi.sendPkg(&sendBuffer);
}

size_t WiFiUDP::write(uint8_t d){
    if(!pkgBegin) return 0;
    sendBuffer.data[sendBuffer.len] = d;
    sendBuffer.len++;
}

size_t WiFiUDP::write(const uint8_t *buffer, size_t size){
    if(!pkgBegin) return 0;
    size_t l = size > PKG_SIZE ? PKG_SIZE : size;
    for(size_t i = 0; i < l; i++){
        write(buffer[i]);
    }
    return l;
}