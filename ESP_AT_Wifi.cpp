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

#ifndef ESP32
#include "ESP_AT_Wifi.h"

ESP_AT_Wifi::ESP_AT_Wifi(HardwareSerial *serial, const char * ssid, const char * pwd, WiFiMode mode){
    ser = serial;
    _ssid = ssid;
    _pwd = pwd;
    _mode = mode;
}

bool ESP_AT_Wifi::begin(uint16_t port, IPAddress ip, IPAddress subnetMask){
    _port = port;
    _ip = ip;
    _subnet = subnetMask;
    if(!WiFi.init(ser, 115200)){
        Serial.println(F("No WiFi module found"));
        return false;
    }
    
    if(_mode == Hybrid_MODE){
        Serial.println(F("Start Wifi in Hybrid mode"));
        //the reconnect state machine starts this accesspoint later on its own
        //if the station gets lost and can not be reached again
        WiFi.enableFallbackAP(WIFI_AP_SSID, WIFI_AP_PWD, ip);
        if(!connectToAP(_ssid, _pwd)){
            Serial.println(F("FAILED!"));
            //accesspoint and station at the same time, the station is
            //searched in the background and replaces the accesspoint
            if(!startAP(WIFI_AP_SSID, WIFI_AP_PWD, true)){
                return false;
            }
            WiFi.startReconnect();
        }
        else{
            Serial.println(F("CONNECTED!"));
            if(setIP(ip)){
                Serial.print(F("IP Address: "));
                Serial.println(ip);
            }
            if(WiFi.setHostname("MobaStation")){
                Serial.println(F("Hostname: Mobastation"));
            }
        }
    }
    else if(_mode == Station_Mode){
        Serial.println(F("Start Wifi in Station mode"));
        if(!connectToAP(_ssid, _pwd)){
            Serial.println(F("FAILED!"));
            return false;
        }
        else{
            Serial.println(F("CONNECTED!"));
            if(setIP(ip)){
                Serial.print(F("IP Address: "));
                Serial.println(ip);
            }
            if(WiFi.setHostname("MobaStation")){
                Serial.println(F("Hostname: Mobastation"));
            }
        }
    }
    else if(_mode == SoftAP_Mode){
        Serial.println(F("Start Wifi in Accesspoint mode"));
        if(!startAP(WIFI_AP_SSID, WIFI_AP_PWD)){
            Serial.println(F("FAILED!"));
            return false;
        }
    }
    
    
    Udp.begin(port);
    Serial.print(F("MobaStation is listen on Port: "));
    Serial.println(port);
    return true;
}

bool ESP_AT_Wifi::connectToAP(const char * ssid, const char * pwd){

    _ssid = ssid;
    _pwd = pwd;

    Serial.print(F("Connect to "));
    Serial.print(ssid);
    Serial.print(F("..."));

    return WiFi.connectToAP(ssid, pwd); 
}

void ESP_AT_Wifi::reconnect(){
    WiFi.startReconnect();
}

bool ESP_AT_Wifi::setIP(IPAddress ip){
    return WiFi.setIP(ip);
}

bool ESP_AT_Wifi::setIP(const char *ip){
    return WiFi.setIP(ip);
}

bool ESP_AT_Wifi::startAP(const char * ssid, const char * pwd, bool keepStation){
    Serial.println(F("Start AccessPoint!"));
    //the accesspoint gets the ip of the station, so the clients need no other settings
    if(WiFi.startAP(ssid, pwd, _ip, keepStation)){
        printAP(ssid, pwd);
        return true;
    }
    return false;
}

bool ESP_AT_Wifi::send(IPAddress client, uint16_t dstPort, uint8_t *data){
    #ifdef DEBUG
    Serial.print(F("Send Packet to "));
    Serial.println(client);
    #endif
    //--------------------------------------------
    if(!Udp.beginPacket(client, dstPort)){
        #ifdef DEBUG
        Serial.println(F("Send Packet failed"));
        #endif
        return false;
    }
    Udp.write(data, data[0]);
    return Udp.endPacket();
}

//-------------------------- PRIVATE ---------------------------------

void ESP_AT_Wifi::printAP(const char * ssid, const char * pwd){
    Serial.println(F("Accesspoint startet!"));
    Serial.print(F("SSID: "));
    Serial.print(ssid);
    Serial.print(F("  PWD: "));
    Serial.println(pwd);
}

bool ESP_AT_Wifi::receivePacket(UdpPacket *pkg){
    //runs the receive parser, the send queue and the non blocking reconnect
    //including the accesspoint fallback of the hybrid mode
    WiFi.loop();
    int pkgSize = Udp.parsePacket();
    if(pkgSize > 0){
        if(pkgSize > Z21_MAX_UDP_SIZE){
            #ifdef DEBUG
            Serial.println(F("Received packet too long, truncated"));
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

#endif
