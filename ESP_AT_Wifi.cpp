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
    if(!WiFi.init(ser, 115200)){
        Serial.println(F("No WiFi module found"));
        return false;
    }
    
    if(_mode == Hybrid_MODE){
        Serial.println(F("Start Wifi in Hybrid mode"));
        if(!connectToAP(_ssid, _pwd)){
            Serial.println(F("FAILED!"));
            if(!startAP(STANDARD_AP_SSID, STANDARD_AP_PWD)){
                return false;
            }
            Serial.println(F("Accesspoint startet!"));
            Serial.print(F("SSID: "));
            Serial.print(STANDARD_AP_SSID);
            Serial.print(F("  PWD: "));
            Serial.println(STANDARD_AP_PWD);
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
        if(!startAP(STANDARD_AP_SSID, STANDARD_AP_PWD)){
            Serial.println(F("FAILED!"));
            return false;
        }
        Serial.println(F("Accesspoint startet!"));
        Serial.print(F("SSID: "));
        Serial.print(STANDARD_AP_SSID);
        Serial.print(F("  PWD: "));
        Serial.println(STANDARD_AP_PWD);
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

bool ESP_AT_Wifi::setIP(IPAddress ip){
    return WiFi.setIP(ip);
}

bool ESP_AT_Wifi::setIP(const char *ip){
    return WiFi.setIP(ip);
}

bool ESP_AT_Wifi::startAP(const char * ssid, const char * pwd){
    Serial.println(F("Start AccessPoint!"));
   _ssid = ssid;
   _pwd = pwd;
   if(WiFi.startAP(ssid, pwd)){
       Serial.println(F("Accesspoint startet!"));
        Serial.print(F("SSID: "));
        Serial.print(ssid);
        Serial.print(F("  PWD: "));
        Serial.println(pwd);
        return true;
   }
   return false;
}

bool ESP_AT_Wifi::send(IPAddress client, uint8_t *data){
    #ifdef DEBUG
    Serial.print(F("Send Packet to "));
    Serial.println(client);
    #endif
    //--------------------------------------------   
    if(!Udp.beginPacket(client, _port)){
        #ifdef DEBUG
        Serial.println(F("Send Packet failed"));
        #endif
        return false;
    }
    Udp.write(data, data[0]);
    return Udp.endPacket();
}

bool ESP_AT_Wifi::receivePacket(UdpPacket *pkg){
    WiFi.loop();
    int pkgSize = Udp.parsePacket();
    if(pkgSize > 0){
        pkg->ip = Udp.remoteIP();
        pkg->dataLength = pkgSize;
        Udp.read(pkg->data, Z21_MAX_UDP_SIZE);
        return true;
    }
    return false;
}
#endif