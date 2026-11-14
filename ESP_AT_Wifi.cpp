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
        if(!connectToAP(_ssid, _pwd)){
            Serial.println(F("FAILED!"));
            //keep trying to reach the accesspoint in the background
            WiFi.startReconnect();
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

void ESP_AT_Wifi::reconnect(){
    if(!WiFi.needsReconnect()){
        return;
    }
    if(_mode == Hybrid_MODE){
        Serial.println(F("Wifi reconnect failed, starting the accesspoint"));
        reconnectFailures = 0;
        WiFi.startAP(STANDARD_AP_SSID, STANDARD_AP_PWD);
    }
    else{
        WiFi.startReconnect();
    }
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

//-------------------------- PRIVATE ---------------------------------

void ESP_AT_Wifi::handleReconnect(){
    RECONNECT_STATE rs = WiFi.getReconnectState();
    if(rs == RECONNECT_IDLE){
        if(reconnectFailures != 0){
            reconnectFailures = 0;
        }
        return;
    }
    //the reconnect itself runs non blocking inside WiFi.loop()
    if(WiFi.getStatus() == WL_CONNECTED){
        reconnectFailures = 0;
    }
    if(rs != RECONNECT_WAIT) return;
    if(!WiFi.needsReconnect()){
        //no station connection that must be kept alive,
        //eg. the accesspoint fallback is already running
        return;
    }

    //all attempts of this wifi outage failed
    if(_mode != SoftAP_Mode && reconnectFailures < 255){
        reconnectFailures++;
    }
    if(_mode == Hybrid_MODE && reconnectFailures >= 3 && reconnectFailures % 3 == 0){
        Serial.println(F("Wifi reconnect failed, starting the accesspoint"));
        WiFi.startAP(STANDARD_AP_SSID, STANDARD_AP_PWD);
    }
}

bool ESP_AT_Wifi::receivePacket(UdpPacket *pkg){
    WiFi.loop();
    handleReconnect();
    int pkgSize = Udp.parsePacket();
    if(pkgSize > 0){
        if(pkgSize > Z21_MAX_UDP_SIZE){
            #ifdef DEBUG
            Serial.println(F("Received packet too long, truncated"));
            #endif
            pkgSize = Z21_MAX_UDP_SIZE;
        }
        pkg->ip = Udp.remoteIP();
        pkg->dataLength = pkgSize;
        Udp.read(pkg->data, pkgSize);
        return true;
    }
    return false;
}

#endif
