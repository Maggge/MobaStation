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

ESP_AT_Wifi::ESP_AT_Wifi(HardwareSerial *serial, const char * hostname, const char * ssid, const char * pwd){
    ser = serial;
    _hostname = hostname;
    _ssid = ssid;
    _pwd = pwd;
    _APmode = false;
}

ESP_AT_Wifi::ESP_AT_Wifi(HardwareSerial *serial, const char * APname, const char * pwd){
    ser = serial;
    _hostname = APname;
    _pwd = pwd;
    _APmode = true;
}

bool ESP_AT_Wifi::begin(uint16_t port, const char *ip){
    _port = port;
    if(!WiFi.init(ser, 115200)) return false;

    if(_APmode){
        startAP(_ssid, _pwd);
        Serial.println("Accesspoint startet!");
        Serial.print("SSID: ");
        Serial.print(_ssid);
        Serial.print("  PWD: ");
        Serial.println(_pwd);
    }
    else{
        if(!connectToAP(_ssid, _pwd)){
            Serial.println("FAILED!");
            if(startAP(STANDARD_AP_SSID, STANDARD_AP_PWD)){
                Serial.println("Accesspoint startet!");
                Serial.print("SSID: ");
                Serial.print(STANDARD_AP_SSID);
                Serial.print("  PWD: ");
                Serial.println(STANDARD_AP_PWD);
            }
        }
        else{
            Serial.println("CONNECTED!");
            if(ip != NULL){
                if(setIP(ip)){
                    Serial.print("IP Address: ");
                    Serial.println(ip);
                }
            }
            if(_hostname != NULL){
                if(setHostname(_hostname)){
                    Serial.print("Hostname: ");
                    Serial.println(_hostname);
                }
            }
        }      
    } 
    
    Udp.begin(port);
    return true;
}

bool ESP_AT_Wifi::connectToAP(const char * ssid, const char * pwd){

    _ssid = ssid;
    _pwd = pwd;

    Serial.print("Connect to ");
    Serial.print(ssid);
    Serial.print("...");

    return WiFi.connectToAP(ssid, pwd); 
}

bool ESP_AT_Wifi::setIP(IPAddress ip){
    return WiFi.setIP(ip);
}

bool ESP_AT_Wifi::setIP(const char *ip){
    return WiFi.setIP(ip);
}

bool ESP_AT_Wifi::setHostname(const char *hostname){
    _hostname = hostname;
    return WiFi.setHostname(hostname);
}

bool ESP_AT_Wifi::startAP(const char * ssid, const char * pwd){
    Serial.println("Start AccessPoint!");
   _ssid = ssid;
   _pwd = pwd;
   WiFi.startAP(ssid, pwd);
   return true;
}

bool ESP_AT_Wifi::send(IPAddress client, uint16_t DataLen, uint16_t Header, uint8_t *dataString, boolean withXOR){
    uint8_t data[24]; 			//z21 send storage
	
	//--------------------------------------------        
	//XOR bestimmen:
	data[0] = DataLen & 0xFF;
	data[1] = DataLen >> 8;
	data[2] = Header & 0xFF;
	data[3] = Header >> 8;
	data[DataLen - 1] = 0;	//XOR

    for (byte i = 0; i < (DataLen-5+!withXOR); i++) { //Ohne Length und Header und XOR
        if (withXOR)
			data[DataLen-1] = data[DataLen-1] ^ *dataString;
		data[i+4] = *dataString;
        dataString++;
    }
    #ifdef DEBUG
    Serial.print("Send Packet to ");
    Serial.println(client);
    #endif
    //--------------------------------------------   
    if(!Udp.beginPacket(client, _port)){
        #ifdef DEBUG
        Serial.println("Send Packet failed");
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