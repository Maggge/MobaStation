/**
  MobaStation Wifi Esp AT

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
//#define DEBUG

#include "WiFi_ESP_AT.h"

//-------------------------- PUBLIC ---------------------------------

bool WiFiClass::init(HardwareSerial *ser, unsigned long baud){
    ser->begin(baud);
    return init(ser);
}

bool WiFiClass::init(Stream *ser){
    wifi = ser;
    //Check if there is something that understand AT-commands
    wifi->print("AT\r\n");
    if(!checkForOk(200)) {
        state = WL_NO_MODULE;
        return false;
    }
    state = WL_IDLE_STATUS;

    //Check Version info - only for debugging 
    wifi->print("AT+GMR\r\n");
    checkForOk(2000);

    //Checks if the module understand the new commands
    wifi->print("AT+CWJAP_CUR?\r\n");
    if(!checkForOk(2000)){
        oldCmd = true;
        while (wifi->available()){
            char ch = wifi->read();
            #ifdef DEBUG
            Serial.print(ch);
            #endif
        }
    }

    return true;
}

bool WiFiClass::setIP(IPAddress ip){
    if(state != WL_CONNECTED) return false;
    wifi->print("AT+CIPSTA_CUR=\"");
    wifi->print(ip[0]);
    wifi->print(".");
    wifi->print(ip[1]);
    wifi->print(".");
    wifi->print(ip[2]);
    wifi->print(".");
    wifi->print(ip[3]);
    wifi->print("\"\r\n");
    return checkForOk(2000);
}

bool WiFiClass::setIP(const char *ip){
    if(state != WL_CONNECTED) return false;
    wifi->print("AT+CIPSTA_CUR=\"");
    wifi->print(ip);
    wifi->print("\"\r\n");
    return checkForOk(2000);
}

bool WiFiClass::setHostname(const char *hostname){
    if(state != WL_CONNECTED) return false;
    //Set the hostname
    wifi->print("AT+CWHOSTNAME=\"");
    wifi->print(hostname);
    wifi->print("\"\r\n");
    return checkForOk(2000);
}

bool WiFiClass::connectToAP(const char * ssid, const char * pwd){
    //Set the wifi Mode
    wifi->print("AT+CWMODE=1\r\n");
    if(!checkForOk(1000)) return false;
    activeMode = Station_Mode;
    #ifdef DEBUG
    Serial.print("Connect to ");
    Serial.print(ssid);
    #endif

    bool ipOk = false;

    if(oldCmd){
        wifi->print("AT+CWJAP=\"");
        wifi->print(ssid);
        wifi->print("\",\"");
        wifi->print(pwd);
        wifi->print("\"\r\n");
        ipOk = checkForOk(WIFI_CONNECT_TIMEOUT);
    }
    else{      
        wifi->print("AT+CWJAP_CUR=\"");
        wifi->print(ssid);
        wifi->print("\",\"");
        wifi->print(pwd);
        wifi->print("\"\r\n");
        ipOk = checkForOk(WIFI_CONNECT_TIMEOUT);
    }

    if(ipOk){
        state = WL_CONNECTED;
        #ifdef DEBUG
        Serial.println("CONNECTED!");
        #endif
    }
    else{
        state = WL_CONNECT_FAILED;
        #ifdef DEBUG
        Serial.println("FAILED!");
        #endif
    }

    return ipOk;
}

bool WiFiClass::startAP(const char * ssid, const char * pwd){
    //Set the wifi Mode
    wifi->print("AT+CWMODE=2\r\n");
    if(!checkForOk(1000)) return false;
    activeMode = SoftAP_Mode;
    #ifdef DEBUG
    Serial.print("Start Access Point...");
    #endif
    wifi->print("AT+CWSAP_CUR=\"");
    wifi->print(ssid);
    wifi->print("\",\"");
    wifi->print(pwd);
    wifi->print("\",1,4,8,0\r\n");
    if(checkForOk(5000)){
        #ifdef DEBUG
        Serial.println("OK!");
        Serial.print("SSID: ");
        Serial.println(ssid);
        Serial.print("PWD: ");
        Serial.println(pwd);
        #endif
        wifi->print("AT+CIPAP_CUR=\"192.168.0.111\",\"192.168.0.111\",\"255.255.255.0\"\r\n");
        if(checkForOk(5000)){
            wifi->print("AT+CWDHCPS_CUR=1,3,\"192.168.0.10\",\"192.168.0.25\"\r\n");
            if(checkForOk(5000)){
                state = WL_AP_LISTENING;
                return true;
            }
        }
    }
    
    #ifdef DEBUG
    Serial.println("FAILED!");
    #endif
    state = WL_AP_FAILED;
    return false;
    
}

WiFiMode WiFiClass::getMode(){
    return activeMode;
}

uint8_t WiFiClass::connect(CONN_TYPE type, const char *host, uint16_t port){
    if(state != WL_CONNECTED || state != WL_AP_LISTENING || state != WL_AP_CONNECTED)
    //Set the mode for multiple connections
    wifi->print("AT+CIPMUX=1\r\n");
    if(!checkForOk(2000)) return false;

    wifi->print("AT+CIPDINFO=1\r\n");
    if(!checkForOk(2000)) return false;

    uint8_t linkID = getFreeLinkID();
    if(linkID == 255) return linkID;
    char *t;
    
    switch (type) {
        case CONN_TYPE::UDP:
            t = "UDP";
            break;
        case TCP:
            t = "TCP";
            break;
        default:
            t = "SSL";
            break;
    }

    //Start the Connection
    wifi->print("AT+CIPSTART=");
    wifi->print(linkID);
    wifi->print(",\"");
    wifi->print(t);
    wifi->print("\",\"");
    wifi->print(host);
    wifi->print("\",");
    wifi->print(port);
    if(type == UDP){
        wifi->print(",");
        wifi->print(port);
        wifi->print(",2");
    }        
    wifi->print("\r\n");
       
    if(!checkForOk(2000)) return 255;

    linkIdOccupied[linkID] = true;

    return linkID;
}

uint8_t WiFiClass::startTcpServer(uint16_t port){
    wifi->print("AT+CIPSERVER=1,");
    wifi->print(port);
    wifi->print("\r\n");
    if(!checkForOk(2000)) return false;

    return true;
}

bool WiFiClass::stopTcpServer(){
    wifi->print("AT+CIPSERVER=0\r\n");
    if(!checkForOk(2000)) return false;

    return true;
}

void WiFiClass::loop(){
    if(!pendingCipsend && waitForSend){
        sendNext();
    }
    else if(pendingCipsend && sendTime + SEND_TIMEOUT <= millis()){
        #ifdef DEBUG
        Serial.println("Send timeout! Send next!");
        #endif
        pendingCipsend = false;
        sendIdx++;
        sendNext();
    }

    while (wifi->available()){
        switch (receiveState) {
            case STANDBY:{
                uint8_t ch = wifi->read();
                if (ch == '+') {
                    receiveState = IPD;
                    #ifdef DEBUG
                    Serial.println("---- Packet received ----");
                    #endif
                    break; 
                }
                else if(ch =='>'){
                    if(pendingCipsend){ 
                        txData();
                    }
                    receiveState=SKIPTOEND;
                }
                else if (ch=='R') { // Received ... bytes 
                    receiveState=SKIPTOEND;
                break;
                } 
                else if (ch=='S') { // SEND OK probably
                    if(pendingCipsend){
                        #ifdef DEBUG
                        if(checkForAnswer(200, F("END OK"))){
                            Serial.println("SEND OK!");
                        }
                        else{
                            Serial.println("SEND FAILED!");                       
                        }
                        #endif
                        pendingCipsend = false;
                    } 
                    receiveState=SKIPTOEND;
                    
                break;
                }  
                else if (ch=='b') {   // This is a busy indicator... probabaly must restart a CIPSEND  
                    pendingCipsend=true;
                    receiveState=SKIPTOEND; 
                break; 
                }        
                else if (ch >= '0' && ch <= '9') { 
                    receiveState=SKIPTOEND;
                    break;
                }
                else if(ch =='E' || ch =='l') { // ERROR or "link is not valid"
                    if (pendingCipsend) {
                        // A CIPSEND was errored... just toss it away
                        //sendNext();
                    }
                    receiveState=SKIPTOEND;
                }
                break;
            }
            case IPD:
                if(wifi->readStringUntil(',') == "IPD"){            
                    receiveState = LINK_ID;
                    #ifdef DEBUG
                    Serial.println("LINK ID");
                    #endif
                }
                else{
                    receiveState = SKIPTOEND;
                }
                break;
            case LINK_ID:
                recBuffer.linkId = wifi->parseInt();
                
                receiveState = LENGTH;
                #ifdef DEBUG
                Serial.print("linkID: ");
                Serial.println(recBuffer.linkId);
                #endif
                break; 
            case LENGTH:
                recBuffer.len = wifi->parseInt();
                #ifdef DEBUG
                Serial.print("Length: ");
                Serial.println(recBuffer.len);
                #endif
                if(wifi->read() == ','){
                    receiveState = IP;
                }
                else{
                    receiveState = SKIPTOEND;
                    #ifdef DEBUG
                    Serial.println("Parse ERROR, skip this message");
                    #endif
                }
                break;
            case IP:
                recBuffer.ip[0] = wifi->parseInt();
                recBuffer.ip[1] = wifi->parseInt();
                recBuffer.ip[2] = wifi->parseInt();
                recBuffer.ip[3] = wifi->parseInt();
                #ifdef DEBUG
                Serial.print("IP: ");
                Serial.println(recBuffer.ip);
                #endif
                receiveState = PORT;
                break;
            case PORT:
                recBuffer.port = wifi->parseInt();
                #ifdef DEBUG
                Serial.print("PORT: ");
                Serial.println(recBuffer.port);
                #endif
                if(wifi->read() == ':'){
                    receiveState = DATA;
                }
                else{
                    receiveState = SKIPTOEND;
                    #ifdef DEBUG
                    Serial.println("Parse ERROR, skip this message");
                    #endif
                }       
                break;
            case DATA:
                recBuffer.data[readDataIdx] = wifi->read();
                #ifdef DEBUG
                Serial.print("0x");
                Serial.print(recBuffer.data[readDataIdx], HEX);
                Serial.print("; ");
                #endif
                readDataIdx++;
                if(readDataIdx >= recBuffer.len){
                    readDataIdx = 0;
                    pendingReceive = true;
                    receiveState = BUFFER;
                    #ifdef DEBUG
                    Serial.println("\nReceive pending!");
                    #endif
                }
                break;
            case BUFFER:
                if(!pendingReceive){
                    receiveState = STANDBY;
                    break;
                }
                return;
            case SKIPTOEND:
                if(wifi->read() == '\n'){
                    receiveState = STANDBY;
                }
                break;
        }    
    }
    return;
}

bool WiFiClass::disconnect(uint8_t linkID){
    wifi->print("AT+CIPCLOSE=");
    wifi->println(linkID);
    if(!checkForOk(2000)) return false;
    linkIdOccupied[linkID] = false;
    return true;
}

bool WiFiClass::packetAvailable(uint8_t linkId){
    if(pendingReceive && linkId == recBuffer.linkId){
        #ifdef DEBUG
        Serial.print("Packet available!");
        #endif
        return true;
    }
    return false;
}

bool WiFiClass::getPacket(uint8_t linkId, WIFI_PACKET *pkg){
    if(!pendingReceive || linkId != recBuffer.linkId) return false;
    #ifdef DEBUG
    Serial.print("get Packet linkID: ");
    Serial.println(linkId);
    #endif
    pkg->ip = recBuffer.ip;
    pkg->linkId = recBuffer.linkId;
    pkg->len = recBuffer.len;
    pkg->ip = recBuffer.ip;
    pkg->port = recBuffer.port;
    for(uint16_t i = 0; i < pkg->len; i++){
        pkg->data[i] = recBuffer.data[i];
    }
    pendingReceive = false;
    #ifdef DEBUG
    Serial.print("PAcket received from ");
    Serial.println(pkg->ip);
    #endif
    return true;
}

bool WiFiClass::sendPkg(WIFI_PACKET *pkg){
    #ifdef DEBUG
    Serial.print("Send Packet to ");
    Serial.println(pkg->ip);
    #endif
    if(waitForSend && tail == sendIdx) return false;
    sendBuffer[tail].linkId = pkg->linkId;
    sendBuffer[tail].ip = pkg->ip;
    sendBuffer[tail].port = pkg->port;
    sendBuffer[tail].len = pkg->len;
    for(int i = 0; i < pkg->len; i++){
        sendBuffer[tail].data[i] = pkg->data[i];
    }
    tail++;
    if(tail >= SEND_BUF_SIZE){
        tail = 0;
    }
    waitForSend = true;
    return true;
}

//-------------------------- PRIVATE ---------------------------------

void WiFiClass::sendNext(){
    if(!waitForSend || pendingCipsend) return;
    #ifdef DEBUG
    Serial.println("----- SEND -----");
    Serial.print("Client IP: ");
    Serial.println(sendBuffer[sendIdx].ip);
    #endif
    wifi->print("AT+CIPSEND=");
    wifi->print(sendBuffer[sendIdx].linkId);
    wifi->print(",");
    wifi->print(sendBuffer[sendIdx].len);
    wifi->print(",\"");
    wifi->print(sendBuffer[sendIdx].ip[0]);
    wifi->print(".");
    wifi->print(sendBuffer[sendIdx].ip[1]);
    wifi->print(".");
    wifi->print(sendBuffer[sendIdx].ip[2]);
    wifi->print(".");
    wifi->print(sendBuffer[sendIdx].ip[3]);
    wifi->print("\",");
    wifi->print(sendBuffer[sendIdx].port);
    wifi->print("\r\n");

    sendTime = millis();
    pendingCipsend = true;
}

void WiFiClass::txData(){
    if(!pendingCipsend) return;
    for(uint16_t i = 0; i  < sendBuffer[sendIdx].len; i++){
        wifi->write(sendBuffer[sendIdx].data[i]);
    }
    sendIdx++;
    if(sendIdx >= SEND_BUF_SIZE){
        sendIdx = 0;
    }
    if(sendIdx == tail){
        waitForSend = false;
    }
    return;
}

uint8_t WiFiClass::getFreeLinkID(){
    for(int i = 0; i < 5; i++){
        if(!linkIdOccupied[i]){
            return i;
        }
    }
    return 255;
}

bool WiFiClass::checkForAnswer( const unsigned int timeout, const FSH * waitfor) {
    unsigned long  startTime = millis();
    char *locator = (char *)waitfor;

    #ifdef DEBUG
    Serial.print("wifi Check: [");
    Serial.print(waitfor);
    Serial.println("]");
    #endif
    while ( millis() - startTime < timeout) {
        while (wifi->available()) {
        int ch = wifi->read();
        #ifdef DEBUG
        Serial.print((char)ch);
        #endif
        if (ch != GETFLASH(locator)) locator = (char *)waitfor;
        if (ch == GETFLASH(locator)) {
            locator++;
            if (!GETFLASH(locator)) {
            #ifdef DEBUG
            Serial.print("FOUND after");
            Serial.print(millis() - startTime);
            Serial.println(" ms");
            #endif
            return true;
            }
        }
        }
    }
    #ifdef DEBUG
    Serial.print("TIMEOUT after");
    Serial.print(timeout);
    Serial.println(" ms");
    #endif
    return false;
}

bool WiFiClass::checkForOk(const unsigned int timeout){
    return checkForAnswer(timeout, F("\r\nOK\r\n"));
}

WiFiClass WiFi;