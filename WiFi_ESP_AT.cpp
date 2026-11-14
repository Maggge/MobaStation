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

    //switch the echo off, otherwise the module mirrors the binary payload written
    //after the CIPSEND prompt and the receive parser has to skip it blindly
    wifi->print("ATE0\r\n");
    echo = !checkForOk(500);

    //Check Version info - only for debugging 
    wifi->print("AT+GMR\r\n");
    checkForOk(2000);

    //Checks if the module understand the new commands
    wifi->print("AT+CWJAP_CUR?\r\n");
    if(!checkForOk(2000)){
        oldCmd = true;
        while (wifi->available()){
            //drop the answer, the variable is only interesting for debugging
            wifi->read();
        }
    }

    return true;
}

bool WiFiClass::setIP(IPAddress ip){
    staticIpSet = true;
    staticIp[0] = ip[0];
    staticIp[1] = ip[1];
    staticIp[2] = ip[2];
    staticIp[3] = ip[3];
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
    uint8_t oct[4] = {0};
    uint8_t part = 0;
    uint16_t val = 0;
    bool valid = true;
    const char *p = ip;
    while(*p && valid){
        if(*p >= '0' && *p <= '9'){
            val = val * 10 + (*p - '0');
            if(val > 255){
                valid = false;
            }
        }
        else if(*p == '.'){
            if(part >= 3){
                valid = false;
                break;
            }
            oct[part++] = (uint8_t)val;
            val = 0;
        }
        else{
            valid = false;
        }
        p++;
    }
    if(valid && part == 3){
        oct[part++] = (uint8_t)val;
    }
    else{
        valid = false;
    }
    if(valid){
        //remembered for the reconnect, the module forgets the static ip
        staticIpSet = true;
        staticIp[0] = oct[0];
        staticIp[1] = oct[1];
        staticIp[2] = oct[2];
        staticIp[3] = oct[3];
    }
    wifi->print("AT+CIPSTA_CUR=\"");
    wifi->print(ip);
    wifi->print("\"\r\n");
    return checkForOk(2000);
}

bool WiFiClass::setHostname(const char *hostname){
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
    stationMode = true;

    //remember the credentials for the reconnect
    strncpy(this->ssid, ssid, WIFI_SSID_MAX_LEN);
    this->ssid[WIFI_SSID_MAX_LEN] = '\0';
    strncpy(this->pwd, pwd, WIFI_PWD_MAX_LEN);
    this->pwd[WIFI_PWD_MAX_LEN] = '\0';

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
    //an accesspoint does not reconnect to another accesspoint
    stationMode = false;
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

WL_STATE WiFiClass::getStatus(){
    return state;
}

uint8_t WiFiClass::connect(CONN_TYPE type, const char *host, uint16_t port){
    //only possible if the module is connected as station or runs an accesspoint
    if(!canSend()) return 255;

    //Set the mode for multiple connections
    wifi->print("AT+CIPMUX=1\r\n");
    if(!checkForOk(2000)) return 255;

    wifi->print("AT+CIPDINFO=1\r\n");
    if(!checkForOk(2000)) return 255;

    uint8_t linkID = getFreeLinkID();
    if(linkID == 255) return linkID;
    const char *t;
    
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

    //remember the link, the reconnect opens it again after a wifi outage
    linkId = linkID;
    strncpy(linkHost, host, sizeof(linkHost) - 1);
    linkHost[sizeof(linkHost) - 1] = '\0';
    linkPort = port;

    return linkID;
}

uint8_t WiFiClass::startTcpServer(uint16_t port){
    if(!canSend()) return 255;

    tcpServerPort = port;

    wifi->print("AT+CIPSERVER=1,");
    wifi->print(port);
    wifi->print("\r\n");
    if(!checkForOk(2000)) return 255;

    return 1;
}

bool WiFiClass::stopTcpServer(){
    tcpServerPort = 0;
    wifi->print("AT+CIPSERVER=0\r\n");
    if(!checkForOk(2000)) return false;

    return true;
}

void WiFiClass::loop(){
    if(wifi == NULL) return;

    uint32_t now = millis();

    if(reconnectState != RECONNECT_IDLE){
        reconnectStateMachine(now);
    }
    if(state == WL_CONNECTED){
        joinRetries = 0;
    }

    if(!pendingCipsend && waitForSend && canSend()){
        sendNext();
    }
    else if(pendingCipsend && now - sendTime >= SEND_TIMEOUT){
        #ifdef DEBUG
        Serial.println("Send timeout! Send next!");
        #endif
        //no prompt or no SEND OK, drop this packet and continue with the next one
        finishSend();
        if(waitForSend && canSend()){
            sendNext();
        }
    }
    else if(waitForSend && !pendingCipsend && !canSend()){
        //nothing can be sent while the wifi is down
        clearSendQueue();
    }

    while (wifi->available()){
        switch (receiveState) {
            case STANDBY:{
                int ch = wifi->read();
                if(ch < 0) break;
                if (ch == '+'){
                    receiveState = IPD;     //+IPD... or +WIFI DISCONNECT
                    break;
                }
                else if(ch =='>'){
                    if(pendingCipsend && !promptSeen){
                        uint16_t sent = sendBuffer[sendIdx].len;
                        txData();
                        promptSeen = true;
                        if(echo){
                            //the module echoes the bytes we have just written, skip them
                            dropBytes = sent;
                            receiveState = SKIPDATA;
                        }
                        //without echo "Recv x bytes" and "SEND OK" follow as normal lines,
                        //the blank after the prompt is ignored in STANDBY
                    }
                    else{
                        receiveState = SKIPTOEND;
                    }
                }
                else if (ch=='R') { // Received ... bytes
                    receiveState=SKIPTOEND;
                break;
                }
                else if (ch=='S') { // SEND OK or SEND FAIL
                    //only valid after the data of the pending packet was written,
                    //a late answer to a timed out packet must not finish the next one
                    if(pendingCipsend && promptSeen){
                        #ifdef DEBUG
                        if(checkForAnswer(200, F("END OK"))){
                            Serial.println("SEND OK!");
                        }
                        else{
                            Serial.println("SEND FAILED!");
                        }
                        #endif
                        finishSend();
                    }
                    receiveState=SKIPTOEND;

                break;
                }
                else if (ch=='b') {   // "busy p..." / "busy s...": the module rejected the command
                    //a rejected CIPSEND never gets a prompt, the send timeout drops the packet.
                    //don't mark a send as pending here, there might be none
                    receiveState=SKIPTOEND;
                break;
                }
                else if ((ch >= '0' && ch <= '9') || ch == 'C' || ch == 'c' || ch == 'W' || ch == 'w'
                         || ch == 'O' || ch == 'o' || ch == 'E' || ch == 'e' || ch == 'F' || ch == 'f'
                         || ch == 'A' || ch == 'a' || ch == 'l'){
                    //unsolicited result code or the response of an AT command
                    lineIdx = 0;
                    line[lineIdx++] = (char)ch;
                    receiveState=URC_LINE;
                }
                else if(ch =='\r' || ch =='\n' || ch == ' '){
                    //empty line or the blank after the CIPSEND prompt
                }
                else {  //echo of a command or something we do not understand
                    receiveState=SKIPTOEND;
                }
                break;
            }
            case IPD:{
                //after the '+': "IPD," or "WIFI DISCONNECT"
                int ch = wifi->read();
                if(ch < 0) break;
                if(ch == 'I'){
                    matchIdx = 1;             //I of "IPD"
                }
                else if(ch == 'P' && matchIdx == 1){
                    matchIdx = 2;
                }
                else if(ch == 'D' && matchIdx == 2){
                    matchIdx = 3;
                }
                else if(ch == ',' && matchIdx == 3){
                    matchIdx = 0;
                    recBuffer.linkId = 0;
                    parseValue = 0;
                    receiveState = LINK_ID;
                    #ifdef DEBUG
                    Serial.println("---- Packet received ----");
                    #endif
                }
                else if(ch == 'W' || ch == 'O' || ch == 'C'){
                    //+WIFI DISCONNECT, +CLOSED, +CWJAP...
                    matchIdx = 0;
                    lineIdx = 0;
                    line[lineIdx++] = '+';
                    line[lineIdx++] = (char)ch;
                    line[lineIdx] = '\0';
                    receiveState = URC_LINE;
                }
                else{
                    matchIdx = 0;
                    receiveState = SKIPTOEND;
                }
                break;
            }
            case LINK_ID:{
                //+IPD,<link id>,<length>,<remote ip>,<remote port>:<data>
                int ch = wifi->read();
                if(ch < 0) break;
                if(ch >= '0' && ch <= '9'){
                    recBuffer.linkId = (uint8_t)(recBuffer.linkId * 10 + (ch - '0'));
                    break;
                }
                if(ch == ',' && recBuffer.linkId < WIFI_LINKS){
                    receiveState = LENGTH;
                }
                else{
                    receiveState = SKIPTOEND;
                }
                break;
            }
            case LENGTH:{
                int ch = wifi->read();
                if(ch < 0) break;
                if(ch >= '0' && ch <= '9'){
                    if(parseValue > 0xffff / 10)
                        parseValue = 0xffff;        //cannot be a valid length anymore
                    else
                        parseValue = (uint16_t)(parseValue * 10 + (ch - '0'));
                    break;
                }
                uint16_t len = parseValue;
                if(len == 0 || len > 0x3fff){
                    //not a valid length, the stream is out of sync
                    parseValue = 0;
                    receiveState = SKIPTOEND;
                    break;
                }
                if(len > PKG_SIZE){
                    //payload bigger than our buffer - store what fits and discard the rest
                    #ifdef DEBUG
                    Serial.println("Packet too long, truncated");
                    #endif
                    dropBytes = len - PKG_SIZE;
                    len = PKG_SIZE;
                }
                else{
                    dropBytes = 0;
                }
                recBuffer.len = len;
                readDataIdx = 0;
                #ifdef DEBUG
                Serial.print("linkID: ");
                Serial.print(recBuffer.linkId);
                Serial.print("  Length: ");
                Serial.println(recBuffer.len);
                #endif
                if(ch == ','){
                    parseValue = 0;
                    receiveState = IP;       //AT+CIPDINFO=1, remote ip and port follow
                }
                else if(ch == ':'){
                    parseValue = 0;
                    receiveState = DATA;     //AT+CIPDINFO=0
                }
                else{
                    receiveState = SKIPTOEND;
                    #ifdef DEBUG
                    Serial.println("Parse ERROR, skip this message");
                    #endif
                }
                break;
            }
            case IP:{
                //4 numbers separated by "," (ESP8266 style: 192,168,1,100) or "."
                int ch = wifi->read();
                if(ch < 0) break;
                if(ch >= '0' && ch <= '9'){
                    if(parseValue > 0xffff / 10)
                        parseValue = 0xffff;        //cannot be a valid octet anymore
                    else
                        parseValue = (uint16_t)(parseValue * 10 + (ch - '0'));
                    break;
                }
                if((ch == ',' || ch == '.') && matchIdx < 3 && parseValue <= 255){
                    recBuffer.ip[matchIdx++] = (uint8_t)parseValue;
                    parseValue = 0;
                    break;
                }
                if(ch == ',' && matchIdx == 3 && parseValue <= 255){
                    recBuffer.ip[matchIdx++] = (uint8_t)parseValue;
                    parseValue = 0;
                    matchIdx = 0;
                    receiveState = PORT;
                }
                else{
                    parseValue = 0;
                    matchIdx = 0;
                    receiveState = SKIPTOEND;
                    #ifdef DEBUG
                    Serial.println("Parse ERROR, skip this message");
                    #endif
                }
                break;
            }
            case PORT:{
                int ch = wifi->read();
                if(ch < 0) break;
                if(ch >= '0' && ch <= '9'){
                    if(parseValue > 0xffff / 10)
                        parseValue = 0xffff;        //cannot be a valid port anymore
                    else
                        parseValue = (uint16_t)(parseValue * 10 + (ch - '0'));
                    break;
                }
                if(ch == ':'){
                    recBuffer.port = parseValue;
                    parseValue = 0;
                    receiveState = DATA;
                    #ifdef DEBUG
                    Serial.print("IP: ");
                    Serial.print(recBuffer.ip);
                    Serial.print("  PORT: ");
                    Serial.println(recBuffer.port);
                    #endif
                }
                else{
                    parseValue = 0;
                    receiveState = SKIPTOEND;
                    #ifdef DEBUG
                    Serial.println("Parse ERROR, skip this message");
                    #endif
                }
                break;
            }
            case DATA:{
                int byteRead = wifi->read();
                if(byteRead < 0) break;
                if(readDataIdx < recBuffer.len){
                    recBuffer.data[readDataIdx] = (uint8_t)byteRead;
                    #ifdef DEBUG
                    Serial.print("0x");
                    Serial.print(recBuffer.data[readDataIdx], HEX);
                    Serial.print("; ");
                    #endif
                    readDataIdx++;
                }
                else if(dropBytes){
                    dropBytes--;    //payload was truncated, throw the rest away
                }
                if(readDataIdx >= recBuffer.len && !dropBytes){
                    readDataIdx = 0;
                    if(pendingReceive){
                        //the last packet was not read yet, throw this one away
                        #ifdef DEBUG
                        Serial.println("Receive buffer full, packet dropped");
                        #endif
                        receiveState = STANDBY;
                    }
                    else{
                        pendingReceive = true;
                        receiveState = BUFFER;
                        #ifdef DEBUG
                        Serial.println("\nReceive pending!");
                        #endif
                    }
                }
                break;
            }
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
            case SKIPDATA:
                if(wifi->read() < 0) break;
                if(dropBytes)
                    dropBytes--;
                else
                    receiveState = STANDBY;
                break;
            case URC_LINE:{
                int ch = wifi->read();
                if(ch < 0) break;
                if(ch != '\n'){
                    if(ch != '\r' && lineIdx < WIFI_LINE_SIZE - 1){
                        line[lineIdx++] = (char)ch;
                    }
                    break;
                }
                line[lineIdx] = '\0';
                handleLine();
                receiveState = STANDBY;
                break;
            }
        }
    }
    return;
}

//--------------------- URC and AT response handling ---------------------

//case insensitive search for a string that starts with start
bool WiFiClass::lineContains(const char *start, const char *str){
    while(*str){
        char a = *start++;
        char b = *str++;
        if(a >= 'a' && a <= 'z') a -= 'a' - 'A';
        if(b >= 'a' && b <= 'z') b -= 'a' - 'A';
        if(a != b) return false;
    }
    return true;
}

void WiFiClass::linkClosed(uint8_t id){
    if(id < WIFI_LINKS){
        #ifdef DEBUG
        Serial.print("Link ");
        Serial.print(id);
        Serial.println(" closed!");
        #endif
        linkIdOccupied[id] = false;
        if(id == linkId){
            if(cmdCmd == WIFI_CMD_OPEN){
                cmdFail();
            }
            if(!cmdActive()){
                reconnectState = RECONNECT_LINK;
                reconnectTime = millis();
            }
        }
    }
    else if(cmdCmd == WIFI_CMD_OPEN){
        //the link could not be opened
        cmdFail();
    }
}

void WiFiClass::handleLine(){
    char ch = line[0];
    if(lineIdx == 0 || ch == '\0') return;
    if(ch == 'A' && line[1] == 'T'){   //echo of one of our own commands
        return;
    }

    switch (ch) {
        case '+':
            //+WIFI DISCONNECT, +CWJAP:<error>
            if(lineIdx >= 4 && line[1] == 'W'){
                if(lineContains(line + 1, "WIFI DISCONNECT")){
                    #ifdef DEBUG
                    Serial.println("WiFi disconnected!");
                    #endif
                    markWifiLost();
                }
                else{   //+WIFI CONNECTED, +WIFI GOT IP
                    if(cmdCmd == WIFI_CMD_JOIN && state != WL_CONNECTED){
                        //the accesspoint is up again, ip and link are set next
                        cmdResult(true);
                    }
                }
            }
            else if(lineContains(line + 1, "CWJAP")){
                cmdFail();
            }
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':{
            //"<link id>,CLOSED"
            uint8_t id = 0;
            uint8_t i = 0;
            while(i < lineIdx && line[i] >= '0' && line[i] <= '9'){
                id = (uint8_t)(id * 10 + (line[i] - '0'));
                i++;
            }
            if(i < lineIdx && line[i] == ',' && lineContains(line + i + 1, "CLOSED")){
                linkClosed(id);
            }
            break;
        }
        case 'C':
        case 'c':
            if(lineContains(line, "CLOSED")){
                //the AT firmware closed the only connection
                linkClosed(linkId);
            }
            break;
        case 'W':
        case 'w':
            //WIFI CONNECTED / WIFI GOT IP / WIFI DISCONNECT without a '+'
            if(lineContains(line, "WIFI DISCONNECT")){
                #ifdef DEBUG
                Serial.println("WiFi disconnected!");
                #endif
                markWifiLost();
            }
            else if(lineContains(line, "WIFI ")){
                if(cmdCmd == WIFI_CMD_JOIN && state != WL_CONNECTED){
                    //the accesspoint is up again, ip and link are set next
                    cmdResult(true);
                }
            }
            break;
        case 'O':
        case 'o':
            if(line[1] == 'K'){
                cmdResult(true);
            }
            break;
        case 'E':
        case 'e':
        case 'F':
        case 'f':
            cmdFail();
            break;
        case 'A':
        case 'a':
            if(lineContains(line, "ALREADY CONNECTED")){
                cmdResult(true);
            }
            break;
        case 'l':
            if(lineContains(line, "NOT VALID")){
                cmdFail();
            }
            break;
        default:
            if(lineContains(line, "NOT VALID")){
                cmdFail();
            }
            break;
    }
}

bool WiFiClass::cmdActive(){
    return cmdExpected;     //the response of a reconnect command is still missing
}

void WiFiClass::cmdResult(bool success){
    if(!cmdExpected) return;
    cmdExpected = false;
    #ifdef DEBUG
    Serial.print("wifi cmd ");
    Serial.print(cmdCmd);
    Serial.println(success ? " ok" : " failed");
    #endif
    if(cmdCmd == WIFI_CMD_SERVER){
        //the server runs inside the AT firmware, the answer also arrives while
        //the module is not connected to the accesspoint, continue in both cases
        #ifdef DEBUG
        Serial.println(success ? "tcp server started!" : "tcp server failed!");
        #endif
        if(reconnectState == RECONNECT_LINK){
            reconnectNext();
        }
        return;
    }
    if(!success) return;

    switch (cmdCmd) {
        case WIFI_CMD_JOIN:
            state = WL_CONNECTED;
            joinRetries = 0;
            //the module also forgets the static ip and the hostname
            reconnectState = staticIpSet ? RECONNECT_IP : RECONNECT_HOSTNAME;
            #ifdef DEBUG
            Serial.println("reconnect CONNECTED!");
            #endif
            break;
        case WIFI_CMD_IP:
            reconnectState = RECONNECT_HOSTNAME;
            break;
        case WIFI_CMD_HOSTNAME:
            reconnectState = RECONNECT_CIPMUX;
            break;
        case WIFI_CMD_OPEN:
            if(linkId < WIFI_LINKS){
                linkIdOccupied[linkId] = true;
            }
            reconnectState = RECONNECT_IDLE;
            clearSendQueue();
            #ifdef DEBUG
            Serial.println("wifi link opened!");
            #endif
            break;
        case WIFI_CMD_CIPMUX:
            reconnectState = RECONNECT_DINFO;
            break;
        case WIFI_CMD_CIPDINFO:
            reconnectState = RECONNECT_OPEN;
            break;
        default:
            break;
    }
}

void WiFiClass::cmdFail(){
    if(cmdExpected){
        cmdResult(false);
    }
}

void WiFiClass::sendCmd(const __FlashStringHelper *cmd, uint8_t what, uint32_t timeout){
    if(wifi == NULL) return;
    cmdCmd = what;
    cmdExpected = true;
    cmdStart = millis();
    cmdTimeout = timeout;
    wifi->print(cmd);
}

//--------------------- wifi reconnect ---------------------

RECONNECT_STATE WiFiClass::getReconnectState(){
    return reconnectState;
}

uint8_t WiFiClass::getJoinRetries(){
    return joinRetries;
}

bool WiFiClass::needsReconnect(){
    return stationMode && state != WL_CONNECTED && reconnectState == RECONNECT_WAIT;
}

void WiFiClass::startReconnect(){
    if(state != WL_CONNECTED && (reconnectState == RECONNECT_WAIT || reconnectState == RECONNECT_IDLE)){
        joinRetries = 0;        //start a fresh set of join attempts
        reconnectState = RECONNECT_LINK;
        reconnectTime = millis();
    }
}

void WiFiClass::markWifiLost(){
    if(!stationMode) return;
    if(state == WL_CONNECTED){
        state = WL_CONNECTION_LOST;
        clearSendQueue();
        for(uint8_t i = 0; i < WIFI_LINKS; i++){
            linkIdOccupied[i] = false;   //all links are gone with the wifi connection
        }
        #ifdef DEBUG
        Serial.println("wifi connection lost!");
        #endif
    }
    if(!cmdActive()){
        reconnectState = RECONNECT_LINK;
        reconnectTime = millis();
    }
}

void WiFiClass::clearSendQueue(){
    sendIdx = 0;
    tail = 0;
    waitForSend = false;
    pendingCipsend = false;
    promptSeen = false;
}

void WiFiClass::reconnectStateMachine(uint32_t now){
    if(cmdExpected){
        if(cmdStart + cmdTimeout <= now){
            //the module did not answer
            #ifdef DEBUG
            Serial.println("reconnect timeout!");
            #endif
            cmdExpected = false;
            if(cmdCmd == WIFI_CMD_SERVER){
                //the server is not important for the wifi connection,
                //continue with the join even if it did not answer
                if(reconnectState == RECONNECT_LINK){
                    reconnectNext();
                }
            }
            else if(reconnectState != RECONNECT_WAIT && reconnectState != RECONNECT_IDLE){
                //start again, reconnectJoin() gives up after a few attempts
                reconnectState = RECONNECT_LINK;
                reconnectTime = now;
            }
        }
        return;
    }

    switch (reconnectState) {
        case RECONNECT_LINK:
            if(tcpServerPort){
                //wait for the answer, cmdResult() continues with the join,
                //a second AT command would destroy the pending one
                sendCmd(F("AT+CIPSERVER=1,"), WIFI_CMD_SERVER, 3000);
                wifi->println(tcpServerPort);
            }
            else{
                reconnectNext();
            }
            break;
        case RECONNECT_JOIN:
            reconnectJoin();
            break;
        case RECONNECT_IP:
            reconnectSetIp();
            break;
        case RECONNECT_HOSTNAME:
            reconnectHostname();
            break;
        case RECONNECT_OPEN:
            reconnectStartLink();
            break;
        case RECONNECT_CIPMUX:
            reconnectCipmux();
            break;
        case RECONNECT_DINFO:
            reconnectDinfo();
            break;
        case RECONNECT_WAIT:
            if(now >= reconnectTime + WIFI_RECONNECT_INTERVAL){
                joinRetries = 0;
                reconnectState = RECONNECT_LINK;
                reconnectTime = now;
            }
            break;
        default:
            break;
    }
}

void WiFiClass::reconnectNext(){
    if(state != WL_CONNECTED){
        reconnectJoin();
    }
    else{
        //the wifi is up, only the link is gone
        reconnectState = RECONNECT_CIPMUX;
    }
}

void WiFiClass::reconnectJoin(){
    if(!stationMode){
        reconnectState = RECONNECT_WAIT;
        reconnectTime = millis();
        return;
    }
    if(joinRetries >= 3){
        reconnectState = RECONNECT_WAIT;
        reconnectTime = millis();
        return;
    }
    joinRetries++;
    reconnectState = RECONNECT_JOIN;
    //drop the leftovers of a previous AT command before starting the join
    while (wifi->available()){
        wifi->read();
    }
    if(ssid[0] == '\0'){
        //the credentials of the AT firmware are kept in flash, reconnect to them
        #ifdef DEBUG
        Serial.println("reconnect to the stored accesspoint...");
        #endif
        sendCmd(F("AT+CWJAP_CUR\r\n"), WIFI_CMD_JOIN, WIFI_RECONNECT_JOIN_TIMEOUT);
        return;
    }
    #ifdef DEBUG
    Serial.print("reconnect to ");
    Serial.print(ssid);
    Serial.print("...");
    #endif
    sendCmd(F("AT+CWJAP_CUR=\""), WIFI_CMD_JOIN, WIFI_RECONNECT_JOIN_TIMEOUT);
    wifi->print(ssid);
    wifi->print("\",\"");
    wifi->print(pwd);
    wifi->println("\"");
}

void WiFiClass::reconnectSetIp(){
    reconnectState = RECONNECT_IP;
    if(!staticIpSet){
        //nothing to set, the next loop iteration continues with the hostname
        reconnectState = RECONNECT_HOSTNAME;
        return;
    }
    sendCmd(F("AT+CIPSTA_CUR=\""), WIFI_CMD_IP, 3000);
    wifi->print(staticIp[0]);
    wifi->print('.');
    wifi->print(staticIp[1]);
    wifi->print('.');
    wifi->print(staticIp[2]);
    wifi->print('.');
    wifi->print(staticIp[3]);
    wifi->println("\"");
}

void WiFiClass::reconnectHostname(){
    reconnectState = RECONNECT_HOSTNAME;
    sendCmd(F("AT+CWHOSTNAME=\"MobaStation\"\r\n"), WIFI_CMD_HOSTNAME, 3000);
}

void WiFiClass::reconnectCipmux(){
    //multiple connections, the remote ip and port are needed to know the client
    reconnectState = RECONNECT_CIPMUX;
    sendCmd(F("AT+CIPMUX=1\r\n"), WIFI_CMD_CIPMUX, 3000);
}

void WiFiClass::reconnectDinfo(){
    reconnectState = RECONNECT_DINFO;
    sendCmd(F("AT+CIPDINFO=1\r\n"), WIFI_CMD_CIPDINFO, 3000);
}

void WiFiClass::reconnectStartLink(){
    reconnectState = RECONNECT_OPEN;
    if(linkId >= WIFI_LINKS){
        reconnectState = RECONNECT_WAIT;
        reconnectTime = millis();
        return;
    }
    sendCmd(F("AT+CIPSTART="), WIFI_CMD_OPEN, 5000);
    wifi->print(linkId);
    wifi->print(",\"UDP\",\"");
    wifi->print(linkHost);
    wifi->print("\",");
    wifi->print(linkPort);
    wifi->print(",");
    wifi->print(linkPort);
    wifi->print(",2\r\n");
}

bool WiFiClass::disconnect(uint8_t linkID){
    if(linkID >= WIFI_LINKS) return false;
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
    uint16_t len = recBuffer.len;
    if(len > PKG_SIZE) len = PKG_SIZE;
    pkg->ip = recBuffer.ip;
    pkg->linkId = recBuffer.linkId;
    pkg->len = len;
    pkg->port = recBuffer.port;
    for(uint16_t i = 0; i < len; i++){
        pkg->data[i] = recBuffer.data[i];
    }
    pendingReceive = false;
    receiveState = STANDBY;
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
    uint8_t next = tail + 1;
    if(next >= SEND_BUF_SIZE){
        next = 0;
    }
    //ring buffer full, nothing space left
    if(waitForSend && next == sendIdx) return false;

    uint16_t len = pkg->len;
    if(len > PKG_SIZE) len = PKG_SIZE;

    sendBuffer[tail].linkId = pkg->linkId;
    sendBuffer[tail].ip = pkg->ip;
    sendBuffer[tail].port = pkg->port;
    sendBuffer[tail].len = len;
    for(uint16_t i = 0; i < len; i++){
        sendBuffer[tail].data[i] = pkg->data[i];
    }
    tail = next;
    waitForSend = true;
    return true;
}

//-------------------------- PRIVATE ---------------------------------

void WiFiClass::sendNext(){
    if(!waitForSend || pendingCipsend) return;
    if(wifi == NULL || sendIdx >= SEND_BUF_SIZE) return;
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
    promptSeen = false;
}

void WiFiClass::txData(){
    if(!pendingCipsend || sendIdx >= SEND_BUF_SIZE) return;
    //the packet stays in the queue until SEND OK or the timeout, finishSend() moves on
    for(uint16_t i = 0; i  < sendBuffer[sendIdx].len; i++){
        wifi->write(sendBuffer[sendIdx].data[i]);
    }
}

void WiFiClass::finishSend(){
    if(!pendingCipsend) return;     //the queue was cleared in the meantime
    pendingCipsend = false;
    promptSeen = false;
    //the only place where the read index moves, so a packet is never skipped twice
    sendIdx++;
    if(sendIdx >= SEND_BUF_SIZE){
        sendIdx = 0;
    }
    waitForSend = sendIdx != tail;
}

bool WiFiClass::canSend(){
    return state == WL_CONNECTED || state == WL_AP_LISTENING || state == WL_AP_CONNECTED;
}

uint8_t WiFiClass::getFreeLinkID(){
    for(int i = 0; i < WIFI_LINKS; i++){
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
        if(!wifi->available()){
            //nothing received yet, give the module time to answer without spinning
            delay(1);
            continue;
        }
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
