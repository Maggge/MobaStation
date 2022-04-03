/**
  MobaStation

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

#include "MobaStation.h"

#include "Eth_Adapter.h"
#include "ESP_AT_Wifi.h"

//------------- Variables -----------------

Eth_Interface *MobaStation::Eth[ETH_INTERFACES];

void (*MobaStation::broadcastEventHandler)(uint16_t DataLen, uint16_t Header, byte *data, boolean withXOR);

MobaBus *MobaStation::mobaBus;

XpressNetMasterClass *MobaStation::XpressNet;
uint8_t MobaStation::xpressPin;
XpressNetClient MobaStation::xpressNetClients[MAX_XPRESSNET_CLIENTS];

uint16_t MobaStation::_port;

TurnoutMode MobaStation::turnoutMode = Combined;
uint16_t MobaStation::dccAddrRange = 2047; 

float MobaStation::voltageMulti = 25.58f;



uint32_t MobaStation::z21IPlastIPcheck; //store the last time of check ip's active
uint32_t MobaStation::z21LastInfoBroadcast;
IPClient MobaStation::z21Clients[z21clientMAX];

uint8_t MobaStation::buffer[Z21_MAX_UDP_SIZE];

uint8_t MobaStation::rBus[20];

bool MobaStation::railPower;
uint8_t MobaStation::railState;

uint32_t MobaStation::ledLastBlink;
uint32_t MobaStation::pwrBtnLast;
bool MobaStation::pwrBtnState;

UdpPacket MobaStation::pkgBuffer;
//Ackmanager
IPAddress MobaStation::ackClient;
bool MobaStation::waitForAck;
uint8_t MobaStation::writeVal;
bool MobaStation::cvWrite;
int16_t MobaStation::ackCv;
//########################################

bool MobaStation::attachEthInterface(Eth_Interface *eth, IPAddress ip, uint16_t port, IPAddress subnetMask){
  for(int i = 0; i < ETH_INTERFACES; i++){
    if(Eth[i] == NULL){
      Eth[i] = eth;
      return eth->begin(port, ip, subnetMask);
    }
  }
  return false;
}

void MobaStation::begin(const FSH * motorShieldName, MotorDriver *mainDriver, MotorDriver *progDriver, bool driveOnProg){
  Serial.println("---------- MobaStation ----------");

  DCC::begin(motorShieldName, mainDriver, progDriver);
  DCC::setProgTrackSyncMain(driveOnProg);
  
  pinMode(POWER_BUTTON, INPUT_PULLUP);
  pinMode(POWER_LED, OUTPUT);
  digitalWrite(POWER_LED, LOW);

  setRailPower(false);
}

void MobaStation::attachMobaBus(MobaBus *m){
  mobaBus = m;
}

void MobaStation::loop(){
  uint32_t currentTime = millis();

  if(!railPower){
    if(currentTime - ledLastBlink > 100){
      digitalWrite(POWER_LED, !digitalRead(POWER_LED));
      ledLastBlink = currentTime;
    }
  }
  
  if(currentTime - pwrBtnLast > 50){
    if(pwrBtnState && digitalRead(POWER_BUTTON) == HIGH){
      pwrBtnState = false;
      pwrBtnLast = currentTime;
    }
    else if(!pwrBtnState && digitalRead(POWER_BUTTON) == LOW){
      setRailPower(!railPower);
      pwrBtnState = true;
      pwrBtnLast = currentTime;
    }
  }
 
  DCC::loop();

  processMobaBus();

  if(XpressNet != NULL){
    XpressNet->update();
  }
   

  //---------- Z21 loop ------------------
  for(int i = 0; i < ETH_INTERFACES; i++){
    if(Eth[i] != NULL){
      if(Eth[i]->receivePacket(&pkgBuffer)){
          processPacket(&pkgBuffer);
      }
    }
  }

  //check if IP is still used:
  if ((currentTime - z21IPlastIPcheck) > z21IPinterval) {
    z21IPlastIPcheck = currentTime;   
    for (uint8_t i = 0; i < z21clientMAX; i++) {
      if (z21Clients[i].time > 0) {
        z21Clients[i].time--;    //Zeit herrunterrechnen
        if(z21Clients[i].time == 0){
          clearClient(i); 	//clear IP DATA
        }
      }
    } 
  }

  if((currentTime - z21InfoInterval) > z21LastInfoBroadcast){
    z21LastInfoBroadcast = currentTime;
    broadcastSystemInfo();
  }
}

void MobaStation::processMobaBus(){
  if(mobaBus == NULL) return;
  MobaBus_Packet pkg;
  if(mobaBus->loop(&pkg)){
    #ifdef DEBUG
    Serial.println("MobaBus - Packet received!");
    Serial.print("Dev Type = 0x");
    Serial.println(pkg.meta.type, HEX);
    Serial.print("Address = 0x");
    Serial.println(pkg.meta.address, HEX);
    Serial.print("Data Len = ");
    Serial.println(pkg.len);
    for (int i = 0; i < pkg.len; i++){
      Serial.print(pkg.data[i]);
      Serial.print("; ");
    }
    Serial.println();
    #endif
    if(pkg.meta.cmd != INFO) return;

    switch (pkg.meta.type)
    {
    case ACCESSORIE:
      broadcastTurnout(pkg.meta.address , pkg.data[0]);
      break; 
    case FEEDBACK:
      rBus[pkg.meta.address - 1] = pkg.data[0];
      uint8_t group = (pkg.meta.address - 1) / 10;
      broadcastRbus(group);
      break;
    default:
      break;
    }
    
  }
}

void MobaStation::assembleData(uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR, uint8_t * dataOutput){
  //XOR bestimmen:
	dataOutput[0] = DataLen & 0xFF;
	dataOutput[1] = DataLen >> 8;
	dataOutput[2] = Header & 0xFF;
	dataOutput[3] = Header >> 8;
	dataOutput[DataLen - 1] = 0;	//XOR

  for (byte i = 0; i < (DataLen-5+!withXOR); i++) { //Ohne Length und Header und XOR
      if (withXOR)
        dataOutput[DataLen-1] = dataOutput[DataLen-1] ^ *dataString;
      dataOutput[i+4] = *dataString;
      dataString++;
  }
}

void MobaStation::sendLanPacket(IPAddress client , uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR){
  if(client == IPAddress()){
    if(broadcastEventHandler != NULL){
      broadcastEventHandler(DataLen, Header, dataString, withXOR);
    }
    return;
  }

  uint8_t data[24];                 //z21 send storage
  assembleData(DataLen, Header, dataString, withXOR, data);

  for(int i = 0; i < ETH_INTERFACES; i++){
    if(Eth[i] != NULL){
      Eth[i]->send(client, data);
    }
  }
}

void MobaStation::broadcastRbus(uint8_t group){
  uint8_t data[16];
  data[0] = group;
  rBusGetData(group, data);

  for(int i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].time > 0 && ((Z21bcRBus & z21Clients[i].BCFlag) > 0)){
      sendLanPacket(z21Clients[i].ip, 0x0F, LAN_RMBUS_DATACHANGED, data, false);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x0F, LAN_RMBUS_DATACHANGED, data, false);
  }
}

void MobaStation::broadcastTurnout(uint16_t addr, uint8_t dir, IPAddress *force){
  uint8_t data[4];
  addr--; //z21Protocoll 0=1,1=2....
  data[0] = LAN_X_TURNOUT_INFO;
  data[1] = (addr >> 8) & 0x3F;
  data[2] = addr & 0xFF;
  if(dir == 0){
    data[3] = B0000001;
  }
  else if(dir == 1){
    data[3] = B0000010;
  }
  else{
    data[3] = B0000011;
  }

  for(int i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].time > 0 && ((Z21bcAll_s & z21Clients[i].BCFlag) > 0 || *force == z21Clients[i].ip)){
      sendLanPacket(z21Clients[i].ip, 0x09, LAN_X_HEADER, data, true);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x09, LAN_X_HEADER, data, true);
  }
}

void MobaStation::processPacket(UdpPacket *packet){
    #ifdef DEBUG
    Serial.print("Pkg received from: ");
    Serial.println(packet->ip);
    #endif
    
    addClientToSlot(packet->ip, 0);

    int header = (packet->data[3]<<8) + packet->data[2];

    uint8_t data[16];

    switch (header) {
        case LAN_GET_SERIAL_NUMBER:{
            #ifdef DEBUG
            Serial.println("LAN_GET_SERIAL_NUMBER");
            #endif
            data[0] = FSTORAGE.read(CONFz21SnLSB);
            data[1] = FSTORAGE.read(CONFz21SnMSB);
            data[2] = 0x00; 
            data[3] = 0x00;
            sendLanPacket(packet->ip, 0x08, LAN_GET_SERIAL_NUMBER, data, false); //Seriennummer 32 Bit (little endian)
            break;
        }
        case LAN_GET_HWINFO: {
            #ifdef DEBUG
            Serial.println("LAN_GET_HWINFO");
            #endif
            data[0] = z21HWTypeLSB;  //HwType 32 Bit
            data[1] = z21HWTypeMSB;
            data[2] = 0x00; 
            data[3] = 0x00;
            data[4] = z21FWVersionLSB;  //FW Version 32 Bit
            data[5] = z21FWVersionMSB;
            data[6] = 0x00; 
            data[7] = 0x00;
            sendLanPacket(packet->ip, 0x0C, LAN_GET_HWINFO, data, false);
            break;
        }
        case LAN_LOGOFF: {
            #ifdef DEBUG
            Serial.println("LAN_LOGOFF");
            #endif
            clearIPSlot(packet->ip);
            break;
        case LAN_GET_CODE:
            #ifdef DEBUG
            Serial.println("LAN_GET_CODE");
            #endif
            /*#define Z21_NO_LOCK        0x00  // keine Features gesperrt 
            #define z21_START_LOCKED   0x01  // �z21 start�: Fahren und Schalten per LAN gesperrt 
            #define z21_START_UNLOCKED 0x02  // �z21 start�: alle Feature-Sperren aufgehoben */
            data[0] = 0x02; //keine Features gesperrt
            sendLanPacket(packet->ip, 0x05, LAN_GET_CODE, data, false);
            break;
        }
        case LAN_X_HEADER: {
            #ifdef DEBUG
            Serial.print("LAN_X_HEADER: ");
            Serial.print("0x");
            Serial.println(packet->data[5], HEX);
            #endif
            processLanXPacket(packet);
            break;
        }
        case LAN_SET_BROADCASTFLAGS: {
            #ifdef DEBUG
            Serial.println("LAN_SET_BROADCASTFLAGS");
            #endif
            uint32_t bcflag = packet->data[7];
            bcflag = packet->data[6] | (bcflag << 8);
            bcflag = packet->data[5] | (bcflag << 8);
            bcflag = packet->data[4] | (bcflag << 8);
            addClientToSlot(packet->ip, getLocalBcFlag(bcflag));
            data[0] = LAN_X_BC_TRACK_POWER;
            data[1] = railState;
            sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
            #ifdef DEBUG
            Serial.print("Broadcastflag: ");
            Serial.println(addClientToSlot(packet->ip, 0), BIN);
            #endif
            break;
        }
        case LAN_GET_BROADCASTFLAGS: {
            #ifdef DEBUG
            Serial.println("LAN_GET_BROADCASTFLAGS");
            #endif
            uint32_t flag = getz21BcFlag(addClientToSlot(packet->ip, 0x00));  
            data[0] = flag;
            data[1] = flag >> 8;
            data[2] = flag >> 16;
            data[3] = flag >> 24;
            sendLanPacket(packet->ip, 0x08, LAN_GET_BROADCASTFLAGS, data, false);
            #ifdef DEBUG
            Serial.print("Broadcastflag: ");
            Serial.println(flag, BIN);
            #endif
            break;
        }
        case LAN_GET_LOCOMODE: {
            #ifdef DEBUG
            Serial.println("LAN_GET_LOCOMODE");
            #endif
            break;
        }
        case LAN_SET_LOCOMODE: {
            #ifdef DEBUG
            Serial.println("LAN_SET_LOCOMODE");
            #endif
            break;
        }
        case LAN_GET_TURNOUTMODE: {
            #ifdef DEBUG
            Serial.println("LAN_GET_TURNOUTMODE");
            #endif
            break;
        }
        case LAN_SET_TURNOUTMODE: {
            #ifdef DEBUG
            Serial.println("LAN_SET_TURNOUTMODE");
            #endif
            break;
        }
        case LAN_RMBUS_GETDATA: {  
            #ifdef DEBUG        
            Serial.println("LAN_RMBUS_GETDATA");
            Serial.print("Group: ");
            Serial.println(packet->data[4]);
            #endif
            data[0] = packet->data[4];
            rBusGetData(packet->data[4], data);
            sendLanPacket(packet->ip, 0x0F, LAN_RMBUS_DATACHANGED, data, false);
            break;
        }
        case LAN_RMBUS_PROGRAMMODULE: {
            #ifdef DEBUG
            Serial.print("Data Length: ");
            Serial.println(packet->data[0]);
            Serial.println("LAN_RMBUS_PROGRAMMODULE");
            Serial.print("Modul Address: ");
            Serial.println(packet->data[4]);
            #endif
            mobaBus->sendPkg(FEEDBACK, packet->data[4], SET, 0, data);
            break;
        }
        case LAN_SYSTEMSTATE_GETDATA: {
            #ifdef DEBUG
            Serial.println("LAN_SYSTEMSTATE_GETDATA");
            #endif
            broadcastSystemInfo(&packet->ip);
            break;
        }
        case LAN_RAILCOM_GETDATA: {
            #ifdef DEBUG
            Serial.println("LAN_RAILCOM_GETDATA");
            #endif
            break;
        }
        case LAN_LOCONET_FROM_LAN: {
            #ifdef DEBUG
            Serial.println("LAN_LOCONET_FROM_LAN");
            #endif
            break;
        }
        case LAN_LOCONET_DISPATCH_ADDR: {
            #ifdef DEBUG
            Serial.println("LAN_LOCONET_DISPATCH_ADDR");
            #endif
            break;
        }
        case LAN_LOCONET_DETECTOR: {
            #ifdef DEBUG
            Serial.println("LAN_LOCONET_DETECTOR");
            #endif
            break;
        }
        case LAN_CAN_DETECTOR: {
            #ifdef DEBUG
            Serial.println("LAN_CAN_DETECTOR");
            #endif
            break;
        }
        default: {
            #ifdef DEBUG
            Serial.println("UNKNOWN_COMMAND");
            #endif
            break;
        }
    }
}

void MobaStation::processLanXPacket(UdpPacket *packet){
  uint8_t lanXheader = packet->data[4];
  uint8_t data[16];

  switch (lanXheader)
  {
  case LAN_X_GET_SETTING: 
    //---------------------- Switch BD0 BEGIN ---------------------------	
    switch (packet->data[5]) {  //DB0
    case 0x21:
      #ifdef DEBUG
      Serial.println("X_GET_VERSION"); 
      #endif
      data[0] = LAN_X_GET_VERSION;	//X-Header: 0x63
      data[1] = 0x21;	//DB0
      data[2] = 0x30;   //X-Bus Version
      data[3] = 0x12;  //ID der Zentrale
      sendLanPacket(packet->ip, 0x09, LAN_X_HEADER, data, true);
      break;
    case 0x24:
      #ifdef DEBUG
      Serial.println("X_GET_STATUS"); 
      #endif
      data[0] = LAN_X_STATUS_CHANGED;	//X-Header: 0x62
      data[1] = 0x22;			//DB0
      data[2] = railState;		//DB1: Status
      sendLanPacket(packet->ip, 0x08, LAN_X_HEADER, data, true);
      break;
    case 0x80:
      #ifdef DEBUG
      Serial.println("X_SET_TRACK_POWER_OFF"); 
      #endif
      setRailPower(false, &packet->ip);	  
      break;
    case 0x81:
      #ifdef DEBUG
      Serial.println("X_SET_TRACK_POWER_ON"); 
      #endif
      setRailPower(true, &packet->ip);
      break;  
    }
    break;  //ENDE DB0
    //---------------------- Switch DB0 ENDE ---------------------------	
  case LAN_X_CV_READ:
    if(packet->data[5] == 0x11){
      uint16_t cv = word(packet->data[6] & 0x3F, packet->data[7]);
      if(waitForAck){
        uint8_t data[2];
        data[0] = LAN_X_CV_NACK;
        data[1] = 0x13;
        sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
      }
      else{
        #ifdef DEBUG
        Serial.print("Read CV: ");
        Serial.println(cv);
        #endif
        waitForAck = true;
        ackClient = packet->ip;
        ackCv = cv;
        cv++; //1 hochzählen da bei z21 0 = 1;
        cvWrite = false;
        DCC::readCV(cv, cvCallback);
      }   
    }
    break;             
  case LAN_X_CV_WRITE: 
    if (packet->data[5] == 0x12) {  //DB0
      uint16_t cv = word(packet->data[6] & 0x3F, packet->data[7]);
      
      if(waitForAck){
        uint8_t data[2];
        data[0] = LAN_X_CV_NACK;
        data[1] = 0x13;
        sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
      }
      else{
        #ifdef DEBUG
        Serial.print("Write CV: ");
        Serial.println(cv); 
        #endif
        waitForAck = true;
        ackClient = packet->ip;
        ackCv = cv;
        cv++; //1 hochzählen da bei z21 0 = 1;
        cvWrite = true;
        writeVal = packet->data[8];
        DCC::writeCVByte(cv, writeVal, cvCallback);
      }    
    }
    break;
  case LAN_X_CV_POM: 
    if (packet->data[5] == 0x30) {  //DB0
      uint16_t addr = ((packet->data[6] & 0x3F) << 8) + packet->data[7];
      uint16_t cv = ((packet->data[8] & B11) << 8) + packet->data[9]; 
  
      if ((packet->data[8] & 0xFC) == 0xEC) {
        #ifdef DEBUG
        Serial.println("LAN_X_CV_POM_WRITE_BYTE"); 
        #endif
        DCC::writeCVByteMain(addr, cv, packet->data[10]);
      }
      else if ((packet->data[8] & 0xFC) == 0xE8) {
        #ifdef DEBUG
        Serial.println("LAN_X_CV_POM_WRITE_BIT"); 
        #endif
        byte bitNum = packet->data[10] & B00000111;
        bool value = packet->data[10] >> 3;
        DCC::writeCVBitMain(addr, cv, bitNum, value);
      }
      else {
        #ifdef DEBUG
        Serial.println("LAN_X_CV_POM_READ_BYTE - NOT IMPLEMENTED YET"); 
        #endif
      }
    }
    else if (packet->data[5] == 0x31) {  //DB0
      #ifdef DEBUG
      Serial.println("LAN_X_CV_POM_ACCESSORY - NOT IMPLEMENTED YET"); 
      #endif
    }
    break;      
  case LAN_X_GET_TURNOUT_INFO: {
    uint16_t addr = ((packet->data[5] << 8) + packet->data[6]) + 1; //Z21 - Protocol 0=1, 1=2, .....

    if(turnoutMode == MobaBus_Only || (turnoutMode == Combined && addr > dccAddrRange)){
      if(mobaBus != NULL){
        mobaBus->sendPkg(ACCESSORIE, addr, GET, 0, data);
      }
    }
    else{

    }
      break;
    }
  case LAN_X_SET_TURNOUT: {

    uint16_t addr = ((packet->data[5] << 8) + packet->data[6]) + 1; //Z21 - Protocol 0=1, 1=2, .....
    bool dir = bitRead(packet->data[7], 0);
    bool power = bitRead(packet->data[7], 3);
    
    #ifdef DEBUG
    Serial.print("X_SET_TURNOUT Adr.: ");
    Serial.print(addr);
    Serial.print(" Dir: ");
    Serial.print(dir);
    Serial.print(" - ");
    Serial.println(power);
    #endif
    if(turnoutMode == DCC_Only || (turnoutMode == Combined && addr <= dccAddrRange)){
      DCC::setAccessory(addr, dir, power); //Not tested yet!!
      broadcastTurnout(addr , dir);
    }
    if(turnoutMode == MobaBus_Only || (turnoutMode == Combined && addr > dccAddrRange)){
      if(mobaBus != NULL){
        data[0] = dir;
        data[1] = power;
        mobaBus->sendPkg(ACCESSORIE, addr, SET, 2, data);
      }
    }
    break;  
    }
  case LAN_X_SET_EXT_ACCESSORY: {
    /*#if defined(SERIALDEBUG)
    ZDebug.print("X_SET_EXT_ACCESSORY RAdr.:");
    ZDebug.print((packet[5] << 8) + packet[6]);
    ZDebug.print(":0x");
    ZDebug.println(packet[7], HEX);
    #endif
    setExtACCInfo((packet[5] << 8) + packet[6], packet[7]);*/
    break;
    }
  case LAN_X_GET_EXT_ACCESSORY_INFO: {
    /*#if defined(SERIALDEBUG)
    ZDebug.print("X_EXT_ACCESSORY_INFO RAdr.:");
    ZDebug.print((packet[5] << 8) + packet[6]);
    ZDebug.print(":0x");
    ZDebug.println(packet[7], HEX);	//DB2 Reserviert f�r zuk�nftige Erweiterungen
    #endif  
    setExtACCInfo((packet[5] << 8) + packet[6], packet[7]);*/
    break;  
    }
  case LAN_X_SET_STOP:
    #ifdef DEBUG
    Serial.println("X_SET_STOP");
    #endif
    DCC::setThrottle(0,1,1);
    data[0] = LAN_X_BC_STOPPED;
    data[1] = 0x00;
    sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
    break;  
  case LAN_X_GET_LOCO_INFO:
    if (packet->data[5] == 0xF0) {  //DB0
      uint16_t addr = word(packet->data[6] & 0x3F, packet->data[7]);
      #ifdef DEBUG
      Serial.print("Get Loco Info Addr: ");
      Serial.println(addr);
      #endif
      setLocoAbo(packet->ip, addr);
      broadcastLocoInfo(addr, &packet->ip);
    }
    break;  
  case LAN_X_SET_LOCO:{
    uint16_t addr = word(packet->data[6] & 0x3F, packet->data[7]);

    if(packet->data[5] == LAN_X_SET_LOCO_FUNCTION){
      uint8_t func = packet->data[8] & B00111111; 
      uint8_t val = packet->data[8] >> 6;
      #ifdef DEBUG
      Serial.print("SET LOCO FUNCTION: Addr: ");
      Serial.print(addr);
      Serial.print(" func: ");
      Serial.print(func);
      Serial.print(" val: ");
      Serial.println(val);
      #endif
      DCC::setFn(addr, func, val);
    }
    else{
      bool dir = (packet->data[8] > 127);
      uint8_t speed = (packet->data[8] & B01111111);

      uint8_t speedSteps = 128;	//default value S=3; DCC 128 Fahrstufen
      if (packet->data[5] == 0x12){	//S=2; DCC 28 Fahrstufen
        speedSteps = 28;
      }
      else if (packet->data[5] == 0x10){	//S=0; DCC 14 Fahrstufen
        speedSteps = 14;
      }

      DCC::setThrottle(addr, speed, speedSteps, dir);

      #ifdef DEBUG
      Serial.print("Loco Drive: Addr: ");
      Serial.print(addr);
      Serial.print("Speed: ");
      Serial.print(speed);
      Serial.print(" Dir: ");
      Serial.print(dir);
      Serial.print(" Speedsteps: ");
      Serial.println(speedSteps);
      #endif
    }
    broadcastLocoInfo(addr, &packet->ip);
    break;  }
  case LAN_X_GET_FIRMWARE_VERSION:
    #ifdef DEBUG
    Serial.println("X_GET_FIRMWARE_VERSION"); 
    #endif
    data[0] = 0xF3;		//identify Firmware (not change)
    data[1] = 0x0A;		//identify Firmware (not change)
    data[2] = z21FWVersionMSB;   //V_MSB
    data[3] = z21FWVersionLSB;  //V_LSB
    sendLanPacket(packet->ip, 0x09, LAN_X_HEADER, data, true);
    break;     
  case 0x73:
    //LAN_X_??? WLANmaus periodische Abfrage: 
    //0x09 0x00 0x40 0x00 0x73 0x00 0xFF 0xFF 0x00
    //length X-Header	XNet-Msg			  speed?
    /*#if defined(SERIALDEBUG)
    ZDebug.println("LAN-X_WLANmaus"); 
    #endif
    //set Broadcastflags for WLANmaus:
    if (addIPToSlot(client, 0x00) == 0)
      addIPToSlot(client, Z21bcAll);*/
    break;
  default:
    #ifdef DEBUG
    Serial.println("UNKNOWN_LAN-X_COMMAND"); 
    #endif
    data[0] = 0x61;
    data[1] = 0x82;
    sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
    break;
  }
}

void MobaStation::cvCallback(int16_t result){
  if(cvWrite && result == 1){
    result = writeVal;
  }
  #ifdef DEBUG
  Serial.print("Callback: ");
  Serial.println(result);
  #endif
  
  uint8_t len;
  uint8_t data[5];
  

  if(result < 0){
    len = 0x07;
    data[0] = LAN_X_CV_NACK;
    data[1] = 0x13;
  }
  else{
    len = 0x0A;
    data[0] = LAN_X_CV_RESULT;
    data[1] = 0x14;
    data[2] = ackCv >> 8;
    data[3] = ackCv & 0xFF; 
    data[4] = result;
  }
  sendLanPacket(ackClient, len, LAN_X_HEADER, data, true);
  cvWrite = false;
  waitForAck = false;
}

uint8_t MobaStation::addClientToSlot(IPAddress ip, uint8_t BCFlag){
    if(ip == IPAddress()) return 0;

    uint8_t slot = z21clientMAX;

    #if defined(ESP32)
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    #endif

    for (uint8_t i = 0; i < z21clientMAX; i++){
        if(z21Clients[i].ip == ip){
            z21Clients[i].time = z21ActTimeIP;
            if(BCFlag != 0){
                z21Clients[i].BCFlag = BCFlag;

                saveEEPROMBCFlag(ip, BCFlag);
                
            }
            return z21Clients[i].BCFlag;
        }
        else if(z21Clients[i].time == 0 && slot == z21clientMAX){
            slot = i;
        }
    } 
    z21Clients[slot].ip = ip;
    z21Clients[slot].time = z21ActTimeIP;

    setRailPower(railPower); //inform the client with last powerState;

    if(BCFlag == 0){
        #if defined(ESP32)
        portENTER_CRITICAL(&myMutex);
        #endif
        z21Clients[slot].BCFlag = getEEPROMBCFlag(ip);
        #if defined(ESP32)	
        portEXIT_CRITICAL(&myMutex);
        #endif
    }
    else{
        z21Clients[slot].BCFlag = BCFlag;       
        saveEEPROMBCFlag(ip, BCFlag);

    }
    	
    return z21Clients[slot].BCFlag;   //BC Flag 4. Byte R�ckmelden
}

uint8_t MobaStation::getClientId(IPAddress ip){
  for(int i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].ip == ip){
      return i;
    }
  }
  return 255;
}

IPAddress MobaStation::getIP(uint8_t clientID){
  return z21Clients[clientID].ip;
}

// delete the stored IP-Address
void MobaStation::clearClient (uint8_t pos) {
			z21Clients[pos].ip = IPAddress();
			z21Clients[pos].BCFlag = 0;
			z21Clients[pos].time = 0;
      for(int i = 0; i < CLIENT_MAX_LOCO_ABO; i++){
        z21Clients[pos].locoAbo[i] = 0;
      }			
}

//--------------------------------------------------------------------------------------------
void MobaStation::clearIPSlots() {
  for (int i = 0; i < z21clientMAX; i++) 
    clearClient(i);
}

//--------------------------------------------------------------------------------------------
void MobaStation::clearIPSlot(IPAddress ip) {
  for (int i = 0; i < z21clientMAX; i++) {
	  if (z21Clients[i].ip == ip) {
		  clearClient(i);
		  return;
	  }
  }
}

uint8_t MobaStation::getEEPROMBCFlag(IPAddress ip){
    
    uint16_t addr = ip.operator[](3) + CLIENTBCFLAGSTORE;
    if(FSTORAGE.length() >= addr){
        return FSTORAGE.read(addr);
    }
    return 0;
}

void MobaStation::saveEEPROMBCFlag(IPAddress ip, uint8_t BCFlag){
    uint16_t addr = ip.operator[](3) + CLIENTBCFLAGSTORE;
    if(FSTORAGE.length() >= addr){
        FSTORAGE.FSTORAGEMODE(addr, BCFlag);
    }
}

void MobaStation::broadcastSystemInfo(IPAddress *force){

  uint8_t data[16];

  int16_t mainCurrent = DCCWaveform::mainTrack.getCurrentmA();
  int16_t progCurrent = DCCWaveform::progTrack.getCurrentmA();

  uint16_t mainvoltage = (uint16_t)((float)analogRead(VOLTAGE_SENSE_PIN) * voltageMulti);

  //Serial.println(mainvoltage);

  int16_t temp = 41;

	data[0] = mainCurrent & 0xFF;  //MainCurrent mA
	data[1] = mainCurrent >> 8;  //MainCurrent mA
	data[2] = progCurrent & 0xFF;  //ProgCurrent mA
	data[3] = progCurrent >> 8;  //ProgCurrent mA        
	data[4] = data[0];  //FilteredMainCurrent
	data[5] = data[1];  //FilteredMainCurrent
	data[6] = temp & 0xFF;  //Temperature
	data[7] = temp >> 8;  //Temperature
	data[8] = mainvoltage & 0xFF;  //SupplyVoltage
	data[9] = mainvoltage >> 8;  //SupplyVoltage
	data[10] = data[8];  //VCCVoltage
	data[11] = data[9];  //VCCVoltage
	data[12] = railState;  //CentralState
  /*Bitmasken f�r CentralState: 
    #define csEmergencyStop  0x01 // Der Nothalt ist eingeschaltet 
    #define csTrackVoltageOff  0x02 // Die Gleisspannung ist abgeschaltet 
    #define csShortCircuit  0x04 // Kurzschluss 
    #define csProgrammingModeActive 0x20 // Der Programmiermodus ist aktiv 	
  */	
	data[13] = 0x00;  //CentralStateEx
  /* Bitmasken f�r CentralStateEx: 
    #define cseHighTemperature  0x01 // zu hohe Temperatur 
    #define csePowerLost  0x02 // zu geringe Eingangsspannung 
    #define cseShortCircuitExternal 0x04 // am externen Booster-Ausgang 
    #define cseShortCircuitInternal 0x08 // am Hauptgleis oder Programmiergleis 	
  */	
	data[14] = 0x00;  //reserved
	data[15] = 0x00;  //reserved

  for(uint8_t i = 0; i < z21clientMAX; i++){
    if((z21Clients[i].time > 0 && (Z21bcSystemInfo_s & z21Clients[i].BCFlag) > 0) || (force != NULL && *force == z21Clients[i].ip)){
      #ifdef DEBUG
      Serial.print("Send System info to ");
      Serial.println(z21Clients[i].ip);
      #endif
      sendLanPacket(z21Clients[i].ip, 0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false);
  }
}

void MobaStation::setRailPower(bool power, IPAddress *client = NULL){
  railPower = power;
  #ifndef ESP32
  POWERMODE p = power ? POWERMODE::ON : POWERMODE::OFF;

  DCCWaveform::mainTrack.setPowerMode(p);
  DCCWaveform::progTrack.setPowerMode(p);
  #endif
  if(power){
    railState = csNormal;
    digitalWrite(POWER_LED, HIGH);
  }else{
    railState = csTrackVoltageOff;
  }

  if(XpressNet != NULL ){
    XpressNet->setPower(railState);
  }

  broadcastRailState(client);
}

void MobaStation::broadcastRailState(IPAddress * force){
  uint8_t data[2];
  data[0] = LAN_X_BC_TRACK_POWER;
  if(railPower){
    data[1] = 0x01;
  }else{
    data[1] = 0x00;
  }

  for(uint8_t i = 0; i < z21clientMAX; i++){
    if((z21Clients[i].time > 0 && (Z21bcAll_s & z21Clients[i].BCFlag) > 0) || (force != NULL && *force == z21Clients[i].ip)){
      #ifdef DEBUG
      Serial.print("Send RailState to ");
      Serial.println(ip);
      #endif
      sendLanPacket(z21Clients[i].ip, 0x07, LAN_X_HEADER, data, true);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x07, LAN_X_HEADER, data, true);
  }
}

uint8_t MobaStation::getLocoInfo(uint16_t addr, uint8_t *data){
  
  data[0] = LAN_X_LOCO_INFO;
  data[1] = (addr >> 8) & 0x3F;
	data[2] = addr & 0xFF;

  uint8_t speedStepsInfo = DCC::getSpeedSteps(addr); 	

  if(speedStepsInfo == 14){
    speedStepsInfo = 0;
  }
  else if(speedStepsInfo == 28){
    speedStepsInfo = 2;
  }
  else{
    speedStepsInfo = 4;//default value; DCC 128 Fahrstufen
  }

  data[3] = ( speedStepsInfo | 0x08); //BUSY! 
  
  uint8_t speed = DCC::getThrottleSpeed(addr);
  uint8_t dir = DCC::getThrottleDirection(addr);

  data[4] = (dir << 7 | speed);

  uint8_t doppelTraktion = 0;
  unsigned long functions = DCC::getFns(addr);

  data[5] = (doppelTraktion << 6 | bitRead(functions, 0) << 4 | (functions & B11110) >> 1 );
  data[6] = (functions & 0x1FE0) >> 5; //Funktion 5 -12
  data[7] = (functions & 0x1FE000) >> 13; //Funktion 13 - 20
  data[8] = (functions & 0x1FE00000) >> 21; //Funktion 21 - 28
  #ifdef DEBUG
  Serial.print("Loco Info: ");
  Serial.print("Speed = ");
  Serial.print(speed);
  Serial.print("; Dir = ");
  Serial.print(dir);
  Serial.print("; steps = ");
  Serial.print(speedStepsInfo);
  Serial.print("; Func0 = ");
  Serial.println(functions & B1);
  #endif
  return 0x09;
}

void MobaStation::setLocoAbo(IPAddress client, uint16_t locoAddr){
  uint8_t firstFreeSlot = 255;
  for(int i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].ip == client){
      for(size_t j = 0; j < CLIENT_MAX_LOCO_ABO; j++){
        if(z21Clients[i].locoAbo[j] == locoAddr){
          return;
        }
        else if(z21Clients[i].locoAbo[j] == 0 && firstFreeSlot == 255){
          firstFreeSlot = j;
        }
      }
      if(firstFreeSlot != 255){
        z21Clients[i].locoAbo[firstFreeSlot] = locoAddr;
        
      }
      return;
    }
  }
}

void MobaStation::broadcastLocoInfo(uint16_t addr, IPAddress *force = NULL){
  uint8_t data[9];
  uint8_t len = getLocoInfo(addr, data) + 5;
  for(uint8_t i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].time == 0) break;
    if(force != NULL && *force == z21Clients[i].ip){
      #ifdef DEBUG
      Serial.print("Force Loco Info to");
      Serial.println(z21Clients[i].ip);
      #endif
      sendLanPacket( z21Clients[i].ip, len, LAN_X_HEADER, data, true);
    }
    else if((Z21bcAll_s & z21Clients[i].BCFlag) > 0){
      for (size_t j = 0; j < CLIENT_MAX_LOCO_ABO; j++){
          if(z21Clients[i].locoAbo[j] == addr){
            #ifdef DEBUG
            Serial.print("Send Loco Info to ");
            Serial.println(z21Clients[i].ip);
            #endif
            sendLanPacket( z21Clients[i].ip, len, LAN_X_HEADER, data, true);
          }
      }
    }
  }

  if(broadcastEventHandler != NULL){
    broadcastEventHandler(len, LAN_X_HEADER, data, true);
  }

  if(XpressNet != NULL){
    unsigned long func = DCC::getFns(addr);
    uint8_t speedStepsInfo = DCC::getSpeedSteps(addr);
    if(speedStepsInfo == 14){
      speedStepsInfo = 0;
    }
    else if(speedStepsInfo == 28){
      speedStepsInfo = 2;
    }
    else{
      speedStepsInfo = 4;//default value; DCC 128 Fahrstufen
    }

    uint8_t speed = DCC::getThrottleSpeed(addr);
    speed = DCC::getThrottleDirection(addr) << 7 | speed;
    for(int i = 0; i < MAX_XPRESSNET_CLIENTS; i++){
      if(xpressNetClients[i].client > 0){
        XpressNet->SetLocoInfoMM(xpressNetClients[i].client, speedStepsInfo, speed, ((uint8_t)func >> 1 & B00001111) | ((uint8_t)func << 4 & B00010000), (uint8_t)(func >> 5) & B00001111, (uint8_t)(func >> 9) & B00001111, (uint8_t)(func >> 13));
      }
    }

    XpressNet->setSpeed(addr, speedStepsInfo, speed);
    XpressNet->setFunc0to4(addr, ((uint8_t)func >> 1 & B00001111) | ((uint8_t)func << 4 & B00010000)); //Gruppe 1: 0 0 0 F0 F4 F3 F2 F1
	  XpressNet->setFunc5to8(addr, (uint8_t)(func >> 5) & B00001111); //Gruppe 2: 0 0 0 0 F8 F7 F6 F5 
	  XpressNet->setFunc9to12(addr, (uint8_t)(func >> 9) & B00001111); //Gruppe 3: 0 0 0 0 F12 F11 F10 F9 
	  XpressNet->setFunc13to20(addr, (uint8_t)(func >> 13)); //Gruppe 4: F20 F19 F18 F17 F16 F15 F14 F13  
	  XpressNet->setFunc21to28(addr, (uint8_t)(func >> 21)); //Gruppe 5: F28 F27 F26 F25 F24 F23 F22 F21
  }
}

//--------------------------------------------------------------------------------
//Convert local stored flag back into a Z21 Flag
uint32_t MobaStation::getz21BcFlag (uint8_t flag) {
  unsigned long outFlag = 0;
  if ((flag & Z21bcAll_s) != 0)
    outFlag |= Z21bcAll;
  if ((flag & Z21bcRBus_s) != 0)
    outFlag |= Z21bcRBus;
  if ((flag & Z21bcSystemInfo_s) != 0)
    outFlag |= Z21bcSystemInfo;
  if ((flag & Z21bcNetAll_s) != 0)
    outFlag |= Z21bcNetAll;
  if ((flag & Z21bcLocoNet_s) != 0)
    outFlag |= Z21bcLocoNet;
  if ((flag & Z21bcLocoNetLocos_s) != 0)
    outFlag |= Z21bcLocoNetLocos;
  if ((flag & Z21bcLocoNetSwitches_s) != 0)
    outFlag |= Z21bcLocoNetSwitches;
  if ((flag & Z21bcLocoNetGBM_s) != 0)    
    outFlag |= Z21bcLocoNetGBM;
  return outFlag;
}

//Convert Z21 LAN BC flag to local stored flag
uint8_t MobaStation::getLocalBcFlag (uint32_t flag) {
  uint8_t outFlag = 0;
  if ((flag & Z21bcAll) != 0)
    outFlag |= Z21bcAll_s;
  if ((flag & Z21bcRBus) != 0) 
    outFlag |= Z21bcRBus_s;
  if ((flag & Z21bcSystemInfo) != 0)
    outFlag |= Z21bcSystemInfo_s;
  if ((flag & Z21bcNetAll) != 0)
    outFlag |= Z21bcNetAll_s;
  if ((flag & Z21bcLocoNet) != 0)
    outFlag |= Z21bcLocoNet_s;
  if ((flag & Z21bcLocoNetLocos) != 0)
    outFlag |= Z21bcLocoNetLocos_s;
  if ((flag & Z21bcLocoNetSwitches) != 0)
    outFlag |= Z21bcLocoNetSwitches_s;
  if ((flag & Z21bcLocoNetGBM) != 0) 
    outFlag |= Z21bcLocoNetGBM_s;
  return outFlag;  
}



void MobaStation::rBusGetData(uint8_t group, uint8_t *data){
  uint8_t idx = group * 10;
  for(uint8_t i = 0; i < 10; i++){
    data[i+1] = rBus[idx + i];
  }
}

void MobaStation::setTurnoutMode(TurnoutMode mode, uint16_t addrRange = 2047){
  turnoutMode = mode;
  dccAddrRange = addrRange;
}

void MobaStation::setVoltageMultiplier(float multi){
  voltageMulti = multi;
}

//----------   Xpress-Net   ----------

void MobaStation::attachXpressNet(XpressNetMasterClass *xpressNet, uint8_t sendReceivePin){
  if(xpressNet == NULL) return;
  XpressNet = xpressNet;
  XpressNet->setup(Loco128, sendReceivePin);
}

void MobaStation::addXpressNetClient(uint8_t client, uint16_t loco){
  for(int i = 0; i < MAX_XPRESSNET_CLIENTS; i++){
    if(xpressNetClients[i].client == client){
      xpressNetClients[i].loco = loco;
      return;
    }
  }
  for(int i = 0; i < MAX_XPRESSNET_CLIENTS; i++){
    if(xpressNetClients[i].client == 0){
      xpressNetClients[i].client = client;
      xpressNetClients[i].loco = loco;
      return;
    }
  }
  return;
}

void notifyXNetgiveLocoMM(uint8_t UserOps, uint16_t Address){
  #ifdef DEBUG
  Serial.println("MM get Loco Info");
  #endif
  unsigned long func = DCC::getFns(Address);
  uint8_t speedStepsInfo = DCC::getSpeedSteps(Address);
  if(speedStepsInfo == 14){
    speedStepsInfo = 0;
  }
  else if(speedStepsInfo == 28){
    speedStepsInfo = 2;
  }
  else{
    speedStepsInfo = 4;//default value; DCC 128 Fahrstufen
  }
  MobaStation::addXpressNetClient(UserOps, Address);
  uint8_t speed = DCC::getThrottleSpeed(Address);
  speed = DCC::getThrottleDirection(Address) << 7 | speed;
  MobaStation::XpressNet->SetLocoInfoMM(UserOps, speedStepsInfo, speed, ((uint8_t)func >> 1 & B00001111) | ((uint8_t)func << 4 & B00010000), (uint8_t)(func >> 5) & B00001111, (uint8_t)(func >> 9) & B00001111, (uint8_t)(func >> 13));
}

void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) {
  #ifdef DEBUG
  Serial.print("Get Loco Info UserOps: ");
  Serial.print(UserOps);
  Serial.print("  Addr: ");
  Serial.println(Address);
  #endif
  unsigned long func = DCC::getFns(Address);
  uint8_t speedStepsInfo = DCC::getSpeedSteps(Address);
  if(speedStepsInfo == 14){
    speedStepsInfo = 0;
  }
  else if(speedStepsInfo == 28){
    speedStepsInfo = 2;
  }
  else{
    speedStepsInfo = 4;//default value; DCC 128 Fahrstufen
  }
  MobaStation::addXpressNetClient(UserOps, Address);
  uint8_t speed = DCC::getThrottleSpeed(Address);
  speed = DCC::getThrottleDirection(Address) << 7 | speed;

  MobaStation::XpressNet->SetLocoInfo(UserOps, speedStepsInfo, speed, ((uint8_t)func >> 1 & B00001111) | ((uint8_t)func << 4 & B00010000), (uint8_t)(func >> 5) & B00001111); //UserOps,Speed,F0,F1
}

void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) {
  #ifdef DEBUG
  Serial.print("Get Loco Func UserOps: ");
  Serial.print(UserOps);
  Serial.print("  Addr: ");
  Serial.println(Address);
  #endif
  MobaStation::addXpressNetClient(UserOps, Address);
  unsigned long func = DCC::getFns(Address);
  MobaStation::XpressNet->SetFktStatus(UserOps, (uint8_t)(func >> 13), (uint8_t)(func >> 21)); //Fkt4, Fkt5
}



void notifyXNetPower(uint8_t State) {
  #ifdef DEBUG
  Serial.print("Power: ");
  Serial.println(State, HEX);
  #endif
  MobaStation::setRailPower(State == 0);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S14:");
  Serial.println(Speed, BIN);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S28:");
  Serial.println(Speed, BIN);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S128:");
  Serial.println(Speed, BIN);
  #endif
}

//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F1:");
  Serial.println(Func1, BIN);
  #endif
  DCC::setFn(Address, 0, (bool)(Func1 >> 4 | B0000001));
  for(int i = 0; i < 4; i++){
    DCC::setFn(Address, i+1, (bool)(Func1 >> i | B0000001));
  }
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F2:");
  Serial.println(Func2, BIN);
  #endif
  for(int i = 0; i < 4; i++){
    DCC::setFn(Address, i+5, bitRead(Func2, i));
  }
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F3:");
  Serial.println(Func3, BIN);
  #endif
  for(int i = 0; i < 4; i++){
    DCC::setFn(Address, i+9, bitRead(Func3, i));
  }
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc4(uint16_t Address, uint8_t Func4) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F4:");
  Serial.println(Func4, BIN);
  #endif
  for(int i = 0; i < 8; i++){
    DCC::setFn(Address, i+13, bitRead(Func4, i));
  }
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc5(uint16_t Address, uint8_t Func5) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F5:");
  Serial.println(Func5, BIN);
  #endif
  for(int i = 0; i < 8; i++){
    DCC::setFn(Address, i+21, bitRead(Func5, i));
  }
  MobaStation::broadcastLocoInfo(Address);
}
void notifyXNetTrnt(uint16_t Address, uint8_t data) {
  if (bitRead(data,3) == 0x01) {  //Weiche Aktiv == HIGH?
    #ifdef DEBUG
    Serial.print("XNet TA:");
    Serial.print(Address);
    Serial.print(", P:");
    Serial.println(data & 0x01);
    #endif
  }
}

void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data) {
  int adr = ((Address * 4) + ((data & 0x01) * 2));
  byte pos = data << 4;
  bitWrite(pos, 7, 1);  //command completed!
/*  
  if (dcc.getBasicAccessoryInfo(adr) == false)
    bitWrite(pos, 0, 1);
  else bitWrite(pos, 1, 1);  
  if (dcc.getBasicAccessoryInfo(adr+1) == false)
    bitWrite(pos, 2, 1);  
  else bitWrite(pos, 3, 1);    
*/  
  MobaStation::XpressNet->SetTrntStatus(UserOps, Address, pos);
}