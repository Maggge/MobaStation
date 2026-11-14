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

TurnoutMode MobaStation::turnoutMode = DEFAULT_TURNOUT_MODE;
uint16_t MobaStation::dccAddrRange = DCC_TURNOUT_ADDRESS_RANGE; 

float MobaStation::voltageMulti = VOLTAGE_MULTIPLIER;



uint32_t MobaStation::z21IPlastIPcheck; //store the last time of check ip's active
uint32_t MobaStation::z21LastInfoBroadcast;
IPClient MobaStation::z21Clients[z21clientMAX];

uint8_t MobaStation::rBus[RBUS_GROUPS * RBUS_GROUP_SIZE];
uint8_t MobaStation::turnoutStates[(TURNOUT_STATE_COUNT + 3) / 4];

bool MobaStation::railPower;
uint8_t MobaStation::railState;

uint32_t MobaStation::ledLastBlink;
uint32_t MobaStation::pwrBtnLast;
bool MobaStation::pwrBtnState;

UdpPacket MobaStation::pkgBuffer;
//Ackmanager
IPAddress MobaStation::ackClient;
bool MobaStation::ackXNet;
bool MobaStation::waitForAck;
uint32_t MobaStation::ackStartTime;
uint8_t MobaStation::writeVal;
bool MobaStation::cvWrite;
int16_t MobaStation::ackCv;
//########################################

//---------- speed step conversion ----------
//Z21 and XpressNet encode the speed the same way:
//  14 steps:  R000 VVVV  0 = stop, 1 = emergency stop, 2..15 = step 1..14
//  28 steps:  R00V VVVV  bit 4 is the least significant speed bit,
//             code = VVVV << 1 | V: 0/1 = stop, 2/3 = emergency stop, 4..31 = step 1..28
//  128 steps: RVVV VVVV  0 = stop, 1 = emergency stop, 2..127 = step 1..126
//R = direction (1 = forward). The DCC library always works with the 128 step code.

static uint8_t speedToDcc128(uint8_t value, uint8_t steps, bool &dir){
  dir = value & 0x80;
  if(steps == 14){
    uint8_t n = value & 0x0F;
    if(n <= 1) return n;
    return 2 + (uint16_t)(n - 2) * 125 / 13;
  }
  if(steps == 28){
    uint8_t code = ((value & 0x0F) << 1) | ((value >> 4) & 0x01);
    if(code <= 1) return 0;
    if(code <= 3) return 1;
    return 2 + (uint16_t)(code - 4) * 125 / 27;
  }
  return value & 0x7F;
}

//inverse of speedToDcc128(), the rounding makes the round trip exact
static uint8_t speedFromDcc128(uint8_t speed, bool dir, uint8_t steps){
  uint8_t value = speed & 0x7F;   //stop (0) and emergency stop (1) are the same in all formats
  if(value > 1){
    if(steps == 14){
      value = 2 + ((uint16_t)(value - 2) * 13 + 62) / 125;
    }
    else if(steps == 28){
      uint8_t code = 4 + ((uint16_t)(value - 2) * 27 + 62) / 125;
      value = (code >> 1) | ((code & 0x01) << 4);
    }
  }
  return (dir ? 0x80 : 0x00) | value;
}

bool MobaStation::attachEthInterface(Eth_Interface *eth, uint16_t port){
  NetworkConfig net;
  net.dhcp = NETWORK_DHCP;
  net.ip = IPAddress(NETWORK_IP);
  net.subnet = IPAddress(NETWORK_SUBNET);
  net.gateway = IPAddress(NETWORK_GATEWAY);
  return attachEthInterface(eth, net, port);
}

bool MobaStation::attachEthInterface(Eth_Interface *eth, IPAddress ip, uint16_t port, IPAddress subnetMask){
  NetworkConfig net;
  net.dhcp = false;
  net.ip = ip;
  net.subnet = subnetMask;
  net.gateway = IPAddress();
  return attachEthInterface(eth, net, port);
}

bool MobaStation::attachEthInterface(Eth_Interface *eth, const NetworkConfig &net, uint16_t port){
  if(eth == NULL) return false;
  _port = port;   //answers to clients that are not registered go to this port
  for(int i = 0; i < ETH_INTERFACES; i++){
    if(Eth[i] == NULL){
      Eth[i] = eth;
      return eth->begin(port, net);
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

//route a turnout command to DCC and/or MobaBus according to the current turnout mode
//addr is the turnout number starting at 1
void MobaStation::setTurnout(uint16_t addr, bool dir, bool power){
  if(addr == 0) return;
  if(turnoutMode == DCC_Only || (turnoutMode == Combined && addr <= dccAddrRange)){
    //RCN-213: turnout 1-4 = decoder address 1 port 0-3. The library takes
    //(decoder address, port, output) and always sends the packet with the coil on,
    //so only the activate command is sent, the deactivate would switch once more
    if(power){
      uint16_t a = addr - 1;
      DCC::setAccessory(a / 4 + 1, a % 4, dir);
    }
    broadcastTurnout(addr, dir);
  }
  if(turnoutMode == MobaBus_Only || (turnoutMode == Combined && addr > dccAddrRange)){
    if(mobaBus != NULL){
      uint8_t data[2];
      data[0] = dir;
      data[1] = power;
      mobaBus->sendPkg(ACCESSORIE, addr, SET, 2, data);
    }
  }
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

  checkShortCircuit();

  //the DCC library should always report a result, but if it does not,
  //answer the client and release the programming track instead of blocking forever
  if(waitForAck && (currentTime - ackStartTime) > CV_ACK_TIMEOUT){
    #ifdef DEBUG
    Serial.println("CV operation timeout");
    #endif
    sendCvResult(-1);
    cvWrite = false;
    waitForAck = false;
  }

  processMobaBus();

  if(XpressNet != NULL){
    XpressNet->update();
  }
   

  //---------- Z21 loop ------------------
  for(int i = 0; i < ETH_INTERFACES; i++){
    if(Eth[i] != NULL){
      if(Eth[i]->receivePacket(&pkgBuffer)){
          pkgBuffer.iface = i;
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

  //written as difference so it survives the millis() overflow and does not fire
  //in every loop during the first seconds after the start
  if((currentTime - z21LastInfoBroadcast) > z21InfoInterval){
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

    if(pkg.len < 1) return;   //every info carries at least one data byte

    switch (pkg.meta.type)
    {
    case ACCESSORIE:
      broadcastTurnout(pkg.meta.address , pkg.data[0]);
      break;
    case FEEDBACK:{
      //the address comes from the bus, never trust it as an array index
      if(pkg.meta.address < 1 || pkg.meta.address > sizeof(rBus)) break;
      rBus[pkg.meta.address - 1] = pkg.data[0];
      uint8_t group = (pkg.meta.address - 1) / RBUS_GROUP_SIZE;
      broadcastRbus(group);
      break;
    }
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

  uint8_t slot = getClientId(client);
  if(slot < z21clientMAX && z21Clients[slot].time > 0){
    sendToClient(slot, DataLen, Header, dataString, withXOR);
    return;
  }

  //unknown client (eg. timed out while a CV operation was running),
  //try the standard port on every interface
  uint8_t data[24];                 //z21 send storage
  assembleData(DataLen, Header, dataString, withXOR, data);

  for(int i = 0; i < ETH_INTERFACES; i++){
    if(Eth[i] != NULL){
      Eth[i]->send(client, _port, data);
    }
  }
}

void MobaStation::sendToClient(uint8_t slot, uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR){
  IPClient &c = z21Clients[slot];
  if(c.iface >= ETH_INTERFACES || Eth[c.iface] == NULL) return;

  uint8_t data[24];                 //z21 send storage
  assembleData(DataLen, Header, dataString, withXOR, data);

  //only the interface the client talks over, with Ethernet and WiFi attached the
  //slow AT command queue of the ESP would otherwise carry every Ethernet packet too
  Eth[c.iface]->send(c.ip, c.port ? c.port : _port, data);
}

void MobaStation::broadcastRbus(uint8_t group){
  uint8_t data[16];
  data[0] = group;
  rBusGetData(group, data);

  for(int i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].time > 0 && ((Z21bcRBus_s & z21Clients[i].BCFlag) > 0)){
      sendToClient(i, 0x0F, LAN_RMBUS_DATACHANGED, data, false);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x0F, LAN_RMBUS_DATACHANGED, data, false);
  }
}

void MobaStation::storeTurnout(uint16_t addr, uint8_t dir){
  if(addr == 0 || addr > TURNOUT_STATE_COUNT || dir > 1) return;
  uint16_t a = addr - 1;
  uint8_t shift = (a % 4) * 2;
  turnoutStates[a / 4] = (turnoutStates[a / 4] & ~(0x03 << shift)) | ((dir + 1) << shift);
}

uint8_t MobaStation::getTurnoutInfo(uint16_t addr){
  if(addr == 0 || addr > TURNOUT_STATE_COUNT) return 0;   //unknown
  uint16_t a = addr - 1;
  return (turnoutStates[a / 4] >> ((a % 4) * 2)) & 0x03;
}

void MobaStation::broadcastTurnout(uint16_t addr, uint8_t dir, IPAddress *force){
  uint8_t data[4];
  storeTurnout(addr, dir);
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
    if(z21Clients[i].time > 0 && ((Z21bcAll_s & z21Clients[i].BCFlag) > 0 || (force != NULL && *force == z21Clients[i].ip))){
      sendToClient(i, 0x09, LAN_X_HEADER, data, true);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x09, LAN_X_HEADER, data, true);
  }
}

//a UDP datagram can carry several Z21 records, each starts with its 16 bit length
void MobaStation::processPacket(UdpPacket *packet){
    #ifdef DEBUG
    Serial.print("Pkg received from: ");
    Serial.println(packet->ip);
    #endif

    uint16_t offset = 0;
    while(offset + 4 <= packet->dataLength){
        uint16_t len = packet->data[offset] | (packet->data[offset + 1] << 8);
        if(len < 4 || offset + len > packet->dataLength){
            //corrupt or truncated, never parse the bytes of an old packet
            #ifdef DEBUG
            Serial.println("Invalid Z21 record length");
            #endif
            return;
        }
        //zero padded copy, a short record reads zeros instead of foreign bytes
        uint8_t rec[Z21_MAX_RECORD];
        memset(rec, 0, sizeof(rec));
        memcpy(rec, &packet->data[offset], len < sizeof(rec) ? len : sizeof(rec));
        processRecord(packet, rec, len);
        offset += len;
    }
}

void MobaStation::processRecord(UdpPacket *packet, uint8_t *rec, uint16_t len){
    addClientToSlot(packet, 0);

    int header = (rec[3]<<8) + rec[2];

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
        }
        case LAN_GET_CODE: {
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
            Serial.println(rec[5], HEX);
            #endif
            if(len < 6) break;   //length, header, X-header and XOR at least
            processLanXPacket(packet, rec);
            break;
        }
        case LAN_SET_BROADCASTFLAGS: {
            #ifdef DEBUG
            Serial.println("LAN_SET_BROADCASTFLAGS");
            #endif
            uint32_t bcflag = rec[7];
            bcflag = rec[6] | (bcflag << 8);
            bcflag = rec[5] | (bcflag << 8);
            bcflag = rec[4] | (bcflag << 8);
            addClientToSlot(packet, getLocalBcFlag(bcflag));
            //railState is a bitmask (csNormal = 0), the client needs the track power message
            sendLanPacket(packet->ip, railStateMessage(data), LAN_X_HEADER, data, true);
            #ifdef DEBUG
            Serial.print("Broadcastflag: ");
            Serial.println(addClientToSlot(packet, 0), BIN);
            #endif
            break;
        }
        case LAN_GET_BROADCASTFLAGS: {
            #ifdef DEBUG
            Serial.println("LAN_GET_BROADCASTFLAGS");
            #endif
            uint32_t flag = getz21BcFlag(addClientToSlot(packet, 0x00));  
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
            Serial.println(rec[4]);
            #endif
            if(rec[4] >= RBUS_GROUPS) break;
            data[0] = rec[4];
            rBusGetData(rec[4], data);
            sendLanPacket(packet->ip, 0x0F, LAN_RMBUS_DATACHANGED, data, false);
            break;
        }
        case LAN_RMBUS_PROGRAMMODULE: {
            #ifdef DEBUG
            Serial.print("Data Length: ");
            Serial.println(rec[0]);
            Serial.println("LAN_RMBUS_PROGRAMMODULE");
            Serial.print("Modul Address: ");
            Serial.println(rec[4]);
            #endif
            if(mobaBus != NULL){
              mobaBus->sendPkg(FEEDBACK, rec[4], SET, 0, data);
            }
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

void MobaStation::processLanXPacket(UdpPacket *packet, uint8_t *rec){
  uint8_t lanXheader = rec[4];
  uint8_t data[16];

  switch (lanXheader)
  {
  case LAN_X_GET_SETTING: 
    //---------------------- Switch BD0 BEGIN ---------------------------	
    switch (rec[5]) {  //DB0
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
    if(rec[5] == 0x11){
      uint16_t cv = word(rec[6] & 0x3F, rec[7]);  //0 based
      #ifdef DEBUG
      Serial.print("Read CV: ");
      Serial.println(cv);
      #endif
      if(!startCv(false, cv, 0, &packet->ip)){
        //another CV operation (Z21 or XpressNet) is still running
        data[0] = LAN_X_CV_NACK;
        data[1] = 0x13;
        sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
      }
    }
    break;
  case LAN_X_CV_WRITE: 
    if (rec[5] == 0x12) {  //DB0
      uint16_t cv = word(rec[6] & 0x3F, rec[7]);  //0 based
      #ifdef DEBUG
      Serial.print("Write CV: ");
      Serial.println(cv);
      #endif
      if(!startCv(true, cv, rec[8], &packet->ip)){
        data[0] = LAN_X_CV_NACK;
        data[1] = 0x13;
        sendLanPacket(packet->ip, 0x07, LAN_X_HEADER, data, true);
      }
    }
    break;
  case LAN_X_CV_POM: 
    if (rec[5] == 0x30) {  //DB0
      uint16_t addr = ((rec[6] & 0x3F) << 8) + rec[7];
      uint16_t cv = ((rec[8] & B11) << 8) + rec[9]; 
  
      if ((rec[8] & 0xFC) == 0xEC) {
        #ifdef DEBUG
        Serial.println("LAN_X_CV_POM_WRITE_BYTE"); 
        #endif
        DCC::writeCVByteMain(addr, cv, rec[10]);
      }
      else if ((rec[8] & 0xFC) == 0xE8) {
        #ifdef DEBUG
        Serial.println("LAN_X_CV_POM_WRITE_BIT"); 
        #endif
        byte bitNum = rec[10] & B00000111;
        bool value = rec[10] >> 3;
        DCC::writeCVBitMain(addr, cv, bitNum, value);
      }
      else {
        #ifdef DEBUG
        Serial.println("LAN_X_CV_POM_READ_BYTE - NOT IMPLEMENTED YET"); 
        #endif
      }
    }
    else if (rec[5] == 0x31) {  //DB0
      #ifdef DEBUG
      Serial.println("LAN_X_CV_POM_ACCESSORY - NOT IMPLEMENTED YET"); 
      #endif
    }
    break;      
  case LAN_X_GET_TURNOUT_INFO: {
    uint16_t addr = ((rec[5] << 8) + rec[6]) + 1; //Z21 - Protocol 0=1, 1=2, .....

    if(turnoutMode == MobaBus_Only || (turnoutMode == Combined && addr > dccAddrRange)){
      //the module answers on the bus, processMobaBus() broadcasts the position
      if(mobaBus != NULL){
        mobaBus->sendPkg(ACCESSORIE, addr, GET, 0, data);
      }
    }
    else{
      //DCC decoders give no feedback, answer with the last switched position
      data[0] = LAN_X_TURNOUT_INFO;
      data[1] = rec[5] & 0x3F;
      data[2] = rec[6];
      data[3] = getTurnoutInfo(addr);
      sendLanPacket(packet->ip, 0x09, LAN_X_HEADER, data, true);
    }
      break;
    }
  case LAN_X_SET_TURNOUT: {

    uint16_t addr = ((rec[5] << 8) + rec[6]) + 1; //Z21 - Protocol 0=1, 1=2, .....
    bool dir = bitRead(rec[7], 0);
    bool power = bitRead(rec[7], 3);
    
    #ifdef DEBUG
    Serial.print("X_SET_TURNOUT Adr.: ");
    Serial.print(addr);
    Serial.print(" Dir: ");
    Serial.print(dir);
    Serial.print(" - ");
    Serial.println(power);
    #endif
    setTurnout(addr, dir, power);
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
    //all clients and throttles get LAN_X_BC_STOPPED, not only the sender
    emergencyStop(&packet->ip, true);
    break;
  case LAN_X_GET_LOCO_INFO:
    if (rec[5] == 0xF0) {  //DB0
      uint16_t addr = word(rec[6] & 0x3F, rec[7]);
      #ifdef DEBUG
      Serial.print("Get Loco Info Addr: ");
      Serial.println(addr);
      #endif
      setLocoAbo(packet->ip, addr);
      broadcastLocoInfo(addr, &packet->ip);
    }
    break;  
  case LAN_X_SET_LOCO:{
    uint16_t addr = word(rec[6] & 0x3F, rec[7]);

    if(rec[5] == LAN_X_SET_LOCO_FUNCTION){
      uint8_t func = rec[8] & B00111111; 
      uint8_t val = rec[8] >> 6;
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
      uint8_t speedSteps = 128;	//default value S=3; DCC 128 Fahrstufen
      if (rec[5] == 0x12){	//S=2; DCC 28 Fahrstufen
        speedSteps = 28;
      }
      else if (rec[5] == 0x10){	//S=0; DCC 14 Fahrstufen
        speedSteps = 14;
      }

      //the library always drives with the 128 step code, the speed steps are only
      //remembered to report the speed back in the format of the client
      bool dir;
      uint8_t speed = speedToDcc128(rec[8], speedSteps, dir);
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

bool MobaStation::startCv(bool write, uint16_t cv, uint8_t value, IPAddress *client){
  //the DCC library runs one programming track operation at a time, Z21 and
  //XpressNet share this lock so neither of them loses its answer
  if(waitForAck) return false;
  waitForAck = true;
  ackStartTime = millis();
  ackXNet = (client == NULL);
  if(client != NULL) ackClient = *client;
  ackCv = cv;
  cvWrite = write;
  writeVal = value;
  //Z21 and XpressNet count from 0, the library from 1
  if(write) DCC::writeCVByte(cv + 1, value, cvCallback);
  else DCC::readCV(cv + 1, cvCallback);
  return true;
}

void MobaStation::xnetReadCV(uint16_t cv){
  if(XpressNet == NULL) return;
  if(cv > 1023 || !startCv(false, cv, 0, NULL)) XpressNet->setCVNack();
}

void MobaStation::xnetWriteCV(uint16_t cv, uint8_t value){
  if(XpressNet == NULL) return;
  if(cv > 1023 || !startCv(true, cv, value, NULL)) XpressNet->setCVNack();
}

void MobaStation::sendCvResult(int16_t result){
  if(ackXNet){
    //the throttle polls for the result, the library keeps it until then
    if(XpressNet == NULL) return;
    if(result < 0) XpressNet->setCVNack();
    else XpressNet->setCVReadValue((uint8_t)ackCv, (uint8_t)result);
    return;
  }

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
}

void MobaStation::cvCallback(int16_t result){
  //already answered with a timeout, don't answer twice
  if(!waitForAck) return;

  if(cvWrite && result == 1){
    result = writeVal;
  }
  #ifdef DEBUG
  Serial.print("Callback: ");
  Serial.println(result);
  #endif

  sendCvResult(result);
  cvWrite = false;
  waitForAck = false;
}

uint8_t MobaStation::addClientToSlot(UdpPacket *packet, uint8_t BCFlag){
    IPAddress ip = packet->ip;
    if(ip == IPAddress()) return 0;

    uint8_t slot = z21clientMAX;

    for (uint8_t i = 0; i < z21clientMAX; i++){
        if(z21Clients[i].ip == ip){
            z21Clients[i].time = z21ActTimeIP;
            //the client may have switched between WiFi and Ethernet
            z21Clients[i].port = packet->port;
            z21Clients[i].iface = packet->iface;
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
    if(slot >= z21clientMAX) return 0; //no free client slot!
    z21Clients[slot].ip = ip;
    z21Clients[slot].port = packet->port;
    z21Clients[slot].iface = packet->iface;
    z21Clients[slot].time = z21ActTimeIP;

    if(BCFlag == 0){
        z21Clients[slot].BCFlag = getEEPROMBCFlag(ip);
    }
    else{
        z21Clients[slot].BCFlag = BCFlag;       
        saveEEPROMBCFlag(ip, BCFlag);
    }

    //tell only the new client the current power state, switching the
    //power again would also disturb the boosters and all other clients
    uint8_t data[2];
    sendToClient(slot, railStateMessage(data), LAN_X_HEADER, data, true);
    	
    return z21Clients[slot].BCFlag;   //BC Flag 4. Byte zurueckmelden
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

uint8_t MobaStation::getIPKey(IPAddress ip){
    //key for the broadcast flag store, 1..128 so it never collides with the
    //value of an erased EEPROM byte (0xFF)
    return (uint8_t)(((ip.operator[](0) ^ ip.operator[](1) ^ ip.operator[](2) ^ ip.operator[](3)) & 0x7F) + 1);
}

uint8_t MobaStation::getEEPROMBCFlag(IPAddress ip){
    //2 bytes per client: key of the IP + the broadcast flags
    uint16_t addr = CLIENTBCFLAGSTORE + ((uint16_t)ip.operator[](3) * 2);
    if(addr + 1 >= CLIENTBCFLAGSTORE + CLIENTSTORELENGTH) return 0;	//outside of the storage area
    if(addr + 1 >= FSTORAGE.length()) return 0;

    //another client used this address before
    if(FSTORAGE.read(addr) != getIPKey(ip)) return 0;

    return FSTORAGE.read(addr + 1);
}

void MobaStation::saveEEPROMBCFlag(IPAddress ip, uint8_t BCFlag){
    uint16_t addr = CLIENTBCFLAGSTORE + ((uint16_t)ip.operator[](3) * 2);
    if(addr + 1 >= CLIENTBCFLAGSTORE + CLIENTSTORELENGTH) return;	//outside of the storage area
    if(addr + 1 >= FSTORAGE.length()) return;

    uint8_t key = getIPKey(ip);
    //only write if something changed, EEPROM cells have a limited lifetime
    if(FSTORAGE.read(addr) != key || FSTORAGE.read(addr + 1) != BCFlag){
        FSTORAGE.FSTORAGEMODE(addr, key);
        FSTORAGE.FSTORAGEMODE(addr + 1, BCFlag);
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
      sendToClient(i, 0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(0x14, LAN_SYSTEMSTATE_DATACHANGED, data, false);
  }
}

void MobaStation::setRailPower(bool power, IPAddress *client){
  applyRailPower(power, client, true);
}

void MobaStation::applyRailPower(bool power, IPAddress *client, bool notifyXNet){
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

  //the library reports the new state back with notifyXNetPower(),
  //xnetPowerChanged() ignores it because railState is already set
  if(notifyXNet && XpressNet != NULL ){
    XpressNet->setPower(railState);
  }

  broadcastRailState(client);
}

void MobaStation::emergencyStop(IPAddress *client, bool notifyXNet){
  //speed 1 on loco address 0 is the DCC broadcast emergency stop, the track power stays on
  DCC::setThrottle(0, 1, 1);
  railState = csEmergencyStop;
  if(notifyXNet && XpressNet != NULL){
    XpressNet->setPower(csEmergencyStop);
  }
  broadcastRailState(client);
}

void MobaStation::xnetPowerChanged(uint8_t state){
  //our own change, reported back by the XpressNet library
  if(state == railState) return;

  switch(state){
    case csNormal:
      applyRailPower(true, NULL, false);
      break;
    case csTrackVoltageOff:
      applyRailPower(false, NULL, false);
      break;
    case csEmergencyStop:
      //a throttle wants all locos stopped, this is not a power off
      emergencyStop(NULL, false);
      break;
    default:
      //short circuit and service mode are only reported by the station itself
      break;
  }
}

//the DCC library switches the main track off on overload and tries again by itself,
//the clients and throttles only have to be told about it
void MobaStation::checkShortCircuit(){
  #ifndef ESP32
  if(!railPower) return;
  bool overload = DCCWaveform::mainTrack.getPowerMode() == POWERMODE::OVERLOAD;
  if(overload && railState != csShortCircuit){
    railState = csShortCircuit;
  }
  else if(!overload && railState == csShortCircuit){
    railState = csNormal;
  }
  else{
    return;
  }
  if(XpressNet != NULL){
    XpressNet->setPower(railState);
  }
  broadcastRailState();
  #endif
}

uint8_t MobaStation::railStateMessage(uint8_t *data){
  if(railState == csEmergencyStop){
    data[0] = LAN_X_BC_STOPPED;     //0x81 0x00
    data[1] = 0x00;
    return 0x07;
  }
  data[0] = LAN_X_BC_TRACK_POWER;
  switch(railState){
    case csNormal:        data[1] = 0x01; break;   //LAN_X_BC_TRACK_POWER_ON
    case csShortCircuit:  data[1] = 0x08; break;   //LAN_X_BC_TRACK_SHORT_CIRCUIT
    case csServiceMode:   data[1] = 0x02; break;   //LAN_X_BC_PROGRAMMING_MODE
    default:              data[1] = 0x00; break;   //LAN_X_BC_TRACK_POWER_OFF
  }
  return 0x07;
}

void MobaStation::broadcastRailState(IPAddress * force){
  uint8_t data[2];
  uint8_t len = railStateMessage(data);

  for(uint8_t i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].time > 0 && ((Z21bcAll_s & z21Clients[i].BCFlag) > 0 || (force != NULL && *force == z21Clients[i].ip))){
      #ifdef DEBUG
      Serial.print("Send RailState to ");
      Serial.println(z21Clients[i].ip);
      #endif
      sendToClient(i, len, LAN_X_HEADER, data, true);
    }
  }
  if(broadcastEventHandler != NULL){
    broadcastEventHandler(len, LAN_X_HEADER, data, true);
  }
}

uint8_t MobaStation::getLocoInfo(uint16_t addr, uint8_t *data){
  
  data[0] = LAN_X_LOCO_INFO;
  data[1] = (addr >> 8) & 0x3F;
	data[2] = addr & 0xFF;

  uint8_t speedSteps = DCC::getSpeedSteps(addr);
  uint8_t speedStepsInfo;

  if(speedSteps == 14){
    speedStepsInfo = 0;
  }
  else if(speedSteps == 28){
    speedStepsInfo = 2;
  }
  else{
    speedSteps = 128;   //unknown loco
    speedStepsInfo = 4;//default value; DCC 128 Fahrstufen
  }

  data[3] = ( speedStepsInfo | 0x08); //BUSY!

  uint8_t speed = DCC::getThrottleSpeed(addr);
  bool dir = DCC::getThrottleDirection(addr);

  //the library stores the 128 step code, the client expects the format it drives with
  data[4] = speedFromDcc128(speed, dir, speedSteps);

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
  uint8_t slot = getClientId(client);
  if(slot >= z21clientMAX || locoAddr == 0) return;
  uint16_t *abo = z21Clients[slot].locoAbo;
  for(uint8_t j = 0; j < CLIENT_MAX_LOCO_ABO; j++){
    if(abo[j] == locoAddr) return;
    if(abo[j] == 0){
      abo[j] = locoAddr;
      return;
    }
  }
  //all slots used: like the Z21 the oldest subscription is dropped (FIFO)
  for(uint8_t j = 1; j < CLIENT_MAX_LOCO_ABO; j++){
    abo[j - 1] = abo[j];
  }
  abo[CLIENT_MAX_LOCO_ABO - 1] = locoAddr;
}

void MobaStation::broadcastLocoInfo(uint16_t addr, IPAddress *force){
  uint8_t data[9];
  uint8_t len = getLocoInfo(addr, data) + 5;
  for(uint8_t i = 0; i < z21clientMAX; i++){
    if(z21Clients[i].time == 0) continue;
    bool send = (force != NULL && *force == z21Clients[i].ip);
    //Z21bcNetAll: PC control software gets every loco without subscribing
    if(!send && (Z21bcNetAll_s & z21Clients[i].BCFlag) > 0){
      send = true;
    }
    if(!send && (Z21bcAll_s & z21Clients[i].BCFlag) > 0){
      for (uint8_t j = 0; j < CLIENT_MAX_LOCO_ABO; j++){
        if(z21Clients[i].locoAbo[j] == addr){
          send = true;
          break;
        }
      }
    }
    if(send){
      #ifdef DEBUG
      Serial.print("Send Loco Info to ");
      Serial.println(z21Clients[i].ip);
      #endif
      sendToClient(i, len, LAN_X_HEADER, data, true);
    }
  }

  if(broadcastEventHandler != NULL){
    broadcastEventHandler(len, LAN_X_HEADER, data, true);
  }

  if(XpressNet != NULL){
    unsigned long func = DCC::getFns(addr);
    uint8_t speedStepsInfo = Loco128; //DCC library always reports speed in 128 steps -> wire must say 128 too

    uint8_t speed = DCC::getThrottleSpeed(addr);
    speed = DCC::getThrottleDirection(addr) << 7 | speed;
    for(int i = 0; i < MAX_XPRESSNET_CLIENTS; i++){
      //the loco info frame carries no address, the throttle applies it to the loco it shows,
      //so only the throttles that control this loco may get it
      if(xpressNetClients[i].client > 0 && xpressNetClients[i].loco == addr){
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
  if(group >= RBUS_GROUPS) return;
  uint8_t idx = group * RBUS_GROUP_SIZE;
  for(uint8_t i = 0; i < RBUS_GROUP_SIZE; i++){
    data[i+1] = rBus[idx + i];
  }
}

void MobaStation::setTurnoutMode(TurnoutMode mode, uint16_t addrRange){
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
  uint8_t speedStepsInfo = Loco128; //speed is always reported in 128 steps by the DCC library
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
  uint8_t speedStepsInfo = Loco128; //speed is always reported in 128 steps by the DCC library
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
  //power on/off and emergency stop are separate states, the guard against
  //the echo of our own changes is in xnetPowerChanged()
  MobaStation::xnetPowerChanged(State);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S14:");
  Serial.println(Speed, BIN);
  #endif
  bool dir;
  uint8_t speed = speedToDcc128(Speed, 14, dir);
  DCC::setThrottle(Address, speed, 14, dir);
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive27(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S27:");
  Serial.println(Speed, BIN);
  #endif
  //27 steps use the 28 step format, the decoder just never gets the highest step
  bool dir;
  uint8_t speed = speedToDcc128(Speed, 28, dir);
  DCC::setThrottle(Address, speed, 28, dir);
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S28:");
  Serial.println(Speed, BIN);
  #endif
  bool dir;
  uint8_t speed = speedToDcc128(Speed, 28, dir);
  DCC::setThrottle(Address, speed, 28, dir);
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S128:");
  Serial.println(Speed, BIN);
  #endif
  bool dir;
  uint8_t speed = speedToDcc128(Speed, 128, dir);
  DCC::setThrottle(Address, speed, 128, dir);
  MobaStation::broadcastLocoInfo(Address);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {
  #ifdef DEBUG
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F1:");
  Serial.println(Func1, BIN);
  #endif
  //Gruppe 1: 0 0 0 F0 F4 F3 F2 F1
  DCC::setFn(Address, 0, bitRead(Func1, 4));
  for(int i = 0; i < 4; i++){
    DCC::setFn(Address, i+1, bitRead(Func1, i));
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
    //data = 0000 ABBP: A = Spulenspannung EIN, P = Ausgang (0 = gerade, 1 = abzweigen)
    bool dir = data & B00000001;
    bool power = bitRead(data, 3);
    //XpressNet counts from 0 (AAAAAAAA << 2 | BB), the station from 1 like the Z21 path
    MobaStation::setTurnout(Address + 1, dir, power);
  }
}

void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data) {
  //Address = group of 4 turnouts, N (bit 0 of data) selects the lower or upper pair
  uint16_t first = (uint16_t)Address * 4 + (data & 0x01) * 2 + 1;   //turnout number from 1
  //ITTN ZZZZ: Z1Z0 = first turnout, Z3Z2 = second, 01 = position 0, 10 = position 1,
  //the encoding is the same as the Z21 one of getTurnoutInfo()
  byte pos = (data & 0x01) << 4;
  pos |= MobaStation::getTurnoutInfo(first);
  pos |= MobaStation::getTurnoutInfo(first + 1) << 2;
  bitWrite(pos, 7, 1);  //command completed!
  MobaStation::XpressNet->SetTrntStatus(UserOps, Address, pos);
}
//----------   Xpress-Net CV / POM   ----------

//0x22 0x15 Direct Mode CV read request, cvAdr is 0 based
void notifyXNetDirectReadCV(uint16_t cvAdr){
  #ifdef DEBUG
  Serial.print("XNet CV read: ");
  Serial.println(cvAdr + 1);
  #endif
  //shares the programming track lock with the Z21 clients
  MobaStation::xnetReadCV(cvAdr);
}

//0x23 0x16 Direct Mode CV write request, cvAdr is 0 based
void notifyXNetDirectCV(uint16_t cvAdr, uint8_t data){
  #ifdef DEBUG
  Serial.print("XNet CV write: ");
  Serial.print(cvAdr + 1);
  Serial.print(" = ");
  Serial.println(data);
  #endif
  MobaStation::xnetWriteCV(cvAdr, data);
}

//POM (Operations Mode) write byte
void notifyXNetPOMwriteByte(uint16_t Adr, uint16_t CV, uint8_t data){
  #ifdef DEBUG
  Serial.print("XNet POM byte: Addr ");
  Serial.print(Adr);
  Serial.print(" CV ");
  Serial.print(CV);
  Serial.print(" = ");
  Serial.println(data);
  #endif
  DCC::writeCVByteMain(Adr, CV, data);
}

//POM (Operations Mode) write bit, data = 0 0 0 P MMM CCCC (P = value, CCCC = bit number)
void notifyXNetPOMwriteBit(uint16_t Adr, uint16_t CV, uint8_t data){
  #ifdef DEBUG
  Serial.print("XNet POM bit: Addr ");
  Serial.print(Adr);
  Serial.print(" CV ");
  Serial.print(CV);
  Serial.print(" bit ");
  Serial.println(data & B00001111);
  #endif
  uint8_t bitNum = data & B00000111;
  bool value = bitRead(data, 3);
  DCC::writeCVBitMain(Adr, CV, bitNum, value);
}
