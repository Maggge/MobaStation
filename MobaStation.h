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

#ifndef __MobaStation__
#define __MobaStation__

#include <Arduino.h>
#include <Udp.h>

#if __has_include ( "config.h")
  #include "config.h"
#else
  #warning config.h not found. Using defaults from config.example.h 
  #include "config.example.h"
#endif

#ifndef TURNOUTS
#define TURNOUT TURNOUT_MobaBus
#endif
#ifndef DCC_TURNOUT_ADDRESS_RANGE
#define DCC_TURNOUT_ADDRESS_RANGE 2047
#endif

#if defined(__arm__)
#include <DueFlashStorage.h>
DueFlashStorage FlashStore;
#define FSTORAGE 	FlashStore
#define FSTORAGEMODE write
#else
// AVR based Boards
#include <EEPROM.h>
#define FSTORAGE 	EEPROM
	#if defined(ESP8266) || defined(ESP32) //ESP8266 or ESP32
		#define FSTORAGEMODE write
	#else
		#define FSTORAGEMODE update
	#endif
#endif

#include "MobaStation_Protocol.h"
#include "Eth_Interface.h"


#include "DCC.h"
#include "DCCWaveform.h"


#include <MobaBus.h>

#ifdef XPRESSNET
#include <XpressNetMaster.h>
#endif

//Firmware-Version der Z21:
#define z21FWVersionMSB 0x01
#define z21FWVersionLSB 0x40

//Hardware-Typ: 0x00000211 // 10870 �Z21 XL Series� (ab 2020) 
#define z21HWTypeMSB 0x02
#define z21HWTypeLSB 0x11
//Seriennummer inside EEPROM:
#define CONFz21SnMSB 0		//0x01
#define CONFz21SnLSB 1		//0xE8

//Store Z21 configuration inside EEPROM:
#define CONF1STORE 50 	//(10x Byte)	- Prog, RailCom, etc.
#define CONF2STORE 60	//(15x Byte)	- Voltage: Prog, Rail, etc.

#define CLIENTBCFLAGSTORE 100			//Start where Client-Hash is stored
#define CLIENTSTORELENGTH 255		//Length of Client Broadcast-Flag store


//--------------------------------------------------------------
#define z21clientMAX 20        //Speichergr��e f�r IP-Adressen
#define z21ActTimeIP 20    //Aktivhaltung einer IP f�r (sec./2)
#define z21IPinterval 2000   //interval at milliseconds
#define z21InfoInterval 2000 //Interval for broadcast System Info

class MobaStation
{
private:
    static Eth_Interface *Eth;
    
    static uint16_t _port;

    static uint32_t z21IPlastIPcheck; //store the last time of check ip's active
    static uint32_t z21LastInfoBroadcast;
    static IPClient z21Clients[z21clientMAX];

    static uint8_t buffer[Z21_MAX_UDP_SIZE];

    static MobaBus *mobaBus;

    static uint8_t xpressPin;

    static void processPacket(UdpPacket *packet);

    static uint8_t addClientToSlot(IPAddress ip, uint8_t BCFlag);
  
    static void clearClient (uint8_t pos);
    static void clearIPSlots();
    static void clearIPSlot(IPAddress ip);

    static uint32_t getz21BcFlag (uint8_t flag);
    static uint8_t getLocalBcFlag (uint32_t flag);

    static bool driveOnProg;
    static bool railPower;
    static uint8_t railState;

    static uint32_t ledLastBlink;

    static uint32_t pwrBtnLast;
    static bool pwrBtnState;

    static uint8_t rBus[20];

    static void rBusGetData(uint8_t group, uint8_t *data);
    static void broadcastRbus(uint8_t group);

    static void broadcastRailState(IPAddress *force = NULL);
    static void sendRailState(IPAddress ip);

    static uint8_t getLocoInfo(uint16_t addr, uint8_t *data);
    static void setLocoAbo(IPAddress client, uint16_t locoAddr);

    static void processMobaBus();

    static void broadcastTurnout(uint16_t addr, uint8_t dir, IPAddress *force = NULL);

    static void broadcastSystemInfo(IPAddress *force = NULL);

    static uint8_t getEEPROMBCFlag(IPAddress ip);
    static void saveEEPROMBCFlag(IPAddress ip, uint8_t BCFlag);

    static UdpPacket pkgBuffer;

    static void processLanXPacket(UdpPacket *packet);

    //Ackmanager
    static IPAddress ackClient;
    static bool waitForAck;
    static int16_t ackCv;
    static uint8_t writeVal;
    static bool cvWrite;
    static void cvCallback(int16_t result);

public:
    #ifdef XPRESSNET
    static XpressNetMasterClass *XpressNet;
    static XpressNetClient xpressNetClients[MAX_XPRESSNET_CLIENTS];

    static void addXpressNetClient(uint8_t client, uint16_t loco);
    static void attachXpressNet(XpressNetMasterClass *xpressNet, uint8_t sendReceivePin);
    #endif

    static void attachMobaBus(MobaBus *m);    

    static void begin(Eth_Interface *eth, const char * ip = NULL, bool driveOnProg = false);

    static void loop();

    static void driveOnProgTrack(bool activate);

    static void setRailPower(bool power, IPAddress *client = NULL);

    static uint8_t getClientId(IPAddress ip);

    static IPAddress getIP(uint8_t clientID);

    static void broadcastLocoInfo(uint16_t addr, IPAddress *force = NULL);

};

#endif