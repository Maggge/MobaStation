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

#include <XpressNetMaster.h>

//Optional user configuration: copy MobaStationConfig.example.h to MobaStationConfig.h in this
//(the library) folder to change pins and default settings without editing the library
#if defined(__has_include)
#  if __has_include("MobaStationConfig.h")
#    include "MobaStationConfig.h"
#  endif
#endif


//Config Storage inside EEPROM
#ifndef CONFIG_STORAGE
#define CONFIG_STORAGE 0 //EEPROM Address where the config is stored
#endif


#ifndef POWER_BUTTON
#define POWER_BUTTON 24
#endif
#ifndef POWER_LED
#define POWER_LED 25
#endif
#ifndef VOLTAGE_SENSE_PIN
#define VOLTAGE_SENSE_PIN A15
#endif
//multiplier to get millivolts from the analog reading of the voltage divider
#ifndef VOLTAGE_MULTIPLIER
#define VOLTAGE_MULTIPLIER 25.58f
#endif

//default turnout routing, can be changed with MobaStation::setTurnoutMode()
#ifndef DEFAULT_TURNOUT_MODE
#define DEFAULT_TURNOUT_MODE Combined
#endif
//in Combined mode all turnouts up to this address are handled by DCC, the ones above by MobaBus
#ifndef DCC_TURNOUT_ADDRESS_RANGE
#define DCC_TURNOUT_ADDRESS_RANGE 2047
#endif

//Firmware-Version der Z21:
#define z21FWVersionMSB 0x01
#define z21FWVersionLSB 0x40

//Hardware-Typ: 0x00000211 // 10870 �Z21 XL Series� (ab 2020) 
#define z21HWTypeMSB 0x02
#define z21HWTypeLSB 0x11
//Seriennummer:
#define CONFz21SnMSB 0		//0x01
#define CONFz21SnLSB 1		//0xE8

//Store Z21 configuration inside EEPROM:
#define CONF1STORE 50 	//(10x Byte)	- Prog, RailCom, etc.
#define CONF2STORE 60	//(15x Byte)	- Voltage: Prog, Rail, etc.

#define CLIENTBCFLAGSTORE 3000			//Start where Client-Hash is stored
#define CLIENTSTORELENGTH 512		//Length of Client Broadcast-Flag store (2 byte per client)


//--------------------------------------------------------------
#ifndef z21clientMAX
#define z21clientMAX 20        //Speichergr��e f�r IP-Adressen
#endif
#define z21ActTimeIP 20    //Aktivhaltung einer IP f�r (sec./2)
#define z21IPinterval 2000   //interval at milliseconds
#define z21InfoInterval 2000 //Interval for broadcast System Info

#ifndef ETH_INTERFACES
#define ETH_INTERFACES 2
#endif

//R-Bus feedback: groups of 10 modules with 8 inputs each, the Z21 protocol knows group 0 and 1
#define RBUS_GROUPS 2
#define RBUS_GROUP_SIZE 10

#ifndef MAX_XPRESSNET_CLIENTS
#define MAX_XPRESSNET_CLIENTS 5
#endif

//after this time a CV read/write on the programming track is aborted if the
//DCC library did not report a result, so the programming track stays usable
#ifndef CV_ACK_TIMEOUT
#define CV_ACK_TIMEOUT 10000
#endif

//the last position of the DCC turnouts is kept to answer turnout info requests,
//2 bit per turnout: 512 turnouts = 128 bytes RAM. Turnouts above report "unknown"
#ifndef TURNOUT_STATE_COUNT
#define TURNOUT_STATE_COUNT 512
#endif


enum TurnoutMode{
    MobaBus_Only,
    DCC_Only,
    Combined,   
};


class MobaStation
{
private:
    static Eth_Interface *Eth[ETH_INTERFACES];
    
    static uint16_t _port;

    static TurnoutMode turnoutMode;
    static uint16_t dccAddrRange; 

    static float voltageMulti;

    static uint32_t z21IPlastIPcheck; //store the last time of check ip's active
    static uint32_t z21LastInfoBroadcast;
    static IPClient z21Clients[z21clientMAX];

    static MobaBus *mobaBus;

    static uint8_t xpressPin;

    //registers the sender of the packet (ip, port, interface), returns its broadcast flags
    static uint8_t addClientToSlot(UdpPacket *packet, uint8_t BCFlag);
  
    static void clearClient (uint8_t pos);
    static void clearIPSlots();
    static void clearIPSlot(IPAddress ip);

    static uint32_t getz21BcFlag (uint8_t flag);
    static uint8_t getLocalBcFlag (uint32_t flag);

    static bool railPower;
    static uint8_t railState;

    static uint32_t ledLastBlink;

    static uint32_t pwrBtnLast;
    static bool pwrBtnState;

    static uint8_t rBus[RBUS_GROUPS * RBUS_GROUP_SIZE];

    //2 bit per turnout in the Z21 format: 0 = unknown, 1 = position 0, 2 = position 1
    static uint8_t turnoutStates[(TURNOUT_STATE_COUNT + 3) / 4];
    static void storeTurnout(uint16_t addr, uint8_t dir);

    //answer to one client, it is reached over the interface and port it used
    static void sendLanPacket(IPAddress client , uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR);
    static void sendToClient(uint8_t slot, uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR);

    static void rBusGetData(uint8_t group, uint8_t *data);
    static void broadcastRbus(uint8_t group);

    //the LAN_X message that describes railState, returns the record length
    static uint8_t railStateMessage(uint8_t *data);
    static void broadcastRailState(IPAddress *force = NULL);
    //switch the track power, notifyXNet = false when the change came from the XpressNet
    static void applyRailPower(bool power, IPAddress *client, bool notifyXNet);
    static void emergencyStop(IPAddress *client, bool notifyXNet);
    static void checkShortCircuit();

    static uint8_t getLocoInfo(uint16_t addr, uint8_t *data);
    static void setLocoAbo(IPAddress client, uint16_t locoAddr);

    static void processMobaBus();

    static void broadcastTurnout(uint16_t addr, uint8_t dir, IPAddress *force = NULL);

    static void broadcastSystemInfo(IPAddress *force = NULL);

    static uint8_t getIPKey(IPAddress ip);
    static uint8_t getEEPROMBCFlag(IPAddress ip);
    static void saveEEPROMBCFlag(IPAddress ip, uint8_t BCFlag);

    static UdpPacket pkgBuffer;

    //rec is one zero padded Z21 record of a received datagram
    static void processRecord(UdpPacket *packet, uint8_t *rec, uint16_t len);
    static void processLanXPacket(UdpPacket *packet, uint8_t *rec);


    //Ackmanager, shared by the Z21 clients and the XpressNet
    static IPAddress ackClient;
    static bool ackXNet;            //the running CV operation was started by the XpressNet
    static bool waitForAck;
    static uint32_t ackStartTime;
    static int16_t ackCv;           //0 based
    static uint8_t writeVal;
    static bool cvWrite;
    //start a CV read/write on the programming track, cv is 0 based,
    //client == NULL for the XpressNet. Returns false while another operation runs
    static bool startCv(bool write, uint16_t cv, uint8_t value, IPAddress *client);
    static void sendCvResult(int16_t result);
    static void cvCallback(int16_t result);

public:

    static void (*broadcastEventHandler)(uint16_t DataLen, uint16_t Header, byte *data, boolean withXOR);

    static void processPacket(UdpPacket *packet);

    static XpressNetMasterClass *XpressNet;
    static XpressNetClient xpressNetClients[MAX_XPRESSNET_CLIENTS];

    static void assembleData(uint16_t DataLen, uint16_t Header, byte *dataString, boolean withXOR, uint8_t * dataOutput);

    static void addXpressNetClient(uint8_t client, uint16_t loco);
    static void attachXpressNet(XpressNetMasterClass *xpressNet, uint8_t sendReceivePin);

    static bool attachEthInterface(Eth_Interface *eth, IPAddress ip, uint16_t port, IPAddress subnetMask);

    static void attachMobaBus(MobaBus *m);    

    static void setTurnout(uint16_t addr, bool dir, bool power);

    /**
     * begin - initialize
     * 
     * @param motorShieldName 
     * @param mainDriver 
     * @param progDriver 
     * @param driveOnProg 
     */
    static void begin(const FSH * motorShieldName, MotorDriver *mainDriver, MotorDriver *progDriver, bool driveOnProg = false);

    static void loop();

    static void driveOnProgTrack(bool activate);

    static void setRailPower(bool power, IPAddress *client = NULL);

    //called by the XpressNet library when a throttle changed the power state
    static void xnetPowerChanged(uint8_t state);

    //turnout position in the Z21 format: 0 = unknown, 1 = position 0, 2 = position 1
    static uint8_t getTurnoutInfo(uint16_t addr);

    //CV read/write requested by an XpressNet throttle, cv is 0 based
    static void xnetReadCV(uint16_t cv);
    static void xnetWriteCV(uint16_t cv, uint8_t value);

    static uint8_t getClientId(IPAddress ip);

    static IPAddress getIP(uint8_t clientID);

    static void broadcastLocoInfo(uint16_t addr, IPAddress *force = NULL);

    static void setTurnoutMode(TurnoutMode mode, uint16_t addrRange = 2047);

    static void setVoltageMultiplier(float multi);

};

#endif