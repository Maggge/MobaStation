/**
  MobaStation Protocol

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

#ifndef __MOBASTATION_Protocol__
#define __MOBASTATION_Protocol__

#include <Arduino.h>
#include <IPAddress.h>

#define Z21_STANDARD_PORT 21105

#define WIFI_CONNECT_TIMEOUT 16000

#define Z21_MAX_UDP_SIZE 15

//------------   Z21 Header   ------------
#define LAN_GET_SERIAL_NUMBER        0x10
#define LAN_LOGOFF                   0x30
#define LAN_X_HEADER                 0x40
#define LAN_SET_BROADCASTFLAGS       0x50
#define LAN_GET_BROADCASTFLAGS       0x51
#define LAN_SYSTEMSTATE_DATACHANGED  0x84
#define LAN_SYSTEMSTATE_GETDATA      0x85
#define LAN_GET_HWINFO               0x1A
#define LAN_GET_CODE                 0x18
#define LAN_GET_LOCOMODE             0x60
#define LAN_SET_LOCOMODE             0x61
#define LAN_GET_TURNOUTMODE          0x70
#define LAN_SET_TURNOUTMODE          0x71
#define LAN_RMBUS_DATACHANGED        0x80
#define LAN_RMBUS_GETDATA            0x81
#define LAN_RMBUS_PROGRAMMODULE      0x82
#define LAN_RAILCOM_DATACHANGED      0x88
#define LAN_RAILCOM_GETDATA          0X89 
#define LAN_LOCONET_Z21_RX           0xA0
#define LAN_LOCONET_Z21_TX           0xA1
#define LAN_LOCONET_FROM_LAN         0xA2
#define LAN_LOCONET_DISPATCH_ADDR    0xA3
#define LAN_LOCONET_DETECTOR         0xA4
#define LAN_CAN_DETECTOR 			 0xC4

//-----------   Z21 X-Header   -----------
#define LAN_X_GET_SETTING            0x21  
#define LAN_X_BC_TRACK_POWER         0x61
#define LAN_X_UNKNOWN_COMMAND        0x61
#define LAN_X_STATUS_CHANGED         0x62
#define LAN_X_GET_VERSION			 0x63
#define LAN_X_SET_STOP               0x80
#define LAN_X_BC_STOPPED             0x81
#define LAN_X_GET_FIRMWARE_VERSION   0xF1
#define LAN_X_GET_LOCO_INFO          0xE3
#define LAN_X_SET_LOCO               0xE4
#define LAN_X_SET_LOCO_FUNCTION      0xF8 
#define LAN_X_LOCO_INFO              0xEF
#define LAN_X_GET_TURNOUT_INFO       0x43 
#define LAN_X_SET_TURNOUT            0x53
#define LAN_X_TURNOUT_INFO           0x43 
#define LAN_X_SET_EXT_ACCESSORY		 0x54
#define LAN_X_GET_EXT_ACCESSORY_INFO 0x44
#define LAN_X_CV_READ                0x23
#define LAN_X_CV_WRITE               0x24
#define LAN_X_CV_NACK_SC             0x61
#define LAN_X_CV_NACK                0x61
#define LAN_X_CV_RESULT              0x64
#define LAN_X_CV_POM                 0xE6 
#define LAN_X_MM_WRITE_BYTE          0x24
#define LAN_X_DCC_READ_REGISTER      0x22
#define LAN_X_DCC_WRITE_REGISTER     0x23

//**************************************************************
//Z21 BC Flags
#define Z21bcNone                B00000000
#define Z21bcAll    		0x00000001
#define Z21bcAll_s               B00000001
#define Z21bcRBus   		0x00000002
#define Z21bcRBus_s              B00000010
#define Z21bcRailcom    	0x00000004    //RailCom-Daten für Abo Loks
#define Z21bcRailcom_s			0x100

#define Z21bcSystemInfo 	0x00000100	//LAN_SYSTEMSTATE_DATACHANGED
#define Z21bcSystemInfo_s        B00000100

//ab FW Version 1.20:
#define Z21bcNetAll          0x00010000 // Alles, auch alle Loks ohne vorher die Lokadresse abonnieren zu müssen (für PC Steuerung)
#define Z21bcNetAll_s            B00001000

#define Z21bcLocoNet         0x01000000 // LocoNet Meldungen an LAN Client weiterleiten (ohne Loks und Weichen)
#define Z21bcLocoNet_s           B00010000
#define Z21bcLocoNetLocos    0x02000000 // Lok-spezifische LocoNet Meldungen an LAN Client weiterleiten
#define Z21bcLocoNetLocos_s      B00100000
#define Z21bcLocoNetSwitches 0x04000000 // Weichen-spezifische LocoNet Meldungen an LAN Client weiterleiten
#define Z21bcLocoNetSwitches_s   B01000000

//ab FW Version 1.22:
#define Z21bcLocoNetGBM      0x08000000  //Status-Meldungen von Gleisbesetztmeldern am LocoNet-Bus
#define Z21bcLocoNetGBM_s        B10000000

//ab FW Version 1.29:
#define Z21bcRailComAll		 0x00040000 //alles: Änderungen bei RailCom-Daten ohne Lok Abo! -> LAN_RAILCOM_DATACHANGED
#define Z21bcRailComAll_s		 	0x200

//ab FW Version 1.30:
#define Z21bcCANDetector	 0x00080000	//Meldungen vom Gelisbesetztmeldern am CAN-Bus
#define Z21bcCANDetector_s			0x400

//certain global XPressnet status indicators:
#define csNormal 0x00 			// Normal Operation Resumed ist eingeschaltet
#define csEmergencyStop 0x01	// Der Nothalt ist eingeschaltet
#define csTrackVoltageOff 0x02  // Die Gleisspannung ist abgeschaltet
#define csShortCircuit 0x04 	// Kurzschluss
#define csServiceMode 0x08 		// Der Programmiermodus ist aktiv - Service Mode
//Bitmask CentralStateEx:
#define cseHighTemperature  0x01 // zu hohe Temperatur 
#define csePowerLost  0x02 // zu geringe Eingangsspannung 
#define cseShortCircuitExternal 0x04 // am externen Booster-Ausgang 
#define cseShortCircuitInternal 0x08 // am Hauptgleis oder Programmiergleis 

#define CLIENT_MAX_LOCO_ABO 16
#define LOCO_SLOTS 50

struct UdpPacket{
    IPAddress ip;
    uint16_t dataLength;
    uint8_t data[Z21_MAX_UDP_SIZE];
};

struct IPClient{
    IPAddress ip;
    uint8_t BCFlag;
    uint8_t time;
    uint16_t locoAbo[CLIENT_MAX_LOCO_ABO];
};

enum TURNOUT_MODE{
    TURNOUT_MobaBus,
    TURNOUT_DCC,
    TURNOUT_HYBRID,
};

struct XpressNetClient{
    uint8_t client;
    uint16_t loco;
};

#endif