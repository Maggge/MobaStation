/**
  MobaStation config file

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

/**********************************************************************

Copy this file into the folder of the MobaStation library (next to MobaStation.h) and
rename it to "MobaStationConfig.h", MobaStation.h picks it up automatically.
It is included once through MobaStationUserConfig.h, it should only contain #defines.

Every setting is optional, the value in the comment is the default.
The example sketches take all their settings (WiFi, Ethernet, motor shield,
MobaBus, XpressNet) from this file, they need no changes.

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////
//
// HARDWARE (used by the example sketches to build the objects)
//
// motor shield of the DCC library (MotorDrivers.h), eg. STANDARD_MOTOR_SHIELD
//#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD
//
// 1 = trains can also drive on the programming track
//#define DRIVE_ON_PROG 1
//
// MobaBus over CAN with a MCP2515 module, 0 = no MobaBus
// (without a CAN module connected it must be 0)
//#define MOBABUS_ENABLED 1
//#define MOBABUS_EEPROM 1000
//
// programming button (to GND) and status LED of MobaBus. Never use 8 and 9 with the
// Arduino Motor Shield, these are its brake pins: pin 8 reads as a pressed button and
// MobaBus erases the whole EEPROM after 10 seconds
//#define MOBABUS_PROG_PIN 26
//#define MOBABUS_LED_PIN 27
//#define MOBABUS_CAN_CS 53
//#define MOBABUS_CAN_INT 21
//#define MOBABUS_CAN_SPEED CAN_250KBPS
//#define MOBABUS_CAN_CLOCK MCP_8MHZ
//
// XpressNet with a MAX485 on Serial1 (TX1/RX1), 0 = no XpressNet
//#define XPRESSNET_ENABLED 1
//#define XPRESSNET_CONTROL_PIN 22

/////////////////////////////////////////////////////////////////////////////////
//
// ETHERNET MODULE (example MobaStationEth)
//
// chip select pin of the W5x00
//#define ETH_CS_PIN 49
//
// MAC address, must be unique in your network
//#define ETH_MAC 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED

/////////////////////////////////////////////////////////////////////////////////
//
// WIFI (example MobaStationWiFi, ESP8266 with AT firmware 1.7)
//
// your router. Quotes, commas and backslashes are not possible, they break the AT command
//#define WIFI_SSID "ssid"
//#define WIFI_PWD "password"
//
// Station_Mode, SoftAP_Mode or Hybrid_MODE (station, own accesspoint as fallback)
//#define WIFI_MODE Hybrid_MODE
//
// serial port of the ESP8266, Serial3 on the Mega + WiFi R3 boards
//#define WIFI_SERIAL Serial3

/////////////////////////////////////////////////////////////////////////////////
//
// NETWORK (used by MobaStation::attachEthInterface(interface))
//
// addresses are written with commas: 192,168,0,111
//
// 1 = get the address from the router (DHCP). The address is printed on the
// serial console at the start, NETWORK_IP is used when no DHCP server answers.
// A fixed address is easier for the Z21 apps, you can also reserve one in the router.
//#define NETWORK_DHCP 0
//
// static address, with DHCP the fallback. The WiFi accesspoint (SoftAP mode and
// hybrid fallback) always uses this address
//#define NETWORK_IP 192,168,0,111
//#define NETWORK_SUBNET 255,255,255,0
//
// 0,0,0,0 = first address of the subnet (192.168.0.1), the Z21 protocol needs no gateway
//#define NETWORK_GATEWAY 0,0,0,0
//
// how long the Ethernet shield waits for a DHCP server, this blocks
// at the start and when the cable is plugged in again
//#define ETH_DHCP_TIMEOUT 10000

/////////////////////////////////////////////////////////////////////////////////
//
// TRACK POWER BUTTON AND STATUS LED
//
// the button must connect the pin to GND
//#define POWER_BUTTON 24
//#define POWER_LED 25

/////////////////////////////////////////////////////////////////////////////////
//
// TRACK VOLTAGE MEASUREMENT
//
// analog pin of the voltage divider, A15 on the Arduino Mega
//#define VOLTAGE_SENSE_PIN A15
//
// multiplier that converts the analog reading into millivolts,
// it depends on the resistors of your voltage divider
//#define VOLTAGE_MULTIPLIER 25.58f

/////////////////////////////////////////////////////////////////////////////////
//
// TURNOUT SETTINGS
//
// how turnouts are driven: MobaBus_Only, DCC_Only or Combined
// can also be changed at runtime with MobaStation::setTurnoutMode()
//#define DEFAULT_TURNOUT_MODE Combined
//
// in Combined mode all turnouts up to this address are driven by DCC,
// the addresses above by MobaBus. With 2047 MobaBus gets no turnout commands at all,
// eg. 1000: DCC decoders 1-1000, MobaBus modules (also local ones) from 1001. Max = 2047
//#define DCC_TURNOUT_ADDRESS_RANGE 2047

/////////////////////////////////////////////////////////////////////////////////
//
// MEMORY LAYOUT AND SIZES
//
// EEPROM address of the MobaStation configuration
// the client broadcast flags use 3000-3511, the MobaBus module storage must
// not overlap these areas (the examples use 1000 for MobaBus)
//#define CONFIG_STORAGE 0
//
// maximum number of LAN clients, each client uses 43 bytes of RAM
//#define z21clientMAX 20
//
// number of Ethernet/WiFi interfaces that can be attached
//#define ETH_INTERFACES 2
//
// number of throttles on the XpressNet
//#define MAX_XPRESSNET_CLIENTS 5

/////////////////////////////////////////////////////////////////////////////////
//
// WIFI RECONNECT
//
// timeout of the AT+CWJAP while MobaStation is started
//#define WIFI_CONNECT_TIMEOUT 16000
//
// time between two wifi reconnect attempts, the reconnect never blocks
//#define WIFI_RECONNECT_INTERVAL 5000
//
// how long a single AT+CWJAP of a reconnect may take
//#define WIFI_RECONNECT_JOIN_TIMEOUT 12000
//
// hybrid mode: after this many failed reconnect rounds (3 joins each) the own
// accesspoint is started next to the station, the station is still searched
//#define WIFI_FALLBACK_ROUNDS 3
//
// time between two station attempts while the fallback accesspoint runs,
// every attempt makes the accesspoint unreachable for a moment
//#define WIFI_FALLBACK_RECONNECT_INTERVAL 60000

/////////////////////////////////////////////////////////////////////////////////
//
// WIFI ACCESSPOINT (SoftAP mode and hybrid fallback)
//
// the accesspoint uses the ip address of MobaStation, change the password,
// the default one is public. The password needs at least 8 characters.
//#define WIFI_AP_SSID "MobaStation"
// only letters and digits, the password is passed inside an AT command
//#define WIFI_AP_PWD "MobaStation"

/////////////////////////////////////////////////////////////////////////////////
//
// DCC TURNOUT STATE
//
// the last position of the first turnouts is kept to answer turnout info requests,
// 2 bit per turnout (512 turnouts = 128 bytes RAM)
//#define TURNOUT_STATE_COUNT 512
