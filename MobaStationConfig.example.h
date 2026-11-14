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

Every setting is optional, the value in the comment is the default.
The motor shield and whether the prog track is driven are defined in the
sketch itself (MOTOR_SHIELD_TYPE and DRIVE_ON_PROG), the XpressNet is
attached at runtime with MobaStation::attachXpressNet().

**********************************************************************/

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
// the addresses above by MobaBus. Max = 2047
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
// maximum number of LAN clients, each client uses 38 bytes of RAM
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
