/**
  MobaStation config file

  © 2021, Markus Mair. All rights reserved.

  This file is the config file of the MobaStation

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

The configuration file

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
//  NOTE: Before connecting these boards and selecting one in this software
//        check the quick install guides!!! Some of these boards require a voltage
//        generating resitor on the current sense pin of the device. Failure to select
//        the correct resistor could damage the sense pin on your Arduino or destroy
//        the device.
//
// DEFINE MOTOR_SHIELD_TYPE BELOW ACCORDING TO THE FOLLOWING TABLE:
//
//  STANDARD_MOTOR_SHIELD : Arduino Motor shield Rev3 based on the L298 with 18V 2A per channel
//  POLOLU_MOTOR_SHIELD   : Pololu MC33926 Motor Driver (not recommended for prog track)
//  FUNDUMOTO_SHIELD      : Fundumoto Shield, no current sensing (not recommended, no short protection)
//  FIREBOX_MK1           : The Firebox MK1                    
//  FIREBOX_MK1S          : The Firebox MK1S
//  IBT_2_WITH_ARDUINO    : Arduino Motor Shield for PROG and IBT-2 for MAIN
//   |
//   +-----------------------v
//
#define MOTOR_SHIELD_TYPE STANDARD_MOTOR_SHIELD

/////////////////////////////////////////////////////////////////////////////////////
//
// ENABLES/DISABLE DRIVING ON PROG-TRACK
// 4
#define DRIVE_ON_PROG true

/////////////////////////////////////////////////////////////////////////////////////
//
// TURNOUT SETTINGS
//
// Sets the Tournout-Mode
// TURNOUT_MobaBus
// TURNOUT_HYBRID
// TURNOUT_DCC
//
#define TURNOUTS TURNOUT_MobaBus
//
// if TURNOUT - MODE is set to TURNOUT_HYBRID this value
// spezifies the address range (from 0 to DCC_TURNOUT_ADDRESS_RANGE) of DCC Turnouts. 
// Above start the address range of MobaBus 
// Max = 2047
//
#define DCC_TURNOUT_ADDRESS_RANGE 2047

//#define XPRESSNET
//#define MAX_XPRESSNET_CLIENTS 5

#define POWER_LED 25
#define POWER_BUTTON 24

#define VOLTAGE_PIN A15
#define VOLTAGE_MULTI 25.58f


