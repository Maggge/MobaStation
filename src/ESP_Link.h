/**
  MobaStation ESP Link

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

#ifndef __ESP_LINK__
#define __ESP_LINK__

#include <Arduino.h>
#include "MobaStation.h"
#include "esplink/EspLinkProtocol.h"
#include "MobaStationWiFiConfig.h"     //WiFiMode and the WIFI_* settings

/**
 * Network over an ESP8266 with the MobaStation firmware (extras/esp/MobaStationESP)
 * instead of the AT firmware: the ESP does WiFi, UDP and the web interface, the Mega
 * exchanges Z21 packets with it in short frames (EspLinkProtocol.h).
 * The WiFi settings of the sketch are sent to the ESP as defaults, settings saved in
 * its web interface win over them.
 */
class ESP_Link : public Eth_Interface, public MobaStation_Management {
private:
    HardwareSerial *ser;
    const char *ssid;
    const char *pwd;
    const char *apSsid;
    const char *apPwd;
    uint8_t mode;
    uint16_t port = 21105;
    NetworkConfig net;

    EspLinkCodec rx;
    uint32_t lastRx = 0;
    uint32_t lastBeat = 0;
    bool espSeen = false;
    uint8_t netMode = ESPLINK_NET_DOWN;
    uint8_t netIp[4] = {0, 0, 0, 0};

    void sendFrame(uint8_t type, const uint8_t *data, uint8_t len);
    void sendText(uint8_t type, uint8_t key, const char *text);
    void sendDefaults();
    void sendStatus();
    void sendSetting(uint8_t key, uint8_t status);
    bool handleFrame(UdpPacket *pkg);

public:
    /**
     * @param mode Station_Mode, SoftAP_Mode or Hybrid_MODE like ESP_AT_Wifi
     */
    ESP_Link(HardwareSerial *serial, const char *ssid, const char *pwd, WiFiMode mode = Hybrid_MODE,
             const char *apSsid = "MobaStation", const char *apPwd = "MobaStation");

    bool begin(uint16_t port, const NetworkConfig &net);
    bool send(IPAddress client, uint16_t dstPort, uint8_t *data);
    bool receivePacket(UdpPacket *pkg);

    //the ESP answered within ESP_LINK_TIMEOUT
    bool linkUp();
    uint16_t linkErrors();

    void cvResult(uint8_t type, uint16_t address, uint16_t cv, uint8_t value, uint8_t status) override;
    void moduleFound(uint8_t type, uint16_t address, uint8_t typeId, uint8_t channels, uint8_t slot, uint8_t version) override;
};

#endif
