/**
  MobaStation Wifi Udp

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

#ifndef __WIFI_UDP__
#define __WIFI_UDP__

#include <Arduino.h>
#include <Udp.h>
#include "WiFi_ESP_AT.h"


class WiFiUDP : public UDP {
private:
    uint8_t link_ID = 255;

    WIFI_PACKET pkgBuffer;

    bool recPending = false;
    size_t recIdx = 0;

    WIFI_PACKET sendBuffer;
    bool pkgBegin = false;
    
public:
    uint8_t begin(uint16_t port);  // initialize, start listening on specified port. Returns 1 if successful, 0 if there are no sockets available to use
    uint8_t beginMulticast(IPAddress ip, uint16_t port); // initialize, start listening on specified multicast IP address and port. Returns 1 if successful, 0 on failure
    void stop();  // Finish with the UDP socket

    // Sending UDP packets

    // Start building up a packet to send to the remote host specific in ip and port
    // Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
    int beginPacket(IPAddress ip, uint16_t port);
    // Start building up a packet to send to the remote host specific in host and port
    // Returns 1 if successful, 0 if there was a problem resolving the hostname or port
    int beginPacket(const char *host, uint16_t port);
    // Finish off this packet and send it
    // Returns 1 if the packet was sent successfully, 0 if there was an error
    int endPacket();
    // Write a single byte into the packet
    size_t write(uint8_t d);
    // Write size bytes from buffer into the packet
    size_t write(const uint8_t *buffer, size_t size);

    // Start processing the next available incoming packet
    // Returns the size of the packet in bytes, or 0 if no packets are available
    int parsePacket();
    // Number of bytes remaining in the current packet
    int available();
    // Read a single byte from the current packet
    int read();
    // Read up to len bytes from the current packet and place them into buffer
    // Returns the number of bytes read, or 0 if none are available
    int read(unsigned char* buffer, size_t len);
    // Read up to len characters from the current packet and place them into buffer
    // Returns the number of characters read, or 0 if none are available
    int read(char* buffer, size_t len);
    // Return the next byte from the current packet without moving on to the next byte
    int peek();
    void flush();	// Finish reading the current packet

    // Return the IP address of the host who sent the current incoming packet
    IPAddress remoteIP();
    // Return the port of the host who sent the current incoming packet
    uint16_t remotePort();

};

#endif