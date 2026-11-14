#pragma once
#include "Arduino.h"
#include <string>
#include <vector>
enum { EthernetNoHardware, EthernetW5500 }; enum { LinkON, LinkOFF };
struct EthernetClass {
  std::vector<std::string> log; int dhcpResult = 1; IPAddress ip;
  void init(uint8_t){}
  int begin(uint8_t*, unsigned long t, unsigned long){ log.push_back("dhcp " + std::to_string(t)); if(dhcpResult) ip = IPAddress(192,168,0,57); return dhcpResult; }
  void begin(uint8_t*, IPAddress i, IPAddress dns, IPAddress gw, IPAddress m){ ip = i; char b[96]; snprintf(b, sizeof(b), "static %d.%d.%d.%d gw %d.%d.%d.%d mask %d.%d.%d.%d", i[0],i[1],i[2],i[3], gw[0],gw[1],gw[2],gw[3], m[0],m[1],m[2],m[3]); log.push_back(b); }
  int maintain(){ log.push_back("maintain"); return 0; }
  int hardwareStatus(){ return EthernetW5500; } int linkStatus(){ return LinkON; } IPAddress localIP(){ return ip; } };
extern EthernetClass Ethernet;
