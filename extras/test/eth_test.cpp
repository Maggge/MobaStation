/*
  Host test of the Ethernet adapter: DHCP, static fallback, lease renewal. make test
*/
#include "Eth_Adapter.h"
uint32_t fakeMillis = 1000; HardwareSerial Serial; EthernetClass Ethernet;
int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } }while(0)
int main(){
  byte mac[6] = {0};
  NetworkConfig n{true, IPAddress(192,168,0,111), IPAddress(255,255,255,0), IPAddress()};
  { Eth_Adapter e(10, mac); Ethernet.log.clear(); Ethernet.dhcpResult = 1;
    CHECK(e.begin(21105, n), "dhcp begin");
    CHECK(Ethernet.log.size() == 1 && Ethernet.log[0] == "dhcp " + std::to_string(ETH_DHCP_TIMEOUT), "dhcp only [%s]", Ethernet.log[0].c_str());
    UdpPacket p; e.receivePacket(&p);
    CHECK(Ethernet.log.back() == "maintain", "lease is maintained"); }
  { Eth_Adapter e(10, mac); Ethernet.log.clear(); Ethernet.dhcpResult = 0;
    e.begin(21105, n);
    CHECK(Ethernet.log.size() == 2 && Ethernet.log[1] == "static 192.168.0.111 gw 192.168.0.1 mask 255.255.255.0", "fallback [%s]", Ethernet.log.back().c_str());
    UdpPacket p; Ethernet.log.clear(); e.receivePacket(&p);
    CHECK(Ethernet.log.empty(), "no maintain without lease"); }
  { Eth_Adapter e(10, mac); Ethernet.log.clear(); n.dhcp = false; n.gateway = IPAddress(192,168,0,254);
    e.begin(21105, n);
    CHECK(Ethernet.log.size() == 1 && Ethernet.log[0] == "static 192.168.0.111 gw 192.168.0.254 mask 255.255.255.0", "static [%s]", Ethernet.log[0].c_str()); }
  printf("%s (%d failures)\n", fails ? "FAILED" : "all ethernet tests passed", fails);
  return fails != 0;
}
