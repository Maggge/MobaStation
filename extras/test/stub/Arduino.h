#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <string>
#include <deque>
typedef uint8_t byte;
typedef bool boolean;
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))
#define pgm_read_byte_near(a) (*(const uint8_t*)(a))
extern uint32_t fakeMillis;
inline uint32_t millis(){ return fakeMillis; }
inline void delay(unsigned long ms){ fakeMillis += ms; }
#define HEX 16
#define BIN 2
class IPAddress;
class Stream {
public:
  void print(const IPAddress &ip);
  std::string out; std::deque<int> in;
  void push(const std::string &s){ for(unsigned char c: s) in.push_back(c); }
  int available(){ return in.size(); }
  int read(){ if(in.empty()) return -1; int c = in.front(); in.pop_front(); return c; }
  size_t write(uint8_t c){ out += (char)c; return 1; }
  void print(const char *s){ out += s; }
  void print(const __FlashStringHelper *s){ out += (const char*)s; }
  void print(char c){ out += c; }
  void print(long v, int=10){ out += std::to_string(v); }
  void print(int v, int=10){ out += std::to_string(v); }
  void print(unsigned v, int=10){ out += std::to_string(v); }
  void print(unsigned long v, int=10){ out += std::to_string(v); }
  void print(unsigned char v, int=10){ out += std::to_string(v); }
  template<class T> void println(T v){ print(v); out += "\r\n"; }
  void println(){ out += "\r\n"; }
};
class HardwareSerial : public Stream { public: void begin(unsigned long){} };
extern HardwareSerial Serial;
class IPAddress {
  uint8_t a[4];
public:
  IPAddress(){ memset(a,0,4); }
  IPAddress(uint8_t x,uint8_t y,uint8_t z,uint8_t w){ a[0]=x;a[1]=y;a[2]=z;a[3]=w; }
  uint8_t operator[](int i) const { return a[i]; }
  uint8_t& operator[](int i){ return a[i]; }
  bool operator==(const IPAddress &o) const { return !memcmp(a,o.a,4); }
  bool fromString(const char *str){ unsigned v[4]; char c;
    if(sscanf(str, "%u.%u.%u.%u%c", &v[0], &v[1], &v[2], &v[3], &c) != 4) return false;
    for(int i = 0; i < 4; i++){ if(v[i] > 255) return false; a[i] = v[i]; } return true; }
};
#include "binary.h"
#define bitRead(v,b) (((v) >> (b)) & 0x01)
#define bitWrite(v,b,x) ((x) ? ((v) |= (1UL<<(b))) : ((v) &= ~(1UL<<(b))))
inline uint16_t word(uint8_t h, uint8_t l){ return (h<<8)|l; }
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A15 69
inline int analogRead(int){ return 0; } inline int digitalRead(int){ return 1; }
inline void digitalWrite(int,int){} inline void pinMode(int,int){}
inline unsigned long micros(){ return 0; }


inline void Stream::print(const IPAddress &ip){ for(int i = 0; i < 4; i++){ if(i) out += "."; out += std::to_string(ip[i]); } }
