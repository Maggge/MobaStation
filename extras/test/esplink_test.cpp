/*
  Host test of the ESP link codec (EspLinkProtocol.h): make test
*/
#include "esplink/EspLinkProtocol.h"
#include <stdio.h>
#include <random>
#include <vector>
int fails = 0;
#define CHECK(c, ...) do{ if(!(c)){ printf("FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } }while(0)

static std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t> &d){
  uint8_t out[ESP_LINK_MAX_FRAME]; uint8_t n = EspLinkCodec::encode(type, d.data(), d.size(), out);
  return std::vector<uint8_t>(out, out + n);
}
static int feedAll(EspLinkCodec &c, const std::vector<uint8_t> &bytes, std::vector<std::vector<uint8_t>> *got = nullptr){
  int frames = 0;
  for(uint8_t b: bytes) if(c.feed(b)){ frames++; if(got) got->push_back(std::vector<uint8_t>(c.data(), c.data() + c.len)); }
  return frames;
}

int main(){
  std::mt19937 rng(7);
  EspLinkCodec rx;
  // random payloads with many zero bytes, every length
  for(int len = 0; len <= ESP_LINK_MAX_PAYLOAD - 1; len++){
    std::vector<uint8_t> d(len); for(auto &b: d) b = (rng() % 3 == 0) ? 0 : rng();
    auto f = frame(0x10, d);
    CHECK(!f.empty() && f.back() == 0, "frame ends with 0x00 (len %d)", len);
    int zeros = 0; for(size_t i = 0; i + 1 < f.size(); i++) if(f[i] == 0) zeros++;
    CHECK(zeros == 0, "no 0x00 inside the frame (len %d)", len);
    CHECK(f.size() <= ESP_LINK_MAX_FRAME, "frame fits (len %d, %zu)", len, f.size());
    std::vector<std::vector<uint8_t>> got;
    CHECK(feedAll(rx, f, &got) == 1 && rx.type == 0x10 && got[0] == d, "round trip len %d", len);
  }
  std::vector<uint8_t> big(ESP_LINK_MAX_PAYLOAD, 1); uint8_t out[ESP_LINK_MAX_FRAME];
  CHECK(EspLinkCodec::encode(1, big.data(), big.size(), out) == 0, "too long payload is refused");

  // a flipped bit is detected
  auto f = frame(0x20, {1, 2, 3, 0, 5});

  for(size_t i = 0; i + 1 < f.size(); i++){
    for(int bit = 0; bit < 8; bit++){
      auto g = f; g[i] ^= (1 << bit);
      if(g[i] == 0) continue;          //becomes a delimiter, a different case
      EspLinkCodec c; CHECK(feedAll(c, g) == 0, "flipped bit %zu/%d accepted", i, bit);
    }
  }
  // garbage before, between and after frames (boot messages of the ESP at 74880 baud)
  std::vector<uint8_t> stream = {0x12, 0xFF, 0x80, 0x33, 0x00, 'e', 't', 's', 0x00};
  auto a = frame(0x11, {192, 168, 30, 20, 0x51, 0x52, 4, 0, 0x10, 0});
  auto b = frame(0x02, {});
  stream.insert(stream.end(), a.begin(), a.end());
  for(int i = 0; i < 200; i++) stream.push_back(rng() % 255 + 1);   //long garbage without 0x00: overflow
  stream.push_back(0);
  stream.insert(stream.end(), b.begin(), b.end());
  stream.insert(stream.end(), a.begin(), a.begin() + 4);            //cut frame
  stream.push_back(0);
  stream.insert(stream.end(), a.begin(), a.end());
  EspLinkCodec c; std::vector<std::vector<uint8_t>> got;
  int n = feedAll(c, stream, &got);
  CHECK(n == 3, "3 valid frames found in the noise (%d)", n);
  CHECK(c.errors >= 3, "broken parts counted (%d)", c.errors);
  CHECK(espLinkGet16((const uint8_t*)"\x51\x52") == 0x5251 && espLinkGet32((const uint8_t*)"\x01\x02\x03\x04") == 0x04030201, "little endian");
  printf("%s (%d failures)\n", fails ? "FAILED" : "all ESP link tests passed", fails);
  return fails != 0;
}
