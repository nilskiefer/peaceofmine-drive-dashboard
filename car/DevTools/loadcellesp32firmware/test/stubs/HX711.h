#pragma once
#include <cassert>
#include <deque>
struct HX711 {
  std::deque<long> samples;
  void begin(unsigned char data, unsigned char clock, int gain) {
    assert(data == 0 && clock == 1 && gain == 128);
  }
  void power_up() {}
  bool wait_ready_timeout(unsigned long, int) { return !samples.empty(); }
  long read() {
    assert(!samples.empty());
    long sample = samples.front();
    samples.pop_front();
    return sample;
  }
};
