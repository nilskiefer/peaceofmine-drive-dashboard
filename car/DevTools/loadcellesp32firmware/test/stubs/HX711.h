#pragma once
#include <cassert>
#include <deque>
struct HX711 {
  std::deque<long> samples;
  unsigned int powerCycles = 0;
  void begin(unsigned char data, unsigned char clock, int gain) {
    assert(data == 0 && clock == 1 && gain == 128);
  }
  void power_up() {}
  void power_down() { ++powerCycles; }
  bool is_ready() { return !samples.empty(); }
  bool wait_ready_timeout(unsigned long, int) { return !samples.empty(); }
  long read() {
    assert(!samples.empty());
    long sample = samples.front();
    samples.pop_front();
    return sample;
  }
};
