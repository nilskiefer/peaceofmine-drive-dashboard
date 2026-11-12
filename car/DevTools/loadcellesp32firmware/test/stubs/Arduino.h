#pragma once
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <string>

constexpr uint8_t D0 = 0, D1 = 1;
constexpr int INPUT_PULLUP = 2;
inline unsigned long nowMs = 0;
inline unsigned long millis() { return nowMs; }
inline void delay(unsigned long ms) { nowMs += ms; }
inline void pinMode(uint8_t, int) {}
struct FakeSerial {
  std::string input, output;
  void begin(int) {}
  int available() { return input.size(); }
  char read() { char c = input.front(); input.erase(0, 1); return c; }
  void print(const char *s) { output += s; }
  void println(const char *s = "") { output += std::string(s) + "\n"; }
  void printf(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    output += buffer;
  }
};
inline FakeSerial Serial;
