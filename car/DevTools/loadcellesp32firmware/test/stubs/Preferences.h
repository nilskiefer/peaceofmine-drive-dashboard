#pragma once
#include <cmath>
#include <cstddef>
struct Preferences {
  bool failSave = false;
  double saved = NAN;
  bool begin(const char *, bool) { return true; }
  bool isKey(const char *) { return std::isfinite(saved); }
  double getDouble(const char *, double) { return saved; }
  size_t putDouble(const char *, double value) {
    if (failSave) return 0;
    saved = value;
    return sizeof(double);
  }
};
