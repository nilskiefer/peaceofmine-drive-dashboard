#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <cmath>
#include <cstdlib>
#include <cstring>

constexpr uint8_t DATA_PIN = D0;   // XIAO ESP32-C6 GPIO0 -> HX711 DAT
constexpr uint8_t CLOCK_PIN = D1;  // XIAO ESP32-C6 GPIO1 -> HX711 CLK
constexpr unsigned long SENSOR_TIMEOUT_MS = 1000;
constexpr unsigned int AVERAGE_SAMPLES = 10;

HX711 scale;
Preferences settings;
double countsPerGram = NAN;
double tareCounts = 0;
bool tared = false;

bool readCounts(long &raw) {
  if (!scale.wait_ready_timeout(SENSOR_TIMEOUT_MS, 1)) {
    Serial.println("# ERROR hx711_timeout: check power, DAT and CLK");
    return false;
  }
  raw = scale.read();
  if (raw <= -8388608L || raw >= 8388607L) {
    Serial.println("# ERROR adc_saturated: check load-cell wiring and load");
    return false;
  }
  return true;
}

bool readAverage(double &average) {
  int64_t sum = 0;
  for (unsigned int i = 0; i < AVERAGE_SAMPLES; ++i) {
    long raw;
    if (!readCounts(raw)) return false;
    sum += raw;
  }
  average = static_cast<double>(sum) / AVERAGE_SAMPLES;
  return true;
}

void printStatus() {
  Serial.printf("# status tared=%d calibrated=%d tare_counts=%.1f counts_per_gram=%.9g\n",
                tared, std::isfinite(countsPerGram), tareCounts, countsPerGram);
}

void command(const char *line) {
  if (strcmp(line, "help") == 0) {
    Serial.println("# commands: help | status | tare | calibrate <grams>");
    Serial.println("# unload and tare, add a known mass, then calibrate 500 (for 500 g)");
    Serial.println("# calibration is saved; tare is required after every reboot");
    Serial.println("# CSV: ms,raw_counts,net_counts,grams; empty fields mean unavailable");
  } else if (strcmp(line, "status") == 0) {
    printStatus();
  } else if (strcmp(line, "tare") == 0) {
    double average;
    if (!readAverage(average)) return;
    tareCounts = average;
    tared = true;
    Serial.println("# OK tared");
    printStatus();
  } else if (strncmp(line, "calibrate ", 10) == 0) {
    char *end;
    const double grams = strtod(line + 10, &end);
    if (end == line + 10 || *end != '\0' || !std::isfinite(grams) || grams <= 0) {
      Serial.println("# ERROR mass must be a finite positive number in grams");
      return;
    }
    if (!tared) {
      Serial.println("# ERROR unload and tare before calibration");
      return;
    }
    double average;
    if (!readAverage(average)) return;
    const double factor = (average - tareCounts) / grams;
    if (!std::isfinite(factor) || factor == 0) {
      Serial.println("# ERROR no usable change from tare");
      return;
    }
    if (settings.putDouble("counts_per_g", factor) != sizeof(double)) {
      Serial.println("# ERROR calibration save failed");
      return;
    }
    countsPerGram = factor;
    Serial.println("# OK calibration saved");
    printStatus();
  } else {
    Serial.println("# ERROR unknown command; enter help");
  }
}

void pollCommands() {
  static char line[64];
  static size_t length = 0;
  static bool overflow = false;
  for (unsigned int i = 0; i < sizeof(line) && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("# ERROR command too long");
      else if (length) {
        line[length] = '\0';
        command(line);
      }
      length = 0;
      overflow = false;
    } else if (c == '\0' || length == sizeof(line) - 1) {
      overflow = true;
    } else if (!overflow) {
      line[length++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  scale.begin(DATA_PIN, CLOCK_PIN, 128);
  scale.power_up();
  // A disconnected DAT wire must read high instead of floating as a fake sample.
  pinMode(DATA_PIN, INPUT_PULLUP);
  if (!settings.begin("loadcell", false)) {
    while (true) {
      Serial.println("# ERROR calibration storage unavailable; halted");
      delay(1000);
    }
  }
  if (settings.isKey("counts_per_g")) {
    countsPerGram = settings.getDouble("counts_per_g", NAN);
    if (!std::isfinite(countsPerGram) || countsPerGram == 0) {
      while (true) {
        Serial.println("# ERROR invalid saved calibration; halted");
        delay(1000);
      }
    }
  }
}

void loop() {
  static unsigned long lastSampleMs = 0;
  pollCommands();
  // Limit USB output to 10 Hz even if the HX711 RATE jumper selects 80 SPS.
  if (millis() - lastSampleMs < 100) {
    delay(1);
    return;
  }
  long raw;
  if (!readCounts(raw)) {
    lastSampleMs = millis();
    delay(100);
    return;
  }
  lastSampleMs = millis();
  Serial.printf("%lu,%ld,", lastSampleMs, raw);
  if (tared) Serial.printf("%.1f", raw - tareCounts);
  Serial.print(",");
  if (tared && std::isfinite(countsPerGram)) {
    Serial.printf("%.3f", (raw - tareCounts) / countsPerGram);
  }
  Serial.println();
}
