#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <cmath>
#include <cstdlib>
#include <cstring>

constexpr uint8_t DATA_PIN = D0;   // XIAO ESP32-C6 GPIO0 -> HX711 DAT
constexpr uint8_t CLOCK_PIN = D1;  // XIAO ESP32-C6 GPIO1 -> HX711 CLK
constexpr unsigned long SENSOR_TIMEOUT_MS = 1000;
constexpr unsigned int AVERAGE_SAMPLES = 10;
constexpr double FILTER_TAU_MS = 40.0;
constexpr unsigned long WATCHDOG_TIMEOUT_MS = 3000;
constexpr unsigned long SENSOR_RECOVERY_MS = 3000;

HX711 scale;
Preferences settings;
double countsPerGram = NAN;
double tareCounts = 0;
bool tared = false;
bool filterReady = false;
double filteredCounts = 0;
unsigned long filterAtMs = 0;
unsigned long lastSampleMs = 0;
unsigned long lastErrorMs = 0;
unsigned long lastRecoveryMs = 0;

void startWatchdog() {
  esp_task_wdt_config_t config = {};
  config.timeout_ms = WATCHDOG_TIMEOUT_MS;
  config.idle_core_mask = 0;
  config.trigger_panic = true;
  esp_err_t result = esp_task_wdt_init(&config);
  if (result == ESP_ERR_INVALID_STATE) result = esp_task_wdt_reconfigure(&config);
  ESP_ERROR_CHECK(result);
  if (esp_task_wdt_status(nullptr) != ESP_OK) ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));
}

void feedWatchdog() { ESP_ERROR_CHECK(esp_task_wdt_reset()); }

double filterCounts(long raw, unsigned long now) {
  const unsigned long elapsed = now - filterAtMs;
  if (!filterReady || elapsed >= SENSOR_TIMEOUT_MS) filteredCounts = raw;
  else filteredCounts += (1.0 - std::exp(-static_cast<double>(elapsed) / FILTER_TAU_MS)) * (raw - filteredCounts);
  filterReady = true;
  filterAtMs = now;
  return filteredCounts;
}

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
    feedWatchdog();
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
    Serial.println("# CSV: ms,raw_counts,net_counts,grams,filtered_counts; empty fields mean unavailable");
    Serial.println("# Native HX711 rate (10/80 SPS via RATE jumper); low-pass time constant 40 ms");
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
  startWatchdog();
#ifdef LOAD_CELL_WATCHDOG_SELF_TEST
  // Test builds deliberately stop feeding the watchdog. Never enable this
  // flag for the firmware left on the probe after validation.
  Serial.println("# watchdog self-test: deliberately stalling");
  while (true) delay(100);
#endif
  filterReady = false;
  lastSampleMs = lastErrorMs = lastRecoveryMs = millis();
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
  feedWatchdog();
  pollCommands();
  // Read every real conversion. Never emit duplicate samples to imitate a
  // higher rate: the HX711 RATE jumper chooses 10 or 80 samples/second.
  if (!scale.is_ready()) {
    const unsigned long now = millis();
    if (now - lastSampleMs >= SENSOR_TIMEOUT_MS && now - lastErrorMs >= SENSOR_TIMEOUT_MS) {
      Serial.println("# ERROR hx711_timeout: check power, DAT and CLK");
      lastErrorMs = now;
      filterReady = false;
    }
    if (now - lastSampleMs >= SENSOR_RECOVERY_MS && now - lastRecoveryMs >= SENSOR_RECOVERY_MS) {
      scale.power_down();
      delay(1);
      scale.power_up();
      pinMode(DATA_PIN, INPUT_PULLUP);
      lastRecoveryMs = millis();
      filterReady = false;
      Serial.println("# HX711 recovery: power cycle requested");
    }
    delay(1);
    return;
  }
  long raw;
  if (!readCounts(raw)) {
    filterReady = false;
    delay(1);
    return;
  }
  lastSampleMs = millis();
  const double filtered = filterCounts(raw, lastSampleMs);
  Serial.printf("%lu,%ld,", lastSampleMs, raw);
  if (tared) Serial.printf("%.1f", raw - tareCounts);
  Serial.print(",");
  if (tared && std::isfinite(countsPerGram)) {
    Serial.printf("%.3f", (raw - tareCounts) / countsPerGram);
  }
  Serial.printf(",%.1f\n", filtered);
}
