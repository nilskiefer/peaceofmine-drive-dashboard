// Run: c++ -std=c++17 -Wall -Wextra -Werror -Itest/stubs test/test_firmware.cpp -o /tmp/loadcell-test && /tmp/loadcell-test
#include <cassert>
#include <climits>
#include "../src/main.cpp"

void samples(long value) { scale.samples.assign(AVERAGE_SAMPLES, value); }
bool outputHas(const char *text) { return Serial.output.find(text) != std::string::npos; }
void send(const std::string &text) {
  Serial.output.clear();
  Serial.input = text;
  while (Serial.available()) pollCommands();
}

int main() {
  setup();
  assert(watchdogSubscribed && watchdogConfig.timeout_ms == 3000 && watchdogConfig.trigger_panic);
  nowMs = 100;
  scale.samples.push_back(1000);
  loop();
  assert(Serial.output == "100,1000,,,1000.0\n");
  send("calibrate 500\n");
  assert(outputHas("ERROR unload and tare"));

  samples(1000);
  send("tare\r\n");
  assert(tared && tareCounts == 1000 && outputHas("OK tared"));
  samples(6000);
  send("calibrate 500\n");
  assert(countsPerGram == 10 && settings.saved == 10);
  Serial.output.clear();
  nowMs = 200;
  scale.samples.push_back(3500);
  loop();
  assert(Serial.output.find("200,3500,2500.0,250.000,") == 0);
  assert(filteredCounts > 1000 && filteredCounts < 3500);

  for (const char *invalid : {"0", "-1", "nan", "inf", "1e999", "1e-999", "500junk", ""}) {
    send(std::string("calibrate ") + invalid + "\n");
    assert(outputHas("ERROR mass") && countsPerGram == 10);
  }
  samples(1000);
  send("calibrate 500\n");
  assert(outputHas("ERROR no usable change") && countsPerGram == 10);

  // Reversed signal polarity is a valid calibration.
  samples(-4000);
  send("calibrate 500\n");
  assert(countsPerGram == -10);
  settings.failSave = true;
  samples(6000);
  send("calibrate 500\n");
  assert(outputHas("ERROR calibration save") && countsPerGram == -10);
  settings.failSave = false;

  scale.samples.clear();
  send("tare\n");
  assert(outputHas("ERROR hx711_timeout") && tareCounts == 1000);
  for (long rail : {-8388608L, 8388607L}) {
    samples(rail);
    send("tare\n");
    assert(outputHas("ERROR adc_saturated") && tareCounts == 1000);
  }
  send(std::string(64, 'x') + "tare\nstatus\n");
  assert(outputHas("ERROR command too long") && outputHas("# status"));
  send("tare" + std::string(1, '\0') + "\n");
  assert(outputHas("ERROR command too long"));
  send("invalid\n");
  assert(outputHas("ERROR unknown command"));

  tared = false;
  countsPerGram = NAN;
  setup();
  assert(!tared && countsPerGram == -10);
  // 80 SPS conversions are emitted immediately, without the old 100 ms cap.
  scale.samples.clear();
  Serial.output.clear();
  nowMs += 13;
  scale.samples.push_back(1000);
  loop();
  assert(!Serial.output.empty());
  const double before = filteredCounts;
  nowMs += 13;
  scale.samples.push_back(2000);
  loop();
  assert(filteredCounts > before && filteredCounts < 2000);
  // A stalled sensor emits no pretend samples and resets filter history.
  Serial.output.clear();
  scale.samples.clear();
  nowMs += SENSOR_TIMEOUT_MS;
  loop();
  assert(outputHas("ERROR hx711_timeout") && !filterReady);
  Serial.output.clear();
  nowMs += 13;
  scale.samples.push_back(-5000);
  loop();
  assert(filteredCounts == -5000);
  // Filter timing also works across millis() rollover on the ESP32.
  filterReady = true;
  filterAtMs = ULONG_MAX - 5;
  filteredCounts = 0;
  const double wrapped = filterCounts(1000, 7);
  assert(wrapped > 0 && wrapped < 1000);
  // Missing conversions retry the HX711 while the main task remains watched.
  scale.samples.clear();
  nowMs += SENSOR_RECOVERY_MS;
  const unsigned int feeds = watchdogFeeds;
  loop();
  assert(scale.powerCycles == 1 && watchdogFeeds > feeds && !filterReady);
  puts("PASS: native-rate streaming, low-pass, watchdog setup/feeds, sensor recovery, tare, calibration, persistence, invalid commands, saturation");
}
