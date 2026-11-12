// Run: c++ -std=c++17 -Wall -Wextra -Werror -Itest/stubs test/test_firmware.cpp -o /tmp/loadcell-test && /tmp/loadcell-test
#include <cassert>
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
  nowMs = 100;
  scale.samples.push_back(1000);
  loop();
  assert(Serial.output == "100,1000,,\n");
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
  assert(Serial.output == "200,3500,2500.0,250.000\n");

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
  puts("PASS: streaming, tare, calibration, persistence, invalid commands, timeout, saturation");
}
