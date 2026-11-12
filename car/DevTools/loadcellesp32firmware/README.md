# XIAO ESP32-C6 load-cell firmware

Reads a SparkFun HX711 on channel A at gain 128 and streams CSV through the
XIAO's USB-C port. No Wi-Fi, I2C, UART adapter, or Raspberry Pi GPIO wiring is
needed. Connect the USB cable directly to the Mac or Raspberry Pi.

## Wire it with USB unplugged

Use the **XIAO ESP32-C6** header labels, not the pin map of a C3 or S3.

| XIAO pin | SparkFun HX711 pin | Purpose |
| --- | --- | --- |
| **5V** | **VCC** | Analog/load-cell supply, powered by USB |
| **3V3** | **VDD** | 3.3 V digital logic supply |
| **GND** | **GND** | Common ground |
| **D0 / GPIO0** | **DAT / DOUT** | HX711 data to ESP32 |
| **D1 / GPIO1** | **CLK / SCK** | ESP32 clock to HX711 |

**VDD must be 3.3 V. Do not join VCC and VDD when VCC is 5 V.** The ESP32
GPIOs are not 5 V tolerant. The XIAO's 5V pin is USB VBUS; this setup uses USB
power. Keep the load-cell signal wires short and away from motor/power wiring.

Connect a four-wire full-bridge load cell to the HX711's **load-cell side**:

| Load-cell datasheet signal | HX711 connection |
| --- | --- |
| Excitation + / E+ | E+ / RED |
| Excitation - / E- | E- / BLK |
| Signal + / S+ | A+ |
| Signal - / S- | A- |
| Cable shield, if present | Shield / YLW |

Follow the load cell's datasheet for **wire colors**. SparkFun documentation
has differing white/green descriptions; use A+/A- from the board schematic
and your sensor's signal labels. Leave channel B unused. A three-wire
half-bridge sensor needs a completed bridge; do not wire it as a four-wire cell.
The sensor's exact model and colors have not been established for this project.

References: [Seeed pin map](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/),
[SparkFun v11 schematic](https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/SparkFun_HX711_Load_Cell_v11.pdf),
[PlatformIO board](https://docs.platformio.org/en/latest/boards/espressif32/seeed_xiao_esp32c6.html).

## Build, flash, and open the serial monitor

Run from this folder with PlatformIO Core installed:

```sh
pio run
pio device list
pio run -t upload --upload-port /dev/cu.usbmodem1101
pio device monitor --port /dev/cu.usbmodem1101
```

The port above was detected on the development Mac. Choose the actual port
reported by `pio device list`; on the Pi it is usually `/dev/ttyACM0`.
On this Mac, PlatformIO is installed at `~/.platformio/penv/bin/pio` if `pio`
is not on PATH. Close any other serial monitor before uploading. To enter
download mode if necessary, hold BOOT, press/release RESET, then release BOOT.
Exit the PlatformIO monitor with Ctrl+C.

## First measurement

The monitor uses 115200 baud and sends commands when you press Enter.

1. Mount the load cell correctly. Remove the test load and let readings settle.
2. Enter `tare`. It averages ten new readings and sets the current load to zero.
3. Place a known mass on the cell, wait for settling, and enter `calibrate 500`
   for a **500 gram** mass (replace 500 with the actual mass).
4. Wait for `# OK calibration saved`. Check that the known mass reads correctly,
   then remove it and check that the reading returns near zero. Check another
   known mass before relying on the measurement.

Calibration is saved in ESP32 flash. **Tare is deliberately not saved and must
be repeated after each reboot**, with the cell unloaded. No automatic tare can
accidentally zero a load present at startup. Recalibrate after changing the
sensor, wiring, gain, or mechanical mounting. A negative counts-per-gram factor
is valid when the cell's signal polarity is reversed.

Commands (lowercase, LF or CRLF terminated):

| Command | Result |
| --- | --- |
| `help` | Print commands and CSV format |
| `status` | Print tare/calibration state and factors |
| `tare` | Average ten readings to zero the unloaded cell |
| `calibrate <grams>` | Average ten readings under a known positive mass and save the factor |

Tare and calibration pause streaming while collecting the ten samples (about
one second at the default 10 SPS). Each sensor wait times out after one second.
An error leaves the previous tare and calibration unchanged.

## USB data format

One CSV record per reading, capped at 10 Hz. Default HX711 hardware rate is
10 SPS; this firmware does not require changing its RATE jumper.

```text
ms,raw_counts,net_counts,grams
1200,123456,,
2400,123460,4.0,
3600,133456,10000.0,500.000
```

The header above describes the columns; it is not emitted on every connection.
`ms` is time since boot (wraps after about 49.7 days). Raw counts are signed
24-bit ADC values. Net counts are empty until tare; grams are empty until both
tare and calibration are available. Blank means **unavailable**, never zero.
Streaming samples are unfiltered; only tare and calibration use averaging.

All command responses and errors start with `#`. A host CSV reader should
handle/skip those lines. Examples:

```text
# ERROR hx711_timeout: check power, DAT and CLK
# ERROR adc_saturated: check load-cell wiring and load
```

On either error, no measurement record is emitted for that sample. The next
sample is attempted normally. Never keep using the last reading as if it were
fresh. A missing analog bridge is not always detectable in software; plausible
counts alone do not verify the sensor wiring or calibration. These are sensing
tools only; this project does not control vehicle motion or integrate with ROS.

## Host tests

The tests compile the actual firmware against small serial/sensor/storage fakes:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -Itest/stubs test/test_firmware.cpp -o /tmp/loadcell-test
/tmp/loadcell-test
```

They check CSV availability, tare, positive and negative calibration, saved
factor reload, malformed/oversized commands, storage-write errors, sensor
timeouts, and ADC saturation. They do not verify electrical timing or accuracy;
those require the wired HX711 and a known test mass.
