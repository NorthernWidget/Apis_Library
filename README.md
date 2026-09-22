# Apis_Library

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4572371.svg)](https://doi.org/10.5281/zenodo.4572371)

Arduino library for the [Apis](https://github.com/NorthernWidget/Project-Apis) LiDAR rangefinder board. Apis manages power supply, firmware watchdog, and I2C communication for a LiDAR Lite unit, and reads a MEMS accelerometer to report pitch and roll — useful when the sensor is not mounted level.

**Requires firmware patch 2 or later** (the [NW-Device-Specification](https://github.com/NorthernWidget/NW-Device-Specification) Schema 1 register map, on Project-Apis `master`) on a board provisioned with [NW-Provision](https://github.com/NorthernWidget/NW-Provision). The default I2C address is `0x41`. Boards running the v0.1.x firmware (address `0x50`) are refused by `begin()`; reflash and provision them, or use Apis_Library v0.1.0.

**Installation:** included in [NorthernWidget-libraries](https://github.com/NorthernWidget/NorthernWidget-libraries). Also available via the Arduino Library Manager. Requires [NW_Core](https://github.com/NorthernWidget/NW_Core), the shared foundation (device protocol, readings and statistics, faults), which the Library Manager installs as a dependency once both are registered; until then, install it beside this library.

```cpp
#include <Apis.h>

Apis rangefinder;

void setup() {
    Serial.begin(9600);
    if (!rangefinder.begin()) {          // false: no ACK, not Schema 1, wrong name, or firmware too old
        Serial.print("Apis refused; firmware patch ");
        Serial.println(rangefinder.getFirmwareVersion());
        while (1);
    }
    Serial.println(rangefinder.getHeader());
}

void loop() {
    Serial.println(rangefinder.getString());
    delay(1000);
}
```

See [examples/](examples/) for a complete demo, a many-readings logger, and Margay logger integration (the Margay example reports a failed `begin()` or a latched device fault in one word through the logger's `Note` column, serial monitor, and LED).

## Readings

`getString()` takes a reading (or several, with statistics, when configured) and returns one CSV row. For writing many individual readings to a file or the serial monitor without building `String`s, use the reading interface, which prints to any Arduino `Print` (an SdFat `File`, `Serial`, ...):

```cpp
rangefinder.beginReadings(Apis::RANGE, 100);  // Apis::ALL, Apis::RANGE, or Apis::ORIENT; 100 readings follow
rangefinder.printHeader(file);                // "Range [cm],"
for (int i = 0; i < 100; i++) {
    rangefinder.logReading(file);             // takes ONE reading and prints it: "312,"
    file.println();
}
rangefinder.endReadings();
Serial.println(rangefinder.getRangeMean());   // statistics of those 100, no further acquisition
```

The count passed to `beginReadings()` is written to the device (registers 0x24-0x25), which then keeps the LiDAR powered and initialised for exactly that many readings instead of powering it up and down around each one; `updateMeasurements()` does the same for its N readings. A run that stops short is abandoned by the device after 2 s with a latched fault.

Every acquisition, whether from `updateMeasurements()`, `logReading()`, or a single-chip update, appends to that measurement's array; `updateMeasurements()` and `beginReadings()` start a fresh set. The scalar getters return the latest reading and the statistics getters compute from the array on demand, so there is one record of the readings and no second acquisition to summarise them.

`printReading(out)` prints the stored reading without acquiring; `updateMeasurements(component)` reads one chip alone (`Apis::RANGE` or `Apis::ORIENT`) or both; `getRangeCount()` and `getOrientCount()` report how many valid readings are behind the current statistics.

Every reading is requested from the device and waited for through its reading counter, so repeated readings are independent measurements. `setRangeReadings(n)` and `setOrientReadings(n)` set how many are taken per `updateMeasurements()` (clamped to `APIS_RANGE_CAPACITY`, default 64, and `APIS_ORIENT_CAPACITY`, default 8; override either before the include). Statistics over them: `getRangeMean()`, `getRangeStd()`, `getRangeSterr()`, `getRangeMedian()`, and the same for pitch and roll, plus `getRangeCount()`. With `setRangeStats(true)` the std and sterr columns join `getString()`.

Handshake and faults, for sketches that want them: `requestReading()`, `ready()`, `newReading()`; `faulted(chip)`, `anyFault()`, `faultChip()`, `faultKind()`, `printFault(Serial)`; `getHardwareMajor()`, `getHardwareMinor()`, `getFirmwareVersion()`.

The v0.1.x names `beginRawReadings()`, `takeRawReading(buf, offset)`, `endRawReadings()` and the `NW_READING_*` selectors still work and are deprecated.

## Testing

`extras/test/run.sh` compiles the library on a desktop against stub `Arduino.h` and `Wire.h` and checks that `getHeader()`, `getString()` and the reading interface produce byte-identical output to `extras/test/baseline.txt` for fixed register images. Run it after any change; `--record` rewrites the baseline when an output change is intended.

**Full API reference:** https://docs.northernwidget.com/Apis_Library/
