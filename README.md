# Apis_Library

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4572371.svg)](https://doi.org/10.5281/zenodo.4572371)

Arduino library for the [Apis](https://github.com/NorthernWidget/Project-Apis) LiDAR rangefinder board. Apis manages power supply, firmware watchdog, and I2C communication for a LiDAR Lite unit, and reads a MEMS accelerometer to report pitch and roll — useful when the sensor is not mounted level.

**Installation:** included in [NorthernWidget-libraries](https://github.com/NorthernWidget/NorthernWidget-libraries). Also available via the Arduino Library Manager.

```cpp
#include <Apis.h>

Apis rangefinder;

void setup() {
    Serial.begin(9600);
    rangefinder.begin();
    Serial.println(rangefinder.getHeader());
}

void loop() {
    Serial.println(rangefinder.getString());
    delay(1000);
}
```

See [examples/](examples/) for a complete demo, a many-readings logger, and Margay logger integration.

## Readings

`getString()` takes a reading (or several, with statistics, when configured) and returns one CSV row. For writing many individual readings to a file or the serial monitor without building `String`s, use the reading interface, which prints to any Arduino `Print` (an SdFat `File`, `Serial`, ...):

```cpp
rangefinder.beginReadings(Apis::RANGE);   // Apis::ALL, Apis::RANGE, or Apis::ORIENT
rangefinder.printHeader(file);            // "Range [cm],"
for (int i = 0; i < 100; i++) {
    rangefinder.logReading(file);         // takes ONE reading and prints it: "312,"
    file.println();
}
rangefinder.endReadings();
```

`printReading(out)` prints the stored reading without acquiring; `updateMeasurements(component)` reads one chip alone (`Apis::RANGE` or `Apis::ORIENT`) or both; `getRangeCount()` and `getOrientCount()` report how many valid readings are behind the current statistics.

The v0.1.x names `beginRawReadings()`, `takeRawReading(buf, offset)`, `endRawReadings()` and the `NW_READING_*` selectors still work and are deprecated.

## Testing

`extras/test/run.sh` compiles the library on a desktop against stub `Arduino.h` and `Wire.h` and checks that `getHeader()`, `getString()` and the reading interface produce byte-identical output to `extras/test/baseline.txt` for fixed register images. Run it after any change; `--record` rewrites the baseline when an output change is intended.

**Full API reference:** https://docs.northernwidget.com/Apis_Library/
