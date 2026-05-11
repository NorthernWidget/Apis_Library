---
title: Apis_Library

layout: single
toc: true
toc_sticky: true
sidebar:
  nav: api
---

# Apis_Library



[![https://zenodo.org/badge/DOI/10.5281/zenodo.4572371.svg](/Apis_Library/images/https://zenodo.org/badge/DOI/10.5281/zenodo.4572371.svg)](https://doi.org/10.5281/zenodo.4572371)

Arduino library for the [Apis](https://github.com/NorthernWidget/Project-Apis) LiDAR rangefinder board. [Apis](/Apis_Library/Classes/classApis/) manages power supply, firmware watchdog, and I2C communication for a LiDAR Lite unit, and reads a MEMS accelerometer to report pitch and roll — useful when the sensor is not mounted level.

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

See [examples/](examples/) for a complete demo and Margay logger integration.

**Full API reference:**[https://docs.northernwidget.com/Apis_Library/](https://docs.northernwidget.com/Apis_Library/)

-------------------------------

Updated on 2026-05-11 at 23:56:23 +0000
