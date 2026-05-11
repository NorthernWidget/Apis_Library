---
title: Apis
summary: Arduino library for the Apis board, which manages a LiDAR Lite unit (roll/pitch, firmware lock/reset, power supply). 

layout: single
toc: true
toc_sticky: true
sidebar:
  nav: api
---

# Apis



Arduino library for the [Apis]() board, which manages a LiDAR Lite unit (roll/pitch, firmware lock/reset, power supply).  [More...](#detailed-description)


`#include <Apis.h>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[Apis](/Apis_Library/Classes/classApis/#function-apis)**(uint16_t nRangeReadings =1, bool rangeStats =false, uint16_t nOrientReadings =1, bool orientStats =false)<br>Instantiate [Apis](/Apis_Library/Classes/classApis/) object.  |
| bool | **[begin](/Apis_Library/Classes/classApis/#function-begin)**(uint8_t address =ADR_DEFAULT, SensitivityMode sensitivity =SENSITIVITY_BALANCED)<br>Begin communications with the [Apis](/Apis_Library/Classes/classApis/) using a prescribed address.  |
| void | **[setNRangeReadings](/Apis_Library/Classes/classApis/#function-setnrangereadings)**(uint16_t n)<br>Set number of range readings to average.  |
| void | **[setRangeStats](/Apis_Library/Classes/classApis/#function-setrangestats)**(bool enable)<br>Enable or disable range std and sterr in [getString()]().  |
| void | **[setNOrientReadings](/Apis_Library/Classes/classApis/#function-setnorientreadings)**(uint16_t n)<br>Set number of orientation readings to average.  |
| void | **[setOrientStats](/Apis_Library/Classes/classApis/#function-setorientstats)**(bool enable)<br>Enable or disable orientation std and sterr in [getString()]().  |
| void | **[setRangefinderSensitivity](/Apis_Library/Classes/classApis/#function-setrangefindersensitivity)**(SensitivityMode mode)<br>Change the rangefinder sensitivity mode after [begin()](/Apis_Library/Classes/classApis/#function-begin).  |
| bool | **[updateRange](/Apis_Library/Classes/classApis/#function-updaterange)**()<br>Measure range [cm] only, without reading the accelerometer. Intended for rapid repeated range readings (e.g. for averaging). Returns false if the sensor returns an error value.  |
| bool | **[updateOrientation](/Apis_Library/Classes/classApis/#function-updateorientation)**()<br>Measure pitch [deg] and roll [deg] only, without ranging. Intended for rapid repeated orientation readings (e.g. for averaging). FIX: Independent readings require firmware support; current firmware caches accelerometer values each loop (~200 ms). Returns false if the sensor returns an error value.  |
| bool | **[updateMeasurements](/Apis_Library/Classes/classApis/#function-updatemeasurements)**()<br>Measure range [cm], roll [deg] and pitch [deg]. Uses Welford's online algorithm to compute mean, std, and sterr over nRangeReadings and nOrientReadings respectively. Returns false if any sensor returns an error value.  |
| int16_t | **[getRange](/Apis_Library/Classes/classApis/#function-getrange)**()<br>Return range mean [cm], rounded to nearest cm.  |
| float | **[getRoll](/Apis_Library/Classes/classApis/#function-getroll)**()<br>Return roll mean [deg].  |
| float | **[getPitch](/Apis_Library/Classes/classApis/#function-getpitch)**()<br>Return pitch mean [deg].  |
| float | **[getRangeMean](/Apis_Library/Classes/classApis/#function-getrangemean)**()<br>Return range mean [cm] as float.  |
| float | **[getRangeStd](/Apis_Library/Classes/classApis/#function-getrangestd)**()<br>Return range standard deviation [cm].  |
| float | **[getRangeSterr](/Apis_Library/Classes/classApis/#function-getrangesterr)**()<br>Return range standard error [cm].  |
| float | **[getPitchStd](/Apis_Library/Classes/classApis/#function-getpitchstd)**()<br>Return pitch standard deviation [deg].  |
| float | **[getPitchSterr](/Apis_Library/Classes/classApis/#function-getpitchsterr)**()<br>Return pitch standard error [deg].  |
| float | **[getRollStd](/Apis_Library/Classes/classApis/#function-getrollstd)**()<br>Return roll standard deviation [deg].  |
| float | **[getRollSterr](/Apis_Library/Classes/classApis/#function-getrollsterr)**()<br>Return roll standard error [deg].  |
| String | **[getHeader](/Apis_Library/Classes/classApis/#function-getheader)**()<br>Return a comma-separated header matching [getString()]() output. Includes statistics columns when rangeStats or orientStats are enabled and nReadings > 1.  |
| String | **[getString](/Apis_Library/Classes/classApis/#function-getstring)**(bool takeNewReadings =true)<br>Return comma-separated data values.  |
| void | **[beginRawReadings](/Apis_Library/Classes/classApis/#function-beginrawreadings)**(uint8_t component =NW_READING_ALL)<br>Prepare for raw reading collection.  |
| uint16_t | **[takeRawReading](/Apis_Library/Classes/classApis/#function-takerawreading)**(char * buf, uint16_t offset)<br>Take one raw reading and write CSV data into buf at offset. Writes: range [cm] for RANGE; pitch [deg], roll [deg] for ORIENT; range, pitch, roll for ALL. Each value followed by a comma. Max bytes written per call: 7 (RANGE), 18 (ORIENT), 25 (ALL).  |
| void | **[endRawReadings](/Apis_Library/Classes/classApis/#function-endrawreadings)**()<br>End raw reading collection.  |

## Detailed Description

```cpp
class Apis;
```

Arduino library for the [Apis]() board, which manages a LiDAR Lite unit (roll/pitch, firmware lock/reset, power supply). 

Library to communicate with the [Apis](/Apis_Library/Classes/classApis/) module, which connects to a LiDAR Lite rangefinder. The [Apis](/Apis_Library/Classes/classApis/) is equipped with capacitors to handle the large burst power draw from the LiDAR Lite, a MEMS accelerometer to note its orientation, a magnet to note a known orientation (often, but not necessarily, horizontal) and the ability to absorb occasional firmware issues that lead to system hangs. The leveling helps the user to calculate, for example, a water level when the sensor is placed on a cliff or a tree next to the river but does not have water below it. The level loses absolute accuracy when near plumb, so a Hall-effect sensor connected to the magnet allows the user to set a zero value, thereby correcting for this. Managing failures of the LiDAR Lite within the [Apis](/Apis_Library/Classes/classApis/) is essential, and the [Apis](/Apis_Library/Classes/classApis/) therefore acts as a buffer to protect the data logger from raw sensor failures. 

## Public Functions Documentation

### function Apis

```cpp
Apis(
    uint16_t nRangeReadings =1,
    bool rangeStats =false,
    uint16_t nOrientReadings =1,
    bool orientStats =false
)
```

Instantiate [Apis](/Apis_Library/Classes/classApis/) object. 

**Parameters**: 

  * **nRangeReadings** Number of range readings to average (default 1). Paul et al. (2020, WRR, doi:10.1029/2019WR026810) found ~1000 readings needed for stable mean convergence under field conditions. Uses Welford's online algorithm: O(1) memory regardless of count. FIX: Independent readings require firmware support for on-demand triggering; current firmware caches the value each loop (~200 ms). 
  * **rangeStats** If true, [getString()](/Apis_Library/Classes/classApis/#function-getstring) includes range std and sterr. Only meaningful when nRangeReadings > 1. 
  * **nOrientReadings** Number of orientation readings to average (default 1). FIX: Independent readings require firmware support; current firmware caches accelerometer values each loop (~200 ms). 
  * **orientStats** If true, [getString()](/Apis_Library/Classes/classApis/#function-getstring) includes orientation std and sterr. Only meaningful when nOrientReadings > 1. 


### function begin

```cpp
bool begin(
    uint8_t address =ADR_DEFAULT,
    SensitivityMode sensitivity =SENSITIVITY_BALANCED
)
```

Begin communications with the [Apis](/Apis_Library/Classes/classApis/) using a prescribed address. 

**Parameters**: 

  * **address** I2C address (default ADR_DEFAULT = 0x50). 
  * **sensitivity** One of the SensitivityMode values (default SENSITIVITY_BALANCED). Written to firmware register 0x01 and applied each firmware loop() iteration. See SensitivityMode for descriptions of each mode. 


**Return**: true if the device acknowledges on I2C, false otherwise. 

**Warning**: Address selection is not yet implemented in firmware; this parameter is accepted but ignored. The device always uses its fixed firmware address. The default (0x50) also clashes with Haar's default address. Both issues will be resolved in a future firmware update. 

### function setNRangeReadings

```cpp
void setNRangeReadings(
    uint16_t n
)
```

Set number of range readings to average. 

### function setRangeStats

```cpp
void setRangeStats(
    bool enable
)
```

Enable or disable range std and sterr in [getString()](). 

### function setNOrientReadings

```cpp
void setNOrientReadings(
    uint16_t n
)
```

Set number of orientation readings to average. 

### function setOrientStats

```cpp
void setOrientStats(
    bool enable
)
```

Enable or disable orientation std and sterr in [getString()](). 

### function setRangefinderSensitivity

```cpp
void setRangefinderSensitivity(
    SensitivityMode mode
)
```

Change the rangefinder sensitivity mode after [begin()](/Apis_Library/Classes/classApis/#function-begin). 

**Parameters**: 

  * **mode** One of the SensitivityMode values. 


Writes the new mode to firmware register 0x01; the firmware applies it on the next loop() iteration via InitLiDAR(). Must be called after [begin()](/Apis_Library/Classes/classApis/#function-begin); if called before, Wire is uninitialised and the transmission fails silently. 


### function updateRange

```cpp
bool updateRange()
```

Measure range [cm] only, without reading the accelerometer. Intended for rapid repeated range readings (e.g. for averaging). Returns false if the sensor returns an error value. 

### function updateOrientation

```cpp
bool updateOrientation()
```

Measure pitch [deg] and roll [deg] only, without ranging. Intended for rapid repeated orientation readings (e.g. for averaging). FIX: Independent readings require firmware support; current firmware caches accelerometer values each loop (~200 ms). Returns false if the sensor returns an error value. 

### function updateMeasurements

```cpp
bool updateMeasurements()
```

Measure range [cm], roll [deg] and pitch [deg]. Uses Welford's online algorithm to compute mean, std, and sterr over nRangeReadings and nOrientReadings respectively. Returns false if any sensor returns an error value. 

### function getRange

```cpp
int16_t getRange()
```

Return range mean [cm], rounded to nearest cm. 

### function getRoll

```cpp
float getRoll()
```

Return roll mean [deg]. 

### function getPitch

```cpp
float getPitch()
```

Return pitch mean [deg]. 

### function getRangeMean

```cpp
float getRangeMean()
```

Return range mean [cm] as float. 

### function getRangeStd

```cpp
float getRangeStd()
```

Return range standard deviation [cm]. 

### function getRangeSterr

```cpp
float getRangeSterr()
```

Return range standard error [cm]. 

### function getPitchStd

```cpp
float getPitchStd()
```

Return pitch standard deviation [deg]. 

### function getPitchSterr

```cpp
float getPitchSterr()
```

Return pitch standard error [deg]. 

### function getRollStd

```cpp
float getRollStd()
```

Return roll standard deviation [deg]. 

### function getRollSterr

```cpp
float getRollSterr()
```

Return roll standard error [deg]. 

### function getHeader

```cpp
String getHeader()
```

Return a comma-separated header matching [getString()]() output. Includes statistics columns when rangeStats or orientStats are enabled and nReadings > 1. 

### function getString

```cpp
String getString(
    bool takeNewReadings =true
)
```

Return comma-separated data values. 

**Parameters**: 

  * **takeNewReadings** if true, run [updateMeasurements()](/Apis_Library/Classes/classApis/#function-updatemeasurements) before returning values. Otherwise, return stored values. 


This is the most likely function (alongside getHeader) for an end user to use. Always includes: Range [cm], Pitch [deg], Roll [deg]. Appends range std and sterr when rangeStats is true and nRangeReadings > 1. Appends orientation std and sterr when orientStats is true and nOrientReadings > 1. Error values are APIS_ERROR (-9999) for sensor errors and APIS_NOT_MEASURED (-9998) when no measurement has yet been taken. 


### function beginRawReadings

```cpp
void beginRawReadings(
    uint8_t component =NW_READING_ALL
)
```

Prepare for raw reading collection. 

**Parameters**: 

  * **component** NW_READING_ALL, NW_READING_RANGE (primary), or NW_READING_ORIENT (secondary). 


### function takeRawReading

```cpp
uint16_t takeRawReading(
    char * buf,
    uint16_t offset
)
```

Take one raw reading and write CSV data into buf at offset. Writes: range [cm] for RANGE; pitch [deg], roll [deg] for ORIENT; range, pitch, roll for ALL. Each value followed by a comma. Max bytes written per call: 7 (RANGE), 18 (ORIENT), 25 (ALL). 

**Parameters**: 

  * **buf** Caller-managed destination buffer. 
  * **offset** Starting write position in buf. 


**Return**: New offset after writing. 

### function endRawReadings

```cpp
void endRawReadings()
```

End raw reading collection. 

-------------------------------

Updated on 2026-05-11 at 23:56:23 +0000