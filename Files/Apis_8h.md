---
title: src/Apis.h

layout: single
toc: true
toc_sticky: true
sidebar:
  nav: api
---

# src/Apis.h



## Classes

|                | Name           |
| -------------- | -------------- |
| class | **[Apis](/Apis_Library/Classes/classApis/)** <br>Arduino library for the [Apis]() board, which manages a LiDAR Lite unit (roll/pitch, firmware lock/reset, power supply).  |

## Types

|                | Name           |
| -------------- | -------------- |
| enum uint8_t | **[SensitivityMode](/Apis_Library/Files/Apis_8h/#enum-sensitivitymode)** { SENSITIVITY_BALANCED = 0, SENSITIVITY_HIGH = 1, SENSITIVITY_LOW = 2, SENSITIVITY_MAX_RANGE = 3}<br>Sensitivity mode for the LiDAR Lite acquisition pipeline.  |

## Defines

|                | Name           |
| -------------- | -------------- |
|  | **[M_PI](/Apis_Library/Files/Apis_8h/#define-m-pi)**  |
|  | **[ADR_DEFAULT](/Apis_Library/Files/Apis_8h/#define-adr-default)**  |
|  | **[APIS_ERROR](/Apis_Library/Files/Apis_8h/#define-apis-error)**  |
|  | **[APIS_NOT_MEASURED](/Apis_Library/Files/Apis_8h/#define-apis-not-measured)**  |
|  | **[NW_READING_ALL](/Apis_Library/Files/Apis_8h/#define-nw-reading-all)**  |
|  | **[NW_READING_PRIMARY](/Apis_Library/Files/Apis_8h/#define-nw-reading-primary)**  |
|  | **[NW_READING_SECONDARY](/Apis_Library/Files/Apis_8h/#define-nw-reading-secondary)**  |
|  | **[NW_READING_RANGE](/Apis_Library/Files/Apis_8h/#define-nw-reading-range)**  |
|  | **[NW_READING_ORIENT](/Apis_Library/Files/Apis_8h/#define-nw-reading-orient)**  |

## Types Documentation

### enum SensitivityMode

| Enumerator | Value | Description |
| ---------- | ----- | ----------- |
| SENSITIVITY_BALANCED | 0|  Default. SIG_COUNT_VAL=0x80, THRESHOLD_BYPASS=0x00. Balanced range and noise performance.  |
| SENSITIVITY_HIGH | 1|  THRESHOLD_BYPASS=0x80. Lower detection threshold; detects weaker returns at the cost of more false positives.  |
| SENSITIVITY_LOW | 2|  THRESHOLD_BYPASS=0xB0. Higher detection threshold; fewer false positives at the cost of reduced range.  |
| SENSITIVITY_MAX_RANGE | 3|  SIG_COUNT_VAL=0xFF. More acquisitions per measurement; longer maximum range, slower throughput.  |



Sensitivity mode for the LiDAR Lite acquisition pipeline. 

Written to firmware register 0x01 by begin(); applied on every loop() iteration when the firmware reinitialises the LiDAR Lite via InitLiDAR(). Two LiDAR Lite registers drive the behaviour:

* SIG_COUNT_VAL (0x02): maximum acquisition count per measurement. Higher values average more returns, extending usable range but slowing throughput.
* THRESHOLD_BYPASS (0x1C): signal detection threshold. Lower values detect weaker returns (higher sensitivity, more false positives); higher values suppress weak returns (fewer false positives, less range). TODO: consider automatic mode selection based on signal quality feedback from the LiDAR Lite. 





## Macros Documentation

### define M_PI

```cpp
#define M_PI 3.14159265358979323846
```


### define ADR_DEFAULT

```cpp
#define ADR_DEFAULT 0x50
```


### define APIS_ERROR

```cpp
#define APIS_ERROR -9999
```


Sentinel returned by all getters and printed by getString() when a measurement fails due to hardware fault, I2C failure, or out-of-range reading. Applies to all measurement types: range, pitch, roll, and all derived statistics (mean, std, sterr). 


### define APIS_NOT_MEASURED

```cpp
#define APIS_NOT_MEASURED -9998
```


Sentinel returned by all getters and printed by getString() when begin() has been called but no successful updateMeasurements() (or updateRange()/updateOrientation()) has yet completed. Distinct from APIS_ERROR so callers can tell the difference between "the sensor failed" and "we haven't asked yet." 


### define NW_READING_ALL

```cpp
#define NW_READING_ALL 0
```


### define NW_READING_PRIMARY

```cpp
#define NW_READING_PRIMARY 1
```


### define NW_READING_SECONDARY

```cpp
#define NW_READING_SECONDARY 2
```


### define NW_READING_RANGE

```cpp
#define NW_READING_RANGE NW_READING_PRIMARY
```


### define NW_READING_ORIENT

```cpp
#define NW_READING_ORIENT NW_READING_SECONDARY
```


## Source code

```cpp
/******************************************************************************
Apis.h

Library for the Apis interface board for a LiDAR Lite unit.

Andrew Wickert
Based loosely on early code by Bobby Schulz, including
https://github.com/NorthernWidget/Project-Apis/tree/master/Software/LiDARLite_I2CParse

Started 2020.05.01
Hardware located at:
https://github.com/NorthernWidget/Project-Apis

License: GNU GPL v3. You should find a copy in the repository.
******************************************************************************/

#ifndef Apis_h
#define Apis_h

#include <Arduino.h>
#include <Wire.h>

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

#define ADR_DEFAULT 0x50 // Define default address.

enum SensitivityMode : uint8_t {
    SENSITIVITY_BALANCED  = 0, 
    SENSITIVITY_HIGH      = 1, 
    SENSITIVITY_LOW       = 2, 
    SENSITIVITY_MAX_RANGE = 3  
};

#define APIS_ERROR        -9999

#define APIS_NOT_MEASURED -9998

// NW standard component constants for raw reading interface.
// TODO: Move to NW template library when created.
#ifndef NW_READING_ALL
  #define NW_READING_ALL       0
  #define NW_READING_PRIMARY   1
  #define NW_READING_SECONDARY 2
#endif
// Apis-specific aliases
#define NW_READING_RANGE  NW_READING_PRIMARY
#define NW_READING_ORIENT NW_READING_SECONDARY

class Apis
{
    public:
        Apis(uint16_t nRangeReadings = 1, bool rangeStats = false,
             uint16_t nOrientReadings = 1, bool orientStats = false);

        bool begin(uint8_t address = ADR_DEFAULT,
                   SensitivityMode sensitivity = SENSITIVITY_BALANCED);

        // --- Configuration setters ---
        void setNRangeReadings(uint16_t n);
        void setRangeStats(bool enable);
        void setNOrientReadings(uint16_t n);
        void setOrientStats(bool enable);
        void setRangefinderSensitivity(SensitivityMode mode);

        bool updateRange();

        bool updateOrientation();

        bool updateMeasurements();

        // --- Single-value getters ---
        int16_t getRange();
        float getRoll();
        float getPitch();

        // --- Statistics getters ---
        float getRangeMean();
        float getRangeStd();
        float getRangeSterr();
        float getPitchStd();
        float getPitchSterr();
        float getRollStd();
        float getRollSterr();

        String getHeader();

        String getString(bool takeNewReadings = true);

        // --- Raw reading interface (NW standard) ---
        void beginRawReadings(uint8_t component = NW_READING_ALL);

        uint16_t takeRawReading(char* buf, uint16_t offset);

        void endRawReadings();

    private:
        void _waitUntilReady();

        // I2C address
        uint8_t _adr = ADR_DEFAULT;

        // Configuration
        uint16_t _nRangeReadings;
        bool     _rangeStats;
        uint16_t _nOrientReadings;
        bool     _orientStats;

        // Stored measurements and statistics.
        // All initialised to APIS_NOT_MEASURED; set to APIS_ERROR on error.
        int16_t _range = APIS_NOT_MEASURED;
        float   _pitch = APIS_NOT_MEASURED;
        float   _roll  = APIS_NOT_MEASURED;

        // Range statistics
        float _rangeMean  = APIS_NOT_MEASURED;
        float _rangeStd   = APIS_NOT_MEASURED;
        float _rangeSterr = APIS_NOT_MEASURED;

        // Orientation statistics
        float _pitchStd   = APIS_NOT_MEASURED;
        float _pitchSterr = APIS_NOT_MEASURED;
        float _rollStd    = APIS_NOT_MEASURED;
        float _rollSterr  = APIS_NOT_MEASURED;

        // Sensor sensitivity; set initially to default "balanced" mode
        SensitivityMode _sensitivity = SENSITIVITY_BALANCED;

        // True after begin(); cleared after _waitUntilReady() fires once.
        bool _needsStartupDelay = true;

        // Raw reading state
        uint8_t _rawComponent = NW_READING_ALL;
};

#endif
```


-------------------------------

Updated on 2026-05-11 at 23:56:23 +0000
