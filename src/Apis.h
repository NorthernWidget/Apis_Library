/******************************************************************************
Apis.h

Library for the Apis interface board for a LiDAR Lite unit.

Andrew Wickert
Based on code by Bobby Schulz
In particular: https://github.com/NorthernWidget/Project-Apis/tree/master/Software/LiDARLite_I2CParse
and based loosely around Bobby Schulz' general functions for sensor libraries.

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

// Sensitivity
#define sensitivityBalanced  0
#define sensitivityHigh      1
#define sensitivityLow       2
#define sensitivityMaxRange  3

// Sentinel values used by all getters and getString() output:
//   -9999  Sensor error: hardware fault, I2C failure, or out-of-range reading.
//   -9998  Not yet measured: begin() has been called but no successful
//          updateMeasurements() (or updateRange()/updateOrientation()) has
//          completed. Distinct from -9999 so callers can tell the difference
//          between "the sensor failed" and "we haven't asked yet."
// Both sentinels apply to all measurement types: range, pitch, roll,
// and all statistics (mean, std, sterr).

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

/**
 * @class Apis Class to Interface with the Apis module for a laser rangefinder.
 * @brief Arduino library for the Apis board, which manages a LiDAR Lite
 * unit (roll/pitch, firmware lock/reset, power supply).
 * @details Library to communicate with the Apis module, which
 * connects to a LiDAR Lite rangefinder. The Apis is equipped with
 * capacitors to handle the large burst power draw from the LiDAR Lite, a MEMS
 * accelerometer to note its orientation, a magnet to note a known orientation
 * (often, but not necessarily, horizontal) and the ability to absorb
 * occasional firmware issues that lead to system hangs.
 * The leveling helps the user to calculate, for example, a water level
 * when the sensor is placed on a cliff or a tree next to the river but does not
 * have water below it. The level loses absolute accuracy when near plumb, so
 * a Hall-effect sensor connected to the magnet allows the user to set a zero
 * value, thereby correcting for this. Managing failures of the LiDAR Lite
 * within the Apis is essential, and the Apis therefore acts as a
 * buffer to protect the data logger from raw sensor failures.
 *
 * \verbatim [![DOI](https://zenodo.org/badge/260392682.svg)](https://zenodo.org/badge/latestdoi/260392682) \endverbatim
 */
class Apis
{
    public:
        /**
         * @brief Instantiate Apis object.
         * @param nRangeReadings Number of range readings to average (default 1).
         * Paul et al. (2020, WRR, doi:10.1029/2019WR026810) found ~1000
         * readings needed for stable mean convergence under field conditions.
         * Uses Welford's online algorithm: O(1) memory regardless of count.
         * FIX: Independent readings require firmware support for on-demand
         * triggering; current firmware caches the value each loop (~200 ms).
         * @param rangeStats If true, getString() includes range std and sterr.
         * Only meaningful when nRangeReadings > 1.
         * @param nOrientReadings Number of orientation readings to average
         * (default 1).
         * FIX: Independent readings require firmware support; current firmware
         * caches accelerometer values each loop (~200 ms).
         * @param orientStats If true, getString() includes orientation std and
         * sterr. Only meaningful when nOrientReadings > 1.
         */
        Apis(uint16_t nRangeReadings = 1, bool rangeStats = false,
             uint16_t nOrientReadings = 1, bool orientStats = false);

        /**
         * @brief Begin communications with the Apis using a prescribed
         * address.
         * @param address default 0x50
         * DOES NOT YET USE A VARIABLE ADDRESS!
         * DOES NOT MATTER WHAT YOU WRITE HERE.
         * THIS DEFAULT ADDRESS CURRENTLY CLASHES WITH HAAR'S DEFAULT!
         * @param sensitivity Options are:
         * sensitivityBalanced (0)
         * sensitivityHigh (1)
         * sensitivityLow (2)
         * sensitivityMaxRange (3)
         * Default value: sensitivityBalanced (0)
         * THIS IS ALSO NOT YET IMPLEMENTED IN FIRMWARE! JUST STAYS AT DEFAULT
         * (SENSITIVITY_BALANCED = 0)
         * CONSIDER A SET_SENSITIVITY FUNCTION IN THE FUTURE TO ALLOW THE
         * SENSOR TO AUTOMATICALLY ADJUST THE SENSITIVITY TO RECEIVE THE BEST
         * POSSIBLE RETURN
         * @return true if the device acknowledges on I2C, false otherwise.
         */
        bool begin(uint8_t address = ADR_DEFAULT,
                   uint8_t sensitivity = sensitivityBalanced);

        // --- Configuration setters ---
        /** @brief Set number of range readings to average. */
        void setNRangeReadings(uint16_t n);
        /** @brief Enable or disable range std and sterr in getString(). */
        void setRangeStats(bool enable);
        /** @brief Set number of orientation readings to average. */
        void setNOrientReadings(uint16_t n);
        /** @brief Enable or disable orientation std and sterr in getString(). */
        void setOrientStats(bool enable);

        /**
         * @brief Measure range [cm] only, without reading the accelerometer.
         * Intended for rapid repeated range readings (e.g. for averaging).
         * Returns false if the sensor returns an error value.
         */
        bool updateRange();

        /**
         * @brief Measure pitch [deg] and roll [deg] only, without ranging.
         * Intended for rapid repeated orientation readings (e.g. for averaging).
         * FIX: Independent readings require firmware support; current firmware
         * caches accelerometer values each loop (~200 ms).
         * Returns false if the sensor returns an error value.
         */
        bool updateOrientation();

        /**
         * @brief Measure range [cm], roll [deg] and pitch [deg].
         * Uses Welford's online algorithm to compute mean, std, and sterr
         * over nRangeReadings and nOrientReadings respectively.
         * Returns false if any sensor returns an error value.
         */
        bool updateMeasurements();

        // --- Single-value getters ---
        /** @brief Return range mean [cm], rounded to nearest cm. */
        int16_t getRange();
        /** @brief Return roll mean [deg]. */
        float getRoll();
        /** @brief Return pitch mean [deg]. */
        float getPitch();

        // --- Statistics getters ---
        /** @brief Return range mean [cm] as float. */
        float getRangeMean();
        /** @brief Return range standard deviation [cm]. */
        float getRangeStd();
        /** @brief Return range standard error [cm]. */
        float getRangeSterr();
        /** @brief Return pitch standard deviation [deg]. */
        float getPitchStd();
        /** @brief Return pitch standard error [deg]. */
        float getPitchSterr();
        /** @brief Return roll standard deviation [deg]. */
        float getRollStd();
        /** @brief Return roll standard error [deg]. */
        float getRollSterr();

        /**
         * @brief Return a comma-separated header matching getString() output.
         * Includes statistics columns when rangeStats or orientStats are
         * enabled and nReadings > 1.
         */
        String getHeader();

        /**
         * @brief Return comma-separated data values.
         * @details This is the most likely function (alongside getHeader) for
         * an end user to use.
         * Always includes: Range [cm], Pitch [deg], Roll [deg].
         * Appends range std and sterr when rangeStats is true and
         * nRangeReadings > 1.
         * Appends orientation std and sterr when orientStats is true and
         * nOrientReadings > 1.
         * Error values are "-9999" (sensor error) and "-9998" (no measurement
         * yet taken).
         * @param takeNewReadings if true, run updateMeasurements() before
         * returning values. Otherwise, return stored values.
         */
        String getString(bool takeNewReadings = true);

        // --- Raw reading interface (NW standard) ---
        /**
         * @brief Prepare for raw reading collection.
         * @param component NW_READING_ALL, NW_READING_RANGE (primary), or
         * NW_READING_ORIENT (secondary).
         */
        void beginRawReadings(uint8_t component = NW_READING_ALL);

        /**
         * @brief Take one raw reading and write CSV data into buf at offset.
         * Writes: range [cm] for RANGE; pitch [deg], roll [deg] for ORIENT;
         * range, pitch, roll for ALL. Each value followed by a comma.
         * Max bytes written per call: 7 (RANGE), 18 (ORIENT), 25 (ALL).
         * @param buf Caller-managed destination buffer.
         * @param offset Starting write position in buf.
         * @return New offset after writing.
         */
        uint16_t takeRawReading(char* buf, uint16_t offset);

        /** @brief End raw reading collection. */
        void endRawReadings();

    private:
        /**
         * @brief Poll Reg[0] until firmware signals ready, or 150 ms elapses.
         * @details Power-on startup sequence (from board power arriving):
         *   1. TLV61220 boost converter starts immediately (EN tied to VIN+);
         *      outputs stable 5V within ~2 ms. No firmware action needed.
         *   2. MIC5365 LDO derives 3.3V from the 5V rail; ATTiny1634 starts.
         *   3. Firmware setup(): delay(10) → POWER_SW high (MIC2544 enables,
         *      680 µF cap charges at ~227 mA over ~15 ms) → delay(100) →
         *      ENABLE high → InitAccel() → InitLiDAR(). Total: ~115 ms.
         *   4. Wire.begin() is called early in setup(), so the ATTiny is
         *      I2C-addressable before it has finished initialising the LiDAR.
         *      A library call arriving during this window would find the sensor
         *      not yet ready.
         * New firmware (Issue #15) sets Reg[0]=1 after InitLiDAR() completes,
         * allowing the library to exit the poll immediately rather than waiting
         * a fixed time. Old firmware leaves Reg[0]=0 always; the 150 ms timeout
         * then covers the full firmware startup with margin.
         * See: https://github.com/NorthernWidget/Project-Apis/issues/15
         */
        void _waitUntilReady();

        // I2C address
        uint8_t _adr = ADR_DEFAULT;

        // Configuration
        uint16_t _nRangeReadings;
        bool     _rangeStats;
        uint16_t _nOrientReadings;
        bool     _orientStats;

        // Stored measurements and statistics.
        // All initialised to -9998 (not yet measured); set to -9999 on error.
        int16_t _range = -9998;
        float   _pitch = -9998;
        float   _roll  = -9998;

        // Range statistics
        float _rangeMean  = -9998;
        float _rangeStd   = -9998;
        float _rangeSterr = -9998;

        // Orientation statistics
        float _pitchStd   = -9998;
        float _pitchSterr = -9998;
        float _rollStd    = -9998;
        float _rollSterr  = -9998;

        // Sensor sensitivity; set initially to default "balanced" mode
        uint8_t _sensitivity = sensitivityBalanced;

        // True after begin(); cleared after _waitUntilReady() fires once.
        bool _needsStartupDelay = true;

        // Raw reading state
        uint8_t _rawComponent;
};

#endif
