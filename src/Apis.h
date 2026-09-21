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

#define ADR_DEFAULT 0x50

// Register map (firmware v0.2.0+, 32-byte layout).
// Registers not listed here are reserved.
#define REG_STATUS      0x00  // Status flags; bit 0 = ready (poll in _waitUntilReady)
#define REG_NAME_0      0x01  // 'A' — checked by begin() to detect old firmware
#define REG_NAME_1      0x02  // 'p'
#define REG_NAME_2      0x03  // 'i'
#define REG_NAME_3      0x04  // 's'
#define REG_HW_MAJOR    0x05  // Hardware version major
#define REG_HW_MINOR    0x06  // Hardware version minor
#define REG_FW_PATCH    0x07  // Firmware patch version
#define REG_RANGE_L     0x08  // Range low byte  (little-endian int16, cm)
#define REG_RANGE_H     0x09  // Range high byte
#define REG_SIGNAL_STR  0x0A  // LiDAR Lite signal strength (uint8_t, from LiDAR Lite reg 0x0E)
#define REG_CONFIG      0x0B  // Sensitivity mode bits [1:0], writable
#define REG_I2C_ADDR    0x0C  // I2C address, writable; firmware saves to EEPROM byte 6,
                              //   takes effect on next boot; falls back to 0x50 if 0xFF
#define REG_ACCEL_BASE  0x10  // Accel raw X low byte; X/Y/Z span 0x10–0x15, little-endian int16
#define REG_OFFSET_BASE 0x18  // Accel offset X low byte; X/Y/Z span 0x18–0x1D, little-endian int16

/**
 * @brief Sensitivity mode for the LiDAR Lite acquisition pipeline.
 * @details Written to REG_CONFIG (0x0B) by begin(); applied on every
 * loop() iteration when the firmware reinitialises the LiDAR Lite via
 * InitLiDAR(). Two LiDAR Lite registers drive the behaviour:
 *   - SIG_COUNT_VAL (0x02): maximum acquisition count per measurement.
 *     Higher values average more returns, extending usable range but
 *     slowing throughput.
 *   - THRESHOLD_BYPASS (0x1C): signal detection threshold. Lower values
 *     detect weaker returns (higher sensitivity, more false positives);
 *     higher values suppress weak returns (fewer false positives, less range).
 */
enum SensitivityMode : uint8_t {
    SENSITIVITY_BALANCED  = 0, ///< Default. SIG_COUNT_VAL=0x80, THRESHOLD_BYPASS=0x00.
                               ///<   Balanced range and noise performance.
    SENSITIVITY_HIGH      = 1, ///< THRESHOLD_BYPASS=0x80. Lower detection threshold;
                               ///<   detects weaker returns at the cost of more
                               ///<   false positives.
    SENSITIVITY_LOW       = 2, ///< THRESHOLD_BYPASS=0xB0. Higher detection threshold;
                               ///<   fewer false positives at the cost of reduced range.
    SENSITIVITY_MAX_RANGE = 3  ///< SIG_COUNT_VAL=0xFF. More acquisitions per measurement;
                               ///<   longer maximum range, slower throughput.
};

/// Sentinel returned by all getters and printed by getString() when a
/// measurement fails due to hardware fault, I2C failure, or out-of-range
/// reading. Applies to all measurement types: range, pitch, roll, and all
/// derived statistics (mean, std, sterr).
#define APIS_ERROR        -9999

/// Sentinel returned by all getters and printed by getString() when begin()
/// has been called but no successful updateMeasurements() (or
/// updateRange()/updateOrientation()) has yet completed. Distinct from
/// APIS_ERROR so callers can tell the difference between "the sensor failed"
/// and "we haven't asked yet."
#define APIS_NOT_MEASURED -9998

// Deprecated component selectors. Use the class-scoped Apis::ALL, Apis::RANGE,
// Apis::ORIENT instead (see Apis::Component). Kept so existing sketches compile;
// values match the enum and will be removed in a future major version.
#ifndef NW_READING_ALL
  #define NW_READING_ALL       0
  #define NW_READING_PRIMARY   1
  #define NW_READING_SECONDARY 2
#endif
#define NW_READING_RANGE  NW_READING_PRIMARY
#define NW_READING_ORIENT NW_READING_SECONDARY

/**
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
 */
class Apis
{
    public:
        /**
         * @brief Measurement group: which on-board chip a reading covers.
         * @details One group per chip, in the order the NW-Device-Specification
         * Apis appendix numbers them: 0 = LiDAR Lite (range, signal strength),
         * 1 = LIS2DH12 accelerometer (pitch, roll). ALL selects every chip.
         * Written as Apis::RANGE etc. at the call site.
         */
        enum Component : uint8_t {
            ALL    = 0,   ///< Every chip: range, then pitch and roll.
            RANGE  = 1,   ///< LiDAR Lite only.
            ORIENT = 2    ///< Accelerometer only.
        };

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
         * @brief Begin communications with the Apis using a prescribed address.
         * @details Checks that the device acknowledges on I2C, verifies that
         * the firmware reports the "Apis" device name at registers 0x01–0x04
         * (indicating firmware v0.2.0+ with the current register map), and
         * writes the initial sensitivity mode to REG_CONFIG (0x0B).
         * Returns false if: (a) no I2C ACK, or (b) name mismatch (old
         * firmware). The minimum firmware version policy beyond that name
         * check is TBD.
         * @param address I2C address (default ADR_DEFAULT = 0x50).
         * @param sensitivity One of the SensitivityMode values
         * (default SENSITIVITY_BALANCED). Written to REG_CONFIG (0x0B)
         * and applied each firmware loop() iteration. See SensitivityMode
         * for descriptions of each mode.
         * @return true if the device acknowledges and reports correct firmware;
         *         false otherwise.
         */
        bool begin(uint8_t address = ADR_DEFAULT,
                   SensitivityMode sensitivity = SENSITIVITY_BALANCED);

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
         * @brief Change the rangefinder sensitivity mode after begin().
         * @details Writes the new mode to REG_CONFIG (0x0B); the firmware
         * applies it on the next loop() iteration via InitLiDAR(). Must be
         * called after begin(); if called before, Wire is uninitialised and
         * the transmission fails silently.
         * @param mode One of the SensitivityMode values.
         */
        void setRangefinderSensitivity(SensitivityMode mode);

        /**
         * @brief Write a new I2C address to the device.
         * @details The firmware saves it to EEPROM byte 6 and uses it on the
         * next boot; the current session continues on the old address. Falls
         * back to 0x50 if EEPROM byte 6 is 0xFF (erased).
         * @param newAddress The 7-bit I2C address to persist.
         */
        void setI2CAddress(uint8_t newAddress);

        /**
         * @brief Measure range [cm] only, without reading the accelerometer.
         * Intended for rapid repeated range readings (e.g. for averaging).
         * Also updates the cached signal strength (see getSignalStrength()).
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
         * @brief Take a reading: measure range [cm], roll [deg] and pitch [deg],
         * or one chip's measurements alone.
         * Uses Welford's online algorithm to compute mean, std, and sterr
         * over nRangeReadings and nOrientReadings respectively.
         * @param component Apis::ALL (default), Apis::RANGE, or Apis::ORIENT:
         * which chip(s) to read. Fields of a chip not selected are left as
         * they were.
         * @return false if any selected chip returned only error values.
         */
        bool updateMeasurements(uint8_t component = ALL);

        /**
         * @brief Number of valid range readings in the last updateMeasurements()
         * call (0 to nRangeReadings). The statistics are computed over these.
         */
        uint16_t getRangeCount();
        /** @brief Number of valid orientation readings in the last updateMeasurements(). */
        uint16_t getOrientCount();

        // --- Single-value getters ---
        /** @brief Return range mean [cm], rounded to nearest cm. */
        int16_t getRange();
        /** @brief Return roll mean [deg]. */
        float getRoll();
        /** @brief Return pitch mean [deg]. */
        float getPitch();
        /**
         * @brief Return the most recently cached LiDAR Lite signal strength.
         * @details Updated by every call to updateRange() or updateMeasurements().
         * Returns 0 before the first measurement.
         */
        uint8_t getSignalStrength();

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
         * Error values are APIS_ERROR (-9999) for sensor errors and
         * APIS_NOT_MEASURED (-9998) when no measurement has yet been taken.
         * @param takeNewReadings if true, run updateMeasurements() before
         * returning values. Otherwise, return stored values.
         */
        String getString(bool takeNewReadings = true);

        // --- Reading interface (NW standard) ---
        /**
         * @brief Print the header matching printReading(): column names with
         * units, each followed by a comma, for the chips selected by
         * beginReadings(). No statistics columns: one reading has none.
         * @param out Any Print destination (SdFat File, Serial, ...).
         * @return Bytes written.
         */
        size_t printHeader(Print& out);

        /**
         * @brief Print the stored reading of the selected chips, each value
         * followed by a comma. Does not acquire: call updateRange(),
         * updateOrientation(), or updateMeasurements() first, or use
         * logReading(). Writes: range [cm] for RANGE; pitch [deg], roll [deg]
         * for ORIENT; range, pitch, roll for ALL.
         * @return Bytes written.
         */
        size_t printReading(Print& out);

        /**
         * @brief Take ONE reading of the selected chips and print it: the
         * one-reading primitive for collecting many readings to a file. Uses
         * updateRange()/updateOrientation(), not updateMeasurements(), so each
         * call is a single acquisition regardless of nRangeReadings.
         * @return Bytes written.
         */
        size_t logReading(Print& out);

        /**
         * @brief Begin a run of readings, selecting which chips they cover.
         * @param component Apis::ALL, Apis::RANGE, or Apis::ORIENT.
         */
        void beginReadings(uint8_t component = ALL);

        /** @brief End a run of readings. */
        void endReadings();

        // --- Deprecated raw-reading interface (v0.1.x names) ---
        /** @deprecated Use beginReadings(). */
        [[deprecated("Use beginReadings()")]]
        void beginRawReadings(uint8_t component = ALL);

        /**
         * @deprecated Use logReading(Print&) with an SdFat File, Serial, or a
         * buffer-backed Print. Kept for v0.1.x sketches: takes one raw reading
         * and writes CSV into buf at offset (range for RANGE; pitch, roll for
         * ORIENT; all three for ALL). Max bytes: 7 (RANGE), 18 (ORIENT), 25 (ALL).
         * @return New offset after writing.
         */
        [[deprecated("Use logReading(Print&)")]]
        uint16_t takeRawReading(char* buf, uint16_t offset);

        /** @deprecated Use endReadings(). */
        [[deprecated("Use endReadings()")]]
        void endRawReadings();

    private:
        /**
         * @brief Poll REG_STATUS until bit 0 is set, or 150 ms elapses.
         * @details Power-on startup sequence (from board power arriving):
         *   1. TLV61220 boost converter starts immediately (EN tied to VIN+);
         *      outputs stable 5V within ~2 ms. No firmware action needed.
         *   2. MIC5365 LDO derives 3.3V from the 5V rail; ATTiny1634 starts.
         *   3. Firmware setup(): delay(10) -> POWER_SW high (MIC2544 enables,
         *      680 uF cap charges at ~227 mA over ~15 ms) -> delay(100) ->
         *      ENABLE high -> InitAccel() -> InitLiDAR(). Total: ~115 ms.
         *   4. Wire.begin() is called early in setup(), so the ATTiny is
         *      I2C-addressable before it has finished initialising the LiDAR.
         *      A library call arriving during this window would find the sensor
         *      not yet ready.
         * Firmware sets REG_STATUS bit 0 after InitLiDAR() completes,
         * allowing the library to exit the poll immediately rather than waiting
         * a fixed time. Old firmware leaves REG_STATUS=0 always; the 150 ms
         * timeout then covers the full firmware startup with margin.
         * See: https://github.com/NorthernWidget/Project-Apis/issues/15
         */
        void _waitUntilReady();

        /**
         * @brief Read n consecutive registers starting at reg into buf, in one
         * I2C transaction (pointer write, then requestFrom with auto-increment).
         * @return true if the device supplied all n bytes.
         */
        bool _readBytes(uint8_t reg, uint8_t* buf, uint8_t n);

        /** @brief Write one byte to register reg. @return true on ACK. */
        bool _writeByte(uint8_t reg, uint8_t value);

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

        // Valid readings behind the current statistics (set by updateMeasurements)
        uint16_t _rangeCount  = 0;
        uint16_t _orientCount = 0;

        // LiDAR Lite signal strength; updated by updateRange()
        uint8_t _signalStrength = 0;

        // Sensor sensitivity; set initially to default "balanced" mode
        SensitivityMode _sensitivity = SENSITIVITY_BALANCED;

        // True after begin(); cleared after _waitUntilReady() fires once.
        bool _needsStartupDelay = true;

        // Chips covered by the current run of readings (beginReadings)
        uint8_t _rawComponent = ALL;
};

#endif
