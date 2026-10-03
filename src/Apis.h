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
#include <NW_Core.h>   // NW_Core: NW_Device (Schema 1 protocol), NW_Readings (statistics), NW_Report

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// Minimum firmware patch (Page 0 byte 0x0A) this library accepts. Patch 1 is
// the first firmware serving the Schema 1 register map; patch 5 serves the
// zero generation with every reading, which this library reads.
#define APIS_FW_MIN_PATCH 5

// Build identity: this library's version (held equal to library.properties by
// NW-Tests/version_check.py) and its build commit, set by the NW-Build wrapper from
// git and blank in an Arduino IDE build. Both go into a logger's status file.
#define APIS_LIBRARY_VERSION "0.1.0"
#ifndef APIS_LIBRARY_COMMIT
#define APIS_LIBRARY_COMMIT ""
#endif

// Register addresses and bit masks are implementation details and live in
// Apis.cpp (NW convention: no public names for them). Sketches use the API.

// Reading arrays: each measurement keeps its readings from the last
// updateMeasurements() in a statically sized array so that median and
// two-pass statistics can be computed. Capacity is per measurement group;
// a sketch may override before including this header, e.g.
//   #define APIS_RANGE_CAPACITY 200
// No heap is ever used; a request above capacity is clamped to it.
#ifndef APIS_RANGE_CAPACITY
  #define APIS_RANGE_CAPACITY 64
#endif
// Digits to m/s^2 for the accelerometer. The firmware sets CTRL_REG4 = 0x88,
// which is full scale +/-2 g with high resolution, and LIS3DH Table 4 gives
// that configuration a sensitivity of 1 mg per digit; the firmware right-
// shifts the 12-bit word into place before serving it, so a served digit is
// one milli-g. Standard gravity is 9.80665 m/s^2.
#define APIS_ACCEL_M_PER_S2_PER_DIGIT (9.80665e-3f)

#ifndef APIS_ORIENT_CAPACITY
  #define APIS_ORIENT_CAPACITY 8
#endif

/**
 * @brief Sensitivity mode for the LiDAR Lite acquisition pipeline.
 * @details Written to REG_CONFIG (0x46) by begin(); applied on every
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
/// derived statistics (mean, std, sterr). The same value as NW_ERROR, which
/// every NW library uses; APIS_ERROR is the Apis spelling of it.
#define APIS_ERROR        NW_ERROR

/// Sentinel returned by all getters and printed by getString() when begin()
/// has been called but no successful updateMeasurements() (or
/// updateDistance()/updateOrientation()) has yet completed. Distinct from
/// APIS_ERROR so callers can tell the difference between "the sensor failed"
/// and "we haven't asked yet."
#define APIS_NOT_MEASURED -9998

// Deprecated component selectors. Use the class-scoped Apis::ALL, Apis::RANGE,
// Apis::ORIENT instead (see Apis::Component). Kept so existing sketches compile;
// values match the enum and will be removed in a future major version.

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
class Apis : public NW_Sensor
{
    public:
        /** @brief Default I2C address: NW-Device-Specification Schema 1 'A' (0x41). Firmware v0.1.x used 0x50. */
        static constexpr uint8_t DEFAULT_ADDRESS = 0x41;

        /**
         * @brief Measurement group: which on-board chip a reading covers.
         * @details One group per chip, in the order the NW-Device-Specification
         * Apis appendix numbers them: 0 = LiDAR Lite (range, signal strength),
         * 1 = LIS3DH accelerometer (pitch, roll). ALL selects every chip.
         * Written as Apis::RANGE etc. at the call site.
         */
        enum Component : uint8_t {
            RANGE  = 0x01,  ///< LiDAR Lite only (chip 0: the control chip-select bit).
            ORIENT = 0x02,  ///< Accelerometer only (chip 1).
            ALL    = 0x03   ///< Every chip: range, then pitch and roll.
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
         * @details Checks that the device acknowledges on I2C, then reads
         * Page 0 Blocks 0–1 and requires: schema byte 0x01 (Schema 1); the
         * name "Apis"; a firmware patch of at least APIS_FW_MIN_PATCH. Stores
         * the hardware and firmware versions for the getters below, and writes
         * the initial sensitivity mode to REG_CONFIG (0x46).
         * @param address I2C address (default DEFAULT_ADDRESS = 0x41).
         * @param sensitivity One of the SensitivityMode values
         * (default SENSITIVITY_BALANCED). See SensitivityMode.
         * @return true if the device acknowledges and passes all three checks;
         *         false otherwise. Call getFirmwareVersion() after a refusal
         *         to see what the device reported (0 if unreadable).
         */
        bool begin(uint8_t address = DEFAULT_ADDRESS,
                   SensitivityMode sensitivity = SENSITIVITY_BALANCED);

        /** @brief Hardware version major, from Page 0 (valid after begin()). */
        uint8_t getHardwareMajor();
        /** @brief Hardware version minor, from Page 0 (valid after begin()). */
        uint8_t getHardwareMinor();
        /** @brief Firmware patch version, from Page 0 (valid after begin(), even when it refused). */
        uint8_t getFirmwareVersion();

        // --- Configuration setters ---
        /**
         * @brief Set the number of range readings taken per updateMeasurements()
         * (statistics are computed over them). Clamped to APIS_RANGE_CAPACITY.
         * @return The number actually set.
         */
        uint16_t setDistanceReadings(uint16_t n);
        /** @brief Enable or disable range std and sterr in getString(). */
        void setDistanceStats(bool enable);
        /** @brief Set the number of orientation readings per updateMeasurements(). Clamped to APIS_ORIENT_CAPACITY. */
        uint16_t setOrientationReadings(uint16_t n);
        /** @brief Include the g vector's X, Y and Z columns in getString() and getHeader(). Off by default. */
        void setAccelerationColumns(bool enable);
        /** @brief Include the acceleration magnitude column, and its statistics when orientation statistics are on. Off by default. */
        void setMagnitudeColumns(bool enable);
        /** @brief Include the tilt column, and its statistics when orientation statistics are on. Off by default. */
        void setTiltColumns(bool enable);
        /** @brief Enable or disable orientation std and sterr in getString(). */
        void setOrientationStats(bool enable);
        /**
         * @brief Change the rangefinder sensitivity mode after begin().
         * @details Writes the new mode to REG_CONFIG (0x46); the firmware
         * applies it on the next loop() iteration via InitLiDAR(). Must be
         * called after begin(); if called before, Wire is uninitialised and
         * the transmission fails silently.
         * @param mode One of the SensitivityMode values.
         */
        void setRangefinderSensitivity(SensitivityMode mode);

        /**
         * @brief Write a new I2C address to the device.
         * @details The firmware persists it in Page 0 byte 0x1F and uses it on
         * the next boot; the current session continues on the old address.
         * Falls back to 0x41 if that byte is 0xFF (unprogrammed).
         * @param newAddress The 7-bit I2C address to persist.
         */
        void setI2CAddress(uint8_t newAddress);

        // --- Handshake (NW-Device-Specification Page 2 Block 0) ---
        /** @brief True when the status byte's ready bit is set: the data registers hold a complete reading. */
        bool ready();

        /**
         * @brief True when the device's reading counter has advanced since
         * this library last stored a reading, i.e. a reading is available that
         * has not been read yet. Does not acquire.
         */
        bool newReading();

        /**
         * @brief Ask the device for a reading now, of the selected chips.
         * Writes the control register: trigger bit plus the chip-select bits
         * for the component. The device clears ready, measures, and sets ready
         * again with the counter incremented. A write to control also clears
         * the report byte (acknowledgement).
         * @return true if the device acknowledged the write.
         */
        bool requestReading(uint8_t component = ALL);

        // --- Faults (status byte, live; Report register, latched) ---
        /**
         * @brief True if the given chip (0 = LiDAR, 1 = accelerometer) was
         * faulted in the status byte of the last reading taken.
         */
        bool faulted(uint8_t chip);
        /** @brief True if any chip was faulted in the last reading (status pan-fault bit). */
        bool anyFault();
        /**
         * @brief Chip index of the most recent report (0 LiDAR,
         * 1 accelerometer, 7 the unit itself), from the Report register read with
         * the last reading; meaningful only when reportKind() != 0.
         */
        uint8_t reportChip();
        /**
         * @brief Kind of the most recent report, per the spec's table:
         * 0 none, 1 no-acknowledge, 2 timeout, 3 checksum, 4 out of range,
         * 5 not initialised, 6 reset since the controller last wrote control,
         * 7 configuration rejected, 8 supply fault, 16–31 device-specific.
         * The device clears it on the next control write (requestReading()).
         */
        uint8_t reportKind();
        /**
         * @brief Print the report as text, e.g. "LiDAR: timeout" or
         * "unit: restarted since configured"; prints "none" when there is no fault.
         * @return Bytes written.
         */
        size_t printReport(Print& out);
        /**
         * @brief The report as one word for a data-table note column,
         * chip then kind: "LiDARTimeout", "AccelNotAnswering", "UnitRestarted";
         * "UnitNone" when there is no fault (check anyFault() first).
         */
        String reportNote();
        /**
         * @brief Print one status line for a logger's status file: name, serial,
         * versions, the last report (code and note), and Pages 0, 1 and 2 (identity, calibration, data) in hex,
         * comma separated, no newline. The logger prints its timestamp first.
         * Three page reads, no write: the report is not acknowledged.
         */
        size_t printStatus(Print& out, bool boot = false) override;
        // --- NW_Sensor: the logger's view (Margay::watch) ---
        const char* name() const override { return "Apis"; }
        bool reportIsFault() override;
        uint8_t bootReportKind() override;
        void clearBootReport() override;
        /**
         * @brief Why the last begin() refused, as one word: "NotAnswering",
         * "NotSchema1", "WrongName", "OldFirmware", "ReadFailed"; "None"
         * after a successful begin().
         */
        String beginFailure();

        /**
         * @brief Take one range reading [cm]: request it, wait for the device's
         * reading counter to advance (up to the timeout), then read range and
         * signal strength. Each call is a distinct acquisition, so repeated
         * calls give independent readings for statistics.
         * Returns false on timeout, bus error, or a negative (error) range.
         */
        bool updateDistance();

        /**
         * @brief Take one orientation reading: request it, wait for the
         * counter to advance, then read the accelerometer and offsets and
         * compute pitch [deg] and roll [deg]. Each call is a distinct
         * acquisition.
         * Returns false on timeout, bus error, or the accelerometer failure
         * signature.
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
        uint16_t getDistanceCount();
        /** @brief Number of valid orientation readings in the last updateMeasurements(). */
        uint16_t getOrientationCount();

        // --- Single-value getters ---
        /** @brief Distance [cm] as logged: the mean over the burst, or the single
         *  reading when no burst ran. A float like every other measurement getter,
         *  so the burst mean keeps its fraction; the device itself resolves whole
         *  centimetres. The same number as getDistanceMean() when a burst ran. */
        float getDistance();
        /** @brief Return roll mean [deg]. */
        float getRoll();
        /** @brief Return pitch mean [deg]. */
        float getPitch();
        /**
         * @brief Acceleration along the accelerometer X axis [m/s^2].
         * @details The last reading rather than a burst mean, as
         * getSignalStrength() is: the three components carry no statistics
         * columns. getAccelerationMagnitude() is a burst mean, so it does not
         * equal the magnitude of these three when more than one reading was
         * taken. APIS_NOT_MEASURED before the first reading, APIS_ERROR when it
         * failed.
         */
        float getAccelerationX();
        /** @brief Acceleration along the accelerometer Y axis [m/s^2]; see getAccelerationX(). */
        float getAccelerationY();
        /** @brief Acceleration along the accelerometer Z axis [m/s^2]; see getAccelerationX(). */
        float getAccelerationZ();
        /**
         * @brief Magnitude of the acceleration vector [m/s^2], mean over the burst.
         * @details Reads 9.81 whenever the unit is static, so a departure means
         * the Apis moved or the accelerometer is faulted. Pitch and roll cannot
         * report that: a wrong g vector still yields plausible angles.
         */
        float getAccelerationMagnitude();
        /**
         * @brief Angle between the housing and its reference [deg], mean over the burst.
         * @details Measured from the stored zero when one has been taken, and
         * from vertical when none has, which is the same branch getPitch() and
         * getRoll() take. One number for a levelling correction; pitch and roll
         * stay, because recovering the direction of tilt needs both.
         */
        float getTilt();
        /**
         * @brief Return the most recently cached LiDAR Lite signal strength.
         * @details Updated by every call to updateDistance() or updateMeasurements().
         * Returns 0 before the first measurement. It is a column of its own
         * (Andy, 2026-10-01), beside the distance it qualifies: a weak return
         * explains a bad range, and a reader with only the range cannot tell a
         * poor target from a moved one. One byte, not averaged, so it carries
         * the bare standard name rather than a mean_of_ form. A failed reading
         * leaves 0, which is also what the firmware writes when the LiDAR will
         * not power up; a uint8_t cannot carry the -9999 sentinel.
         */
        uint8_t getSignalStrength();
        /**
         * @brief The accelerometer's own temperature at the last orientation
         * reading, in the LIS3DH's relative digits (1 per degree C, no absolute
         * reference; firmware patch 3). Logged beside pitch and roll so that a
         * per-unit drift correction can be fitted later; APIS_NOT_MEASURED before
         * the first reading.
         */
        int16_t getAccelerometerTemperatureADC();
        /**
         * @brief Accelerometer temperature change [C] since the zero was stored.
         * @details The LIS3DH measures change, not temperature: its Table 5
         * specifies only the output change versus temperature, 1 digit per
         * degree C and not guaranteed, with no offset and no reference point.
         * So this is a departure from the temperature at which the Hall-effect
         * zero was taken, which is what the drift correction wants, and it is
         * APIS_NOT_MEASURED when no zero has ever been stored, because there is
         * then nothing to be a change from.
         */
        float getAccelerometerTemperatureChange();
        /** @brief The same digit at the moment the Hall-effect zero was taken (Page 1). */
        int16_t getZeroTemperatureADC();
        /**
         * @brief How many zeros the unit has stored since manufacture (0 =
         * never), from the copy the reading carries (0x58–0x59, firmware
         * patch 5). Read with the axes, so it belongs to the last orientation
         * reading; the value seen at begin() before then.
         *
         * It is a column of its own (Andy, 2026-10-01), last in the row beside
         * the accelerometer temperature change, which is the other zero-relative
         * fact. Pitch, roll and tilt are all measured against a stored zero, and
         * a re-zero mid-deployment silently changes what every later angle means.
         * The generation in each row is what lets an analyst segment the record:
         * equal generation means comparable angles. The firmware anticipates
         * this by mirroring the Page 1 value into Page 2 at 0x58 and serving it
         * with every reading.
         */
        uint16_t getZeroGeneration();
        /**
         * @brief True when the last orientation reading's generation differs
         * from the one seen at the reading before it, or at begin(), which
         * reads Page 1's generation once: the unit has stored a new zero.
         */
        bool zeroChanged();
        /**
         * @brief Print the record of zeros from Page 1: the current zero and
         * the two before it, newest first, one line each as
         * generation,X,Y,Z,T (offsets in counts; T the temperature digit at
         * the zero). Zeros never stored print nothing, so a unit zeroed once
         * prints one line. Reads calibration, not a reading: no acquisition.
         * @return Bytes written.
         */
        size_t dumpZeros(Print& out);

        // --- Statistics getters ---
        // Computed two-pass in 32-bit float over the readings stored by the last
        // updateMeasurements(). Adequate for N up to the array capacities
        // (tens to a few hundred); at N in the thousands the sum of squared
        // deviations would want double precision, which the AVR lacks.
        /** @brief Return range mean [cm] as float. */
        float getDistanceMean();
        /** @brief Return the median range [cm] of the stored readings (nearest cm for odd N; mean of the middle pair otherwise). */
        float getDistanceMedian();
        /** @brief Acceleration magnitude median [m/s^2]. */
        float getAccelerationMagnitudeMedian();
        /** @brief Tilt median [deg]. */
        float getTiltMedian();
        /** @brief Return the median pitch [deg] of the stored readings. */
        float getPitchMedian();
        /** @brief Return the median roll [deg] of the stored readings. */
        float getRollMedian();
        /** @brief Return range standard deviation [cm]. */
        float getDistanceStd();
        /** @brief Return range standard error [cm]. */
        float getDistanceSterr();
        /** @brief Acceleration magnitude standard deviation [m/s^2]. */
        float getAccelerationMagnitudeStd();
        /** @brief Acceleration magnitude standard error [m/s^2]. */
        float getAccelerationMagnitudeSterr();
        /** @brief Tilt standard deviation [deg]. */
        float getTiltStd();
        /** @brief Tilt standard error [deg]. */
        float getTiltSterr();
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
        /**
         * @brief Print the summary columns a logger writes: the means, with the
         * statistics columns each chip group has enabled.
         * @details The streaming form of getHeader(), and its definition: that
         * function prints through this one into a String. Pass a `File` to write
         * the card, `Serial` to write the monitor. Distinct from printHeader(),
         * which is the burst interface and carries no statistics.
         * @param out Where to print.
         * @return Bytes printed.
         */
        size_t printDataHeader(Print& out) override;

        /**
         * @brief Print one summary row, in printDataHeader()'s column order.
         * @details Takes no reading: it prints what the last updateMeasurements()
         * left, which is what lets a caller write the same row to two sinks
         * without acquiring twice.
         * @param out Where to print.
         * @return Bytes printed.
         */
        size_t printDataRow(Print& out) override;

        /**
         * @brief Come back on the bus after the logger cut the sensor rail to sleep.
         * @details One of the three calls a logger makes on a watched sensor
         * (LIBRARY-DESIGN.md section 14 step 4). It re-runs begin() at the address
         * this sensor was begun with.
         * @return True when the sensor answered and passed begin()'s gates.
         */
        bool wake() override;

        /**
         * @brief Take this row's readings and store them, for printDataRow() to print.
         * @return True when a reading was taken.
         */
        bool acquire() override;

        /**
         * @brief Print one word for the logger's Note column, with no comma.
         * @param beginFailed print why begin() refused, rather than what the last
         *        reading reported.
         * @return Bytes printed.
         */
        size_t printNote(Print& out, bool beginFailed = false) override;


        String getHeader();

        /**
         * @brief Return comma-separated data values.
         * @details This is the most likely function (alongside getHeader) for
         * an end user to use.
         * Always includes: Range [cm], Pitch [deg], Roll [deg], AccelT [C].
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
         * followed by a comma. Does not acquire: call updateDistance(),
         * updateOrientation(), or updateMeasurements() first, or use
         * logReading(). Writes: range [cm] for RANGE; pitch [deg], roll [deg]
         * for ORIENT; range, pitch, roll for ALL.
         * @return Bytes written.
         */
        size_t printReading(Print& out);

        /**
         * @brief Take ONE reading of the selected chips and print it: the
         * one-reading primitive for collecting many readings to a file. Uses
         * updateDistance()/updateOrientation(), not updateMeasurements(), so each
         * call is a single acquisition regardless of nRangeReadings.
         * @return Bytes written.
         */
        size_t logReading(Print& out);

        /**
         * @brief Begin a run of readings, selecting which chips they cover.
         * @param component Apis::ALL, Apis::RANGE, or Apis::ORIENT.
         * @param n How many readings the run will take (the number of
         * logReading() calls to follow). With n > 1 the device is told in
         * advance (readings-requested word, registers 0x44-0x45) and keeps the
         * LiDAR powered and initialised for exactly that many readings; with
         * 0 or 1 each reading powers the LiDAR up and down. A run that stops
         * short is abandoned by the device after 2 s with a report.
         */
        void beginReadings(uint8_t component = ALL, uint16_t n = 0);

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






        // The Schema 1 device protocol (identity gates, handshake, batches, faults): NW_Core.
        NW_Device _dev;
        /** @brief Chip-select mask for a component: bit 0 LiDAR, bit 1 accelerometer. */
        static uint8_t _chips(uint8_t component);
        // Configuration
        NW_ReadingsConfig _distanceCfg;    // readings per updateMeasurements() and stats columns, LiDAR
        NW_ReadingsConfig _orientationCfg;   // accelerometer
        bool _accelerationColumns = false;   // the g vector's components: off unless asked for
        bool _magnitudeColumns    = false;
        bool _tiltColumns         = false;

        // Stored measurements and statistics.
        // All initialised to APIS_NOT_MEASURED; set to APIS_ERROR on error.
        float _distance = APIS_NOT_MEASURED;
        float   _pitch = APIS_NOT_MEASURED;
        float   _roll  = APIS_NOT_MEASURED;
        float   _accelX = APIS_NOT_MEASURED;   // last reading, m/s^2
        float   _accelY = APIS_NOT_MEASURED;
        float   _accelZ = APIS_NOT_MEASURED;
        float   _magnitude = APIS_NOT_MEASURED;  // burst mean, m/s^2
        float   _tilt      = APIS_NOT_MEASURED;  // burst mean, deg

        // Range statistics

        // Orientation statistics

        // Readings behind the current statistics (set by updateMeasurements):
        // one array per measurement, native type, oldest first, and the count
        // of valid entries.
        // One storage path: every acquisition appends here (updateDistance(),
        // updateOrientation(), whether from updateMeasurements() or logReading());
        // the statistics getters read the arrays; updateMeasurements() and
        // beginReadings() start a fresh set. (NW_Core)
        NW_Readings<int16_t, APIS_RANGE_CAPACITY> _distanceReadings;
        NW_Readings<float, APIS_ORIENT_CAPACITY>  _pitchReadings;
        NW_Readings<float, APIS_ORIENT_CAPACITY>  _rollReadings;
        NW_Readings<float, APIS_ORIENT_CAPACITY>  _magnitudeReadings;
        NW_Readings<float, APIS_ORIENT_CAPACITY>  _tiltReadings;


        // LiDAR Lite signal strength; updated by updateDistance()
        uint8_t _signalStrength = 0;
        // Accelerometer temperature digits (relative); updated by updateOrientation()
        int16_t _accelerometerTemp = APIS_NOT_MEASURED;
        int16_t _zeroTemp  = APIS_NOT_MEASURED;
        // Zero generation carried by the last orientation reading (begin() seeds it
        // from Page 1), and whether it moved between the last two readings
        uint16_t _zeroGen = 0;
        bool _zeroChanged = false;

        // Sensor sensitivity; set initially to default "balanced" mode
        SensitivityMode _sensitivity = SENSITIVITY_BALANCED;



        // Chips covered by the current run of readings (beginReadings)
        uint8_t _rawComponent = ALL;
};

// Deprecated component selectors (v0.1.x): the class-scoped Apis::ALL,
// Apis::RANGE, Apis::ORIENT replace them. Kept so existing sketches compile.
#ifndef NW_READING_ALL
  #define NW_READING_ALL       Apis::ALL
  #define NW_READING_PRIMARY   Apis::RANGE
  #define NW_READING_SECONDARY Apis::ORIENT
#endif
#define NW_READING_RANGE  NW_READING_PRIMARY
#define NW_READING_ORIENT NW_READING_SECONDARY

/** @deprecated Use Apis::DEFAULT_ADDRESS. Every NW library defined this same macro
 *  with a different value, so a sketch including two of them got the last one.
 *  Removed at the next major version. */
#define ADR_DEFAULT Apis::DEFAULT_ADDRESS

#endif
