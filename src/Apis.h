/******************************************************************************
Apis.h

Library for the Apis interface board for a LiDAR Lite unit.

Andrew Wickert
Based on code by Bobby Schulz
In particular: https://github.com/NorthernWidget-Skunkworks/Project-Symbiont-LiDAR/tree/master/Software/LiDARLite_I2CParse
and based loosely around Bobby Schulz' general functions for sensor libraries.

Started 2020.05.01
Hardware located at:
https://github.com/NorthernWidget-Skunkworks/Project-Symbiont-LIDAR

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
#define sensitivityBalanced 0
#define sensitivityHigh 1
#define sensitivityLow 2
#define sensitivityMaxRange 3
/**
 * @class Apis Class to Interface with the Apis module for a laser rangefinder.
 * @brief Arduino library for the Apis board, which manages a LiDAR Lite
 * unit (roll/pitch, firmware lock/reset, power supply).
 * @details Library to communicate with the Apis module, which
 * connects to a LiDAR Lite rangefinder.The Apis is equipped with
 * capacitors to handle the large burst power draw from the LiDAR Lite, a MEMS
 * accelerometer to note its orientation, a magnet to note a known orientation
 * (often, but not necessarily, horizontal)  and the ability to absorb
 * occasional firmware issues that lead to system hangs.
 * The leveling helps the user to calculate, for example, a water level
 * when the sensor is palced on a cliff a tree next to the river but does not
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
         * @brief Instantiate Apis object
         */
        Apis();

        /**
         * @brief Begin communications with the Apis using a prescribed
         * address.
         * @param Addresss_ default 0x50
         * DOES NOT YET USE A VARIABLE ADDRESS!
         * DOES NOT MATTER WHAT YOU WRITE HERE.
         * THIS DEFAULT ADDRESS CURRENTLY CLASHES WITH HAAR'S DEFAULT!
         * @param[in] sensitivity. Options are:
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
         */
        void begin(uint8_t Address_ = ADR_DEFAULT, \
                   uint8_t sensitivity = sensitivityBalanced);

        /**
         * @brief Measure range [cm] only, without reading the accelerometer.
         * Intended for rapid repeated range readings (e.g. for averaging).
         * Returns false if the sensor returns an error value.
         */
        bool updateRange();

        /**
         * @brief Measure range [cm], roll [deg] and pitch [deg].
         * Returns false if any sensor returns an error value.
         */
        bool updateMeasurements();

        /**
         * @brief Return range [cm]
         */
        float getRange();

        /**
         * @brief Return roll [deg]
         */
        float getRoll();

        /**
         * @brief Return pitch [deg]
         */
        float getPitch();

        /**
         * @brief Return a comma-separated header:
         * "Range [cm],Pitch [deg],Roll [deg],"
         */
        String getHeader();

        /**
         * @brief Return comma-separated data values
         * @details This is the most likely function (alongside getHeader) for
         * an end user to use.
         * Values are returned as:
         * String(Range) + "," + String(Pitch) + "," + String(Roll) + ","
         * Error values are "-9999" (sensor error) and "-9998" (no measurement
         * yet taken")
         * @param[in] takeNewReadings: if `true` run `updateMeasurements` before
         * returning values. Otherwise, just return values.
         */
        String getString(bool takeNewReadings = true);

    private:
        //Default address
        uint8_t ADR = ADR_DEFAULT;

        // Variables to save and make accessible to other functions
        // Initialize at a non-error value; just one that indicates that
        // they are unset (will be returned if requested before measurement
        // is taken)
        float Range = -9998;
        float Pitch = -9998;
        float Roll = -9998;

        // Sensor sensitivity; set initially to default "balanced" mode
        uint8_t sensitivity = sensitivityBalanced;
};

#endif
