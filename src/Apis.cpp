#include "Apis.h"

// The column names, from NW_Core's generated table. Aliased here only to keep
// the lines below readable: the strings themselves are never typed, and which
// register each one names is recorded in the Apis appendix of
// NW-Device-Specification, beside the register map.
//
// A summary row carries a mean and a per-reading row carries one reading, and
// the vocabulary says which with its prefix operator. Both forms come from the
// same CSV row, so the pair cannot drift apart.
#define HDR_DISTANCE        NW_HDR_RANGEFINDER_APIS__DISTANCE
#define HDR_DISTANCE_MEAN   NW_HDR_MEAN_OF_RANGEFINDER_APIS__DISTANCE
#define HDR_DISTANCE_STD    NW_HDR_STD_OF_RANGEFINDER_APIS__DISTANCE
#define HDR_DISTANCE_STERR  NW_HDR_STERR_OF_RANGEFINDER_APIS__DISTANCE
#define HDR_SIGNAL          NW_HDR_RANGEFINDER_APIS__SIGNAL_STRENGTH
#define HDR_PITCH           NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__PITCH_ANGLE
#define HDR_PITCH_MEAN      NW_HDR_MEAN_OF_RANGEFINDER_APIS_ACCELEROMETER__PITCH_ANGLE
#define HDR_PITCH_STD       NW_HDR_STD_OF_RANGEFINDER_APIS_ACCELEROMETER__PITCH_ANGLE
#define HDR_PITCH_STERR     NW_HDR_STERR_OF_RANGEFINDER_APIS_ACCELEROMETER__PITCH_ANGLE
#define HDR_ROLL            NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__ROLL_ANGLE
#define HDR_ROLL_MEAN       NW_HDR_MEAN_OF_RANGEFINDER_APIS_ACCELEROMETER__ROLL_ANGLE
#define HDR_ROLL_STD        NW_HDR_STD_OF_RANGEFINDER_APIS_ACCELEROMETER__ROLL_ANGLE
#define HDR_ROLL_STERR      NW_HDR_STERR_OF_RANGEFINDER_APIS_ACCELEROMETER__ROLL_ANGLE
#define HDR_ACCEL_X         NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__X_COMPONENT_OF_ACCELERATION
#define HDR_ACCEL_Y         NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__Y_COMPONENT_OF_ACCELERATION
#define HDR_ACCEL_Z         NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__Z_COMPONENT_OF_ACCELERATION
#define HDR_MAGNITUDE       NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__MAGNITUDE_OF_ACCELERATION
#define HDR_MAGNITUDE_MEAN  NW_HDR_MEAN_OF_RANGEFINDER_APIS_ACCELEROMETER__MAGNITUDE_OF_ACCELERATION
#define HDR_MAGNITUDE_STD   NW_HDR_STD_OF_RANGEFINDER_APIS_ACCELEROMETER__MAGNITUDE_OF_ACCELERATION
#define HDR_MAGNITUDE_STERR NW_HDR_STERR_OF_RANGEFINDER_APIS_ACCELEROMETER__MAGNITUDE_OF_ACCELERATION
#define HDR_TILT            NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__TILT_ANGLE
#define HDR_TILT_MEAN       NW_HDR_MEAN_OF_RANGEFINDER_APIS_ACCELEROMETER__TILT_ANGLE
#define HDR_TILT_STD        NW_HDR_STD_OF_RANGEFINDER_APIS_ACCELEROMETER__TILT_ANGLE
#define HDR_TILT_STERR      NW_HDR_STERR_OF_RANGEFINDER_APIS_ACCELEROMETER__TILT_ANGLE
#define HDR_ACCEL_T         NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__ANOMALY_OF_TEMPERATURE
#define HDR_ZERO_GEN        NW_HDR_RANGEFINDER_APIS_ACCELEROMETER__ZERO_GENERATION

// Register map: NW-Device-Specification Schema 1, Apis appendix. Three 32-byte
// pages; a controller writes a start address and reads up to 32 bytes with
// auto-increment. Registers not listed are reserved.
// Pages renumbered 2026-09-23 (spec 4c3b18d): calibration is Page 1 at 0x20,
// data Page 2 at 0x40 (Block 0 at 0x40–0x47 is NW_Core's).
// Page 0 (0x00–0x1F): identity, EEPROM-backed
// Page 1 (0x20–0x3F): calibration, EEPROM-backed
#define REG_OFFSET_BASE 0x20  // Accel offset X low byte; X/Y/Z span 0x20–0x25, little-endian int16; 0x26–0x27 the temperature word at the zero
// 0x28–0x2F and 0x30–0x37: the two zeros before the current one, same form (firmware patch 5)
#define REG_ZERO_GEN    0x38  // Zero generation, little-endian uint16: zeros stored since manufacture, 0 never (patch 5)
// Page 2 (0x40–0x5F): status and sensor data, SRAM
#define REG_RANGE_L     0x48  // Range low byte  (little-endian int16, cm)
#define REG_RANGE_H     0x49  // Range high byte
#define REG_SIGNAL_STR  0x4A  // LiDAR Lite signal strength (uint8_t, from LiDAR Lite reg 0x0E)
#define REG_ACCEL_BASE  0x50  // Accel raw X low byte; X/Y/Z span 0x50–0x55, little-endian int16; 0x56–0x57 the temperature word
// 0x58–0x59: the zero generation again, mirrored from Page 1 with every reading (patch 5)


Apis::Apis(uint16_t nRangeReadings, bool rangeStats,
           uint16_t nOrientReadings, bool orientStats)
{
    setDistanceReadings(nRangeReadings);
    setOrientationReadings(nOrientReadings);
    _distanceCfg.stats = rangeStats;
    _orientationCfg.stats = orientStats;
}

bool Apis::begin(uint8_t address, SensitivityMode sensitivity)
{
    _sensitivity = sensitivity;
    // NW_Core: ACK, Page 0 read, and the three gates (schema 0x01, name "Apis",
    // firmware patch >= APIS_FW_MIN_PATCH); versions are stored before any refusal.
    if (!_dev.begin(address, "Apis", APIS_FW_MIN_PATCH)) return false;
    _dev.writeConfig((uint8_t)_sensitivity);
    // Page 1's zero generation once, so zeroChanged() has a reference before the first reading
    uint8_t g[2] = {0, 0};
    _dev.readBytes(REG_ZERO_GEN, g, 2);
    _zeroGen = g[0] | (g[1] << 8);
    _zeroChanged = false;
    return true;
}

uint8_t Apis::getHardwareMajor()   { return _dev.hardwareMajor(); }
uint8_t Apis::getHardwareMinor()   { return _dev.hardwareMinor(); }
uint8_t Apis::getFirmwareVersion() { return _dev.firmwareVersion(); }

uint8_t Apis::_chips(uint8_t component) {
    return component & ALL;   // the selectors are the chip-select bits: RANGE chip 0, ORIENT chip 1
}
uint16_t Apis::setDistanceReadings(uint16_t n)  { return _distanceCfg.set(n, APIS_RANGE_CAPACITY); }
uint16_t Apis::setOrientationReadings(uint16_t n) { return _orientationCfg.set(n, APIS_ORIENT_CAPACITY); }
void Apis::setDistanceStats(bool enable)      { _distanceCfg.stats = enable; }
void Apis::setOrientationStats(bool enable)     { _orientationCfg.stats = enable; }

void Apis::setRangefinderSensitivity(SensitivityMode mode) {
    _sensitivity = mode;
    _dev.writeConfig((uint8_t)_sensitivity);
}

void Apis::setI2CAddress(uint8_t newAddress)    { _dev.setI2CAddress(newAddress); }
bool Apis::ready()                              { return _dev.ready(); }
bool Apis::newReading()                         { return _dev.newReading(); }
bool Apis::requestReading(uint8_t component)    { return _dev.requestReading(_chips(component)); }


bool    Apis::faulted(uint8_t chip) { return _dev.faulted(chip); }
bool    Apis::anyFault()            { return _dev.anyFault(); }
uint8_t Apis::reportChip()           { return _dev.reportChip(); }
uint8_t Apis::reportKind()           { return _dev.reportKind(); }

size_t Apis::printReport(Print& out) {
    // The chip names are Apis's own (the spec's chip table); NW_Report prints the rest.
    static const char* const chips[] = {"LiDAR", "accelerometer"};
    return _dev.report().print(out, chips, 2);
}

size_t Apis::printStatus(Print& out, bool boot) {
    static const char* const chips[] = {"LiDAR", "Accel"};
    return _dev.printSnapshot(out, chips, 2, boot, APIS_LIBRARY_VERSION, APIS_LIBRARY_COMMIT);
}

bool    Apis::reportIsFault()  { return _dev.report().isFault(); }
uint8_t Apis::bootReportKind() { return _dev.bootReport().kind(); }
void    Apis::clearBootReport() { _dev.clearBootReport(); }

String Apis::reportNote() {
    // One word for a data-table note: the chip, then the kind ("LiDARTimeout").
    static const char* const chips[] = {"LiDAR", "Accel"};
    return _dev.report().note(chips, 2);
}

String Apis::beginFailure() { return _dev.beginFailure(); }

bool Apis::updateDistance() {
    if (_dev.batchFaulted(0x01)) {        // rest of a batch whose LiDAR did not power up
        _distance = NW_ERROR;
        return false;
    }
    if (!_dev.takeReading(_chips(RANGE))) {
        _distance = NW_ERROR;
        return false;
    }
    if (_dev.batchFaulted(0x01)) {        // not answering / self-test failed: the chip is not coming
        _distance = NW_ERROR;
        return false;
    }
    // Range low/high and signal strength are consecutive (0x48–0x4A): one read.
    uint8_t d[3] = {0xFF, 0xFF, 0xFF};   // 0xFF mirrors what Wire.read() yields on a failed request
    _dev.readData(REG_RANGE_L, d, 3);
    _distance = (float)(int16_t)((d[1] << 8) | d[0]);
    _signalStrength = d[2];

    if (_distance < 0) {
        _distance = NW_ERROR;
        return false;
    }
    _distanceReadings.append(_distance);
    return true;
}

bool Apis::updateOrientation() {
    if (!_dev.takeReading(_chips(ORIENT))) {
        _pitch = _roll = NW_ERROR;
        _accelX = _accelY = _accelZ = NW_ERROR;
        _accelerometerTemp = NW_ERROR;
        return false;
    }
    int16_t dataSet[6];
    uint8_t d[10];

    // Accel raw X/Y/Z at REG_ACCEL_BASE (0x50–0x55), the temperature word (0x56–0x57) and the zero generation (0x58–0x59): one read of ten bytes
    memset(d, 0xFF, sizeof d);           // 0xFF mirrors what Wire.read() yields on a failed request
    _dev.readData(REG_ACCEL_BASE, d, 10);
    for (int i = 0; i < 3; i++) dataSet[i] = ((d[2*i + 1] << 8) | d[2*i]);
    _accelerometerTemp = (int8_t)d[7];           // the LIS3DH digit is the word's high byte (1 per degree C, relative)
    uint16_t generation = d[8] | (d[9] << 8);

    // Accel offsets X/Y/Z at REG_OFFSET_BASE (0x20–0x25, Page 1) and the temperature at the zero (0x26–0x27): one read of eight bytes
    memset(d, 0xFF, sizeof d);
    _dev.readBytes(REG_OFFSET_BASE, d, 8);
    for (int i = 0; i < 3; i++) dataSet[3+i] = ((d[2*i + 1] << 8) | d[2*i]);
    _zeroTemp = (int8_t)d[7];

    float gx = dataSet[0], gy = dataSet[1], gz = dataSet[2];
    float offsetX = dataSet[3], offsetY = dataSet[4], offsetZ = dataSet[5];

    // When software I2C reads fail, the ATTiny returns 0xFF per byte. Two 0xFF
    // bytes assembled as int16_t (0xFFFF) and right-shifted 4 gives -1 on AVR
    // (arithmetic shift). All three axes equal to -1 is therefore the I2C bus
    // failure signature, not a physical accelerometer reading.
    if (gx == gy && gx == gz && gx == -1) {
        _pitch = _roll = NW_ERROR;
        _accelX = _accelY = _accelZ = NW_ERROR;
        _accelerometerTemp = NW_ERROR;
        return false;
    }
    _zeroChanged = (generation != _zeroGen);   // a reading that reached us: its generation against the last one seen
    _zeroGen = generation;
    if (offsetX == offsetY && offsetX == offsetZ && offsetX == 0) {
        _pitch = atan(-gx/gz) * 180. / M_PI;
        _roll  = atan(gy / sqrt(pow(gx, 2) + pow(gz, 2))) * 180. / M_PI;
    } else {
        _pitch = (atan(-gx/gz) - atan(-offsetX/offsetZ)) * 180. / M_PI;
        _roll  = (atan(gy / sqrt(pow(gx, 2) + pow(gz, 2)))
                - atan(offsetY / sqrt(pow(offsetX, 2) + pow(offsetZ, 2)))) * 180. / M_PI;
    }
    _pitchReadings.append(_pitch);
    _rollReadings.append(_roll);

    // The g vector in physical units, and the two quantities derived from it.
    _accelX = gx * APIS_ACCEL_M_PER_S2_PER_DIGIT;
    _accelY = gy * APIS_ACCEL_M_PER_S2_PER_DIGIT;
    _accelZ = gz * APIS_ACCEL_M_PER_S2_PER_DIGIT;
    float magnitude = sqrt(gx*gx + gy*gy + gz*gz);          // digits
    if (magnitude > 0) {
        _magnitudeReadings.append(magnitude * APIS_ACCEL_M_PER_S2_PER_DIGIT);
        // Tilt from the stored zero when one has been taken and from vertical
        // when none has, which is the branch pitch and roll take above.
        float cosine;
        if (offsetX == offsetY && offsetX == offsetZ && offsetX == 0) {
            cosine = gz / magnitude;
        } else {
            float reference = sqrt(offsetX*offsetX + offsetY*offsetY + offsetZ*offsetZ);
            cosine = (reference > 0) ? (gx*offsetX + gy*offsetY + gz*offsetZ) / (magnitude * reference) : 2;
        }
        if (cosine >= -1 && cosine <= 1) _tiltReadings.append(acos(cosine) * 180. / M_PI);
    }
    return true;
}

bool Apis::updateMeasurements(uint8_t component) {
    bool rangeOK  = true;
    bool orientOK = true;
    if (component & RANGE) {
    // Take N range readings; each successful one appends to _distanceReadings[].
    // The device is told how many follow so it holds the LiDAR powered for the
    // batch (0/1 means power down after each), and a LiDAR that will not power
    // up stops the batch (NW_Device::takeReadings). Statistics are read from
    // the array (two-pass in float, exact enough for N up to the array
    // capacity; see the precision note in Apis.h).
    _distanceReadings.reset();
    _dev.takeReadings(0x01, _distanceCfg.n, [this] { return updateDistance(); });
    if (_distanceReadings.count() == 0) {
        _distance = NW_ERROR;
    } else {
        _distance = getDistanceMean();
    }
    // Float comparisons with NW_ERROR are safe: the value is assigned directly,
    // never computed, so the float representation is exact and consistent.
    rangeOK = (_distance != NW_ERROR);
    }
    if (component & ORIENT) {
    // Take N orientation readings; each successful one appends to the arrays.
    _pitchReadings.reset();
    _rollReadings.reset();
    _magnitudeReadings.reset();
    _tiltReadings.reset();
    _dev.takeReadings(0x02, _orientationCfg.n, [this] { return updateOrientation(); });
    if (_pitchReadings.count() == 0) {
        _pitch = _roll = NW_ERROR;
        _magnitude = _tilt = NW_ERROR;
    } else {
        _pitch = _pitchReadings.mean();
        _roll  = _rollReadings.mean();
        _magnitude = (_magnitudeReadings.count() > 0) ? _magnitudeReadings.mean() : NW_ERROR;
        _tilt      = (_tiltReadings.count() > 0)      ? _tiltReadings.mean()      : NW_ERROR;
    }
    orientOK = (_pitch != NW_ERROR) && (_roll != NW_ERROR);
    }
    return rangeOK && orientOK;
}



float Apis::getDistanceMedian() { return _distanceReadings.median(); }
float Apis::getAccelerationMagnitudeMedian() { return _magnitudeReadings.median(); }
float Apis::getTiltMedian()  { return _tiltReadings.median(); }
float Apis::getPitchMedian() { return _pitchReadings.median(); }
float Apis::getRollMedian()  { return _rollReadings.median(); }

uint16_t Apis::getDistanceCount()  { return _distanceReadings.count(); }
uint16_t Apis::getOrientationCount() { return _pitchReadings.count(); }

float   Apis::getDistance()       { return _distance; }
float   Apis::getRoll()           { return _roll; }
float   Apis::getPitch()          { return _pitch; }
uint8_t Apis::getSignalStrength() { return _signalStrength; }

// Statistics are computed from the arrays each call (NW_Readings), so a burst
// logged through logReading() has its statistics without re-acquiring.
float Apis::getDistanceMean()  { return _distanceReadings.mean(); }
float Apis::getDistanceStd()   { return _distanceReadings.std(); }
float Apis::getDistanceSterr() { return _distanceReadings.sterr(); }
float Apis::getAccelerationX() { return _accelX; }
float Apis::getAccelerationY() { return _accelY; }
float Apis::getAccelerationZ() { return _accelZ; }
float Apis::getAccelerationMagnitude() { return _magnitude; }
float Apis::getTilt()       { return _tilt; }
float Apis::getAccelerationMagnitudeStd()   { return _magnitudeReadings.std(); }
float Apis::getAccelerationMagnitudeSterr() { return _magnitudeReadings.sterr(); }
float Apis::getTiltStd()    { return _tiltReadings.std(); }
void  Apis::setAccelerationColumns(bool enable) { _accelerationColumns = enable; }
void  Apis::setMagnitudeColumns(bool enable)    { _magnitudeColumns = enable; }
void  Apis::setTiltColumns(bool enable)         { _tiltColumns = enable; }
float Apis::getTiltSterr()  { return _tiltReadings.sterr(); }
float Apis::getPitchStd()   { return _pitchReadings.std(); }
float Apis::getPitchSterr() { return _pitchReadings.sterr(); }
float Apis::getRollStd()    { return _rollReadings.std(); }
float Apis::getRollSterr()  { return _rollReadings.sterr(); }

String Apis::getString(bool takeNewReadings) {
    if (takeNewReadings) {
        updateMeasurements();
    }
    String s = String(_distance) + ",";
    if (_distanceCfg.columns()) {
        s += String(getDistanceStd()) + "," + String(getDistanceSterr()) + ",";
    }
    s += String(getSignalStrength()) + ",";
    s += String(_pitch) + "," + String(_roll) + ",";
    if (_orientationCfg.columns()) {
        s += String(getPitchStd())   + "," + String(getPitchSterr()) + ","
           + String(getRollStd())    + "," + String(getRollSterr())  + ",";
    }
    if (_accelerationColumns) {
        s += String(_accelX) + "," + String(_accelY) + "," + String(_accelZ) + ",";
    }
    if (_magnitudeColumns) {
        s += String(_magnitude) + ",";
        if (_orientationCfg.columns()) {
            s += String(getAccelerationMagnitudeStd()) + "," + String(getAccelerationMagnitudeSterr()) + ",";
        }
    }
    if (_tiltColumns) {
        s += String(_tilt) + ",";
        if (_orientationCfg.columns()) {
            s += String(getTiltStd()) + "," + String(getTiltSterr()) + ",";
        }
    }
    s += String(getAccelerometerTemperatureChange()) + ",";
    s += String(getZeroGeneration()) + ",";
    return s;
}

int16_t Apis::getAccelerometerTemperatureADC() {
    return _accelerometerTemp;
}

float Apis::getAccelerometerTemperatureChange() {
    if (_accelerometerTemp == APIS_NOT_MEASURED || _accelerometerTemp == NW_ERROR) {
        return (float)_accelerometerTemp;
    }
    if (_zeroGen == 0) return (float)APIS_NOT_MEASURED;   // no zero stored: nothing to be a change from
    return (float)(_accelerometerTemp - _zeroTemp);       // the LIS3DH digit is 1 per degree C
}

int16_t Apis::getZeroTemperatureADC() {
    return _zeroTemp;
}

uint16_t Apis::getZeroGeneration() {
    return _zeroGen;
}

bool Apis::zeroChanged() {
    return _zeroChanged;
}

size_t Apis::dumpZeros(Print& out) {
    // Page 1 in one read (calibration, not a reading, so readBytes): Blocks 0–2 hold
    // the current zero and the two before it, 0x38–0x39 the generation. Newest first;
    // only zeros that were stored are printed (generation counts them).
    uint8_t p[32];
    memset(p, 0, sizeof p);
    if (!_dev.readBytes(REG_OFFSET_BASE, p, 32)) return 0;
    uint16_t generation = p[REG_ZERO_GEN - REG_OFFSET_BASE] | (p[REG_ZERO_GEN - REG_OFFSET_BASE + 1] << 8);
    size_t n = 0;
    for (uint8_t k = 0; k < 3 && k < generation; k++) {
        const uint8_t* z = p + 8*k;
        n += out.print((unsigned int)(generation - k));
        for (int i = 0; i < 3; i++) {
            n += out.print(',');
            n += out.print((int)(int16_t)((z[2*i + 1] << 8) | z[2*i]));
        }
        n += out.print(',');
        n += out.println((int)(int8_t)z[7]);   // the temperature digit at that zero, as getZeroTemperature() gives it
    }
    return n;
}

String Apis::getHeader() {
    // The summary row: the distance and the angles are means over the readings
    // taken, which is why those carry the mean_of_ operator. The signal
    // strength, the temperature anomaly and the zero generation are each one
    // value from the last reading and carry their bare names.
    String h;
    h += F(HDR_DISTANCE_MEAN);  h += ",";
    if (_distanceCfg.columns()) {
        h += F(HDR_DISTANCE_STD);    h += ",";
        h += F(HDR_DISTANCE_STERR);  h += ",";
    }
    h += F(HDR_SIGNAL);      h += ",";
    h += F(HDR_PITCH_MEAN);  h += ",";
    h += F(HDR_ROLL_MEAN);   h += ",";
    if (_orientationCfg.columns()) {
        h += F(HDR_PITCH_STD);    h += ",";
        h += F(HDR_PITCH_STERR);  h += ",";
        h += F(HDR_ROLL_STD);     h += ",";
        h += F(HDR_ROLL_STERR);   h += ",";
    }
    if (_accelerationColumns) {
        h += F(HDR_ACCEL_X);  h += ",";
        h += F(HDR_ACCEL_Y);  h += ",";
        h += F(HDR_ACCEL_Z);  h += ",";
    }
    if (_magnitudeColumns) {
        h += F(HDR_MAGNITUDE_MEAN);  h += ",";
        if (_orientationCfg.columns()) {
            h += F(HDR_MAGNITUDE_STD);    h += ",";
            h += F(HDR_MAGNITUDE_STERR);  h += ",";
        }
    }
    if (_tiltColumns) {
        h += F(HDR_TILT_MEAN);  h += ",";
        if (_orientationCfg.columns()) {
            h += F(HDR_TILT_STD);    h += ",";
            h += F(HDR_TILT_STERR);  h += ",";
        }
    }
    h += F(HDR_ACCEL_T);   h += ",";
    h += F(HDR_ZERO_GEN);  h += ",";
    return h;
}

void Apis::beginReadings(uint8_t component, uint16_t n) {
    _rawComponent = component;
    if (component & RANGE)  _distanceReadings.reset();
    if (component & ORIENT) {
        _pitchReadings.reset(); _rollReadings.reset();
        _magnitudeReadings.reset(); _tiltReadings.reset();
    }
    _dev.resetBatch();
    if (n > 1 && (component & RANGE)) _dev.writeBatch(n);
}

void Apis::endReadings() {
    // No cleanup required currently
}

size_t Apis::printHeader(Print& out) {
    size_t n = 0;
    // These carry the bare standard names rather than getHeader()'s mean_of_
    // forms, because each value here is one reading. The two sets differ for
    // that reason and no other: both come from the same rows of the CSV.
    if (_rawComponent & RANGE) {
        n += out.print(F(HDR_DISTANCE));  n += out.print(',');
        n += out.print(F(HDR_SIGNAL));    n += out.print(',');
    }
    if (_rawComponent & ORIENT) {
        n += out.print(F(HDR_PITCH));  n += out.print(',');
        n += out.print(F(HDR_ROLL));   n += out.print(',');
        if (_accelerationColumns) {
            n += out.print(F(HDR_ACCEL_X));  n += out.print(',');
            n += out.print(F(HDR_ACCEL_Y));  n += out.print(',');
            n += out.print(F(HDR_ACCEL_Z));  n += out.print(',');
        }
        if (_magnitudeColumns) { n += out.print(F(HDR_MAGNITUDE)); n += out.print(','); }
        if (_tiltColumns)      { n += out.print(F(HDR_TILT));      n += out.print(','); }
        n += out.print(F(HDR_ACCEL_T));   n += out.print(',');
        n += out.print(F(HDR_ZERO_GEN));  n += out.print(',');
    }
    return n;
}

size_t Apis::printReading(Print& out) {
    size_t n = 0;
    if (_rawComponent & RANGE) {
        n += out.print(_distance);  n += out.print(',');
        n += out.print(getSignalStrength()); n += out.print(',');
    }
    if (_rawComponent & ORIENT) {
        n += out.print(_pitch);  n += out.print(',');
        n += out.print(_roll);   n += out.print(',');
        if (_accelerationColumns) {
            n += out.print(_accelX); n += out.print(',');
            n += out.print(_accelY); n += out.print(',');
            n += out.print(_accelZ); n += out.print(',');
        }
        if (_magnitudeColumns) { n += out.print(_magnitude); n += out.print(','); }
        if (_tiltColumns)      { n += out.print(_tilt);      n += out.print(','); }
        n += out.print(getAccelerometerTemperatureChange()); n += out.print(',');
        n += out.print(getZeroGeneration()); n += out.print(',');
    }
    return n;
}

size_t Apis::logReading(Print& out) {
    if (_rawComponent & RANGE)  updateDistance();
    if (_rawComponent & ORIENT) updateOrientation();
    return printReading(out);
}

// --- Deprecated raw-reading interface: bodies kept verbatim from v0.1.x so
// --- their output stays byte-identical for existing sketches.

void Apis::beginRawReadings(uint8_t component) {
    _rawComponent = component;
}

uint16_t Apis::takeRawReading(char* buf, uint16_t offset) {
    if (_rawComponent & RANGE) {
        updateDistance();
        offset += snprintf(buf + offset, 8, "%d,", (int)_distance);
    }
    if (_rawComponent & ORIENT) {
        char tmp[10];
        if (updateOrientation()) {
            dtostrf(_pitch, 1, 2, tmp);
            offset += snprintf(buf + offset, 10, "%s,", tmp);
            dtostrf(_roll, 1, 2, tmp);
            offset += snprintf(buf + offset, 10, "%s,", tmp);
        } else {
            offset += snprintf(buf + offset, 13, "-9999,-9999,"); // NW_ERROR twice; string
                                                                  // literal used because the
                                                                  // buffer size (13) is tied to
                                                                  // the digit count of -9999.
        }
    }
    return offset;
}

void Apis::endRawReadings() {
    // No cleanup required currently
}
