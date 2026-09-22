#include "Apis.h"

// Register map: NW-Device-Specification Schema 1, Apis appendix. Three 32-byte
// pages; a controller writes a start address and reads up to 32 bytes with
// auto-increment. Registers not listed are reserved.
// Page 0 (0x00–0x1F) — identity, EEPROM-backed
// Page 1 (0x20–0x3F) — status and sensor data, SRAM
#define REG_RANGE_L     0x28  // Range low byte  (little-endian int16, cm)
#define REG_RANGE_H     0x29  // Range high byte
#define REG_SIGNAL_STR  0x2A  // LiDAR Lite signal strength (uint8_t, from LiDAR Lite reg 0x0E)
#define REG_ACCEL_BASE  0x30  // Accel raw X low byte; X/Y/Z span 0x30–0x35, little-endian int16
// Page 2 (0x40–0x5F) — calibration, EEPROM-backed
#define REG_OFFSET_BASE 0x40  // Accel offset X low byte; X/Y/Z span 0x40–0x45, little-endian int16


Apis::Apis(uint16_t nRangeReadings, bool rangeStats,
           uint16_t nOrientReadings, bool orientStats)
    : _rangeStats(rangeStats), _orientStats(orientStats)
{
    setRangeReadings(nRangeReadings);
    setOrientReadings(nOrientReadings);
}

bool Apis::begin(uint8_t address, SensitivityMode sensitivity)
{
    _sensitivity = sensitivity;
    // NW_Core: ACK, Page 0 read, and the three gates (schema 0x01, name "Apis",
    // firmware patch >= APIS_FW_MIN_PATCH); versions are stored before any refusal.
    if (!_dev.begin(address, "Apis", APIS_FW_MIN_PATCH)) return false;
    _dev.writeConfig((uint8_t)_sensitivity);
    return true;
}

uint8_t Apis::getHardwareMajor()   { return _dev.hardwareMajor(); }
uint8_t Apis::getHardwareMinor()   { return _dev.hardwareMinor(); }
uint8_t Apis::getFirmwareVersion() { return _dev.firmwareVersion(); }

uint8_t Apis::_chips(uint8_t component) {
    uint8_t chips = 0;
    if (component == ALL || component == RANGE)  chips |= 0x01;   // chip 0: LiDAR Lite
    if (component == ALL || component == ORIENT) chips |= 0x02;   // chip 1: accelerometer
    return chips;
}
uint16_t Apis::setRangeReadings(uint16_t n) {
    _nRangeReadings = (n > APIS_RANGE_CAPACITY) ? APIS_RANGE_CAPACITY : n;
    return _nRangeReadings;
}
uint16_t Apis::setOrientReadings(uint16_t n) {
    _nOrientReadings = (n > APIS_ORIENT_CAPACITY) ? APIS_ORIENT_CAPACITY : n;
    return _nOrientReadings;
}
void Apis::setRangeStats(bool enable)      { _rangeStats = enable; }
void Apis::setOrientStats(bool enable)     { _orientStats = enable; }
void Apis::setNRangeReadings(uint16_t n)   { setRangeReadings(n); }
void Apis::setNOrientReadings(uint16_t n)  { setOrientReadings(n); }

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
uint8_t Apis::faultChip()           { return _dev.faultChip(); }
uint8_t Apis::faultKind()           { return _dev.faultKind(); }

size_t Apis::printFault(Print& out) {
    // The chip names are Apis's own; the kind names are universal (NW_Fault).
    static const char* const chips[] = {"LiDAR", "accelerometer"};
    uint8_t chip = faultChip(), kind = faultKind();
    if (kind == 0) return out.print("none");
    size_t n = 0;
    if (chip == 7) n += out.print("unit");
    else if (chip < 2) n += out.print(chips[chip]);
    else { n += out.print("chip "); n += out.print(chip); }
    n += out.print(": ");
    return n + _dev.fault().printKind(out);
}

bool Apis::updateRange() {
    if (_dev.batchFaulted(0x01)) {        // rest of a batch whose LiDAR did not power up
        _range = APIS_ERROR;
        return false;
    }
    if (!_dev.takeReading(_chips(RANGE))) {
        _range = APIS_ERROR;
        return false;
    }
    if (_dev.batchFaulted(0x01)) {        // no acknowledge / not initialised: the chip is not coming
        _range = APIS_ERROR;
        return false;
    }
    // Range low/high and signal strength are consecutive (0x28–0x2A): one read.
    uint8_t d[3] = {0xFF, 0xFF, 0xFF};   // 0xFF mirrors what Wire.read() yields on a failed request
    _dev.readBytes(REG_RANGE_L, d, 3);
    _range = (int16_t)((d[1] << 8) | d[0]);
    _signalStrength = d[2];

    if (_range < 0) {
        _range = APIS_ERROR;
        return false;
    }
    _rangeReadings.append(_range);
    return true;
}

bool Apis::updateOrientation() {
    if (!_dev.takeReading(_chips(ORIENT))) {
        _pitch = _roll = APIS_ERROR;
        return false;
    }
    int16_t dataSet[6];
    uint8_t d[6];

    // Accel raw X/Y/Z at REG_ACCEL_BASE (0x30–0x35): one read of six bytes
    memset(d, 0xFF, sizeof d);           // 0xFF mirrors what Wire.read() yields on a failed request
    _dev.readBytes(REG_ACCEL_BASE, d, 6);
    for (int i = 0; i < 3; i++) dataSet[i] = ((d[2*i + 1] << 8) | d[2*i]);

    // Accel offsets X/Y/Z at REG_OFFSET_BASE (0x40–0x45, Page 2): one read of six bytes
    memset(d, 0xFF, sizeof d);
    _dev.readBytes(REG_OFFSET_BASE, d, 6);
    for (int i = 0; i < 3; i++) dataSet[3+i] = ((d[2*i + 1] << 8) | d[2*i]);

    float gx = dataSet[0], gy = dataSet[1], gz = dataSet[2];
    float offsetX = dataSet[3], offsetY = dataSet[4], offsetZ = dataSet[5];

    // When software I2C reads fail, the ATTiny returns 0xFF per byte. Two 0xFF
    // bytes assembled as int16_t (0xFFFF) and right-shifted 4 gives -1 on AVR
    // (arithmetic shift). All three axes equal to -1 is therefore the I2C bus
    // failure signature, not a physical accelerometer reading.
    if (gx == gy && gx == gz && gx == -1) {
        _pitch = _roll = APIS_ERROR;
        return false;
    } else if (offsetX == offsetY && offsetX == offsetZ && offsetX == 0) {
        _pitch = atan(-gx/gz) * 180. / M_PI;
        _roll  = atan(gy / sqrt(pow(gx, 2) + pow(gz, 2))) * 180. / M_PI;
    } else {
        _pitch = (atan(-gx/gz) - atan(-offsetX/offsetZ)) * 180. / M_PI;
        _roll  = (atan(gy / sqrt(pow(gx, 2) + pow(gz, 2)))
                - atan(offsetY / sqrt(pow(offsetX, 2) + pow(offsetZ, 2)))) * 180. / M_PI;
    }
    _pitchReadings.append(_pitch);
    _rollReadings.append(_roll);
    return true;
}

bool Apis::updateMeasurements(uint8_t component) {
    bool rangeOK  = true;
    bool orientOK = true;
    if (component == ALL || component == RANGE) {
    // Tell the device how many readings follow so it holds the LiDAR powered
    // for the batch (single readings need no word: 0/1 means power down after each).
    if (_nRangeReadings > 1) _dev.writeBatch(_nRangeReadings);
    // Take N range readings; each successful one appends to _rangeReadings[].
    // Statistics are read from the array (two-pass in float, exact enough for N
    // up to the array capacity; see the precision note in Apis.h).
    _rangeReadings.reset();
    _dev.resetBatch();
    for (uint16_t i = 0; i < _nRangeReadings; i++) {
        if (!updateRange() && _dev.batchFaulted(0x01)) break;   // LiDAR will not power up: stop the batch
    }
    if (_rangeReadings.count() == 0) {
        _range = APIS_ERROR;
    } else {
        _range = (int16_t)getRangeMean();
    }
    // Float comparisons with APIS_ERROR are safe: the value is assigned directly,
    // never computed, so the float representation is exact and consistent.
    rangeOK = (_range != APIS_ERROR);
    }
    if (component == ALL || component == ORIENT) {
    // Take N orientation readings; each successful one appends to the arrays.
    _pitchReadings.reset();
    _rollReadings.reset();
    for (uint16_t i = 0; i < _nOrientReadings; i++) updateOrientation();
    if (_pitchReadings.count() == 0) {
        _pitch = _roll = APIS_ERROR;
    } else {
        _pitch = _pitchReadings.mean();
        _roll  = _rollReadings.mean();
    }
    orientOK = (_pitch != APIS_ERROR) && (_roll != APIS_ERROR);
    }
    return rangeOK && orientOK;
}



float Apis::getRangeMedian() { return _rangeReadings.median(); }
float Apis::getPitchMedian() { return _pitchReadings.median(); }
float Apis::getRollMedian()  { return _rollReadings.median(); }

uint16_t Apis::getRangeCount()  { return _rangeReadings.count(); }
uint16_t Apis::getOrientCount() { return _pitchReadings.count(); }

int16_t Apis::getRange()          { return _range; }
float   Apis::getRoll()           { return _roll; }
float   Apis::getPitch()          { return _pitch; }
uint8_t Apis::getSignalStrength() { return _signalStrength; }

// Statistics are computed from the arrays each call (NW_Readings), so a burst
// logged through logReading() has its statistics without re-acquiring.
float Apis::getRangeMean()  { return _rangeReadings.mean(); }
float Apis::getRangeStd()   { return _rangeReadings.std(); }
float Apis::getRangeSterr() { return _rangeReadings.sterr(); }
float Apis::getPitchStd()   { return _pitchReadings.std(); }
float Apis::getPitchSterr() { return _pitchReadings.sterr(); }
float Apis::getRollStd()    { return _rollReadings.std(); }
float Apis::getRollSterr()  { return _rollReadings.sterr(); }

String Apis::getString(bool takeNewReadings) {
    if (takeNewReadings) {
        updateMeasurements();
    }
    String s = String(_range) + ",";
    if (_rangeStats && _nRangeReadings > 1) {
        s += String(getRangeStd()) + "," + String(getRangeSterr()) + ",";
    }
    s += String(_pitch) + "," + String(_roll) + ",";
    if (_orientStats && _nOrientReadings > 1) {
        s += String(getPitchStd())   + "," + String(getPitchSterr()) + ","
           + String(getRollStd())    + "," + String(getRollSterr())  + ",";
    }
    return s;
}

String Apis::getHeader() {
    String h = "Range [cm],";
    if (_rangeStats && _nRangeReadings > 1) {
        h += "Range std [cm],Range sterr [cm],";
    }
    h += "Pitch [deg],Roll [deg],";
    if (_orientStats && _nOrientReadings > 1) {
        h += "Pitch std [deg],Pitch sterr [deg],"
             "Roll std [deg],Roll sterr [deg],";
    }
    return h;
}

void Apis::beginReadings(uint8_t component, uint16_t n) {
    _rawComponent = component;
    if (component == ALL || component == RANGE)  _rangeReadings.reset();
    if (component == ALL || component == ORIENT) { _pitchReadings.reset(); _rollReadings.reset(); }
    _dev.resetBatch();
    if (n > 1 && (component == ALL || component == RANGE)) _dev.writeBatch(n);
}

void Apis::endReadings() {
    // No cleanup required currently
}

size_t Apis::printHeader(Print& out) {
    size_t n = 0;
    if (_rawComponent == ALL || _rawComponent == RANGE) {
        n += out.print("Range [cm],");
    }
    if (_rawComponent == ALL || _rawComponent == ORIENT) {
        n += out.print("Pitch [deg],Roll [deg],");
    }
    return n;
}

size_t Apis::printReading(Print& out) {
    size_t n = 0;
    if (_rawComponent == ALL || _rawComponent == RANGE) {
        n += out.print(_range);  n += out.print(',');
    }
    if (_rawComponent == ALL || _rawComponent == ORIENT) {
        n += out.print(_pitch);  n += out.print(',');
        n += out.print(_roll);   n += out.print(',');
    }
    return n;
}

size_t Apis::logReading(Print& out) {
    if (_rawComponent == ALL || _rawComponent == RANGE)  updateRange();
    if (_rawComponent == ALL || _rawComponent == ORIENT) updateOrientation();
    return printReading(out);
}

// --- Deprecated raw-reading interface: bodies kept verbatim from v0.1.x so
// --- their output stays byte-identical for existing sketches.

void Apis::beginRawReadings(uint8_t component) {
    _rawComponent = component;
}

uint16_t Apis::takeRawReading(char* buf, uint16_t offset) {
    if (_rawComponent == ALL || _rawComponent == RANGE) {
        updateRange();
        offset += snprintf(buf + offset, 8, "%d,", (int)_range);
    }
    if (_rawComponent == ALL || _rawComponent == ORIENT) {
        char tmp[10];
        if (updateOrientation()) {
            dtostrf(_pitch, 1, 2, tmp);
            offset += snprintf(buf + offset, 10, "%s,", tmp);
            dtostrf(_roll, 1, 2, tmp);
            offset += snprintf(buf + offset, 10, "%s,", tmp);
        } else {
            offset += snprintf(buf + offset, 13, "-9999,-9999,"); // APIS_ERROR twice; string
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
