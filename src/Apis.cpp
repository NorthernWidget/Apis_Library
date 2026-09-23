#include "Apis.h"

// Register map: NW-Device-Specification Schema 1, Apis appendix. Three 32-byte
// pages; a controller writes a start address and reads up to 32 bytes with
// auto-increment. Registers not listed are reserved.
// Pages renumbered 2026-09-23 (spec 4c3b18d): calibration is Page 1 at 0x20,
// data Page 2 at 0x40 (Block 0 at 0x40–0x47 is NW_Core's).
// Page 0 (0x00–0x1F): identity, EEPROM-backed
// Page 1 (0x20–0x3F): calibration, EEPROM-backed
#define REG_OFFSET_BASE 0x20  // Accel offset X low byte; X/Y/Z span 0x20–0x25, little-endian int16; 0x26–0x27 the temperature word at the zero
// Page 2 (0x40–0x5F): status and sensor data, SRAM
#define REG_RANGE_L     0x48  // Range low byte  (little-endian int16, cm)
#define REG_RANGE_H     0x49  // Range high byte
#define REG_SIGNAL_STR  0x4A  // LiDAR Lite signal strength (uint8_t, from LiDAR Lite reg 0x0E)
#define REG_ACCEL_BASE  0x50  // Accel raw X low byte; X/Y/Z span 0x50–0x55, little-endian int16; 0x56–0x57 the temperature word


Apis::Apis(uint16_t nRangeReadings, bool rangeStats,
           uint16_t nOrientReadings, bool orientStats)
{
    setRangeReadings(nRangeReadings);
    setOrientReadings(nOrientReadings);
    _rangeCfg.stats = rangeStats;
    _orientCfg.stats = orientStats;
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
    return component & ALL;   // the selectors are the chip-select bits: RANGE chip 0, ORIENT chip 1
}
uint16_t Apis::setRangeReadings(uint16_t n)  { return _rangeCfg.set(n, APIS_RANGE_CAPACITY); }
uint16_t Apis::setOrientReadings(uint16_t n) { return _orientCfg.set(n, APIS_ORIENT_CAPACITY); }
void Apis::setRangeStats(bool enable)      { _rangeCfg.stats = enable; }
void Apis::setOrientStats(bool enable)     { _orientCfg.stats = enable; }
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
uint8_t Apis::reportChip()           { return _dev.reportChip(); }
uint8_t Apis::reportKind()           { return _dev.reportKind(); }

size_t Apis::printReport(Print& out) {
    // The chip names are Apis's own (the spec's chip table); NW_Report prints the rest.
    static const char* const chips[] = {"LiDAR", "accelerometer"};
    return _dev.report().print(out, chips, 2);
}

size_t Apis::printStatus(Print& out, bool boot) {
    static const char* const chips[] = {"LiDAR", "Accel"};
    return _dev.printSnapshot(out, chips, 2, boot);
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

bool Apis::updateRange() {
    if (_dev.batchFaulted(0x01)) {        // rest of a batch whose LiDAR did not power up
        _range = NW_ERROR;
        return false;
    }
    if (!_dev.takeReading(_chips(RANGE))) {
        _range = NW_ERROR;
        return false;
    }
    if (_dev.batchFaulted(0x01)) {        // no acknowledge / not initialised: the chip is not coming
        _range = NW_ERROR;
        return false;
    }
    // Range low/high and signal strength are consecutive (0x48–0x4A): one read.
    uint8_t d[3] = {0xFF, 0xFF, 0xFF};   // 0xFF mirrors what Wire.read() yields on a failed request
    _dev.readData(REG_RANGE_L, d, 3);
    _range = (int16_t)((d[1] << 8) | d[0]);
    _signalStrength = d[2];

    if (_range < 0) {
        _range = NW_ERROR;
        return false;
    }
    _rangeReadings.append(_range);
    return true;
}

bool Apis::updateOrientation() {
    if (!_dev.takeReading(_chips(ORIENT))) {
        _pitch = _roll = NW_ERROR;
        _accelTemp = NW_ERROR;
        return false;
    }
    int16_t dataSet[6];
    uint8_t d[8];

    // Accel raw X/Y/Z at REG_ACCEL_BASE (0x50–0x55) and the temperature word (0x56–0x57): one read of eight bytes
    memset(d, 0xFF, sizeof d);           // 0xFF mirrors what Wire.read() yields on a failed request
    _dev.readData(REG_ACCEL_BASE, d, 8);
    for (int i = 0; i < 3; i++) dataSet[i] = ((d[2*i + 1] << 8) | d[2*i]);
    _accelTemp = (int8_t)d[7];           // the LIS3DH digit is the word's high byte (1 per degree C, relative)

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
        _accelTemp = NW_ERROR;
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
    if (component & RANGE) {
    // Take N range readings; each successful one appends to _rangeReadings[].
    // The device is told how many follow so it holds the LiDAR powered for the
    // batch (0/1 means power down after each), and a LiDAR that will not power
    // up stops the batch (NW_Device::takeReadings). Statistics are read from
    // the array (two-pass in float, exact enough for N up to the array
    // capacity; see the precision note in Apis.h).
    _rangeReadings.reset();
    _dev.takeReadings(0x01, _rangeCfg.n, [this] { return updateRange(); });
    if (_rangeReadings.count() == 0) {
        _range = NW_ERROR;
    } else {
        _range = (int16_t)getRangeMean();
    }
    // Float comparisons with NW_ERROR are safe: the value is assigned directly,
    // never computed, so the float representation is exact and consistent.
    rangeOK = (_range != NW_ERROR);
    }
    if (component & ORIENT) {
    // Take N orientation readings; each successful one appends to the arrays.
    _pitchReadings.reset();
    _rollReadings.reset();
    _dev.takeReadings(0x02, _orientCfg.n, [this] { return updateOrientation(); });
    if (_pitchReadings.count() == 0) {
        _pitch = _roll = NW_ERROR;
    } else {
        _pitch = _pitchReadings.mean();
        _roll  = _rollReadings.mean();
    }
    orientOK = (_pitch != NW_ERROR) && (_roll != NW_ERROR);
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
    if (_rangeCfg.columns()) {
        s += String(getRangeStd()) + "," + String(getRangeSterr()) + ",";
    }
    s += String(_pitch) + "," + String(_roll) + ",";
    if (_orientCfg.columns()) {
        s += String(getPitchStd())   + "," + String(getPitchSterr()) + ","
           + String(getRollStd())    + "," + String(getRollSterr())  + ",";
    }
    s += String(_accelTemp) + ",";
    return s;
}

int16_t Apis::getAccelTemperature() {
    return _accelTemp;
}

int16_t Apis::getZeroTemperature() {
    return _zeroTemp;
}

String Apis::getHeader() {
    String h = "Range [cm],";
    if (_rangeCfg.columns()) {
        h += "Range std [cm],Range sterr [cm],";
    }
    h += "Pitch [deg],Roll [deg],";
    if (_orientCfg.columns()) {
        h += "Pitch std [deg],Pitch sterr [deg],"
             "Roll std [deg],Roll sterr [deg],";
    }
    h += "AccelT [C],";
    return h;
}

void Apis::beginReadings(uint8_t component, uint16_t n) {
    _rawComponent = component;
    if (component & RANGE)  _rangeReadings.reset();
    if (component & ORIENT) { _pitchReadings.reset(); _rollReadings.reset(); }
    _dev.resetBatch();
    if (n > 1 && (component & RANGE)) _dev.writeBatch(n);
}

void Apis::endReadings() {
    // No cleanup required currently
}

size_t Apis::printHeader(Print& out) {
    size_t n = 0;
    if (_rawComponent & RANGE) {
        n += out.print("Range [cm],");
    }
    if (_rawComponent & ORIENT) {
        n += out.print("Pitch [deg],Roll [deg],AccelT [C],");
    }
    return n;
}

size_t Apis::printReading(Print& out) {
    size_t n = 0;
    if (_rawComponent & RANGE) {
        n += out.print(_range);  n += out.print(',');
    }
    if (_rawComponent & ORIENT) {
        n += out.print(_pitch);  n += out.print(',');
        n += out.print(_roll);   n += out.print(',');
        n += out.print(_accelTemp); n += out.print(',');
    }
    return n;
}

size_t Apis::logReading(Print& out) {
    if (_rawComponent & RANGE)  updateRange();
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
        updateRange();
        offset += snprintf(buf + offset, 8, "%d,", (int)_range);
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
