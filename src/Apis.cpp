#include "Apis.h"

Apis::Apis(uint16_t nRangeReadings, bool rangeStats,
           uint16_t nOrientReadings, bool orientStats)
    : _nRangeReadings(nRangeReadings), _rangeStats(rangeStats),
      _nOrientReadings(nOrientReadings), _orientStats(orientStats)
{
}

bool Apis::begin(uint8_t address, SensitivityMode sensitivity)
{
    _adr = address;
    _sensitivity = sensitivity;
    Wire.begin();

    // Check ACK.
    Wire.beginTransmission(_adr);
    if (Wire.endTransmission() != 0) return false;

    // Page 0 Blocks 0–1 (0x00–0x0F): schema, name, versions. Read in one
    // transaction; the firmware serves Page 0 from EEPROM so it is valid before
    // the first reading.
    uint8_t p0[16];
    _hwMajor = _hwMinor = _fwPatch = 0;
    if (!_readBytes(REG_SCHEMA, p0, 16)) return false;
    _hwMajor = p0[REG_HW_MAJOR];
    _hwMinor = p0[REG_HW_MINOR];
    _fwPatch = p0[REG_FW_PATCH];
    if (p0[REG_SCHEMA] != 0x01) return false;               // not Schema 1 (0x00 legacy, 0xFF unprovisioned, other)
    const char expected[7] = {'A', 'p', 'i', 's', 0, 0, 0};   // 7-byte name field, null-padded
    for (uint8_t i = 0; i < 7; i++) {
        if (p0[REG_NAME + i] != expected[i]) return false;
    }
    if (_fwPatch < APIS_FW_MIN_PATCH) return false;          // register map older than this library

    _writeByte(REG_CONFIG, (uint8_t)_sensitivity);

    return true;
}

uint8_t Apis::getHardwareMajor()   { return _hwMajor; }
uint8_t Apis::getHardwareMinor()   { return _hwMinor; }
uint8_t Apis::getFirmwareVersion() { return _fwPatch; }

bool Apis::_readBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
    // One transaction: pointer write, then requestFrom(n). Schema 1 firmware
    // (patch 1+) serves up to 32 bytes with auto-increment.
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(_adr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

bool Apis::_writeByte(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_adr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

void Apis::setNRangeReadings(uint16_t n)  { _nRangeReadings = n; }
void Apis::setRangeStats(bool enable)      { _rangeStats = enable; }
void Apis::setNOrientReadings(uint16_t n) { _nOrientReadings = n; }
void Apis::setOrientStats(bool enable)     { _orientStats = enable; }

void Apis::setRangefinderSensitivity(SensitivityMode mode) {
    _sensitivity = mode;
    _writeByte(REG_CONFIG, (uint8_t)_sensitivity);
}

void Apis::setI2CAddress(uint8_t newAddress) {
    _writeByte(REG_I2C_ADDR, newAddress);
}

bool Apis::ready() {
    uint8_t status = 0;
    return _readBytes(REG_STATUS, &status, 1) && (status & APIS_BIT_READY);
}

uint16_t Apis::_readCounter() {
    uint8_t d[2] = {0xFF, 0xFF};
    _readBytes(REG_COUNTER, d, 2);
    return (uint16_t)((d[1] << 8) | d[0]);
}

bool Apis::newReading() {
    return _readCounter() != _lastCounter;
}

bool Apis::requestReading(uint8_t component) {
    uint8_t ctrl = APIS_CTRL_TRIGGER;
    if (component == ALL || component == RANGE)  ctrl |= APIS_CTRL_LIDAR;
    if (component == ALL || component == ORIENT) ctrl |= APIS_CTRL_ACCEL;
    return _writeByte(REG_CTRL, ctrl);
}

bool Apis::_takeReading(uint8_t component) {
    uint16_t before = _readCounter();
    if (!requestReading(component)) return false;
    unsigned long start = millis();
    while (millis() - start < timeoutGlobal) {
        uint16_t now = _readCounter();
        if (now != before) {
            _lastCounter = now;
            // Block 0 of the new reading: status (0x20) and latched fault (0x27)
            uint8_t b0[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            _readBytes(REG_STATUS, b0, 8);
            _status = b0[0];
            _fault  = b0[7];
            return true;
        }
        delay(1);
    }
    return false;
}

bool    Apis::faulted(uint8_t chip) { return _status & (1 << (chip + 1)); }
bool    Apis::anyFault()            { return _status & APIS_BIT_PANFAULT; }
uint8_t Apis::faultChip()           { return _fault >> 5; }
uint8_t Apis::faultKind()           { return _fault & 0x1F; }

size_t Apis::printFault(Print& out) {
    static const char* const chips[] = {"LiDAR", "accelerometer"};
    static const char* const kinds[] = {"none", "no acknowledge", "timeout", "checksum", "out of range",
                                        "not initialised", "reset since configured", "config rejected",
                                        "supply fault"};
    uint8_t chip = faultChip(), kind = faultKind();
    if (kind == 0) return out.print("none");
    size_t n = 0;
    if (chip == 7) n += out.print("unit");
    else if (chip < 2) n += out.print(chips[chip]);
    else { n += out.print("chip "); n += out.print(chip); }
    n += out.print(": ");
    if (kind < 9) n += out.print(kinds[kind]);
    else { n += out.print("kind "); n += out.print(kind); }
    return n;
}

bool Apis::updateRange() {
    if (!_takeReading(RANGE)) {
        _range = APIS_ERROR;
        return false;
    }
    // Range low/high and signal strength are consecutive (0x28–0x2A): one read.
    uint8_t d[3] = {0xFF, 0xFF, 0xFF};   // 0xFF mirrors what Wire.read() yields on a failed request
    _readBytes(REG_RANGE_L, d, 3);
    _range = (int16_t)((d[1] << 8) | d[0]);
    _signalStrength = d[2];

    if (_range < 0) {
        _range = APIS_ERROR;
        return false;
    }
    return true;
}

bool Apis::updateOrientation() {
    if (!_takeReading(ORIENT)) {
        _pitch = _roll = APIS_ERROR;
        return false;
    }
    int16_t dataSet[6];
    uint8_t d[6];

    // Accel raw X/Y/Z at REG_ACCEL_BASE (0x30–0x35): one read of six bytes
    memset(d, 0xFF, sizeof d);           // 0xFF mirrors what Wire.read() yields on a failed request
    _readBytes(REG_ACCEL_BASE, d, 6);
    for (int i = 0; i < 3; i++) dataSet[i] = ((d[2*i + 1] << 8) | d[2*i]);

    // Accel offsets X/Y/Z at REG_OFFSET_BASE (0x40–0x45, Page 2): one read of six bytes
    memset(d, 0xFF, sizeof d);
    _readBytes(REG_OFFSET_BASE, d, 6);
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
    return true;
}

bool Apis::updateMeasurements(uint8_t component) {
    bool rangeOK  = true;
    bool orientOK = true;

    if (component == ALL || component == RANGE) {
    // Welford's online algorithm for range mean, std, sterr
    float rangeM2 = 0, rangeMean = 0;
    uint16_t rangeN = 0;

    for (uint16_t i = 0; i < _nRangeReadings; i++) {
        if (updateRange()) {
            rangeN++;
            float x = (float)_range;
            float delta = x - rangeMean;
            rangeMean += delta / rangeN;
            rangeM2   += delta * (x - rangeMean);
        }
    }

    if (rangeN == 0) {
        _range = APIS_ERROR;
        _rangeMean = _rangeStd = _rangeSterr = APIS_ERROR;
    } else {
        _rangeMean = rangeMean;
        _range     = (int16_t)rangeMean;
        _rangeStd   = (rangeN > 1) ? sqrt(rangeM2 / (rangeN - 1)) : 0;
        _rangeSterr = (rangeN > 1) ? _rangeStd / sqrt((float)rangeN) : 0;
    }
    _rangeCount = rangeN;
    // Float comparisons with APIS_ERROR are safe: the value is assigned directly,
    // never computed, so the float representation is exact and consistent.
    rangeOK = (_range != APIS_ERROR);
    }

    if (component == ALL || component == ORIENT) {
    // Welford's online algorithm for orientation mean, std, sterr
    float pitchM2 = 0, rollM2 = 0, pitchMean = 0, rollMean = 0;
    uint16_t orientN = 0;

    for (uint16_t i = 0; i < _nOrientReadings; i++) {
        if (updateOrientation()) {
            orientN++;
            float dp = _pitch - pitchMean;
            pitchMean += dp / orientN;
            pitchM2   += dp * (_pitch - pitchMean);

            float dr = _roll - rollMean;
            rollMean += dr / orientN;
            rollM2   += dr * (_roll - rollMean);
        }
    }

    if (orientN == 0) {
        _pitch = _roll = APIS_ERROR;
        _pitchStd = _pitchSterr = _rollStd = _rollSterr = APIS_ERROR;
    } else {
        _pitch = pitchMean;
        _roll  = rollMean;
        _pitchStd   = (orientN > 1) ? sqrt(pitchM2 / (orientN - 1)) : 0;
        _pitchSterr = (orientN > 1) ? _pitchStd / sqrt((float)orientN) : 0;
        _rollStd    = (orientN > 1) ? sqrt(rollM2  / (orientN - 1)) : 0;
        _rollSterr  = (orientN > 1) ? _rollStd  / sqrt((float)orientN) : 0;
    }
    _orientCount = orientN;
    orientOK = (_pitch != APIS_ERROR) && (_roll != APIS_ERROR);
    }

    return rangeOK && orientOK;
}

uint16_t Apis::getRangeCount()  { return _rangeCount; }
uint16_t Apis::getOrientCount() { return _orientCount; }

int16_t Apis::getRange()          { return _range; }
float   Apis::getRoll()           { return _roll; }
float   Apis::getPitch()          { return _pitch; }
uint8_t Apis::getSignalStrength() { return _signalStrength; }

float Apis::getRangeMean()  { return _rangeMean; }
float Apis::getRangeStd()   { return _rangeStd; }
float Apis::getRangeSterr() { return _rangeSterr; }
float Apis::getPitchStd()   { return _pitchStd; }
float Apis::getPitchSterr() { return _pitchSterr; }
float Apis::getRollStd()    { return _rollStd; }
float Apis::getRollSterr()  { return _rollSterr; }

String Apis::getString(bool takeNewReadings) {
    if (takeNewReadings) {
        updateMeasurements();
    }
    String s = String(_range) + ",";
    if (_rangeStats && _nRangeReadings > 1) {
        s += String(_rangeStd) + "," + String(_rangeSterr) + ",";
    }
    s += String(_pitch) + "," + String(_roll) + ",";
    if (_orientStats && _nOrientReadings > 1) {
        s += String(_pitchStd)   + "," + String(_pitchSterr) + ","
           + String(_rollStd)    + "," + String(_rollSterr)  + ",";
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

void Apis::beginReadings(uint8_t component) {
    _rawComponent = component;
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
