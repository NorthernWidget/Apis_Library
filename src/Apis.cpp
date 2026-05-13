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
    _needsStartupDelay = true;
    Wire.begin();

    // Check ACK.
    Wire.beginTransmission(_adr);
    if (Wire.endTransmission() != 0) return false;

    // Verify device identity by reading the 4-byte ASCII name from REG_NAME_0–3
    // (0x01–0x04). New firmware initialises these statically, so they are valid
    // before _waitUntilReady(). Old firmware leaves them 0x00; returning false
    // here prevents misinterpreting the old register map.
    const char expected[4] = {'A', 'p', 'i', 's'};
    for (uint8_t i = 0; i < 4; i++) {
        Wire.beginTransmission(_adr);
        Wire.write(REG_NAME_0 + i);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        if (Wire.read() != expected[i]) return false;
    }

    Wire.beginTransmission(_adr);
    Wire.write(REG_CONFIG);
    Wire.write((uint8_t)_sensitivity);
    Wire.endTransmission();

    return true;
}

void Apis::setNRangeReadings(uint16_t n)  { _nRangeReadings = n; }
void Apis::setRangeStats(bool enable)      { _rangeStats = enable; }
void Apis::setNOrientReadings(uint16_t n) { _nOrientReadings = n; }
void Apis::setOrientStats(bool enable)     { _orientStats = enable; }

void Apis::setRangefinderSensitivity(SensitivityMode mode) {
    _sensitivity = mode;
    Wire.beginTransmission(_adr);
    Wire.write(REG_CONFIG);
    Wire.write((uint8_t)_sensitivity);
    Wire.endTransmission();
}

void Apis::setI2CAddress(uint8_t newAddress) {
    Wire.beginTransmission(_adr);
    Wire.write(REG_I2C_ADDR);
    Wire.write(newAddress);
    Wire.endTransmission();
}

bool Apis::updateRange() {
    if (_needsStartupDelay) {
        _waitUntilReady();
        _needsStartupDelay = false;
    }
    uint8_t data1 = 0;
    uint8_t data2 = 0;

    Wire.beginTransmission(_adr);
    Wire.write(REG_RANGE_L);
    Wire.endTransmission();
    Wire.requestFrom(_adr, 1);
    data1 = Wire.read();

    Wire.beginTransmission(_adr);
    Wire.write(REG_RANGE_H);
    Wire.endTransmission();
    Wire.requestFrom(_adr, 1);
    data2 = Wire.read();

    _range = (int16_t)((data2 << 8) | data1);

    Wire.beginTransmission(_adr);
    Wire.write(REG_SIGNAL_STR);
    Wire.endTransmission();
    Wire.requestFrom(_adr, 1);
    _signalStrength = Wire.read();

    if (_range < 0) {
        _range = APIS_ERROR;
        return false;
    }
    return true;
}

bool Apis::updateOrientation() {
    if (_needsStartupDelay) {
        _waitUntilReady();
        _needsStartupDelay = false;
    }
    uint8_t data1 = 0, data2 = 0;
    int16_t dataSet[6];

    // Accel raw X/Y/Z at REG_ACCEL_BASE (0x10–0x15)
    for (int i = 0; i < 3; i++) {
        Wire.beginTransmission(_adr);
        Wire.write(REG_ACCEL_BASE + 2*i);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        data1 = Wire.read();

        Wire.beginTransmission(_adr);
        Wire.write(REG_ACCEL_BASE + 2*i + 1);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        data2 = Wire.read();

        dataSet[i] = ((data2 << 8) | data1);
    }

    // Accel offsets X/Y/Z at REG_OFFSET_BASE (0x18–0x1D)
    for (int i = 0; i < 3; i++) {
        Wire.beginTransmission(_adr);
        Wire.write(REG_OFFSET_BASE + 2*i);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        data1 = Wire.read();

        Wire.beginTransmission(_adr);
        Wire.write(REG_OFFSET_BASE + 2*i + 1);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        data2 = Wire.read();

        dataSet[3+i] = ((data2 << 8) | data1);
    }

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

void Apis::_waitUntilReady() {
    // Poll REG_STATUS bit 0 until the firmware signals ready or 150 ms elapse.
    // The 150 ms covers the full firmware startup: delay(10) + POWER_SW high +
    // 680 uF cap charge (~15 ms at MIC2544 current limit) + delay(100) +
    // ENABLE high + InitAccel + InitLiDAR ~= 115 ms, with margin.
    // The TLV61220 boost converter is always on (EN tied to VIN+) and produces
    // stable 5V before the ATTiny starts, so no converter startup is added here.
    // Old firmware leaves REG_STATUS=0 always and times out; new firmware
    // (github.com/NorthernWidget/Project-Apis/issues/15)
    // sets bit 0 after InitLiDAR(), allowing an early exit.
    uint32_t start = millis();
    while (millis() - start < 150) {
        Wire.beginTransmission(_adr);
        Wire.write(REG_STATUS);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        if (Wire.read() & 0x01) return;
        delay(5);
    }
}

bool Apis::updateMeasurements() {
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

    // Float comparisons with APIS_ERROR are safe: the value is assigned directly,
    // never computed, so the float representation is exact and consistent.
    return (_range != APIS_ERROR) && (_pitch != APIS_ERROR) && (_roll != APIS_ERROR);
}

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

void Apis::beginRawReadings(uint8_t component) {
    _rawComponent = component;
}

uint16_t Apis::takeRawReading(char* buf, uint16_t offset) {
    if (_rawComponent == NW_READING_ALL || _rawComponent == NW_READING_RANGE) {
        updateRange();
        offset += snprintf(buf + offset, 8, "%d,", (int)_range);
    }
    if (_rawComponent == NW_READING_ALL || _rawComponent == NW_READING_ORIENT) {
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
