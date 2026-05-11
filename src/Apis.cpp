#include "Apis.h"

Apis::Apis(uint16_t nRangeReadings, bool rangeStats,
           uint16_t nOrientReadings, bool orientStats)
    : _nRangeReadings(nRangeReadings), _rangeStats(rangeStats),
      _nOrientReadings(nOrientReadings), _orientStats(orientStats),
      _rawComponent(NW_READING_ALL)
{
}

bool Apis::begin(uint8_t address, uint8_t sensitivity)
{
    _adr = address;
    _sensitivity = sensitivity;
    Wire.begin();
    Wire.beginTransmission(_adr);
    Wire.write(0x01);
    Wire.write(_sensitivity);
    return Wire.endTransmission() == 0;
}

void Apis::setNRangeReadings(uint16_t n)  { _nRangeReadings = n; }
void Apis::setRangeStats(bool enable)      { _rangeStats = enable; }
void Apis::setNOrientReadings(uint16_t n) { _nOrientReadings = n; }
void Apis::setOrientStats(bool enable)     { _orientStats = enable; }

bool Apis::updateRange() {
    uint8_t data1 = 0;
    uint8_t data2 = 0;

    Wire.beginTransmission(_adr);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(_adr, 1);
    data1 = Wire.read();

    Wire.beginTransmission(_adr);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(_adr, 1);
    data2 = Wire.read();

    _range = (int16_t)((data2 << 8) | data1);

    if (_range < 0) {
        _range = -9999;
        return false;
    }
    return true;
}

bool Apis::updateOrientation() {
    uint8_t data1 = 0, data2 = 0;
    int16_t dataSet[6];

    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(_adr);
        Wire.write(2*i + 4);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        data1 = Wire.read();

        Wire.beginTransmission(_adr);
        Wire.write(2*i + 5);
        Wire.endTransmission();
        Wire.requestFrom(_adr, 1);
        data2 = Wire.read();

        dataSet[i] = ((data2 << 8) | data1);
    }

    float gx = dataSet[0], gy = dataSet[1], gz = dataSet[2];
    float offsetX = dataSet[3], offsetY = dataSet[4], offsetZ = dataSet[5];

    if (gx == gy && gx == gz && gx == -1) {
        _pitch = _roll = -9999;
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

bool Apis::updateMeasurements() {
    // Allow capacitor to settle before first reading
    delay(200);

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
        _range = -9999;
        _rangeMean = _rangeStd = _rangeSterr = -9999;
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
        _pitch = _roll = -9999;
        _pitchStd = _pitchSterr = _rollStd = _rollSterr = -9999;
    } else {
        _pitch = pitchMean;
        _roll  = rollMean;
        _pitchStd   = (orientN > 1) ? sqrt(pitchM2 / (orientN - 1)) : 0;
        _pitchSterr = (orientN > 1) ? _pitchStd / sqrt((float)orientN) : 0;
        _rollStd    = (orientN > 1) ? sqrt(rollM2  / (orientN - 1)) : 0;
        _rollSterr  = (orientN > 1) ? _rollStd  / sqrt((float)orientN) : 0;
    }

    return (_range != -9999) && (_pitch != -9999) && (_roll != -9999);
}

int16_t Apis::getRange() {
    // Distance in cm (rounded mean when nRangeReadings > 1)
    return _range;
}

float Apis::getRoll() {
    // Roll in degrees
    return _roll;
}

float Apis::getPitch() {
    // Pitch in degrees
    return _pitch;
}

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
            offset += snprintf(buf + offset, 13, "-9999,-9999,");
        }
    }
    return offset;
}

void Apis::endRawReadings() {
    // No cleanup required currently
}
