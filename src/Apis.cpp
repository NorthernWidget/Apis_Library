#include "Apis.h"

Apis::Apis()
{
}

void Apis::begin(uint8_t ADR_, uint8_t Sensitivity)
{
    // Set address and sensitivity
    ADR = ADR_;
    Wire.begin();
    Wire.beginTransmission(ADR_);
    Wire.write(0x01);
    Wire.write(Sensitivity);
    Wire.endTransmission();
}

bool Apis::updateRange() {
    uint8_t Data1 = 0;
    uint8_t Data2 = 0;

    Wire.beginTransmission(ADR);
    Wire.write(0x02);
    Wire.endTransmission();
    Wire.requestFrom(ADR, 1);
    Data1 = Wire.read();

    Wire.beginTransmission(ADR);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(ADR, 1);
    Data2 = Wire.read();

    Range = (int16_t)((Data2 << 8) | Data1);

    if (Range < 0) {
        Range = -9999;
        return false;
    }
    return true;
}

bool Apis::updateMeasurements(uint8_t nReadings) {
    // Allow capacitor to settle before first reading
    delay(200);

    // Take nReadings range measurements and average successful ones
    float rangeSum = 0;
    uint8_t nSuccess = 0;
    for (uint8_t i = 0; i < nReadings; i++) {
        if (updateRange()) {
            rangeSum += Range;
            nSuccess++;
        }
    }
    Range = (nSuccess > 0) ? (int16_t)(rangeSum / nSuccess) : -9999;

    // Read accelerometer gX/Y/Z (0x04-0x09) and offsets (0x0A-0x0F)
    uint8_t Data1 = 0;
    uint8_t Data2 = 0;
    int16_t DataSet[6];

    for (int i = 0; i < 6; i++) {
        Wire.beginTransmission(ADR);
        Wire.write(2*i + 4);
        Wire.endTransmission();
        Wire.requestFrom(ADR, 1);
        Data1 = Wire.read();

        Wire.beginTransmission(ADR);
        Wire.write(2*i + 5);
        Wire.endTransmission();
        Wire.requestFrom(ADR, 1);
        Data2 = Wire.read();

        DataSet[i] = ((Data2 << 8) | Data1);
    }

    float gX = DataSet[0];
    float gY = DataSet[1];
    float gZ = DataSet[2];
    float OffsetX = DataSet[3];
    float OffsetY = DataSet[4];
    float OffsetZ = DataSet[5];

    if (gX == gY && gX == gZ && gX == -1) {
        Pitch = -9999;
        Roll = -9999;
    } else if (OffsetX == OffsetY && OffsetX == OffsetZ && OffsetX == 0) {
        Pitch = atan(-gX/gZ) * 180. / M_PI;
        Roll = atan(gY / sqrt(pow(gX, 2) + pow(gZ, 2))) * 180. / M_PI;
    } else {
        Pitch = (atan(-gX/gZ) - atan(-OffsetX/OffsetZ)) * 180. / M_PI;
        Roll = (atan(gY / sqrt(pow(gX, 2) + pow(gZ, 2)))
               - atan(OffsetY / sqrt(pow(OffsetX, 2) + pow(OffsetZ, 2)))) * 180. / M_PI;
    }

    return (Range != -9999) && (Pitch != -9999) && (Roll != -9999);
}

float Apis::getRoll() {
    // Roll in degrees
    return Roll;
}

float Apis::getPitch() {
    // Pitch in degrees
    return Pitch;
}

int16_t Apis::getRange() {
    // Distance in cm
    return Range;
}

String Apis::getString(bool takeNewReadings) {
    if (takeNewReadings) {
        updateMeasurements();
    }
    return String(Range) + "," + String(Pitch) + "," + String(Roll) + ",";
}

String Apis::getHeader() {
    return "Range [cm],Pitch [deg],Roll [deg],";
}

