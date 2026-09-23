#include <Margay.h>
#include <Apis.h>

Margay Logger(MODEL_3v0);  // update to match your hardware version
Apis rangefinder;

uint8_t I2CVals[] = {Apis::DEFAULT_ADDRESS};
String header = "";
uint32_t updateRate = 60;  // seconds between readings

void setup() {
    header = rangefinder.getHeader();
    Logger.begin(I2CVals, sizeof(I2CVals), header);
    initialize();
}

void loop() {
    Logger.run(update, updateRate);
}

String update() {
    bool ok = initialize();
    if (!ok) Logger.note(rangefinder.beginFailure());  // e.g. NoACK
    String row = rangefinder.getString();  // -9999 where a reading failed
    if (ok && rangefinder.anyFault()) {
        Logger.note(rangefinder.reportNote());  // e.g. LiDARTimeout
    }
    return row;
}

bool initialize() {
    return rangefinder.begin();
}
