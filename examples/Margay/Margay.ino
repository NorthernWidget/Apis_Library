#include <Margay.h>
#include <Apis.h>

Margay Logger(MODEL_3v0);  // update to match your hardware version
Apis rangefinder;

uint8_t I2CVals[] = {ADR_DEFAULT};
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
    initialize();
    return rangefinder.getString();
}

void initialize() {
    rangefinder.begin();
}
