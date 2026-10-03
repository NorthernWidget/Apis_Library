#include <Margay.h>
#include <Apis.h>

Margay Logger(MODEL_3v0);  // update to match your hardware version
Apis rangefinder;

uint32_t updateRate = 60;  // seconds between readings

void setup() {
    // One line states the sensor, where it is, and its column order. The logger
    // writes the file's header and every row from it, and notes a sensor that
    // refused: this sketch composes nothing and keeps no header.
    Logger.watch(rangefinder);
    Logger.begin();
}

void loop() {
    Logger.run(updateRate);
}
