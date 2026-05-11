#include <Apis.h>

Apis rangefinder;

void setup() {
    Serial.begin(9600);
    Serial.println("Apis LiDAR rangefinder");
    Serial.println("Reads range [cm], pitch [deg], and roll [deg] once per second.");

    if (!rangefinder.begin()) {
        Serial.println("Apis not found. Check wiring.");
        while (1);
    }

    Serial.println(rangefinder.getHeader());
}

void loop() {
    Serial.println(rangefinder.getString());
    delay(1000);
}
