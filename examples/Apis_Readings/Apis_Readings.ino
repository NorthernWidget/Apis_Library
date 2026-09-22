// Apis_Readings: take many individual range readings and print each one,
// using the Print-based reading interface. Output goes to Serial here; pass
// an SdFat File instead to write the same lines to a card.
#include <Apis.h>

Apis rangefinder;
const uint16_t N = 20;

void setup() {
    Serial.begin(9600);
    if (!rangefinder.begin()) {
        Serial.print("Apis refused; firmware patch "); Serial.println(rangefinder.getFirmwareVersion());
        while (1);
    }
    rangefinder.beginReadings(Apis::RANGE, N);   // range only, N readings (LiDAR held powered for the burst); Apis::ALL adds pitch and roll
    Serial.print("n,");
    rangefinder.printHeader(Serial);
    Serial.println();
}

void loop() {
    for (uint16_t i = 0; i < N; i++) {
        Serial.print(i); Serial.print(',');
        rangefinder.logReading(Serial);       // one reading per call
        Serial.println();
    }
    rangefinder.endReadings();
    delay(10000);
    rangefinder.beginReadings(Apis::RANGE);
}
