#include <Apis.h>

Apis rangefinder;

void setup() {
    Serial.begin(9600);
    Serial.println("Apis LiDAR rangefinder");
    Serial.println("Reads range [cm], pitch [deg], and roll [deg] once per second.");

    if (!rangefinder.begin()) {
        // No ACK, not Schema 1, wrong device name, or firmware older than this library needs.
        Serial.print("Apis refused. Firmware patch reported: ");
        Serial.print(rangefinder.getFirmwareVersion());
        Serial.print(" (need >= ");
        Serial.print(APIS_FW_MIN_PATCH);
        Serial.println("). Check wiring, then reflash and provision.");
        while (1);
    }
    Serial.print("Apis hardware v"); Serial.print(rangefinder.getHardwareMajor());
    Serial.print("."); Serial.print(rangefinder.getHardwareMinor());
    Serial.print(", firmware patch "); Serial.println(rangefinder.getFirmwareVersion());

    rangefinder.printDataHeader(Serial);  // straight to the port: no row is built in RAM
    Serial.println();
}

void loop() {
    rangefinder.updateMeasurements();    // take the readings
    rangefinder.printDataRow(Serial);    // print what they left
    Serial.println();
    delay(1000);
}
