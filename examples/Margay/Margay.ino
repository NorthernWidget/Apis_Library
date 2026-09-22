// Apis on a Margay data logger: one row per log interval, with a Note column
// that names a sensor failure in one word so a file reader can see it without
// the serial monitor. Measurement columns keep -9999 when a reading fails.
// Note is always the last column and carries no comma after it, so the row
// ends cleanly: every sensor ends its fields with a comma for the next one.
#include <Margay.h>
#include <Apis.h>

Margay Logger(MODEL_3v0);  // update to match your hardware version
Apis rangefinder;

uint8_t I2CVals[] = {ADR_DEFAULT};
String header = "";
uint32_t updateRate = 60;  // seconds between readings

void setup() {
    header = rangefinder.getHeader() + "Note";
    Logger.begin(I2CVals, sizeof(I2CVals), header);
    initialize();
}

void loop() {
    Logger.run(update, updateRate);
}

// One word naming why the last begin() or reading failed; empty when all is well.
String note = "";

// Report a failure on the serial monitor and the LED, and remember it for the row.
void report(const String& what) {
    note = what;
    Serial.print(F("Apis: "));
    Serial.println(what);
    Logger.LED_Color(ORANGE);
    delay(300);
    Logger.LED_Color(OFF);
}

String update() {
    note = "";
    if (!initialize()) {
        // begin() refuses for four reasons; tell them apart with what it leaves behind.
        Wire.beginTransmission(ADR_DEFAULT);
        if (Wire.endTransmission() != 0)                                  report("NoACK");
        else if (rangefinder.getFirmwareVersion() < APIS_FW_MIN_PATCH)     report("OldFirmware");
        else                                                                report("NotSchema1");
    }
    String row = rangefinder.getString();      // takes the readings; -9999 where one failed
    if (note.length() == 0 && rangefinder.anyFault()) {
        // Latched fault from the device: which chip, what kind (NW-Device-Specification Block 0).
        static const char* const chips[] = {"LiDAR", "Accel"};
        static const char* const kinds[] = {"", "NoACK", "Timeout", "Checksum", "Range",
                                            "NotInit", "Reset", "Config", "Supply"};
        uint8_t chip = rangefinder.faultChip(), kind = rangefinder.faultKind();
        String what = (chip < 2) ? chips[chip] : (chip == 7) ? "Unit" : "Chip" + String(chip);
        what += (kind < 9) ? kinds[kind] : "Kind" + String(kind);
        report(what);
        Serial.print(F("        "));
        rangefinder.printFault(Serial);        // the same fault in words, for a person watching
        Serial.println();
    }
    return row + note;
}

bool initialize() {
    return rangefinder.begin();
}
