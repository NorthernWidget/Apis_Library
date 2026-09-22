// Apis on a Margay data logger: one row per log interval. A failed begin() or
// a latched device fault is reported in one word through Logger.note(), which
// writes it in the logger's Note column (always last, no comma after it),
// prints it to Serial, and pulses the LED. Measurement columns keep -9999.
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

bool failed = false;

// One word for the row's Note column, the serial monitor, and the LED.
void report(const String& what) {
    failed = true;
    Logger.note(what);
}

String update() {
    failed = false;
    if (!initialize()) {
        // begin() refuses for four reasons; tell them apart with what it leaves behind.
        Wire.beginTransmission(Apis::DEFAULT_ADDRESS);
        if (Wire.endTransmission() != 0)                                  report("NoACK");
        else if (rangefinder.getFirmwareVersion() < APIS_FW_MIN_PATCH)     report("OldFirmware");
        else                                                                report("NotSchema1");
    }
    String row = rangefinder.getString();      // takes the readings; -9999 where one failed
    if (!failed && rangefinder.anyFault()) {
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
    return row;
}

bool initialize() {
    return rangefinder.begin();
}
