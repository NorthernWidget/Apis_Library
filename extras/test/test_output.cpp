// Output-regression test for Apis_Library: compiles src/Apis.cpp against the
// stubs in this directory and prints getHeader()/getString()/takeRawReading()
// for fixed register images. run.sh diffs the result against baseline.txt.
#include "Arduino.h"
#include "Wire.h"
TwoWire Wire;
#include "../../src/Apis.cpp"

static void loadImage(int16_t range, uint8_t signal, int16_t ax, int16_t ay, int16_t az,
                      int16_t ox, int16_t oy, int16_t oz) {
    uint8_t* r = Wire.image; memset(r, 0, sizeof(Wire.image));
    r[0x00] = 0x01; r[0x01] = 'A'; r[0x02] = 'p'; r[0x03] = 'i'; r[0x04] = 's';
    r[0x05] = 1; r[0x06] = 0; r[0x07] = 2;
    r[0x08] = range & 0xFF; r[0x09] = (range >> 8) & 0xFF; r[0x0A] = signal; r[0x0C] = 0x50;
    int16_t a[3] = {ax, ay, az}, o[3] = {ox, oy, oz};
    for (int i = 0; i < 3; i++) { r[0x10 + 2*i] = a[i] & 0xFF; r[0x11 + 2*i] = (a[i] >> 8) & 0xFF;
                                  r[0x18 + 2*i] = o[i] & 0xFF; r[0x19 + 2*i] = (o[i] >> 8) & 0xFF; }
}

static void report(const char* name, Apis& a) {
    printf("[%s]\n", name);
    printf("header: %s\n", a.getHeader().c_str());
    printf("string: %s\n", a.getString().c_str());
    char buf[64] = {0}; uint16_t o = 0;
    a.beginRawReadings(NW_READING_ALL); o = a.takeRawReading(buf, 0); a.endRawReadings();
    printf("raw ALL (%u bytes): %s\n", o, buf);
    memset(buf, 0, sizeof buf);
    a.beginRawReadings(NW_READING_RANGE); o = a.takeRawReading(buf, 0); a.endRawReadings();
    printf("raw RANGE (%u bytes): %s\n", o, buf);
    printf("getters: range=%d roll=%.4f pitch=%.4f signal=%u mean=%.4f std=%.4f sterr=%.4f\n",
           a.getRange(), a.getRoll(), a.getPitch(), a.getSignalStrength(),
           a.getRangeMean(), a.getRangeStd(), a.getRangeSterr());
}

int main() {
    // 1. Single reading, no statistics, level board.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; if (!a.begin()) { puts("begin failed"); return 1; } report("N=1, no stats, level", a); }

    // 2. Tilted board with offsets, still N=1.
    loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
    { Apis a; a.begin(); report("N=1, tilted with offsets", a); }

    // 3. N=5 with statistics; the image changes between reads so the
    //    statistics paths are exercised with differing readings.
    { Apis a(5, true, 3, true); a.begin();
      // A new reading begins whenever the register pointer moves backwards
      // (the previous reading finished at a higher address). Only then does the
      // image change, so the values within one reading are always consistent
      // and the count of readings, not of bus transactions, drives the sequence.
      int k = 0; int lastPtr = 0xFF;
      Wire.beforeRead = [&](TwoWire& w, uint8_t ptr) {
          if (ptr <= lastPtr) { k++;
              int16_t r = 300 + 3 * (k % 5);   w.image[0x08] = r & 0xFF;  w.image[0x09] = (r >> 8) & 0xFF;
              int16_t ax = 100 + 7 * (k % 3);  w.image[0x10] = ax & 0xFF; w.image[0x11] = (ax >> 8) & 0xFF; }
          lastPtr = ptr; };
      loadImage(300, 90, 100, 50, 1000, 0, 0, 0);
      report("N=5 range stats, N=3 orient stats, varying", a);
      Wire.beforeRead = nullptr; }

    // 4. Error image: negative range, accelerometer bus-failure signature.
    loadImage(-1, 0, -1, -1, -1, 0, 0, 0);
    { Apis a(3, true, 1, false); a.begin(); report("errors: range<0, accel 0xFFFF", a); }

    // 5. Before any measurement: getString(false) prints the not-measured sentinels.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; a.begin(); printf("[not measured]\nstring: %s\n", a.getString(false).c_str()); }

    // 6. Wrong name: begin() must fail.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0); Wire.image[0x01] = 'X';
    { Apis a; printf("[wrong name] begin=%d\n", a.begin()); }

    fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);   // metric, not output
    return 0;
}
