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

// Print into a fixed buffer: the in-memory Print destination from the design.
class BufferPrint : public Print {
    char* _buf; size_t _cap, _len = 0;
  public:
    BufferPrint(char* buf, size_t cap) : _buf(buf), _cap(cap) { _buf[0] = 0; }
    size_t write(uint8_t c) override { if (_len + 1 >= _cap) return 0; _buf[_len++] = c; _buf[_len] = 0; return 1; }
    size_t length() const { return _len; }
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"   // the deprecated names are under test on purpose
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
    // Print-based interface: header, stored reading, and one logged reading.
    char pb[64]; BufferPrint bp(pb, sizeof pb);
    a.beginReadings(Apis::ALL); a.printHeader(bp); printf("printHeader ALL: %s\n", pb);
    BufferPrint bp2(pb, sizeof pb); a.printReading(bp2); printf("printReading ALL (stored): %s\n", pb);
    BufferPrint bp3(pb, sizeof pb); size_t nb = a.logReading(bp3); a.endReadings();
    printf("logReading ALL (%zu bytes): %s\n", nb, pb);
    a.beginReadings(Apis::RANGE); BufferPrint bp4(pb, sizeof pb); a.printHeader(bp4); printf("printHeader RANGE: %s\n", pb);
    BufferPrint bp5(pb, sizeof pb); a.logReading(bp5); a.endReadings(); printf("logReading RANGE: %s\n", pb);
}
#pragma GCC diagnostic pop

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

    // 5b. Per-chip reading: RANGE alone must not touch pitch/roll; counts reported.
    loadImage(400, 50, 100, 50, 1000, 0, 0, 0);
    { Apis a(4, true, 2, true); a.begin(); a.updateMeasurements();
      Wire.image[0x08] = 0x2C; Wire.image[0x09] = 0x01;   // range -> 300
      Wire.image[0x10] = 0x00; Wire.image[0x11] = 0x00;   // ax -> 0 (would change pitch if read)
      bool ok = a.updateMeasurements(Apis::RANGE);
      printf("[per-chip] RANGE ok=%d string(false): %s counts=%u/%u\n", ok, a.getString(false).c_str(),
             a.getRangeCount(), a.getOrientCount());
      ok = a.updateMeasurements(Apis::ORIENT);
      printf("[per-chip] ORIENT ok=%d string(false): %s\n", ok, a.getString(false).c_str()); }

    // 6. Wrong name: begin() must fail.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0); Wire.image[0x01] = 'X';
    { Apis a; printf("[wrong name] begin=%d\n", a.begin()); }

    fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);   // metric, not output
    return 0;
}
