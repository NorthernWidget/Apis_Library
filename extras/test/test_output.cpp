// Output-regression test for Apis_Library: compiles src/Apis.cpp against the
// stubs in this directory and prints getHeader()/getString()/takeRawReading()
// for fixed register images. run.sh diffs the result against baseline.txt.
#include "Arduino.h"
#include "Wire.h"
TwoWire Wire;
#include "../../src/Apis.cpp"

static uint8_t crc8(const uint8_t* d, uint8_t n) {           // CRC-8/SMBUS, as NW-Provision writes it
    uint8_t c = 0; for (uint8_t i = 0; i < n; i++) { c ^= d[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1); }
    return c;
}

// Build a Schema 1 register image: Page 0 as NW-Provision writes it (with the
// firmware's patch at 0x0A), Page 1 with a complete reading, Page 2 offsets.
static void loadImage(int16_t range, uint8_t signal, int16_t ax, int16_t ay, int16_t az,
                      int16_t ox, int16_t oy, int16_t oz, uint8_t fwPatch = 2, uint8_t schema = 0x01) {
    uint8_t* r = Wire.image; memset(r, 0, sizeof(Wire.image));
    r[0x00] = schema; r[0x01] = 'A'; r[0x02] = 'p'; r[0x03] = 'i'; r[0x04] = 's';
    r[0x08] = 0; r[0x09] = 1; r[0x0A] = fwPatch;                       // HW 0.1, FW patch
    r[0x10] = 0x41; r[0x11] = 0x01; r[0x12] = 0; r[0x13] = 7; r[0x14] = 0; r[0x15] = 42;
    r[0x1D] = 0x4E; r[0x1E] = crc8(r, 0x1E); r[0x1F] = 0x41;
    r[0x20] = 0x01;                                                   // ready
    r[0x21] = 0x06;                                                   // both chips selected
    r[0x22] = 1; r[0x23] = 0;                                         // reading counter = 1
    r[0x26] = 0x00; r[0x27] = 0x00;
    r[0x28] = range & 0xFF; r[0x29] = (range >> 8) & 0xFF; r[0x2A] = signal;
    int16_t a[3] = {ax, ay, az}, o[3] = {ox, oy, oz};
    for (int i = 0; i < 3; i++) { r[0x30 + 2*i] = a[i] & 0xFF; r[0x31 + 2*i] = (a[i] >> 8) & 0xFF;
                                  r[0x40 + 2*i] = o[i] & 0xFF; r[0x41 + 2*i] = (o[i] >> 8) & 0xFF; }
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

// Emulate the Schema 1 firmware's response to a control write: a trigger
// completes a reading at once — counter +1, ready set, trigger and sleep
// bits cleared, fault byte cleared. A per-test hook can vary the data.
static std::function<void(TwoWire&)> onReading;
static uint16_t lastRequest = 0;   // readings-requested word as the stub firmware saw it
static void installFirmwareEmulation() {
    Wire.onWrite = [](TwoWire& w, uint8_t reg, uint8_t val) {
        if (reg == 0x24) lastRequest = (lastRequest & 0xFF00) | val;
        if (reg == 0x25) lastRequest = (lastRequest & 0x00FF) | (val << 8);
        if (reg != 0x21) return;
        w.image[0x27] = 0;                                  // any control write acknowledges the fault
        if (!(val & 0x01)) return;
        w.image[0x21] = val & 0x7E;                         // trigger and sleep consumed
        if (onReading) onReading(w);
        uint16_t c = w.image[0x22] | (w.image[0x23] << 8); c++;
        w.image[0x22] = c & 0xFF; w.image[0x23] = c >> 8;
        w.image[0x20] |= 0x01;
    };
}

int main() {
    installFirmwareEmulation();
    // 1. Single reading, no statistics, level board.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; if (!a.begin()) { puts("begin failed"); return 1; } report("N=1, no stats, level", a); }

    // 2. Tilted board with offsets, still N=1.
    loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
    { Apis a; a.begin(); report("N=1, tilted with offsets", a); }

    // 3. N=5 with statistics; the image changes between reads so the
    //    statistics paths are exercised with differing readings.
    { Apis a(5, true, 3, true); a.begin();
      // Each trigger yields a fresh reading: range steps through five values,
      // the X axis through three, so both statistics paths see spread.
      int k = 0;
      onReading = [&](TwoWire& w) { k++;
          int16_t r = 300 + 3 * (k % 5);   w.image[0x28] = r & 0xFF;  w.image[0x29] = (r >> 8) & 0xFF;
          int16_t ax = 100 + 7 * (k % 3);  w.image[0x30] = ax & 0xFF; w.image[0x31] = (ax >> 8) & 0xFF; };
      loadImage(300, 90, 100, 50, 1000, 0, 0, 0);
      report("N=5 range stats, N=3 orient stats, varying", a);
      onReading = nullptr; }

    // 4. Error image: negative range, accelerometer bus-failure signature.
    loadImage(-1, 0, -1, -1, -1, 0, 0, 0);
    { Apis a(3, true, 1, false); a.begin(); report("errors: range<0, accel 0xFFFF", a); }

    // 5. Before any measurement: getString(false) prints the not-measured sentinels.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; a.begin(); printf("[not measured]\nstring: %s\n", a.getString(false).c_str()); }

    // 5b. Per-chip reading: RANGE alone must not touch pitch/roll; counts reported.
    loadImage(400, 50, 100, 50, 1000, 0, 0, 0);
    { Apis a(4, true, 2, true); a.begin(); a.updateMeasurements();
      Wire.image[0x28] = 0x2C; Wire.image[0x29] = 0x01;   // range -> 300
      Wire.image[0x30] = 0x00; Wire.image[0x31] = 0x00;   // ax -> 0 (would change pitch if read)
      bool ok = a.updateMeasurements(Apis::RANGE);
      printf("[per-chip] RANGE ok=%d string(false): %s counts=%u/%u\n", ok, a.getString(false).c_str(),
             a.getRangeCount(), a.getOrientCount());
      ok = a.updateMeasurements(Apis::ORIENT);
      printf("[per-chip] ORIENT ok=%d string(false): %s\n", ok, a.getString(false).c_str()); }

    // 5b2. Medians and clamping: 7 range readings 300..318 step 3 (median 309), capacity clamp.
    loadImage(300, 90, 100, 50, 1000, 0, 0, 0);
    { Apis a; a.begin(); int k = 0;
      onReading = [&](TwoWire& w) { int16_t r = 300 + 3 * (k++ % 7); w.image[0x28] = r & 0xFF; w.image[0x29] = (r >> 8) & 0xFF; };
      printf("[median] setRangeReadings(7)=%u setRangeReadings(1000)=%u\n", a.setRangeReadings(7), a.setRangeReadings(1000));
      a.setRangeReadings(7); a.setOrientReadings(3); a.updateMeasurements();
      printf("[median] count=%u mean=%.4f median=%.4f std=%.4f pitchMedian=%.4f rollMedian=%.4f\n",
             a.getRangeCount(), a.getRangeMean(), a.getRangeMedian(), a.getRangeStd(), a.getPitchMedian(), a.getRollMedian());
      onReading = nullptr; }

    // 5c. Handshake: ready/newReading/requestReading against the emulated firmware.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; a.begin();
      printf("[handshake] ready=%d newReading(before any)=%d", a.ready(), a.newReading());
      a.updateRange();
      printf(" counter after 1 reading=%u newReading=%d", Wire.image[0x22] | (Wire.image[0x23] << 8), a.newReading());
      Wire.image[0x27] = 0x02;                       // firmware latched a LiDAR timeout
      a.requestReading(Apis::RANGE);
      printf(" ctrl after request=0x%02X fault after ack=0x%02X\n", Wire.image[0x21], Wire.image[0x27]);
      // timeout path: firmware that never answers (no emulation)
      auto saved = Wire.onWrite; Wire.onWrite = nullptr;
      bool ok = a.updateRange();
      printf("[handshake] no firmware response: updateRange=%d range=%d\n", ok, a.getRange());
      Wire.onWrite = saved; }

    // 5d. Faults: the firmware reports a LiDAR fault in status and a latched code; then a unit reset code.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; a.begin(); char pb[48];
      onReading = [](TwoWire& w) { w.image[0x20] = 0x83; w.image[0x27] = 0x02; };   // ready | LiDAR fault | pan; LiDAR: timeout
      a.updateRange(); BufferPrint bp(pb, sizeof pb); a.printFault(bp);
      printf("[faults] faulted(0)=%d faulted(1)=%d any=%d chip=%u kind=%u text='%s'\n",
             a.faulted(0), a.faulted(1), a.anyFault(), a.faultChip(), a.faultKind(), pb);
      onReading = [](TwoWire& w) { w.image[0x20] = 0x01; w.image[0x27] = 0xE6; };   // clean reading; unit: reset since configured
      a.updateRange(); BufferPrint bp2(pb, sizeof pb); a.printFault(bp2);
      printf("[faults] any=%d chip=%u kind=%u text='%s'\n", a.anyFault(), a.faultChip(), a.faultKind(), pb);
      onReading = nullptr; }

    // 6. Bursts: the readings-requested word reaches the device before the readings.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; a.begin(); lastRequest = 0;
      a.setRangeReadings(5); a.updateMeasurements(Apis::RANGE);
      printf("[request] updateMeasurements N=5 -> word=%u\n", lastRequest);
      lastRequest = 0; a.setRangeReadings(1); a.updateMeasurements(Apis::RANGE);
      printf("[request] updateMeasurements N=1 -> word=%u (no write)\n", lastRequest);
      lastRequest = 0; char pb[128]; BufferPrint bp(pb, sizeof pb);
      int k = 0; onReading = [&](TwoWire& w) { int16_t r = 240 + 10 * (k++); w.image[0x28] = r & 0xFF; w.image[0x29] = (r >> 8) & 0xFF; };
      a.beginReadings(Apis::RANGE, 4); for (int i = 0; i < 4; i++) a.logReading(bp); a.endReadings();
      unsigned tx = Wire.transactions;
      printf("[request] beginReadings(RANGE, 4) -> word=%u rows=%s\n", lastRequest, pb);
      printf("[burst stats] count=%u mean=%.2f std=%.4f median=%.2f transactions during stats=%u\n",
             a.getRangeCount(), a.getRangeMean(), a.getRangeStd(), a.getRangeMedian(), Wire.transactions - tx);
      onReading = nullptr; }

    // 7. begin() gates: wrong name, wrong schema, firmware too old, and the versions it reports.
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0); Wire.image[0x01] = 'X';
    { Apis a; printf("[wrong name] begin=%d\n", a.begin()); }
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0, 1, 0x00);
    { Apis a; bool ok = a.begin(); printf("[schema 0x00] begin=%d fw=%u\n", ok, a.getFirmwareVersion()); }
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0, 1, 0xFF);
    { Apis a; printf("[schema 0xFF unprovisioned] begin=%d\n", a.begin()); }
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0, 0);
    { Apis a; bool ok = a.begin(); printf("[fw patch 0 < min %d] begin=%d fw=%u\n", APIS_FW_MIN_PATCH, ok, a.getFirmwareVersion()); }
    loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
    { Apis a; bool ok = a.begin(); printf("[versions] begin=%d hw=%u.%u fw=%u\n", ok, a.getHardwareMajor(), a.getHardwareMinor(), a.getFirmwareVersion()); }

    fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);   // metric, not output
    return 0;
}
