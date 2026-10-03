// Output-regression test for Apis_Library: compiles src/Apis.cpp against the
// stubs in this directory and prints printDataHeader()/printDataRow()/takeRawReading()
// for fixed register images. run.sh diffs the result against baseline.txt.
#include "Arduino.h"
#include "Wire.h"
TwoWire Wire;
#include "../../src/Apis.cpp"

#include "NW_TestSupport.h"

// Build a Schema 1 register image: Page 0 as NW-Provision writes it (with the
// firmware's patch at 0x0A), Page 2 with a complete reading, Page 1 offsets.
// The zero generation (0x38–0x39, mirrored at 0x58–0x59) is 1 when offsets are
// given and 0 (never zeroed) when they are all zero.
static void loadImage(int16_t range, uint8_t signal, int16_t ax, int16_t ay, int16_t az,
                      int16_t ox, int16_t oy, int16_t oz, uint8_t fwPatch = 5, uint8_t schema = 0x01,
                      int8_t accelT = 21, int8_t zeroT = 24) {
  uint8_t* r = Wire.image;
  nwLoadPage0(r, "Apis", 0x41, 1, fwPatch, schema);  // Page 0 and Block 0, HW 0.1
  r[0x48] = range & 0xFF;
  r[0x49] = (range >> 8) & 0xFF;
  r[0x4A] = signal;
  int16_t a[3] = { ax, ay, az }, o[3] = { ox, oy, oz };
  for (int i = 0; i < 3; i++) {
    r[0x50 + 2 * i] = a[i] & 0xFF;
    r[0x51 + 2 * i] = (a[i] >> 8) & 0xFF;
    r[0x20 + 2 * i] = o[i] & 0xFF;
    r[0x21 + 2 * i] = (o[i] >> 8) & 0xFF;
  }
  r[0x56] = 0x00;
  r[0x57] = (uint8_t)accelT;  // OUT_ADC3 word: the digit in the high byte
  r[0x26] = 0x00;
  r[0x27] = (uint8_t)zeroT;
  uint8_t generation = (ox || oy || oz) ? 1 : 0;
  r[0x38] = generation;
  r[0x39] = 0;
  r[0x58] = generation;
  r[0x59] = 0;
}

// Set the zero generation in the image, on Page 1 and in the reading's mirror.
static void setGeneration(uint8_t* r, uint16_t generation) {
  r[0x38] = generation & 0xFF;
  r[0x39] = generation >> 8;
  r[0x58] = r[0x38];
  r[0x59] = r[0x39];
}

// Fill the record of zeros on Page 1: Block k (0x20 + 8k) holds zero k of the
// three given (current, previous, the one before), each X, Y, Z and the
// temperature word with its digit in the high byte; then the generation.
static void loadZeros(uint16_t generation, const int16_t zeros[3][3], const int8_t temps[3]) {
  uint8_t* r = Wire.image;
  for (int k = 0; k < 3; k++) {
    for (int i = 0; i < 3; i++) {
      r[0x20 + 8 * k + 2 * i] = zeros[k][i] & 0xFF;
      r[0x21 + 8 * k + 2 * i] = (zeros[k][i] >> 8) & 0xFF;
    }
    r[0x26 + 8 * k] = 0x00;
    r[0x27 + 8 * k] = (uint8_t)temps[k];
  }
  setGeneration(r, generation);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"  // the deprecated names are under test on purpose
// The four String functions are gone (section 15 family B). These helpers hold
// this harness's output identical by replicating exactly what they did:
// getString(true) acquired first, getString(false) did not, and the header
// never did. 1024 because a full-width header of CSDMS standard names runs
// past 1000 characters here; each helper refuses to hide a truncation.
static const char* head(Apis& a) {
  static char b[2048];
  NW_BufferPrint p(b, sizeof b);
  a.printDataHeader(p);
  if (p.truncated()) printf("  TRUNCATED: head() needs a bigger buffer\n");
  return b;
}

static const char* row(Apis& a, bool takeNewReadings = true) {
  if (takeNewReadings) a.updateMeasurements();
  static char b[2048];
  NW_BufferPrint p(b, sizeof b);
  a.printDataRow(p);
  if (p.truncated()) printf("  TRUNCATED: row() needs a bigger buffer\n");
  return b;
}

static const char* note(Apis& a, bool beginFailed = false) {
  // Two buffers in rotation: one printf takes both a report word and a begin
  // failure, and a single static would have the second overwrite the first
  // before either is printed.
  static char buffers[2][64];
  static uint8_t which = 0;
  char* b = buffers[which];
  which = (uint8_t)(1 - which);
  NW_BufferPrint p(b, sizeof buffers[0]);
  a.printNote(p, beginFailed);
  if (p.truncated()) printf("  TRUNCATED: note() needs a bigger buffer\n");
  return b;
}

static void report(const char* name, Apis& a) {
  printf("[%s]\n", name);
  printf("header: %s\n", head(a));
  printf("string: %s\n", row(a));
  char buf[64] = { 0 };
  uint16_t o = 0;
  a.beginRawReadings(NW_READING_ALL);
  o = a.takeRawReading(buf, 0);
  a.endRawReadings();
  printf("raw ALL (%u bytes): %s\n", o, buf);
  memset(buf, 0, sizeof buf);
  a.beginRawReadings(NW_READING_RANGE);
  o = a.takeRawReading(buf, 0);
  a.endRawReadings();
  printf("raw RANGE (%u bytes): %s\n", o, buf);
  printf("getters: range=%.4f roll=%.4f pitch=%.4f signal=%u mean=%.4f std=%.4f sterr=%.4f\n",
         a.getDistance(), a.getRoll(), a.getPitch(), a.getSignalStrength(),
         a.getDistanceMean(), a.getDistanceStd(), a.getDistanceSterr());
  printf("accel:   x=%.4f y=%.4f z=%.4f |g|=%.4f tilt=%.4f  (mag std=%.4f sterr=%.4f, tilt median=%.4f)\n",
         a.getAccelerationX(), a.getAccelerationY(), a.getAccelerationZ(),
         a.getAccelerationMagnitude(), a.getTilt(),
         a.getAccelerationMagnitudeStd(), a.getAccelerationMagnitudeSterr(), a.getTiltMedian());
  // Print-based interface: header, stored reading, and one logged reading.
  // The buffer must hold a whole header. char[64] silently cut printHeader ALL
  // at "AccelT change [" and the baseline recorded the cut line as if it were
  // the output, so trunc= is printed beside everything streamed from here on.
  char pb[512];
  BufferPrint bp(pb, sizeof pb);
  a.beginReadings(Apis::ALL);
  a.printHeader(bp);
  printf("printHeader ALL: %s trunc=%d\n", pb, bp.truncated());
  BufferPrint bp2(pb, sizeof pb);
  a.printReading(bp2);
  printf("printReading ALL (stored): %s trunc=%d\n", pb, bp2.truncated());
  BufferPrint bp3(pb, sizeof pb);
  size_t nb = a.logReading(bp3);
  a.endReadings();
  printf("logReading ALL (%zu bytes): %s trunc=%d\n", nb, pb, bp3.truncated());
  a.beginReadings(Apis::RANGE);
  BufferPrint bp4(pb, sizeof pb);
  a.printHeader(bp4);
  printf("printHeader RANGE: %s trunc=%d\n", pb, bp4.truncated());
  BufferPrint bp5(pb, sizeof pb);
  a.logReading(bp5);
  a.endReadings();
  printf("logReading RANGE: %s trunc=%d\n", pb, bp5.truncated());
}
#pragma GCC diagnostic pop

int main() {
  Wire.deviceAddress = 0x41;
  installFirmwareEmulation();
  // 1. Single reading, no statistics, level board.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    if (!a.begin()) {
      puts("begin failed");
      return 1;
    }
    report("N=1, no stats, level", a);
  }

  // 2. Tilted board with offsets, still N=1.
  loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
  {
    Apis a;
    a.begin();
    report("N=1, tilted with offsets", a);
  }

  // 3. N=5 with statistics; the image changes between reads so the
  //    statistics paths are exercised with differing readings.
  {
    Apis a(5, true, 3, true);
    a.begin();
    // Each trigger yields a fresh reading: range steps through five values,
    // the X axis through three, so both statistics paths see spread.
    int k = 0;
    onReading = [&](TwoWire& w) {
      k++;
      int16_t r = 300 + 3 * (k % 5);
      w.image[0x48] = r & 0xFF;
      w.image[0x49] = (r >> 8) & 0xFF;
      int16_t ax = 100 + 7 * (k % 3);
      w.image[0x50] = ax & 0xFF;
      w.image[0x51] = (ax >> 8) & 0xFF;
    };
    loadImage(300, 90, 100, 50, 1000, 0, 0, 0);
    report("N=5 range stats, N=3 orient stats, varying", a);
    onReading = nullptr;
  }

  // 4. Error image: negative range, accelerometer bus-failure signature.
  loadImage(-1, 0, -1, -1, -1, 0, 0, 0);
  {
    Apis a(3, true, 1, false);
    a.begin();
    report("errors: range<0, accel 0xFFFF", a);
  }

  // 5. Before any measurement: a row without acquiring prints the not-measured sentinels.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    a.begin();
    printf("[not measured]\nstring: %s\n", row(a, false));
  }

  // 5b. Per-chip reading: RANGE alone must not touch pitch/roll; counts reported.
  loadImage(400, 50, 100, 50, 1000, 0, 0, 0);
  {
    Apis a(4, true, 2, true);
    a.begin();
    a.updateMeasurements();
    Wire.image[0x48] = 0x2C;
    Wire.image[0x49] = 0x01;  // range -> 300
    Wire.image[0x50] = 0x00;
    Wire.image[0x51] = 0x00;  // ax -> 0 (would change pitch if read)
    bool ok = a.updateMeasurements(Apis::RANGE);
    printf("[per-chip] RANGE ok=%d row(no acquire): %s counts=%u/%u\n", ok, row(a, false),
           a.getDistanceCount(), a.getOrientationCount());
    ok = a.updateMeasurements(Apis::ORIENT);
    printf("[per-chip] ORIENT ok=%d row(no acquire): %s\n", ok, row(a, false));
  }

  // 5b2. Medians and clamping: 7 range readings 300..318 step 3 (median 309), capacity clamp.
  loadImage(300, 90, 100, 50, 1000, 0, 0, 0);
  {
    Apis a;
    a.begin();
    int k = 0;
    onReading = [&](TwoWire& w) {
      int16_t r = 300 + 3 * (k++ % 7);
      w.image[0x48] = r & 0xFF;
      w.image[0x49] = (r >> 8) & 0xFF;
    };
    printf("[median] setDistanceReadings(7)=%u setDistanceReadings(1000)=%u\n", a.setDistanceReadings(7), a.setDistanceReadings(1000));
    a.setDistanceReadings(7);
    a.setOrientationReadings(3);
    a.updateMeasurements();
    printf("[median] count=%u mean=%.4f median=%.4f std=%.4f pitchMedian=%.4f rollMedian=%.4f\n",
           a.getDistanceCount(), a.getDistanceMean(), a.getDistanceMedian(), a.getDistanceStd(), a.getPitchMedian(), a.getRollMedian());
    onReading = nullptr;
  }

  // 5b3. The optional column groups: off by default, and each one appears only
  //      when asked for. Statistics on the new columns follow the orientation
  //      statistics flag, as pitch and roll do.
  loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
  {
    Apis a(1, false, 3, true);
    a.begin();
    printf("[groups] default header: %s\n", head(a));
    a.setAccelerationColumns(true);
    a.setMagnitudeColumns(true);
    a.setTiltColumns(true);
    a.updateMeasurements();
    printf("[groups] all on header:  %s\n", head(a));
    printf("[groups] all on string:  %s\n", row(a, false));
    a.setAccelerationColumns(false);
    printf("[groups] accel off:      %s\n", head(a));
    // The invariant that matters: a header cell for every value, in every
    // combination of the groups. A file whose columns and labels disagree
    // is worse than one with fewer columns.
    for (int mask = 0; mask < 8; mask++) {
      a.setAccelerationColumns(mask & 1);
      a.setMagnitudeColumns(mask & 2);
      a.setTiltColumns(mask & 4);
      const char* h = head(a);
      const char* line = row(a, false);
      int hc = 0, sc = 0;
      for (const char* q = h; *q; q++)
        if (*q == ',') hc++;
      for (const char* q = line; *q; q++)
        if (*q == ',') sc++;
      printf("[groups] mask %d: %d labels, %d values%s\n", mask, hc, sc, hc == sc ? "" : "  MISMATCH");
    }
  }

  // 5c. Handshake: ready/newReading/requestReading against the emulated firmware.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    a.begin();
    printf("[handshake] ready=%d newReading(before any)=%d", a.ready(), a.newReading());
    a.updateDistance();
    printf(" counter after 1 reading=%u newReading=%d", Wire.image[0x42] | (Wire.image[0x43] << 8), a.newReading());
    Wire.image[0x47] = 0x02;  // firmware latched a LiDAR timeout
    a.requestReading(Apis::RANGE);
    printf(" ctrl after request=0x%02X fault after ack=0x%02X\n", Wire.image[0x41], Wire.image[0x47]);
    // timeout path: firmware that never answers (no emulation)
    auto saved = Wire.onWrite;
    Wire.onWrite = nullptr;
    bool ok = a.updateDistance();
    printf("[handshake] no firmware response: updateDistance=%d range=%.2f\n", ok, a.getDistance());
    Wire.onWrite = saved;
  }

  // 5d. Faults: the firmware reports a LiDAR fault in status and a latched code; then a unit reset code.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    a.begin();
    char pb[48];
    onReading = [](TwoWire& w) {
      w.image[0x40] = 0x83;
      w.image[0x47] = 0x02;
    };  // ready | LiDAR fault | pan; LiDAR: timeout
    a.updateDistance();
    BufferPrint bp(pb, sizeof pb);
    a.printReport(bp);
    printf("[faults] faulted(0)=%d faulted(1)=%d any=%d chip=%u kind=%u text='%s'\n",
           a.faulted(0), a.faulted(1), a.anyFault(), a.reportChip(), a.reportKind(), pb);
    onReading = [](TwoWire& w) {
      w.image[0x40] = 0x01;
      w.image[0x47] = 0xE6;
    };  // clean reading; unit: restarted since configured
    a.updateDistance();
    BufferPrint bp2(pb, sizeof pb);
    a.printReport(bp2);
    printf("[faults] any=%d chip=%u kind=%u text='%s'\n", a.anyFault(), a.reportChip(), a.reportKind(), pb);
    printf("[faults] note='%s' (unit reset); beginFailure='%s'\n", note(a), note(a, true));
    onReading = [](TwoWire& w) {
      w.image[0x40] = 0x85;
      w.image[0x47] = 0x21;
    };  // accelerometer: not answering
    a.updateDistance();
    printf("[faults] note='%s' (accel no ack)\n", note(a));
    onReading = [](TwoWire& w) {
      w.image[0x40] = 0x01;
      w.image[0x47] = 0x29;
    };  // clean reading; accelerometer: calibration stored (a notice)
    a.updateDistance();
    printf("[notice] any=%d chip=%u kind=%u note='%s' (calibration stored: no status bit, data stand)\n", a.anyFault(), a.reportChip(), a.reportKind(), note(a));
    onReading = nullptr;
  }
  // 5e. The boot report: begin() reads Block 0 before its first write, so a unit reset latched at boot is seen.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  Wire.image[0x47] = 0xE6;
  {
    Apis a;
    a.begin();
    printf("[boot report] note='%s' after begin()", note(a));
    a.updateDistance();
    printf("; after the first reading: '%s'\n", note(a));
  }
  // 5f. The status line for a logger's status file, after a calibration-stored notice.
  {
    Apis a;
    a.begin();
    char sb[320];
    onReading = [](TwoWire& w) {
      w.image[0x40] = 0x01;
      w.image[0x47] = 0x29;
    };
    a.updateOrientation();
    onReading = nullptr;
    BufferPrint sp(sb, sizeof sb);
    size_t k = a.printStatus(sp);
    printf("[status] %zu bytes: %s\n", k, sb);
  }
  {
    Apis a;
    a.begin();
    onReading = [](TwoWire& w) {
      w.image[0x40] = 0x81;
      w.image[0x47] = 0x51;
    };  // chip 2, kind 17 (device-specific)
    a.updateDistance();
    printf("[faults] note='%s' (chip 2 kind 17)\n", note(a));
    onReading = nullptr;
  }

  // 6. Batches: the readings-requested word reaches the device before the readings.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    a.begin();
    lastRequest = 0;
    a.setDistanceReadings(5);
    a.updateMeasurements(Apis::RANGE);
    printf("[request] updateMeasurements N=5 -> word=%u\n", lastRequest);
    lastRequest = 0;
    a.setDistanceReadings(1);
    a.updateMeasurements(Apis::RANGE);
    printf("[request] updateMeasurements N=1 -> word=%u (no write)\n", lastRequest);
    lastRequest = 0;
    char pb[512];
    BufferPrint bp(pb, sizeof pb);
    int k = 0;
    onReading = [&](TwoWire& w) {
      int16_t r = 240 + 10 * (k++);
      w.image[0x48] = r & 0xFF;
      w.image[0x49] = (r >> 8) & 0xFF;
    };
    a.beginReadings(Apis::RANGE, 4);
    for (int i = 0; i < 4; i++) a.logReading(bp);
    a.endReadings();
    unsigned tx = Wire.transactions;
    printf("[request] beginReadings(RANGE, 4) -> word=%u rows=%s\n", lastRequest, pb);
    printf("[batch stats] count=%u mean=%.2f std=%.4f median=%.2f transactions during stats=%u\n",
           a.getDistanceCount(), a.getDistanceMean(), a.getDistanceStd(), a.getDistanceMedian(), Wire.transactions - tx);
    onReading = nullptr;
  }

  // 7. A LiDAR that will not power up: the firmware reports fault chip 0 kind 1 on the first
  //    reading; the library stops the batch instead of waiting out every reading.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    a.begin();
    int k = 0;
    onReading = [&](TwoWire& w) {
      k++;
      w.image[0x40] = 0x83;
      w.image[0x47] = 0x01;
      w.image[0x48] = 0xF1;
      w.image[0x49] = 0xD8;
    };  // -9999
    a.setDistanceReadings(10);
    bool ok = a.updateMeasurements(Apis::RANGE);
    char fb[64];
    BufferPrint fbp(fb, sizeof fb);
    a.printReport(fbp);
    printf("[dead LiDAR] N=10: ok=%d readings taken=%d range=%.2f fault='%s'\n", ok, k, a.getDistance(), fb);
    onReading = nullptr;
  }

  // 8. begin() gates: wrong name, wrong schema, firmware too old, and the versions it reports.
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  Wire.image[0x01] = 'X';
  {
    Apis a;
    bool ok = a.begin();
    printf("[wrong name] begin=%d failure=%s\n", ok, note(a, true));
  }
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0, 1, 0x00);
  {
    Apis a;
    bool ok = a.begin();
    printf("[schema 0x00] begin=%d fw=%u failure=%s\n", ok, a.getFirmwareVersion(), note(a, true));
  }
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0, 1, 0xFF);
  {
    Apis a;
    bool ok = a.begin();
    printf("[schema 0xFF unprovisioned] begin=%d failure=%s\n", ok, note(a, true));
  }
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0, 0);
  {
    Apis a;
    bool ok = a.begin();
    printf("[fw patch 0 < min %d] begin=%d fw=%u failure=%s\n", APIS_FW_MIN_PATCH, ok, a.getFirmwareVersion(), note(a, true));
  }
  loadImage(250, 120, 0, 0, 1024, 0, 0, 0);
  {
    Apis a;
    bool ok = a.begin();
    printf("[versions] begin=%d hw=%u.%u fw=%u failure=%s\n", ok, a.getHardwareMajor(), a.getHardwareMinor(), a.getFirmwareVersion(), note(a, true));
  }

  // 9. The record of zeros (firmware patch 5): the generation rides with the reading;
  //    zeroChanged() flags the reading after the image's generation moves, once.
  loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
  {
    Apis a;
    a.begin();
    printf("[zero generation] at begin=%u changed=%d", a.getZeroGeneration(), a.zeroChanged());
    a.updateOrientation();
    printf("; after a reading: generation=%u changed=%d", a.getZeroGeneration(), a.zeroChanged());
    setGeneration(Wire.image, 2);  // the unit stored a new zero
    a.updateOrientation();
    printf("; generation moved: generation=%u changed=%d", a.getZeroGeneration(), a.zeroChanged());
    a.updateOrientation();
    printf("; next reading: changed=%d\n", a.zeroChanged());
  }
  // 9b. A begin() against a unit already at generation 3 takes that as its reference.
  loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
  setGeneration(Wire.image, 3);
  {
    Apis a;
    a.begin();
    a.updateOrientation();
    printf("[zero generation] begin at 3: generation=%u changed=%d\n", a.getZeroGeneration(), a.zeroChanged());
  }
  // 9c. dumpZeros(): a full ring, newest first; one zero; none.
  loadImage(1234, 77, 200, -150, 980, 10, -5, 1000);
  {
    const int16_t zeros[3][3] = { { 10, -5, 1000 }, { 12, -4, 998 }, { 9, -6, 1001 } };
    const int8_t temps[3] = { 24, 20, 18 };
    loadZeros(5, zeros, temps);
    Apis a;
    a.begin();
    char zb[96];
    BufferPrint zp(zb, sizeof zb);
    size_t k = a.dumpZeros(zp);
    printf("[dumpZeros] generation 5, %zu bytes:\n%s", k, zb);
    loadZeros(1, zeros, temps);
    BufferPrint zp1(zb, sizeof zb);
    k = a.dumpZeros(zp1);
    printf("[dumpZeros] generation 1, %zu bytes:\n%s", k, zb);
    loadZeros(0, zeros, temps);
    BufferPrint zp0(zb, sizeof zb);
    k = a.dumpZeros(zp0);
    printf("[dumpZeros] generation 0, %zu bytes: '%s'\n", k, zb);
  }

  fprintf(stderr, "bus transactions total: %u\n", Wire.transactions);  // metric, not output
  return 0;
}
