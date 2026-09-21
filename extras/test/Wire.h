// Stub TwoWire serving a 32-byte register image, with the transaction
// semantics Apis.cpp relies on: beginTransmission(adr); write(reg) sets the
// pointer; a second write() is a register write; endTransmission() returns 0
// when the address matches; requestFrom(adr, n) queues n bytes from the
// pointer with auto-increment; read() pops, or returns -1 (0xFF when cast)
// when nothing is queued, as the real Wire does.
#pragma once
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>

class TwoWire {
  public:
    uint8_t image[64] = {0};
    uint8_t deviceAddress = 0x50;
    std::function<void(TwoWire&, uint8_t)> beforeRead;   // hook(wire, pointer): mutate the image before a read
    unsigned transactions = 0;                  // count of requestFrom calls (bus-load metric)

    void begin() {}
    void beginTransmission(uint8_t adr) { _adr = adr; _nwrites = 0; }
    size_t write(uint8_t v) {
        if (_nwrites == 0) _ptr = v; else if (_adr == deviceAddress) image[_ptr++ & 0x3F] = v;
        _nwrites++; return 1;
    }
    uint8_t endTransmission(bool = true) { return _adr == deviceAddress ? 0 : 2; }
    uint8_t requestFrom(uint8_t adr, uint8_t n) {
        transactions++;
        if (adr != deviceAddress) return 0;
        if (beforeRead) beforeRead(*this, _ptr);
        for (uint8_t i = 0; i < n; i++) _q.push_back(image[_ptr++ & 0x3F]);
        return n;
    }
    int read() { if (_q.empty()) return -1; int v = _q.front(); _q.pop_front(); return v; }
    int available() { return (int)_q.size(); }
  private:
    uint8_t _adr = 0, _ptr = 0; unsigned _nwrites = 0; std::deque<uint8_t> _q;
};
extern TwoWire Wire;
