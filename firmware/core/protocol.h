// Packet framing for the Mac <-> Pico link (firmware/PROTOCOL.md): COBS + CRC-16/CCITT-FALSE.
// Portable C++ (no Arduino headers) so it is unit-tested on the Mac against sim/link.py.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace proto {

inline uint16_t crc16(const uint8_t* d, size_t n) {          // CRC-16/CCITT-FALSE
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        c ^= uint16_t(d[i]) << 8;
        for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
    }
    return c;
}

// COBS encode: out must hold n + n/254 + 1 bytes. Returns the encoded length (no trailing 0x00).
inline size_t cobs_encode(const uint8_t* in, size_t n, uint8_t* out) {
    size_t code_at = 0, o = 1;
    uint8_t code = 1;
    for (size_t i = 0; i < n; ++i) {
        if (in[i] == 0) { out[code_at] = code; code_at = o++; code = 1; }
        else {
            out[o++] = in[i];
            if (++code == 0xFF) { out[code_at] = code; code_at = o++; code = 1; }
        }
    }
    out[code_at] = code;
    return o;
}

// COBS decode. Returns the decoded length, or 0 on malformed input.
inline size_t cobs_decode(const uint8_t* in, size_t n, uint8_t* out) {
    size_t i = 0, o = 0;
    while (i < n) {
        uint8_t code = in[i++];
        if (code == 0 || i + code - 1 > n) return 0;
        for (uint8_t k = 1; k < code; ++k) out[o++] = in[i++];
        if (code != 0xFF && i < n) out[o++] = 0;
    }
    return o;
}

constexpr size_t MAX_PAYLOAD = 200;
constexpr size_t MAX_FRAME = MAX_PAYLOAD + 4 + (MAX_PAYLOAD + 4) / 254 + 2;

// Build a framed packet (COBS + trailing 0x00). Returns its length.
inline size_t frame(uint8_t type, uint8_t seq, const uint8_t* payload, size_t n, uint8_t* out) {
    uint8_t raw[MAX_PAYLOAD + 4];
    raw[0] = type; raw[1] = seq;
    memcpy(raw + 2, payload, n);
    uint16_t c = crc16(raw, n + 2);
    raw[n + 2] = uint8_t(c & 0xFF); raw[n + 3] = uint8_t(c >> 8);
    size_t m = cobs_encode(raw, n + 4, out);
    out[m] = 0;
    return m + 1;
}

// Parse one COBS-encoded frame (without its 0x00). On success fills type/seq/payload and returns true.
inline bool parse(const uint8_t* enc, size_t n, uint8_t& type, uint8_t& seq, uint8_t* payload, size_t& plen) {
    uint8_t raw[MAX_PAYLOAD + 4];
    if (n > MAX_FRAME) return false;
    size_t m = cobs_decode(enc, n, raw);
    if (m < 4 || m > MAX_PAYLOAD + 4) return false;
    uint16_t want = uint16_t(raw[m - 2]) | (uint16_t(raw[m - 1]) << 8);
    if (crc16(raw, m - 2) != want) return false;
    type = raw[0]; seq = raw[1]; plen = m - 4;
    memcpy(payload, raw + 2, plen);
    return true;
}

// Byte-at-a-time receiver: feed every byte from USB; returns true when a valid packet is complete.
struct Receiver {
    uint8_t buf[MAX_FRAME];
    size_t n = 0;
    bool overflow = false;
    uint8_t type = 0, seq = 0, payload[MAX_PAYLOAD];
    size_t plen = 0;
    uint32_t bad = 0;                                         // frames dropped (CRC / COBS / overflow)
    bool feed(uint8_t b) {
        if (b != 0) {
            if (n < sizeof(buf)) buf[n++] = b; else overflow = true;
            return false;
        }
        bool ok = !overflow && n > 0 && parse(buf, n, type, seq, payload, plen);
        if (!ok && n > 0) ++bad;
        n = 0; overflow = false;
        return ok;
    }
};

}  // namespace proto
