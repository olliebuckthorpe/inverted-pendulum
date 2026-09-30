"""Mac side of the Mac <-> Pico link (firmware/PROTOCOL.md): framing, message packing, and a serial wrapper.
The framing is cross-checked byte-for-byte against the Pico's firmware/core/protocol.h in tests/test_link.py."""
from __future__ import annotations

import struct
import time

from cobs import cobs

HEARTBEAT, PING, SET_MODE, PARAM, WEIGHTS, KICK, HANG = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
TELEMETRY, PONG, ACK, TEXT = 0x81, 0x82, 0x83, 0x84
MODES = {"IDLE": 0, "CLASSICAL": 1, "AI": 2, "LQR_ONLY": 3, "SYSID": 4, "KICK": 5}
TELEMETRY_FMT = "<I5fBHHH"     # t_us, x, x_dot, th, th_dot, duty, mode, faults, exec_us, jitter_us
FAULT_NAMES = ("heartbeat lost", "soft limit", "fell", "end-stop / relay dropped", "E-stop", "sensor error",
               "loop overrun", "rod-1 turn limit", "weights invalid")          # bits 0-8 (latched)
LIVE_NAMES = ("E-stop released (P12)", "left end-stop closed (N1)", "right end-stop closed (N2)",
              "angle read OK")                                                  # bits 12-15 (live levels)


def decode_faults(v: int) -> tuple[list[str], dict[str, bool]]:
    """Split TELEMETRY `faults` into (latched fault names, live input levels). firmware/PROTOCOL.md."""
    return ([n for i, n in enumerate(FAULT_NAMES) if v >> i & 1],
            {n: bool(v >> (12 + i) & 1) for i, n in enumerate(LIVE_NAMES)})


def unpack_telemetry(payload: bytes) -> dict:
    keys = ("t_us", "x", "x_dot", "th", "th_dot", "duty", "mode", "faults", "exec_us", "jitter_us")
    return dict(zip(keys, struct.unpack(TELEMETRY_FMT, payload)))


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    c = 0xFFFF
    for byte in data:
        c ^= byte << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def frame(ptype: int, seq: int, payload: bytes = b"") -> bytes:
    raw = bytes([ptype, seq & 0xFF]) + payload
    raw += struct.pack("<H", crc16(raw))
    return cobs.encode(raw) + b"\x00"


def parse(encoded: bytes) -> tuple[int, int, bytes] | None:
    """One frame without its 0x00. Returns (type, seq, payload) or None if corrupt."""
    try:
        raw = cobs.decode(encoded)
    except cobs.DecodeError:
        return None
    if len(raw) < 4 or crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
        return None
    return raw[0], raw[1], raw[2:-2]


def weights_messages(floats, net: int, chunk: int = 48) -> list[bytes]:
    """Split a policy's 4673 floats into WEIGHTS payloads."""
    return [struct.pack("<BHB", net, i, len(floats[i:i + chunk])) + struct.pack(f"<{len(floats[i:i + chunk])}f", *floats[i:i + chunk])
            for i in range(0, len(floats), chunk)]


class Link:
    """Thin wrapper over a serial port. `send(type, payload)`; `poll()` yields parsed packets."""

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.s = serial.Serial(port, baud, timeout=0)
        self.seq = 0
        self.buf = bytearray()

    def send(self, ptype: int, payload: bytes = b"") -> None:
        self.s.write(frame(ptype, self.seq, payload))
        self.seq = (self.seq + 1) & 0xFF

    def poll(self):
        self.buf += self.s.read(4096)
        while b"\x00" in self.buf:
            enc, _, rest = self.buf.partition(b"\x00")
            self.buf = bytearray(rest)
            if enc and (pkt := parse(bytes(enc))):
                yield pkt

    def heartbeat(self) -> None:
        self.send(HEARTBEAT, struct.pack("<I", int(time.monotonic() * 1000) & 0xFFFFFFFF))
