# Mac ↔ Pico protocol (PLAN 2.5, SPEC §10)

USB CDC serial. Every message is one **packet**, framed with **COBS** and ended by a `0x00` byte (COBS removes
every zero from the packet, so `0x00` can only mean "end of packet"). A corrupted or half-received packet is
dropped when its CRC fails, and the stream re-synchronises at the next `0x00`.

```
packet (before COBS):  [type u8][seq u8][payload …][crc16 u16 LE]
crc16: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, seq and payload
```

All multi-byte fields are little-endian; floats are IEEE-754 float32. `seq` counts per sender (wraps at 255).

| Type | Name | Direction | Payload |
|---|---|---|---|
| 0x01 | HEARTBEAT | Mac → Pico | u32 mac_ms. **No valid heartbeat for 100 ms → motor off** (SPEC §9) |
| 0x02 | PING | Mac → Pico | u32 id, u64 mac_ns (latency test, PLAN 2.5) |
| 0x03 | SET_MODE | Mac → Pico | u8 mode: 0 IDLE, 1 CLASSICAL, 2 AI, 3 LQR_ONLY, 4 SYSID, 5 KICK |
| 0x04 | PARAM | Mac → Pico | u16 id, f32 value (one parameter; ids in `core/params.h`) |
| 0x05 | WEIGHTS | Mac → Pico | u8 net (0 swing-up, 1 balance), u16 first float index, u8 n (≤ 48), f32 × n |
| 0x06 | KICK | Mac → Pico | f32 duty, u16 duration_ms: a cart jolt that **overrides** the controller's duty for that long (SPEC §12) |
| 0x81 | TELEMETRY | Pico → Mac | u32 t_us, f32 x, x_dot, th, th_dot, duty, u8 mode, u16 faults, u16 loop_exec_us, u16 loop_jitter_us |
| 0x82 | PONG | Pico → Mac | the PING payload echoed, + u32 pico_us |
| 0x83 | ACK | Pico → Mac | u8 type, u8 seq, u8 status (0 ok, else an error code) |
| 0x84 | TEXT | Pico → Mac | UTF-8 text (boot message, faults explained) |

Fault bits (TELEMETRY `faults`): 0 heartbeat lost, 1 soft limit, 2 fell, 3 end-stop / relay dropped,
4 E-stop, 5 sensor error (AS5047P parity/error flag), 6 loop overrun, 7 rod-1 turn limit (double), 8 weights invalid.
