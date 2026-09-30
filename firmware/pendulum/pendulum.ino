// Inverted pendulum firmware for the Raspberry Pi Pico 2 (arduino-pico core). PLAN 2.3–2.6, 3.3–3.4, 6.1.
//   core 0: USB link (firmware/PROTOCOL.md): heartbeat, parameters, weights, mode, telemetry out.
//   core 1: the fixed-rate control loop: read sensors → estimate → controller → safety → motor.
// All control logic lives in firmware/core (unit-tested on the Mac against the sim); this file is the glue.
// Pins: hardware/electronics.md. Build: firmware/build.sh (adds firmware/ to the include path).
#include <Arduino.h>
#include <SPI.h>
#include <hardware/pio.h>
#include <pico/mutex.h>
#include "quadrature_encoder.pio.h"
#include "core/control.h"
#include "core/params.h"
#include "core/protocol.h"

// ---- pins (hardware/electronics.md) ----
constexpr int PIN_ENC_A = 2;                 // encoder B on GP3 (PIO needs consecutive pins)
constexpr int PIN_PWM = 14, PIN_DIR = 15;
constexpr int PIN_MISO = 16, PIN_CS1 = 17, PIN_SCK = 18, PIN_MOSI = 19;
constexpr int PIN_SENSE_P12 = 10, PIN_SENSE_N1 = 11, PIN_SENSE_N2 = 12;
constexpr uint32_t PWM_HZ = 20000, PWM_RANGE = 1000;

enum Mode : uint8_t { IDLE = 0, CLASSICAL = 1, AI = 2, LQR_ONLY = 3, SYSID = 4, KICK = 5 };

// ---- state shared between the cores (guarded by `lock`) ----
static mutex_t lock;
struct Shared {
    uint8_t mode = IDLE, requested_mode = IDLE;
    bool reset_request = false;
    bool hang_request = false;                            // PLAN 3.4 watchdog test (HANG, IDLE only)
    uint32_t last_heartbeat_us = 0;
    float kick_duty = 0; uint32_t kick_until_us = 0;
    ctl::ClassicalParams classical;
    ctl::MotorParams motor;
    ctl::SafetyParams safety;
    float x_per_count = 2 * 3.14159265f * 0.00955f / 403.2f, theta_zero = 0, vel_fc = 100;
    float ai_catch = 0.2618f, ai_drop = 0.6109f, policy_hz = 100, smooth_hz = 20;
    int sysid_type = 0; float sysid_amp = 0.25f, sysid_f0 = 0.5f, sysid_f1 = 20, sysid_s = 5;
    uint32_t params_version = 0;
    // telemetry snapshot (written by core 1)
    uint32_t t_us = 0; float x = 0, xd = 0, th = 0, thd = 0, duty = 0; uint16_t faults = 0, exec_us = 0, jitter_us = 0;
    uint32_t tel_seq = 0;
} S;
static policy::Weights weights[2];                       // 0 swing-up, 1 balance
static uint32_t weights_received[2][(policy::N_FLOATS + 31) / 32];
static volatile bool weights_valid[2] = {false, false};

static bool weights_complete(int net) {
    for (int i = 0; i < policy::N_FLOATS; ++i)
        if (!(weights_received[net][i / 32] & (1u << (i % 32)))) return false;
    return true;
}

// ---- AS5047P over SPI (datasheet: mode 1, 16-bit frames, reply arrives with the NEXT frame) ----
static bool even_parity(uint16_t v) { return (__builtin_popcount(v) & 1) == 0; }
static uint16_t spi16(uint16_t out) {
    digitalWrite(PIN_CS1, LOW);
    uint16_t in = SPI.transfer16(out);
    digitalWrite(PIN_CS1, HIGH);
    delayMicroseconds(1);                                 // CSn high >= 350 ns between frames
    return in;
}
static bool read_angle(float& rad) {
    const uint16_t READ_ANGLECOM = 0xFFFF;                // R=1, addr 0x3FFF, even parity -> 0xFFFF
    spi16(READ_ANGLECOM);                                  // this reply belongs to the previous command
    uint16_t r = spi16(READ_ANGLECOM);                     // this one carries a fresh angle
    if (!even_parity(r) || (r & 0x4000)) return false;    // PARD wrong or EF (error flag) set
    rad = float(r & 0x3FFF) * (2.0f * float(M_PI) / 16384.0f);
    return true;
}

// ---- motor (MDD10A channel 1, sign-magnitude: PWM LOW = both outputs LOW = braked; same as the MD13S) ----
static void motor_out(float duty) {
    duty = ctl::clampf(duty, -1.0f, 1.0f);
    digitalWrite(PIN_DIR, duty < 0 ? HIGH : LOW);
    analogWrite(PIN_PWM, int(fabsf(duty) * PWM_RANGE + 0.5f));
}

// ---- system identification: open-loop duty sequences played by the Pico (PLAN 4.1) ----
static float sysid_duty(int type, float amp, float f0, float f1, float T, float t) {
    if (t >= T) return 0.0f;
    if (type == 0) return (fmodf(t, 2.0f) < 1.0f) ? amp : -amp;                           // ±amp steps, 1 s each
    if (type == 1) { float k = (f1 - f0) / T; return amp * sinf(2.0f * float(M_PI) * (f0 * t + 0.5f * k * t * t)); }  // chirp
    float ph = fmodf(t * f0, 1.0f); return amp * (ph < 0.5f ? 4 * ph - 1 : 3 - 4 * ph);    // triangle
}

// =========================================================================================================
// core 0: USB link
// =========================================================================================================
static proto::Receiver rx;
static uint8_t tx_seq = 0;
static void send(uint8_t type, const void* payload, size_t n) {
    uint8_t out[proto::MAX_FRAME + 2];
    size_t m = proto::frame(type, tx_seq++, static_cast<const uint8_t*>(payload), n, out);
    Serial.write(out, m);
}
static void ack(uint8_t type, uint8_t seq, uint8_t status) { uint8_t p[3] = {type, seq, status}; send(0x83, p, 3); }
static uint16_t rd_u16(const uint8_t* p) { uint16_t v; memcpy(&v, p, 2); return v; }   // little-endian on both ends
static float rd_f32(const uint8_t* p) { float v; memcpy(&v, p, 4); return v; }

static void set_param(uint16_t id, float v) {
    auto& c = S.classical; auto& m = S.motor; auto& s = S.safety;
    switch (id) {
        case pid::X_PER_COUNT: S.x_per_count = v; break;   case pid::THETA_ZERO: S.theta_zero = v; break;
        case pid::VEL_FILTER_HZ: S.vel_fc = v; break;
        case pid::SUPPLY_V: m.V = v; break; case pid::MOTOR_R: m.R = v; break; case pid::MOTOR_KE: m.Ke = v; break;
        case pid::PULLEY_R: m.r = v; break; case pid::SUPPLY_CAP_A: m.cap = v; break;
        case pid::K0: c.K[0] = v; break; case pid::K1: c.K[1] = v; break; case pid::K2: c.K[2] = v; break;
        case pid::K3: c.K[3] = v; break; case pid::X_REF: c.x_ref = v; break;
        case pid::POLE_M: c.m = v; break; case pid::POLE_L: c.l = v; break; case pid::POLE_J: c.J = v; break;
        case pid::M_TOTAL: c.M_total = v; break; case pid::MOTOR_G: c.G = v; break; case pid::MOTOR_D: c.D = v; break;
        case pid::F_COULOMB: c.F_coulomb = v; break; case pid::K_ENERGY: c.k_energy = v; break;
        case pid::A_MAX: c.a_max = v; break; case pid::K_X: c.k_x = v; break; case pid::K_V: c.k_v = v; break;
        case pid::NUDGE: c.nudge = v; break; case pid::TARGET_MARGIN: c.target_margin = v; break;
        case pid::CATCH_RAD: c.catch_rad = v; break; case pid::CATCH_ENERGY: c.catch_energy = v; break;
        case pid::DROP_RAD: c.drop_rad = v; break;
        case pid::AI_CATCH_RAD: S.ai_catch = v; break; case pid::AI_DROP_RAD: S.ai_drop = v; break;
        case pid::POLICY_HZ: S.policy_hz = v; break; case pid::SMOOTH_HZ: S.smooth_hz = v; break;
        case pid::HEARTBEAT_MS: s.heartbeat_timeout_us = uint32_t(v * 1000); break;
        case pid::X_SOFT: s.x_soft = v; break; case pid::A_BRAKE: s.a_brake = v; break;
        case pid::FALL_RAD: s.fall_rad = v; break; case pid::DUTY_MAX: s.duty_max = v; break;
        case pid::SLEW: s.slew_per_cycle = v; break;
        case pid::SYSID_TYPE: S.sysid_type = int(v); break; case pid::SYSID_AMP: S.sysid_amp = v; break;
        case pid::SYSID_F0: S.sysid_f0 = v; break; case pid::SYSID_F1: S.sysid_f1 = v; break;
        case pid::SYSID_SECONDS: S.sysid_s = v; break;
    }
    ++S.params_version;
}

static void handle(uint8_t type, uint8_t seq, const uint8_t* p, size_t n) {
    mutex_enter_blocking(&lock);
    uint8_t status = 0;
    switch (type) {
        case 0x01: S.last_heartbeat_us = time_us_32(); break;                                     // HEARTBEAT
        case 0x02: if (n == 12) { uint8_t out[16]; memcpy(out, p, 12); uint32_t t = time_us_32(); memcpy(out + 12, &t, 4);
                                  mutex_exit(&lock); send(0x82, out, 16); return; } status = 1; break; // PING -> PONG
        case 0x03: if (n == 1 && p[0] <= KICK) { S.requested_mode = p[0]; S.reset_request = true; } else status = 2; break;
        case 0x04: if (n == 6 && S.mode == IDLE) set_param(rd_u16(p), rd_f32(p + 2));   // params only when idle
                   else status = 3; break;
        case 0x05: {                                                                                  // WEIGHTS
            uint8_t net = p[0]; uint16_t first = rd_u16(p + 1); uint8_t k = p[3];
            if (n != size_t(4 + 4 * k) || net > 1 || first + k > policy::N_FLOATS || S.mode == AI) { status = 4; break; }
            float* dst = reinterpret_cast<float*>(&weights[net]);
            for (int i = 0; i < k; ++i) { dst[first + i] = rd_f32(p + 4 + 4 * i);
                                          weights_received[net][(first + i) / 32] |= 1u << ((first + i) % 32); }
            weights_valid[net] = weights_complete(net);
            break;
        }
        case 0x06: if (n == 6) { S.kick_duty = rd_f32(p); S.kick_until_us = time_us_32() + rd_u16(p + 4) * 1000u; }
                   else status = 5; break;
        case 0x07: if (n == 0 && S.mode == IDLE) S.hang_request = true; else status = 7; break;   // HANG (test)
        default: status = 6;
    }
    mutex_exit(&lock);
    if (type != 0x01) ack(type, seq, status);
}

void setup() {
    pinMode(PIN_PWM, OUTPUT); digitalWrite(PIN_PWM, LOW);        // motor braked from the first instruction
    pinMode(PIN_DIR, OUTPUT); digitalWrite(PIN_DIR, LOW);
    mutex_init(&lock);
    Serial.begin(115200);
    const char hello[] = "pendulum firmware up: motor braked, mode IDLE";
    send(0x84, hello, sizeof(hello) - 1);
}

void loop() {
    while (Serial.available()) {
        if (rx.feed(uint8_t(Serial.read()))) handle(rx.type, rx.seq, rx.payload, rx.plen);
    }
    static uint32_t sent_seq = 0;
    mutex_enter_blocking(&lock);
    bool fresh = S.tel_seq != sent_seq;
    uint8_t t[31];
    if (fresh) {
        sent_seq = S.tel_seq;
        memcpy(t, &S.t_us, 4); memcpy(t + 4, &S.x, 4); memcpy(t + 8, &S.xd, 4); memcpy(t + 12, &S.th, 4);
        memcpy(t + 16, &S.thd, 4); memcpy(t + 20, &S.duty, 4); t[24] = S.mode;
        memcpy(t + 25, &S.faults, 2); memcpy(t + 27, &S.exec_us, 2); memcpy(t + 29, &S.jitter_us, 2);
    }
    mutex_exit(&lock);
    if (fresh) send(0x81, t, sizeof(t));                               // TELEMETRY (layout: link.TELEMETRY_FMT)
}

// =========================================================================================================
// core 1: fixed-rate control loop (500 Hz)
// =========================================================================================================
constexpr uint32_t PERIOD_US = 2000;
static PIO enc_pio = pio0; static uint enc_sm = 0;

void setup1() {
    pinMode(PIN_SENSE_P12, INPUT); pinMode(PIN_SENSE_N1, INPUT); pinMode(PIN_SENSE_N2, INPUT);
    analogWriteFreq(PWM_HZ); analogWriteRange(PWM_RANGE); analogWrite(PIN_PWM, 0);
    SPI.setRX(PIN_MISO); SPI.setSCK(PIN_SCK); SPI.setTX(PIN_MOSI);
    pinMode(PIN_CS1, OUTPUT); digitalWrite(PIN_CS1, HIGH);
    SPI.begin(false);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));    // 1 MHz to start (datasheet max 10 MHz)
    pio_add_program_at_offset(enc_pio, &quadrature_encoder_program, 0);  // the program is .origin 0
    quadrature_encoder_program_init(enc_pio, enc_sm, PIN_ENC_A, 0);
    rp2040.wdt_begin(50);                                                // hang -> reset -> motor braked on boot
}

void loop1() {
    static ctl::Estimator est; static ctl::Classical classical; static ctl::AiController ai; static ctl::Safety safety;
    static uint32_t next = time_us_32() + PERIOD_US, applied_version = ~0u, mode_start_us = 0;
    static uint8_t mode = IDLE;
    static float angle_prev = 0;

    int32_t lateness = int32_t(time_us_32() - next);
    while (int32_t(time_us_32() - next) < 0) {}                         // wait for the tick (busy: no jitter)
    uint32_t t0 = time_us_32();
    uint16_t jitter = uint16_t(ctl::clampf(float(int32_t(t0 - next)), 0, 65535));
    next += PERIOD_US;
    if (lateness > int32_t(PERIOD_US)) next = t0 + PERIOD_US;           // overran badly: resynchronise

    // ---- take a consistent copy of the configuration ----
    mutex_enter_blocking(&lock);
    if (S.params_version != applied_version) {
        classical.p = S.classical; safety.p = S.safety;
        est.configure(PERIOD_US * 1e-6f, S.vel_fc);
        ai.catch_rad = S.ai_catch; ai.drop_rad = S.ai_drop;
        ai.set_rates(1e6f / PERIOD_US, S.policy_hz, S.smooth_hz);
        applied_version = S.params_version;
    }
    if (S.reset_request) {                                              // mode change: start the controller fresh
        mode = S.requested_mode; S.reset_request = false; mode_start_us = t0;
        classical.mode = (mode == LQR_ONLY) ? ctl::BALANCE : ctl::SWING;
        ai.mode = ctl::SWING; ai.k = 0; ai.duty = 0; ai.action = 0;
        safety.reset();
    }
    ctl::MotorParams motor = S.motor;
    float xpc = S.x_per_count, th0 = S.theta_zero;
    uint32_t hb_age = t0 - S.last_heartbeat_us;
    float kick = S.kick_duty;
    uint32_t S_kick_until = S.kick_until_us;
    int sid_type = S.sysid_type; float sa = S.sysid_amp, sf0 = S.sysid_f0, sf1 = S.sysid_f1, sT = S.sysid_s;
    bool hang = S.hang_request;
    mutex_exit(&lock);
    if (hang) while (true) {}                                           // stop feeding the watchdog: reset in 50 ms

    // ---- sense ----
    float x = float(quadrature_encoder_get_count(enc_pio, enc_sm)) * xpc;
    float angle;
    bool sensor_ok = read_angle(angle);
    if (!sensor_ok) angle = angle_prev;
    angle_prev = angle;
    ctl::State s = est.update(x, ctl::wrap(angle - th0));
    bool p12 = digitalRead(PIN_SENSE_P12), n1 = digitalRead(PIN_SENSE_N1), n2 = digitalRead(PIN_SENSE_N2);
    bool relay_ok = p12 && n1 && n2;

    // ---- decide ----
    float duty = 0; bool balancing = false;
    switch (mode) {
        case CLASSICAL: duty = classical.step(s); balancing = classical.mode == ctl::BALANCE; break;
        case LQR_ONLY:  duty = classical.lqr(s);  balancing = true; break;
        case AI:
            if (weights_valid[0] && weights_valid[1]) { ai.swing = &weights[0]; ai.balance = &weights[1];
                                                        duty = ai.step(s); balancing = ai.mode == ctl::BALANCE; }
            else safety.faults |= ctl::F_WEIGHTS;
            break;
        case SYSID: duty = sysid_duty(sid_type, sa, sf0, sf1, sT, (t0 - mode_start_us) * 1e-6f); break;
        case KICK:  duty = int32_t(S_kick_until - t0) > 0 ? kick : 0.0f; break;
        default: duty = 0;
    }
    // software kick (SPEC §12): a cart jolt that OVERRIDES the controller for its duration (an additive kick is
    // mostly cancelled by the controller pushing back). The controllers keep running, so they resume at once.
    bool kicking = int32_t(S_kick_until - t0) > 0;
    if (kicking && (mode == CLASSICAL || mode == LQR_ONLY || mode == AI)) duty = kick;
    duty = ctl::supply_cap(motor, duty, s.xd);
    if (!p12) safety.faults |= ctl::F_ESTOP;
    float out = (mode == IDLE) ? 0.0f : safety.gate(duty, hb_age, s, balancing, sensor_ok, relay_ok);
    motor_out(out);
    rp2040.wdt_reset();

    // ---- report ----
    uint16_t exec = uint16_t(ctl::clampf(float(time_us_32() - t0), 0, 65535));
    if (exec > PERIOD_US) safety.faults |= ctl::F_OVERRUN;
    mutex_enter_blocking(&lock);
    S.t_us = t0; S.x = s.x; S.xd = s.xd; S.th = s.th; S.thd = s.thd; S.duty = out; S.mode = mode;
    // bits 0-8: latched faults (Safety); bits 12-15: LIVE input levels for bench bring-up and diagnosis, reported
    // in every mode incl. IDLE (never read by Safety): 12 P12 (E-stop released), 13 N1, 14 N2, 15 angle read OK
    S.faults = safety.faults | (p12 << 12) | (n1 << 13) | (n2 << 14) | (sensor_ok << 15);
    S.exec_us = exec; S.jitter_us = jitter; ++S.tel_seq;
    mutex_exit(&lock);
}
