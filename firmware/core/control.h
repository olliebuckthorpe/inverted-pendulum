// The Pico's per-cycle control logic, as portable C++ (no Arduino headers), mirroring the Python sim that the
// results came from:
//   Estimator      ↔ sim/plant.py Estimator        (quantised readings → differenced, low-passed velocities)
//   supply_cap     ↔ sim/motor.py cap_duty_for_supply
//   Classical      ↔ sim/swingup.py EnergySwingUp  (energy pumping → LQR, same hand-over rule)
//   AiController   ↔ sim/ai_handover.py HandOver + the env's action smoothing (sim/envs/pendulum_env.py)
//   Safety         ↔ SPEC §9 / PLAN 3.4 (heartbeat, speed-aware soft limit, fall, clamp + slew)
// Every number the controllers need arrives from the Mac as PARAMs (computed from sim/params.yaml), so the
// Pico and the sim share one source of truth. Parity with the Python is tested in sim/tests/test_control_parity.py.
#pragma once
#include <math.h>
#include <stdint.h>
#include "../nn/mlp_policy.h"

namespace ctl {

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrap(float a) { return atan2f(sinf(a), cosf(a)); }
inline float signf(float v) { return (v > 0) - (v < 0); }

struct State { float x, xd, th, thd; };           // th: 0 = upright, wrapped to (-pi, pi]

// ---------------------------------------------------------------------------------------------------------
// Estimator: velocities by differencing the (already quantised) readings, then a first-order low-pass.
struct Estimator {
    float dt = 0.002f, a = 0.5f;                   // a = dt / (dt + 1/(2π fc))
    bool primed = false;
    float px = 0, pth = 0, vx = 0, vth = 0;
    void configure(float dt_, float fc_hz) { dt = dt_; a = dt_ / (dt_ + 1.0f / (2.0f * float(M_PI) * fc_hz)); primed = false; }
    State update(float x, float th) {
        if (!primed) { px = x; pth = th; primed = true; }
        float rx = (x - px) / dt, rth = wrap(th - pth) / dt;
        vx += a * (rx - vx); vth += a * (rth - vth);
        px = x; pth = th;
        return {x, vx, wrap(th), vth};
    }
};

// ---------------------------------------------------------------------------------------------------------
// Supply-current limiter: shrink |duty| until duty·(V·duty − Ke·ω)/R ≤ cap (roots of V d² − Ke ω d − R cap = 0).
struct MotorParams { float V = 12.0f, R = 2.18f, Ke = 0.069f, r = 0.00955f, cap = 4.5f; };
inline float supply_cap(const MotorParams& m, float duty, float cart_speed) {
    if (m.cap <= 0) return duty;
    float b = m.Ke * cart_speed / m.r;
    float disc = sqrtf(b * b + 4.0f * m.V * m.R * m.cap);
    return clampf(duty, (b - disc) / (2.0f * m.V), (b + disc) / (2.0f * m.V));
}

// ---------------------------------------------------------------------------------------------------------
// Classical: energy-shaping swing-up, hand-over to LQR (u = −K s) near the top, hand back if it falls.
struct ClassicalParams {
    float K[4] = {0, 0, 0, 0};                     // LQR gain over [x, x_dot, th, th_dot]
    float x_ref = 0;
    float m = 0.02f, l = 0.15f, J = 6e-4f, g = 9.81f; // pole: mass, pivot→CoM, inertia about the pivot
    float M_total = 1.76f, G = 30.8f, D = 19.1f, F_coulomb = 2.1f;  // motor-model inverse
    float k_energy = 60, a_max = 6, k_x = 8, k_v = 4, nudge = 0.4f, target_margin = 0.10f;
    float catch_rad = 0.349f, catch_energy = 0.25f, drop_rad = 0.611f;
};
enum Mode : uint8_t { SWING = 2, BALANCE = 1 };

struct Classical {
    ClassicalParams p;
    uint8_t mode = SWING;
    float energy(float th, float thd) const { return 0.5f * p.J * thd * thd + p.m * p.g * p.l * (cosf(th) - 1.0f); }
    float lqr(const State& s) const {
        return -(p.K[0] * (s.x - p.x_ref) + p.K[1] * s.xd + p.K[2] * s.th + p.K[3] * s.thd);
    }
    float step(const State& s) {
        float E = energy(s.th, s.thd), Eb = -2.0f * p.m * p.g * p.l;
        bool near_top = fabsf(s.th) < p.catch_rad;
        if (mode == SWING && near_top && fabsf(E) < p.catch_energy * fabsf(Eb)) mode = BALANCE;
        else if (mode == BALANCE && fabsf(s.th) > p.drop_rad) mode = SWING;
        if (mode == BALANCE) return lqr(s);
        float sv = s.thd * cosf(s.th);
        if (fabsf(sv) < 1e-3f && E < 0.9f * Eb) sv = p.nudge;
        float Eerr = E - p.target_margin * fabsf(Eb);
        float a = clampf(p.k_energy * Eerr * sv, -p.a_max, p.a_max) - p.k_x * s.x - p.k_v * s.xd;
        float force = p.M_total * a + p.D * s.xd + p.F_coulomb * signf(s.xd);
        return force / p.G;
    }
};

// ---------------------------------------------------------------------------------------------------------
// AI: swing-up policy → hand-over → balance policy, deciding every `decimate` control cycles; the chosen action
// is smoothed into the duty every cycle (the env's first-order filter, alpha per control cycle).
struct AiController {
    const policy::Weights* swing = nullptr;
    const policy::Weights* balance = nullptr;
    float catch_rad = 0.2618f, drop_rad = 0.6109f;   // hand over at 15°, hand back past 35° (select_ai.py)
    int decimate = 5;                                 // 500 Hz loop / 100 Hz policy
    float alpha = smoothing(500.0f, 20.0f);           // the env's first-order filter, per control cycle
    // Same formula as sim/envs/pendulum_env.py: alpha = 1 / (1 + ctrl_hz / (2π · smooth_hz))
    static float smoothing(float ctrl_hz, float smooth_hz) { return 1.0f / (1.0f + ctrl_hz / (2.0f * float(M_PI) * smooth_hz)); }
    void set_rates(float ctrl_hz, float policy_hz, float smooth_hz) {
        decimate = int(ctrl_hz / policy_hz + 0.5f);
        alpha = smoothing(ctrl_hz, smooth_hz);
    }
    uint8_t mode = SWING;
    int k = 0;
    float action = 0, duty = 0;
    float step(const State& s) {
        if (k % decimate == 0) {
            float obs[6] = {s.x / 0.4f, s.xd, sinf(s.th), cosf(s.th), s.thd / 10.0f, duty};
            float th = atan2f(obs[2], obs[3]);
            if (mode == SWING && fabsf(th) < catch_rad) mode = BALANCE;
            else if (mode == BALANCE && fabsf(th) > drop_rad) mode = SWING;
            action = policy::act(mode == BALANCE ? *balance : *swing, obs);
        }
        ++k;
        duty += alpha * (action - duty);
        return duty;
    }
};

// ---------------------------------------------------------------------------------------------------------
// Safety (on the Pico, never the Mac). Returns the duty actually allowed and sets fault bits.
enum Fault : uint16_t { F_HEARTBEAT = 1 << 0, F_SOFT_LIMIT = 1 << 1, F_FELL = 1 << 2, F_ENDSTOP = 1 << 3,
                        F_ESTOP = 1 << 4, F_SENSOR = 1 << 5, F_OVERRUN = 1 << 6, F_TURNS = 1 << 7, F_WEIGHTS = 1 << 8 };
struct SafetyParams {
    uint32_t heartbeat_timeout_us = 100000;         // WPILib convention, SPEC §9
    float x_soft = 0.33f;                             // soft travel limit (end-stops trip at ~±0.348)
    float a_brake = 8.0f;                             // assumed braking deceleration for the speed-aware limit
    float fall_rad = 0.7f;                            // balance modes: beyond this the rod is lost
    float duty_max = 1.0f, slew_per_cycle = 0.25f;    // clamp + max change per 2 ms cycle: 0.25 leaves both
                                                      // controllers unaffected in sim (classical 10/10, AI 97/100)
};
struct Safety {
    SafetyParams p;
    uint16_t faults = 0;
    float last = 0;
    // balancing: true in modes that must keep the rod near upright (LQR_ONLY / after hand-over)
    float gate(float duty, uint32_t heartbeat_age_us, const State& s, bool balancing, bool sensor_ok, bool relay_ok) {
        if (heartbeat_age_us > p.heartbeat_timeout_us) faults |= F_HEARTBEAT;
        if (!sensor_ok) faults |= F_SENSOR;
        if (!relay_ok) faults |= F_ENDSTOP;
        if (balancing && fabsf(s.th) > p.fall_rad) faults |= F_FELL;
        // speed-aware soft limit: would the cart pass x_soft even braking hard from here?
        float stop_at = s.x + signf(s.xd) * s.xd * s.xd / (2.0f * p.a_brake);
        if (fabsf(s.x) > p.x_soft || fabsf(stop_at) > p.x_soft) faults |= F_SOFT_LIMIT;
        float out = faults ? 0.0f : clampf(duty, -p.duty_max, p.duty_max);   // duty 0 on the MD13S = motor braked
        out = faults ? 0.0f : clampf(out, last - p.slew_per_cycle, last + p.slew_per_cycle);
        last = out;
        return out;
    }
    void reset() { faults = 0; last = 0; }            // only on an explicit command from the Mac, motor idle
};

}  // namespace ctl
