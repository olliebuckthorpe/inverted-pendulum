// PARAM ids (firmware/PROTOCOL.md 0x04): every number the Pico needs, sent by the Mac from sim/params.yaml
// and the trained / designed controllers, so the Pico and the sim share one source of truth.
#pragma once
#include <stdint.h>

namespace pid {
enum : uint16_t {
    // geometry and sensing
    X_PER_COUNT = 1,        // m per motor-encoder count (2π r / 403.2)
    THETA_ZERO = 2,         // AS5047P reading (rad) when the rod is straight UP
    VEL_FILTER_HZ = 3,      // estimator low-pass
    // motor model / supply limiter
    SUPPLY_V = 10, MOTOR_R = 11, MOTOR_KE = 12, PULLEY_R = 13, SUPPLY_CAP_A = 14,
    // classical
    K0 = 20, K1 = 21, K2 = 22, K3 = 23, X_REF = 24,
    POLE_M = 25, POLE_L = 26, POLE_J = 27, M_TOTAL = 28, MOTOR_G = 29, MOTOR_D = 30, F_COULOMB = 31,
    K_ENERGY = 32, A_MAX = 33, K_X = 34, K_V = 35, NUDGE = 36, TARGET_MARGIN = 37,
    CATCH_RAD = 38, CATCH_ENERGY = 39, DROP_RAD = 40,
    // AI
    AI_CATCH_RAD = 50, AI_DROP_RAD = 51, POLICY_HZ = 52, SMOOTH_HZ = 53,
    // safety
    HEARTBEAT_MS = 60, X_SOFT = 61, A_BRAKE = 62, FALL_RAD = 63, DUTY_MAX = 64, SLEW = 65,
    // system identification (SYSID mode): 0 step, 1 chirp, 2 triangle
    SYSID_TYPE = 70, SYSID_AMP = 71, SYSID_F0 = 72, SYSID_F1 = 73, SYSID_SECONDS = 74,
};
}
