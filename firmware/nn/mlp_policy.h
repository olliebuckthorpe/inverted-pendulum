// Policy network for the Pico (PLAN 6.1): the SAC actor's deterministic path, as plain float maths.
//   obs[6] -> Linear(6,64) -> ReLU -> Linear(64,64) -> ReLU -> Linear(64,1) -> tanh -> duty in [-1, 1]
// Observation order (must match sim/envs/pendulum_env.py): x/0.4, x_dot, sin θ, cos θ, θ_dot/10, duty now.
// Weights come either compiled in (policy_weights.h, from sim/export_policy.py) or uploaded over USB as the
// same floats in the same order (see policy::from_floats). About 4.5k multiply-adds per decision; 4673 floats (18.7 kB).
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace policy {

constexpr int IN = 6, H1 = 64, H2 = 64;
constexpr int N_FLOATS = H1 * IN + H1 + H2 * H1 + H2 + H2 + 1;   // 4673

struct Weights {
    float w1[H1][IN];
    float b1[H1];
    float w2[H2][H1];
    float b2[H2];
    float w3[H2];
    float b3;
};

// Load from a flat float array in the order above (the USB WEIGHTS message carries exactly this).
inline void from_floats(Weights& W, const float* f) { memcpy(&W, f, sizeof(float) * N_FLOATS); }

inline float act(const Weights& W, const float obs[IN]) {
    float h1[H1], h2[H2];
    for (int j = 0; j < H1; ++j) {
        float s = W.b1[j];
        for (int i = 0; i < IN; ++i) s += W.w1[j][i] * obs[i];
        h1[j] = s > 0.0f ? s : 0.0f;
    }
    for (int j = 0; j < H2; ++j) {
        float s = W.b2[j];
        for (int i = 0; i < H1; ++i) s += W.w2[j][i] * h1[i];
        h2[j] = s > 0.0f ? s : 0.0f;
    }
    float y = W.b3;
    for (int i = 0; i < H2; ++i) y += W.w3[i] * h2[i];
    return tanhf(y);
}

}  // namespace policy
