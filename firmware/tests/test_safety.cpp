// Unit tests for ctl::Safety (firmware/core/control.h): run by sim/tests/test_safety.py. Exit code = failures.
#include <cmath>
#include <cstdio>
#include "../core/control.h"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); ++fails; } } while (0)

int main() {
    using namespace ctl;
    const State still{0, 0, 0, 0};
    {   // normal: passes the command through, within clamp and slew
        Safety S; float u = 0;
        for (int i = 0; i < 20; ++i) u = S.gate(1.0f, 1000, still, true, true, true);
        CHECK(fabsf(u - 1.0f) < 1e-6f);                      // reached full duty after ramping
        CHECK(S.faults == 0);
    }
    {   // slew: one cycle can only move 0.25
        Safety S; float u = S.gate(1.0f, 0, still, true, true, true);
        CHECK(fabsf(u - 0.25f) < 1e-6f);
        u = S.gate(-1.0f, 0, still, true, true, true);
        CHECK(fabsf(u - 0.0f) < 1e-6f);
    }
    {   // clamp
        Safety S; S.p.slew_per_cycle = 10; float u = S.gate(3.0f, 0, still, true, true, true);
        CHECK(fabsf(u - 1.0f) < 1e-6f);
    }
    {   // heartbeat lost (>100 ms): motor off, fault latched even after heartbeats resume
        Safety S; S.gate(0.5f, 0, still, true, true, true);
        float u = S.gate(0.5f, 100001, still, true, true, true);
        CHECK(u == 0.0f && (S.faults & F_HEARTBEAT));
        u = S.gate(0.5f, 0, still, true, true, true);
        CHECK(u == 0.0f);                                     // latched until an explicit reset
        S.reset(); u = S.gate(0.2f, 0, still, true, true, true);
        CHECK(fabsf(u - 0.2f) < 1e-6f && S.faults == 0);
    }
    {   // exactly at the timeout is still OK
        Safety S; float u = S.gate(0.2f, 100000, still, true, true, true);
        CHECK(fabsf(u - 0.2f) < 1e-6f);
    }
    {   // fall detection only while balancing
        Safety S; State tilted{0, 0, 0.8f, 0};
        CHECK(S.gate(0.2f, 0, tilted, false, true, true) != 0.0f);   // swinging up: allowed
        CHECK(S.gate(0.2f, 0, tilted, true, true, true) == 0.0f && (S.faults & F_FELL));
    }
    {   // speed-aware soft limit: inside the limit but too fast outward to stop in time
        Safety S; State fast{0.25f, 1.5f, 0, 0};            // stop at 0.25 + 1.5²/16 = 0.39 > 0.33
        CHECK(S.gate(0.5f, 0, fast, true, true, true) == 0.0f && (S.faults & F_SOFT_LIMIT));
        Safety T; State inward{0.25f, -1.5f, 0, 0};         // same place, moving inward: fine
        CHECK(T.gate(0.2f, 0, inward, true, true, true) != 0.0f);
        Safety U; State out{-0.34f, 0, 0, 0};               // past the limit, still: off
        CHECK(U.gate(0.2f, 0, out, true, true, true) == 0.0f);
    }
    {   // sensor error and relay dropped
        Safety S; CHECK(S.gate(0.2f, 0, still, true, false, true) == 0.0f && (S.faults & F_SENSOR));
        Safety T; CHECK(T.gate(0.2f, 0, still, true, true, false) == 0.0f && (T.faults & F_ENDSTOP));
    }
    printf("%d failure(s)\n", fails);
    return fails;
}
