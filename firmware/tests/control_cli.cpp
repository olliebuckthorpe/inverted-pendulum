// Host harness for sim/tests/test_control_parity.py: drives the Pico's control.h with sequences from the sim.
//   argv: classical-params.txt swing.bin balance.bin
//   stdin lines:  "E x th"            -> estimator: prints "x xd th thd"
//                 "C x xd th thd"     -> classical: prints "duty mode"
//                 "A x xd th thd"     -> AI (one control cycle): prints "duty mode"
//                 "L duty speed"      -> supply limiter: prints the allowed duty
//                 "R"                 -> reset the controllers (fresh swing-up)
#include <cstdio>
#include <cstring>
#include <vector>
#include "../core/control.h"

static bool load_bin(const char* path, policy::Weights& W) {
    FILE* f = fopen(path, "rb"); if (!f) return false;
    char h[16]; std::vector<float> v(policy::N_FLOATS);
    bool ok = fread(h, 1, 16, f) == 16 && fread(v.data(), 4, v.size(), f) == v.size();
    fclose(f); if (ok) policy::from_floats(W, v.data()); return ok;
}

int main(int argc, char** argv) {
    ctl::Classical C; ctl::Estimator Est; ctl::MotorParams M; ctl::AiController A;
    static policy::Weights Ws, Wb;
    FILE* pf = fopen(argv[1], "r"); char key[64]; float v;
    while (pf && fscanf(pf, "%63s %f", key, &v) == 2) {
        auto& p = C.p;
        if (!strcmp(key, "K0")) p.K[0] = v; else if (!strcmp(key, "K1")) p.K[1] = v; else if (!strcmp(key, "K2")) p.K[2] = v;
        else if (!strcmp(key, "K3")) p.K[3] = v; else if (!strcmp(key, "m")) p.m = v; else if (!strcmp(key, "l")) p.l = v;
        else if (!strcmp(key, "J")) p.J = v; else if (!strcmp(key, "M_total")) p.M_total = v; else if (!strcmp(key, "G")) p.G = v;
        else if (!strcmp(key, "D")) p.D = v; else if (!strcmp(key, "F_coulomb")) p.F_coulomb = v;
        else if (!strcmp(key, "est_dt")) Est.configure(v, 100.0f);
        else if (!strcmp(key, "V")) M.V = v; else if (!strcmp(key, "R")) M.R = v; else if (!strcmp(key, "Ke")) M.Ke = v;
        else if (!strcmp(key, "r")) M.r = v; else if (!strcmp(key, "cap")) M.cap = v;
        else if (!strcmp(key, "ai_catch")) A.catch_rad = v;
    }
    if (pf) fclose(pf);
    if (argc > 3 && load_bin(argv[2], Ws) && load_bin(argv[3], Wb)) { A.swing = &Ws; A.balance = &Wb; }
    char cmd[2]; float a, b, c, d;
    while (scanf("%1s", cmd) == 1) {
        if (cmd[0] == 'E') { scanf("%f %f", &a, &b); auto s = Est.update(a, b); printf("%.9g %.9g %.9g %.9g\n", s.x, s.xd, s.th, s.thd); }
        else if (cmd[0] == 'C') { scanf("%f %f %f %f", &a, &b, &c, &d); float u = C.step({a, b, c, d}); printf("%.9g %d\n", u, C.mode); }
        else if (cmd[0] == 'A') { scanf("%f %f %f %f", &a, &b, &c, &d); float u = A.step({a, b, c, d}); printf("%.9g %d %.9g\n", u, A.mode, A.action); }
        else if (cmd[0] == 'R') { C.mode = ctl::SWING; A.mode = ctl::SWING; A.k = 0; A.action = 0; A.duty = 0; Est.primed = false; printf("ok\n"); }
        else if (cmd[0] == 'L') { scanf("%f %f", &a, &b); printf("%.9g\n", ctl::supply_cap(M, a, b)); }
        fflush(stdout);
    }
}
