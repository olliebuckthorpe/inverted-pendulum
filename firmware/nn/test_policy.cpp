// Host test (compiled on the Mac by sim/tests/test_export.py): runs the same mlp_policy.h the Pico will run.
// Usage: test_policy weights.bin obs.csv > out.csv   (weights.bin: 4673 little-endian float32 after a 16-byte header)
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "mlp_policy.h"
// COMPILED_IN builds get POLICY_WEIGHTS from a generated weights header passed with -include

int main(int argc, char** argv) {
    static policy::Weights W;
#ifdef COMPILED_IN
    W = POLICY_WEIGHTS;
    const char* obs_path = argv[1];
#else
    FILE* fw = fopen(argv[1], "rb");
    if (!fw) return 2;
    char header[16];
    if (fread(header, 1, 16, fw) != 16) return 3;
    std::vector<float> f(policy::N_FLOATS);
    if (fread(f.data(), sizeof(float), f.size(), fw) != f.size()) return 4;
    fclose(fw);
    policy::from_floats(W, f.data());
    const char* obs_path = argv[2];
#endif
    FILE* fo = fopen(obs_path, "r");
    if (!fo) return 5;
    float o[policy::IN];
    while (fscanf(fo, "%f,%f,%f,%f,%f,%f", &o[0], &o[1], &o[2], &o[3], &o[4], &o[5]) == 6)
        printf("%.9g\n", policy::act(W, o));
    fclose(fo);
    return 0;
}
