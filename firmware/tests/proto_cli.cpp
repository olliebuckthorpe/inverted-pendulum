// Host helper for sim/tests/test_link.py: frame packets from hex on stdin, and parse frames back.
// "F type seq payloadhex" -> prints the framed bytes as hex;  "P framehex" (without 0x00) -> "type seq payloadhex" or "BAD"
#include <cstdio>
#include <cstring>
#include "../core/protocol.h"

static int hexval(char c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; }
static size_t unhex(const char* s, uint8_t* out) { size_t n = strlen(s) / 2; for (size_t i = 0; i < n; ++i) out[i] = uint8_t(hexval(s[2 * i]) * 16 + hexval(s[2 * i + 1])); return n; }

int main() {
    char cmd[2], a[1024], b[1024], c[1024];
    while (scanf("%1s", cmd) == 1) {
        if (cmd[0] == 'F') {
            unsigned t, q; scanf("%u %u %1023s", &t, &q, c);
            uint8_t pl[proto::MAX_PAYLOAD], out[proto::MAX_FRAME + 2];
            size_t n = strcmp(c, "-") ? unhex(c, pl) : 0;
            size_t m = proto::frame(uint8_t(t), uint8_t(q), pl, n, out);
            for (size_t i = 0; i < m; ++i) printf("%02x", out[i]);
            printf("\n");
        } else {
            scanf("%1023s", a);
            uint8_t enc[proto::MAX_FRAME + 2], pl[proto::MAX_PAYLOAD]; uint8_t t, q; size_t n;
            size_t m = unhex(a, enc);
            if (proto::parse(enc, m, t, q, pl, n)) { printf("%u %u ", t, q); for (size_t i = 0; i < n; ++i) printf("%02x", pl[i]); printf("%s\n", n ? "" : "-"); }
            else printf("BAD\n");
        }
        fflush(stdout);
        (void)b;
    }
}
