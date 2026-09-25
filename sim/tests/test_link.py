"""The Mac (sim/link.py) and the Pico (firmware/core/protocol.h) must frame packets byte-for-byte identically,
reject corruption, and survive every payload shape (zeros, 0xFF runs, the 254-byte COBS boundary)."""
import random
import shutil
import subprocess

import pytest

import link
import model

FW = model.SIM_DIR.parent / "firmware"
pytestmark = pytest.mark.skipif(not shutil.which("clang++"), reason="needs clang++")


@pytest.fixture(scope="module")
def cli(tmp_path_factory):
    exe = tmp_path_factory.mktemp("p") / "proto_cli"
    subprocess.run(["clang++", "-std=c++17", "-O2", "-Wall", str(FW / "tests" / "proto_cli.cpp"), "-o", str(exe)], check=True)
    p = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    yield p
    p.kill()


def ask(p, line):
    p.stdin.write(line + "\n"); p.stdin.flush()
    return p.stdout.readline().strip()


def payloads():
    rng = random.Random(0)
    yield b""
    yield b"\x00" * 10
    yield b"\xff" * 200
    yield bytes(range(200))
    for n in (1, 2, 196, 199, 200):
        for _ in range(20):
            yield bytes(rng.randrange(256) for _ in range(n))


def test_cpp_and_python_frame_identically(cli):
    for i, pl in enumerate(payloads()):
        t, q = 0x81, i & 0xFF
        py = link.frame(t, q, pl)
        cpp = bytes.fromhex(ask(cli, f"F {t} {q} {pl.hex() or '-'}"))
        assert py == cpp, (pl.hex(), py.hex(), cpp.hex())
        assert py.count(0) == 1 and py.endswith(b"\x00")               # COBS: the only zero is the delimiter
        # each side parses the other's frame back to the original
        assert link.parse(cpp[:-1]) == (t, q, pl)
        got = ask(cli, f"P {py[:-1].hex()}").split()
        assert (int(got[0]), int(got[1]), bytes.fromhex(got[2]) if got[2] != "-" else b"") == (t, q, pl)


def test_corruption_is_rejected(cli):
    rng = random.Random(1)
    rejected = 0
    for _ in range(300):
        pl = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 60)))
        f = bytearray(link.frame(0x04, 7, pl)[:-1])
        f[rng.randrange(len(f))] ^= 1 << rng.randrange(8)                # flip one bit
        if 0 in f:
            continue                                                        # a new 0x00 would split the frame
        assert link.parse(bytes(f)) is None or link.parse(bytes(f))[2] != pl
        rejected += ask(cli, f"P {bytes(f).hex()}") == "BAD"
    assert rejected > 250
