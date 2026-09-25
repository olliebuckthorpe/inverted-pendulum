"""Every PARAM the Mac sends must be one the Pico's firmware handles, the weights must chunk exactly, and the
telemetry layout must match the firmware's."""
import re
import struct

import link
import model
import pico_setup

FW = model.SIM_DIR.parent / "firmware"


def test_every_param_is_handled_by_the_firmware():
    ids_h = {int(v) for v in re.findall(r"=\s*(\d+)", (FW / "core" / "params.h").read_text())}
    sketch = (FW / "pendulum" / "pendulum.ino").read_text()
    names = dict(re.findall(r"(\w+)\s*=\s*(\d+)", (FW / "core" / "params.h").read_text()))
    handled = {int(names[n]) for n in re.findall(r"case pid::(\w+):", sketch)}
    sent = {i for i, _ in pico_setup.param_messages(model.load_params())}
    assert sent <= ids_h, sent - ids_h                      # declared in params.h
    assert sent <= handled, sent - handled                  # and actually applied by set_param


def test_weights_chunk_exactly():
    f = pico_setup.weight_floats("swingupE_seed3")
    msgs = link.weights_messages(f, 0)
    got = []
    for m in msgs:
        net, first, n = struct.unpack("<BHB", m[:4])
        assert net == 0 and len(m) == 4 + 4 * n and len(m) <= 200 and first == len(got)
        got += struct.unpack(f"<{n}f", m[4:])
    assert got == f and len(got) == 4673


def test_telemetry_layout_matches_firmware():
    assert struct.calcsize(link.TELEMETRY_FMT) == 31        # the sketch sends uint8_t t[31]
    assert "uint8_t t[31];" in (FW / "pendulum" / "pendulum.ino").read_text()
