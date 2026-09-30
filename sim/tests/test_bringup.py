import math
import struct

import numpy as np

import bringup
import link


def test_noise_stats_white_noise_and_averaging():
    rng = np.random.default_rng(0)
    th = 1.0 + np.radians(0.068) * rng.standard_normal(5000)
    st = bringup.noise_stats(th)
    assert abs(st["std_deg"] - 0.068) < 0.004
    assert abs(st["std_avg4_deg"] - 0.068 / 2) < 0.004        # white noise falls as 1/sqrt(N)


def test_noise_stats_survives_the_wrap_at_pi():
    rng = np.random.default_rng(1)
    raw = math.pi + np.radians(0.1) * rng.standard_normal(2000)
    th = np.arctan2(np.sin(raw), np.cos(raw))                  # the reading flips between +π and -π
    assert bringup.noise_stats(th)["std_deg"] < 0.12


def test_ruler_error_and_loop_stats():
    assert abs(bringup.ruler_error(0.010, 0.515, 500.0) - 1.0) < 1e-9
    assert bringup.loop_stats([300, 420, 999], [1, 2, 3])["pass"]
    assert not bringup.loop_stats([300, 1000], [1, 2])["pass"]


def test_decode_faults_splits_latched_faults_from_live_inputs():
    faults, live = link.decode_faults((1 << 4) | (1 << 12) | (1 << 15))
    assert faults == ["E-stop"]
    assert live == {"E-stop released (P12)": True, "left end-stop closed (N1)": False,
                    "right end-stop closed (N2)": False, "angle read OK": True}


class FakePico:
    """Plays back TELEMETRY frames through the real framing, and records what the Mac sends."""

    def __init__(self, frames):
        self.pending, self.sent = [link.frame(link.TELEMETRY, i, f) for i, f in enumerate(frames)], []
        self.buf = bytearray()

    def heartbeat(self):
        self.sent.append(link.HEARTBEAT)

    def send(self, ptype, payload=b""):
        self.sent.append(ptype)

    poll = link.Link.poll

    class _S:
        def __init__(self, outer):
            self.o = outer

        def read(self, n):
            return self.o.pending.pop(0) if self.o.pending else b""

    @property
    def s(self):
        return FakePico._S(self)


def test_telemetry_decodes_frames_and_heartbeats():
    tel = struct.pack(link.TELEMETRY_FMT, 123, 0.25, 0.0, math.radians(3.0), 0.0, 0.0, 0,
                      (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15), 410, 2)
    pico = FakePico([tel, tel])
    rows = list(bringup.telemetry(pico, 0.1))
    assert len(rows) == 2 and rows[0]["x"] == 0.25 and rows[0]["exec_us"] == 410
    assert link.HEARTBEAT in pico.sent
    assert "none" in bringup.status_line(rows[0])       # no latched faults, all inputs healthy


def test_zero_from_hanging_makes_hanging_read_180_and_handles_the_wrap():
    for raw_hang in (0.3, 3.1, -3.1, -1.2, math.pi):              # the sensor's raw reading with the rod hanging
        for th0_now in (0.0, 1.0):
            th = [math.atan2(math.sin(raw_hang - th0_now + e), math.cos(raw_hang - th0_now + e)) for e in (-0.001, 0.0, 0.001)]
            new = bringup.zero_from_hanging(np.array(th), th0_now)
            hanging_now = math.atan2(math.sin(raw_hang - new), math.cos(raw_hang - new))
            assert abs(abs(hanging_now) - math.pi) < 1e-6
