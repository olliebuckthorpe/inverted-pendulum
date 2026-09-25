"""Build and run the Pico's safety unit tests (firmware/tests/test_safety.cpp) on the Mac."""
import shutil
import subprocess

import pytest

import model

FW = model.SIM_DIR.parent / "firmware"


@pytest.mark.skipif(not shutil.which("clang++"), reason="needs clang++")
def test_safety_unit_tests(tmp_path):
    exe = tmp_path / "t"
    subprocess.run(["clang++", "-std=c++17", "-O2", "-Wall", str(FW / "tests" / "test_safety.cpp"), "-o", str(exe)], check=True)
    r = subprocess.run([str(exe)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout
