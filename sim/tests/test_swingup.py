"""PLAN 1.5 check: from hanging, swing up and balance within 10 s, cart inside ±0.40 m, in >= 9 of 10 runs."""

import model
import swingup


def test_energy_swingup_9_of_10():
    results = swingup.batch(model.load_params(), n=10)
    assert sum(r["ok"] for r in results) >= 9, [(r["t_catch"], r["max_x"]) for r in results]
