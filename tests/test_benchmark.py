import csv
import json
import re

import numpy as np

from drone_vo_lab import benchmark
from drone_vo_lab.benchmark import CASES, MODES, perturb
from drone_vo_lab.scene import Sequence


def fixture_sequence():
    count = 20
    return Sequence(
        np.arange(count, dtype=np.uint8)[:, None, None, None]
        * np.ones((count, 8, 8, 3), dtype=np.uint8),
        np.arange(count, dtype=np.float32)[:, None, None]
        * np.ones((count, 8, 8), dtype=np.float32),
        np.arange(count) / 15,
        np.repeat(np.eye(4)[None], count, axis=0),
        np.eye(3),
        "warehouse",
        0,
    )


def test_frame_drop_preserves_rgb_depth_timestamp_correspondence():
    sequence = fixture_sequence()
    indices, images = perturb(sequence, "frame_drop", 4)
    np.testing.assert_array_equal(indices, [0, 4, 8, 12, 16])
    np.testing.assert_array_equal(images[:, 0, 0, 0], sequence.depth[indices, 0, 0])
    np.testing.assert_allclose(np.diff(sequence.timestamps[indices]), 4 / 15)


def test_blackout_does_not_modify_source_sequence_or_depth():
    sequence = fixture_sequence()
    original = sequence.rgb.copy()
    indices, images = perturb(sequence, "blackout", 2)
    np.testing.assert_array_equal(sequence.rgb, original)
    assert not images[8:11].any()
    np.testing.assert_array_equal(images[11:], original[11:])
    np.testing.assert_array_equal(sequence.depth[:, 0, 0], np.arange(20))
    np.testing.assert_array_equal(indices, np.arange(20))


def test_benchmark_exports_three_modes_and_recovery_events(monkeypatch, tmp_path):
    sequence = fixture_sequence()
    monkeypatch.setattr(benchmark, "capture", lambda *args: sequence)
    rows = benchmark.run_benchmark(
        tmp_path, ["warehouse"], [0], frames=20, video=False, max_recovery_failures=2
    )
    assert len(rows) == len(CASES) * len(MODES) == 12
    assert {row["estimator"] for row in rows} == set(MODES)
    manifest = json.loads((tmp_path / "results.json").read_text())
    assert manifest["metadata"]["settings"]["max_recovery_failures"] == 2
    assert manifest["metadata"]["settings"]["estimators"] == list(MODES)

    for row in rows:
        assert row["recovered_updates"] == 0
        expected_reanchors = (row["frames"] - 1) // 2 if row["estimator"] == "recovery" else 0
        assert row["reanchors"] == expected_reanchors
        log = tmp_path / "trajectories" / f"warehouse-0-{row['case']}-{row['estimator']}.csv"
        with log.open() as handle:
            observations = list(csv.DictReader(handle))
        assert len(observations) == row["frames"]
        assert (
            sum(obs["reason"].startswith("reanchored") for obs in observations)
            == expected_reanchors
        )

    html = (tmp_path / "index.html").read_text()
    match = re.search(
        r'<script id="experiment-data" type="application/json">(.*?)</script>', html, re.S
    )
    payload = json.loads(match[1])
    assert len(payload["flights"]) == 12
    for flight in payload["flights"]:
        assert len(flight["reasons"]) == len(flight["time"])
    assert not (tmp_path / "demo.mp4").exists()
