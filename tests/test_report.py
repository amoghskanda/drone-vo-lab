import json
import re

import pytest

from drone_vo_lab.benchmark import CASES, MODES
from drone_vo_lab.report import build_report


@pytest.mark.parametrize("modes", [MODES[:2], MODES])
def test_report_supports_legacy_and_three_mode_results(tmp_path, modes):
    rows = [
        {
            "scene": "warehouse",
            "seed": 0,
            "case": case,
            "estimator": mode,
            "position_rmse_m": 0.1,
            "rotation_rmse_deg": 1.0,
            "tracking_coverage": 0.9,
            "median_processing_ms": 2.0,
            "p95_processing_ms": 3.0,
        }
        for case in CASES
        for mode in modes
    ]
    build_report(tmp_path, rows, [], {"video_rendered": False, "cases": CASES})
    html = (tmp_path / "index.html").read_text()
    match = re.search(
        r'<script id="experiment-data" type="application/json">(.*?)</script>', html, re.S
    )
    payload = json.loads(match[1])
    assert {row["estimator"] for row in payload["runs"]} == set(modes)
    assert len({payload["colors"][mode] for mode in modes}) == len(modes)
    assert (tmp_path / "summary.png").is_file()
    assert "__EXPERIMENT_DATA__" not in html
    assert 'id="sensor-replay"' not in html
    assert 'href="demo.mp4"' not in html
