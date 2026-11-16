import os

import cv2
import numpy as np
import pytest

from drone_vo_lab.geometry import evaluate
from drone_vo_lab.odometry import RGBDOdometry
from drone_vo_lab.scene import capture


@pytest.mark.render
@pytest.mark.skipif(os.environ.get("RUN_RENDER_TESTS") != "1", reason="Set RUN_RENDER_TESTS=1")
@pytest.mark.parametrize("mode", ["baseline", "checked", "recovery"])
def test_rendered_sequence_has_consistent_metric_depth_and_camera_conventions(mode):
    # Short inter-frame motion exercises RGB tracking, depth, PnP, and pose composition.
    sequence = capture("warehouse", 0, 90, 15, 320, 240)
    cv2.setNumThreads(1)
    estimator = RGBDOdometry(sequence.calibration, mode=mode)
    estimates = [
        estimator.update(rgb, depth)
        for rgb, depth in zip(sequence.rgb, sequence.depth, strict=True)
    ]
    metrics = evaluate(
        np.array([e.pose for e in estimates]),
        sequence.poses,
        np.array([e.accepted for e in estimates]),
    )
    assert metrics["tracking_coverage"] > 0.95
    assert metrics["position_rmse_m"] < 0.25
