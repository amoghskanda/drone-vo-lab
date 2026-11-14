import cv2
import numpy as np

from drone_vo_lab.geometry import intrinsics, inverse
from drone_vo_lab.odometry import RGBDOdometry, estimate_motion


def test_pnp_recovers_known_six_dof_increment_and_rejects_outliers():
    rng = np.random.default_rng(7)
    points = rng.uniform([-1, -0.7, 2], [1, 0.7, 6], size=(120, 3))
    rotation = np.array([0.02, -0.03, 0.01])
    translation = np.array([0.12, -0.04, 0.08])
    k = intrinsics(320, 240, 65)
    pixels = cv2.projectPoints(points, rotation, translation, k, None)[0].reshape(-1, 2)
    pixels[:20] = rng.uniform([0, 0], [320, 240], size=(20, 2))
    cv2.setRNGSeed(0)
    transform, inliers = estimate_motion(points, pixels, k)
    assert transform is not None
    np.testing.assert_allclose(transform[:3, :3], cv2.Rodrigues(rotation)[0], atol=1e-5)
    np.testing.assert_allclose(transform[:3, 3], translation, atol=1e-5)
    assert len(inliers) >= 95
    # The returned motion maps previous-camera points into the current frame.
    np.testing.assert_allclose(inverse(transform) @ transform, np.eye(4), atol=1e-12)


def test_black_frames_hold_pose_and_explicitly_report_failure():
    estimator = RGBDOdometry(intrinsics(160, 120, 65))
    black = np.zeros((120, 160, 3), np.uint8)
    depth = np.full((120, 160), 3.0, np.float32)
    assert estimator.update(black, depth).accepted
    failed = estimator.update(black, depth)
    assert not failed.accepted
    assert failed.reason == "insufficient features"
    np.testing.assert_allclose(failed.pose, np.eye(4))
