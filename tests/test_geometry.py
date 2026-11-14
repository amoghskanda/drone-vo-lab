import numpy as np

from drone_vo_lab.geometry import backproject, evaluate, intrinsics, inverse, look_at


def test_backprojection_has_metric_axial_depth():
    k = intrinsics(320, 240, 65)
    pixels = np.array([[20.0, 80.0], [159.5, 119.5], [270.0, 190.0]])
    depths = np.array([2.0, 3.0, 4.0])
    points = backproject(pixels, depths, k)
    projected = points @ k.T
    np.testing.assert_allclose(projected[:, :2] / projected[:, 2:], pixels)
    np.testing.assert_allclose(points[:, 2], depths)


def test_look_at_camera_axes_and_rigid_inverse():
    pose = look_at(np.array([1.0, 2.0, 3.0]), np.array([1.0, 3.0, 3.0]))
    np.testing.assert_allclose(pose[:3, 0], [1, 0, 0])
    np.testing.assert_allclose(pose[:3, 1], [0, 0, -1])
    np.testing.assert_allclose(pose[:3, 2], [0, 1, 0])
    np.testing.assert_allclose(pose @ inverse(pose), np.eye(4), atol=1e-12)
    assert np.linalg.det(pose[:3, :3]) > 0.9999


def test_metrics_align_initial_frame_without_fitting_away_drift():
    start = look_at(np.array([4.0, 2.0, 1.0]), np.array([4.0, 3.0, 1.0]))
    relative = np.repeat(np.eye(4)[None], 3, axis=0)
    relative[:, 0, 3] = [0, 1, 2]
    truth = start[None] @ relative
    accepted = np.array([True, True, False])
    perfect = evaluate(relative, truth, accepted)
    assert perfect["position_rmse_m"] < 1e-12
    held = relative.copy()
    held[-1] = held[-2]
    metrics = evaluate(held, truth, accepted)
    np.testing.assert_allclose(metrics["position_rmse_m"], np.sqrt(1 / 3))
    assert metrics["failed_updates"] == 1
    assert metrics["tracking_coverage"] == 0.5
