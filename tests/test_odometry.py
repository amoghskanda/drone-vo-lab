import cv2
import numpy as np
import pytest

from drone_vo_lab.geometry import intrinsics, inverse
from drone_vo_lab.odometry import RGBDOdometry, estimate_motion


@pytest.mark.parametrize("mode", ["baseline", "checked", "recovery"])
def test_reference_handling_after_rejected_update(mode):
    estimator = RGBDOdometry(intrinsics(160, 120, 65), mode=mode)

    # A is uniformly gray, so it contains no trackable corners
    image_a = np.full((120, 160, 3), 80, dtype=np.uint8)
    depth_a = np.full((120, 160), 3.0, dtype=np.float32)

    estimator.update(image_a, depth_a)
    pose_before = estimator.pose.copy()

    # different image and depth make reference replacement observable
    image_b = np.zeros_like(image_a)
    depth_b = np.full_like(depth_a, 7.0)

    result = estimator.update(image_b, depth_b)

    assert not result.accepted
    np.testing.assert_array_equal(result.pose, pose_before)

    if mode == "recovery":
        expected_image, expected_depth = image_a, depth_a
    else:
        expected_image, expected_depth = image_b, depth_b

    np.testing.assert_array_equal(
        estimator.previous_gray,
        cv2.cvtColor(expected_image, cv2.COLOR_RGB2GRAY),
    )
    np.testing.assert_array_equal(estimator.previous_depth, expected_depth)


def test_recovery_estimates_motion_across_blackout():
    cv2.setRNGSeed(0)
    rng = np.random.default_rng(42)

    image_a = rng.integers(0, 256, size=(120, 160, 3), dtype=np.uint8)
    image_a = cv2.GaussianBlur(image_a, (3, 3), 0)

    calibration = intrinsics(160, 120, 65)
    depth = np.full((120, 160), 3.0, dtype=np.float32)

    def shifted_image(pixels):
        transform = np.float32([[1, 0, pixels], [0, 1, 0]])
        return cv2.warpAffine(image_a, transform, (160, 120))

    estimator = RGBDOdometry(calibration, mode="recovery", max_recovery_failures=2)
    estimator.update(image_a, depth)

    # check non-zero pose before losing tracking
    before_gap = estimator.update(shifted_image(4), depth)
    assert before_gap.accepted

    # the blackout must hold that pose
    failed = estimator.update(np.zeros_like(image_a), depth)
    assert not failed.accepted
    assert estimator.recovery_failures == 1
    np.testing.assert_allclose(failed.pose, before_gap.pose)

    # recover with the reference
    recovered = estimator.update(shifted_image(12), depth)
    assert recovered.accepted
    assert recovered.reason == "recovered"
    assert estimator.recovery_failures == 0

    expected_pose = np.eye(4)
    expected_pose[0, 3] = -12 * 3.0 / calibration[0, 0]

    np.testing.assert_allclose(recovered.pose, expected_pose, atol=0.01)

    # A fresh failure must start a new window, not trigger premature expiry.
    failed_again = estimator.update(np.zeros_like(image_a), depth)
    assert not failed_again.accepted
    assert estimator.recovery_failures == 1
    assert not failed_again.reason.startswith("reanchored")
    np.testing.assert_array_equal(
        estimator.previous_gray,
        cv2.cvtColor(shifted_image(12), cv2.COLOR_RGB2GRAY),
    )


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


def test_recovery_reanchors_on_failure_limit_without_changing_pose():
    estimator = RGBDOdometry(intrinsics(160, 120, 65), mode="recovery", max_recovery_failures=2)
    reference = np.full((120, 160, 3), 80, dtype=np.uint8)
    depth = np.full((120, 160), 3.0, dtype=np.float32)
    estimator.update(reference, depth)
    # Represent an already accumulated pose, which expiry must preserve.
    estimator.pose[:3, 3] = [1.0, -2.0, 0.5]
    held_pose = estimator.pose.copy()

    first_failure = estimator.update(np.zeros_like(reference), depth)
    assert not first_failure.accepted
    assert estimator.recovery_failures == 1
    np.testing.assert_array_equal(estimator.previous_gray, reference[:, :, 0])

    new_reference = np.full_like(reference, 20)
    new_depth = np.full_like(depth, 7.0)
    expired = estimator.update(new_reference, new_depth)
    assert not expired.accepted
    assert expired.reason == "reanchored after recovery timeout: insufficient features"
    assert estimator.recovery_failures == 0
    np.testing.assert_array_equal(expired.pose, held_pose)
    np.testing.assert_array_equal(first_failure.pose, held_pose)
    np.testing.assert_array_equal(estimator.previous_gray, new_reference[:, :, 0])
    np.testing.assert_array_equal(estimator.previous_depth, new_depth)

    # Expiry starts another bounded attempt window from the replacement frame.
    next_failure = estimator.update(np.zeros_like(reference), depth)
    assert not next_failure.accepted
    assert estimator.recovery_failures == 1
    assert not next_failure.reason.startswith("reanchored")
    np.testing.assert_array_equal(estimator.previous_gray, new_reference[:, :, 0])


@pytest.mark.parametrize("limit", [0, -1, 1.5, True])
def test_recovery_limit_requires_a_positive_integer(limit):
    with pytest.raises(ValueError, match="positive integer"):
        RGBDOdometry(intrinsics(160, 120, 65), mode="recovery", max_recovery_failures=limit)
