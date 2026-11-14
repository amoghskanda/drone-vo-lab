"""Rigid transforms use T_a_b: points in frame b transform into frame a.

Images and the estimator use OpenCV camera axes: right, down, forward.
MuJoCo cameras use right, up, backward. Conversion is a 180-degree X rotation.
"""

import numpy as np

CV_TO_GL = np.diag([1.0, -1.0, -1.0])


def intrinsics(width: int, height: int, fovy: float) -> np.ndarray:
    focal = 0.5 * height / np.tan(np.deg2rad(fovy) / 2)
    return np.array([[focal, 0, (width - 1) / 2], [0, focal, (height - 1) / 2], [0, 0, 1]])


def inverse(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def look_at(position: np.ndarray, target: np.ndarray, roll: float = 0.0) -> np.ndarray:
    forward = np.asarray(target, dtype=float) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.column_stack([right, down, forward])
    c, s = np.cos(roll), np.sin(roll)
    rotation = rotation @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = position
    return pose


def backproject(pixels: np.ndarray, depth: np.ndarray, calibration: np.ndarray) -> np.ndarray:
    """Backproject axial depth in meters, not Euclidean ray distance."""
    rays = np.column_stack([pixels, np.ones(len(pixels))]) @ np.linalg.inv(calibration).T
    return rays * depth[:, None]


def evaluate(estimated: np.ndarray, truth: np.ndarray, accepted: np.ndarray) -> dict:
    """Align the initial coordinate frame only. Keep held poses in the error metrics."""
    relative_truth = inverse(truth[0])[None] @ truth
    errors = np.linalg.norm(estimated[:, :3, 3] - relative_truth[:, :3, 3], axis=1)
    length = float(np.linalg.norm(np.diff(truth[:, :3, 3], axis=0), axis=1).sum())
    rotations = np.swapaxes(relative_truth[:, :3, :3], 1, 2) @ estimated[:, :3, :3]
    angles = np.rad2deg(np.arccos(np.clip((np.trace(rotations, axis1=1, axis2=2) - 1) / 2, -1, 1)))
    return {
        "position_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "rotation_rmse_deg": float(np.sqrt(np.mean(angles**2))),
        "final_error_m": float(errors[-1]),
        "path_length_m": length,
        "tracking_coverage": float(np.mean(accepted[1:])) if len(accepted) > 1 else 0.0,
        "failed_updates": int(np.count_nonzero(~accepted[1:])),
        "frames": len(estimated),
    }
