"""Frame-to-frame RGB-D odometry. This module has no simulator or truth-pose access."""
"""references:  Lucas and Kanade, 1981: optical-flow tracking via OpenCV and Lepetit et al., IJCV 2009: EPnP camera pose estimation."""

from dataclasses import dataclass
from time import perf_counter

import cv2
import numpy as np

from .geometry import backproject, inverse


@dataclass
class Estimate:
    pose: np.ndarray
    accepted: bool
    tracks: int
    inliers: int
    processing_ms: float
    pixels: np.ndarray
    previous_pixels: np.ndarray
    reason: str


def estimate_motion(points3d: np.ndarray, pixels: np.ndarray, calibration: np.ndarray):
    """Return T_current_previous from previous-camera 3D to current-image 2D."""
    if len(points3d) < 6:
        return None, np.empty(0, dtype=int)
    ok, rotation, translation, inliers = cv2.solvePnPRansac(
        points3d.astype(np.float64),
        pixels.astype(np.float64),
        calibration,
        None,
        iterationsCount=150,
        reprojectionError=2.5,
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < 6:
        return None, np.empty(0, dtype=int)
    indices = inliers.ravel()
    rotation, translation = cv2.solvePnPRefineLM(
        points3d[indices].astype(np.float64),
        pixels[indices].astype(np.float64),
        calibration,
        None,
        rotation,
        translation,
    )
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rotation)[0]
    transform[:3, 3] = translation.ravel()
    return transform, indices


class RGBDOdometry:
    def __init__(self, calibration: np.ndarray, mode: str = "checked"):
        if mode not in {"baseline", "checked"}:
            raise ValueError(f"Unknown estimator: {mode}")
        self.calibration = calibration.copy()
        self.mode = mode
        self.pose = np.eye(4)
        self.previous_gray = None
        self.previous_depth = None

    def update(self, rgb: np.ndarray, depth: np.ndarray) -> Estimate:
        start = perf_counter()
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        empty = np.empty((0, 2), dtype=np.float32)
        if self.previous_gray is None:
            self.previous_gray, self.previous_depth = gray, depth.copy()
            return Estimate(self.pose.copy(), True, 0, 0, 0.0, empty, empty, "initialized")
        previous, current = empty, empty
        indices = np.empty(0, dtype=int)
        transform = None
        reason = "insufficient features"
        points = cv2.goodFeaturesToTrack(
            self.previous_gray, maxCorners=500, qualityLevel=0.01, minDistance=7, blockSize=5
        )
        if points is not None and len(points) >= 6:
            tracked, status, _ = cv2.calcOpticalFlowPyrLK(
                self.previous_gray,
                gray,
                points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            )
            if tracked is not None:
                previous, current = points.reshape(-1, 2), tracked.reshape(-1, 2)
                h, w = gray.shape
                mask = status.ravel().astype(bool) & np.isfinite(current).all(axis=1)
                mask &= (current[:, 0] >= 1) & (current[:, 0] < w - 1)
                mask &= (current[:, 1] >= 1) & (current[:, 1] < h - 1)
                if self.mode == "checked":
                    backward, back_status, _ = cv2.calcOpticalFlowPyrLK(
                        gray,
                        self.previous_gray,
                        tracked,
                        None,
                        winSize=(21, 21),
                        maxLevel=3,
                    )
                    if backward is None:
                        mask[:] = False
                    else:
                        mask &= back_status.ravel().astype(bool)
                        mask &= np.linalg.norm(backward.reshape(-1, 2) - previous, axis=1) < 0.8
                previous, current = previous[mask], current[mask]
                uv = np.rint(previous).astype(int)
                z = self.previous_depth[uv[:, 1], uv[:, 0]]
                valid = np.isfinite(z) & (z > 0.15) & (z < 25.0)
                previous, current, z = previous[valid], current[valid], z[valid]
                points3d = backproject(previous, z, self.calibration)
                transform, indices = estimate_motion(points3d, current, self.calibration)
                reason = "PnP failed"
                # Reject unsupported increments rather than manufacturing a pose.
                if (
                    transform is not None
                    and self.mode == "checked"
                    and (len(indices) < 12 or len(indices) / max(len(current), 1) < 0.35)
                ):
                    transform, reason = None, "low inlier support"
        accepted = transform is not None
        if accepted:
            self.pose = self.pose @ inverse(transform)
            reason = "tracking"
        # Failed increments hold the last pose. No truth-based reset or gap interpolation.
        self.previous_gray, self.previous_depth = gray, depth.copy()
        elapsed = (perf_counter() - start) * 1000
        return Estimate(
            self.pose.copy(),
            accepted,
            len(current),
            len(indices),
            elapsed,
            current[indices] if accepted else current,
            previous[indices] if accepted else previous,
            reason,
        )
