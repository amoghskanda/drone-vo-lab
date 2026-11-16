"""Paired sensor corruptions, estimator runs, and auditable per-frame outputs."""

import csv
import json
import platform
from pathlib import Path

import cv2
import mujoco
import numpy as np

from .geometry import evaluate
from .odometry import RGBDOdometry
from .scene import Sequence, capture

CASES = {
    "clean": "Unmodified synchronized RGB-D",
    "motion_blur": "21-pixel horizontal RGB box blur; depth unchanged",
    "frame_drop": "Keep every nth synchronized RGB-D frame, preserving timestamps",
    "blackout": "Black RGB frames from 40 to 55% of the flight; depth unchanged",
}
MODES = ("baseline", "checked", "recovery")


def perturb(sequence: Sequence, case: str, frame_interval):
    if case not in CASES:
        raise ValueError(f"Unknown corruption: {case}")
    indices = np.arange(len(sequence.rgb))
    # customized to take every nth frame.
    if case == "frame_drop" and frame_interval is not None:
        indices = indices[:: int(frame_interval)]
    images = sequence.rgb[indices].copy()
    if case == "motion_blur":
        kernel = np.zeros((21, 21), dtype=np.float32)
        kernel[10] = 1.0 / 21
        images = np.array([cv2.filter2D(image, -1, kernel) for image in images])
    elif case == "blackout":
        images[int(len(images) * 0.40) : int(len(images) * 0.55)] = 0
    return indices, images


def run_sequence(
    sequence: Sequence,
    case: str,
    mode: str,
    frame_interval: int,
    max_recovery_failures: int = 30,
) -> tuple[dict, list, np.ndarray]:
    indices, images = perturb(sequence, case, frame_interval=frame_interval)
    cv2.setRNGSeed(0)
    estimator = RGBDOdometry(
        sequence.calibration, mode, max_recovery_failures=max_recovery_failures
    )
    estimates = [
        estimator.update(image, sequence.depth[index])
        for image, index in zip(images, indices, strict=True)
    ]
    poses = np.array([estimate.pose for estimate in estimates])
    accepted = np.array([estimate.accepted for estimate in estimates])
    metrics = evaluate(poses, sequence.poses[indices], accepted)
    latency = [estimate.processing_ms for estimate in estimates[1:]]
    metrics.update(
        {
            "scene": sequence.scene,
            "seed": sequence.seed,
            "case": case,
            "estimator": mode,
            "recovered_updates": sum(e.reason == "recovered" for e in estimates),
            "reanchors": sum(e.reason.startswith("reanchored") for e in estimates),
            "median_processing_ms": float(np.median(latency)),
            "p95_processing_ms": float(np.percentile(latency, 95)),
        }
    )
    return metrics, estimates, indices


def save_trajectory(path: Path, sequence: Sequence, estimates: list, indices: np.ndarray):
    """CSV includes every pose, including held poses after failed updates."""
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            [
                "frame",
                "timestamp_s",
                "accepted",
                "tracks",
                "inliers",
                "processing_ms",
                "reason",
                *[f"T_first_current_{i}{j}" for i in range(4) for j in range(4)],
            ]
        )
        for index, estimate in zip(indices, estimates, strict=True):
            writer.writerow(
                [
                    int(index),
                    float(sequence.timestamps[index]),
                    estimate.accepted,
                    estimate.tracks,
                    estimate.inliers,
                    estimate.processing_ms,
                    estimate.reason,
                    *estimate.pose.ravel().tolist(),
                ]
            )


def run_benchmark(
    output: Path,
    scenes: list[str],
    seeds: list[int],
    frames: int = 180,
    fps: float = 15,
    width: int = 320,
    height: int = 240,
    video: bool = True,
    frame_interval: int = 4,
    max_recovery_failures: int = 30,
):
    from .report import build_report, render_demo

    output.mkdir(parents=True, exist_ok=True)
    trajectories = output / "trajectories"
    trajectories.mkdir(exist_ok=True)
    cv2.setNumThreads(1)
    rows, flights, demo = [], [], None
    for scene in scenes:
        for seed in seeds:
            print(f"Capturing {scene}, seed {seed}, {frames} frames ...", flush=True)
            sequence = capture(scene, seed, frames, fps, width, height)
            reference = {
                "scene": scene,
                "seed": seed,
                "timestamps": sequence.timestamps.tolist(),
                "ground_truth": sequence.poses[:, :3, 3].tolist(),
                "camera_poses": sequence.poses.tolist(),
                "calibration": sequence.calibration.tolist(),
            }
            (trajectories / f"{scene}-{seed}-truth.json").write_text(json.dumps(reference))
            if demo is None:
                demo = (sequence, {})
            for case in CASES:
                for mode in MODES:
                    metrics, estimates, indices = run_sequence(
                        sequence, case, mode, frame_interval, max_recovery_failures
                    )
                    rows.append(metrics)
                    stem = f"{scene}-{seed}-{case}-{mode}"
                    save_trajectory(trajectories / f"{stem}.csv", sequence, estimates, indices)
                    world_estimated = sequence.poses[0][None] @ np.array(
                        [e.pose for e in estimates]
                    )
                    flights.append(
                        {
                            "scene": scene,
                            "seed": seed,
                            "case": case,
                            "estimator": mode,
                            "indices": indices.tolist(),
                            "estimated": world_estimated[:, :3, 3].tolist(),
                            "truth": sequence.poses[indices, :3, 3].tolist(),
                            "time": sequence.timestamps[indices].tolist(),
                            "accepted": [e.accepted for e in estimates],
                            "inliers": [e.inliers for e in estimates],
                            "reasons": [e.reason for e in estimates],
                        }
                    )
                    if demo[0] is sequence:
                        demo[1][(case, mode)] = (estimates, indices)
                    print(
                        f"  {case:12} {mode:8} RMSE {metrics['position_rmse_m']:.3f} m, "
                        f"coverage {metrics['tracking_coverage']:.0%}",
                        flush=True,
                    )
    metadata = {
        "video_rendered": video,
        "settings": {
            "scenes": scenes,
            "seeds": seeds,
            "frames": frames,
            "fps": fps,
            "width": width,
            "height": height,
            "frame_interval": frame_interval,
            "max_recovery_failures": max_recovery_failures,
            "estimators": list(MODES),
        },
        "environment": {
            "platform": platform.platform(),
            "python": platform.python_version(),
            "numpy": np.__version__,
            "opencv": cv2.__version__,
            "mujoco": mujoco.__version__,
            "opencv_threads": 1,
        },
        "cases": CASES,
        "methodology": {
            "motion": "Prescribed camera trajectory; no flight dynamics or autonomous control",
            "depth": "Ideal metric axial depth from MuJoCo; no depth noise in these experiments",
            "alignment": "Initial camera frame only, no fitted rotation, translation, or scale",
            "failures": "Hold last pose on rejected updates; include held poses in RMSE",
            "models": "Classical RGB-D VO; no learned model, loop closure, IMU, or relocalization",
            "comparison": "Same inputs, 500 Shi-Tomasi corners, LK, and RANSAC PnP in all modes; "
            "checked and recovery add forward/backward flow and inlier-support checks",
            "recovery": "Retain the reference on rejection; accepted gap-spanning estimates "
            "are labeled recovered. After max_recovery_failures rejected observations, "
            "re-anchor at the held pose and label the unrecovered gap. Past poses are unchanged.",
            "events": "recovered_updates counts accepted estimates after failures; reanchors "
            "counts expired references. Neither count guarantees ground-truth correctness.",
            "runtime": "Estimator-only wall time, excluding rendering, corruption, plotting and encoding",
        },
    }
    (output / "results.json").write_text(json.dumps({"metadata": metadata, "runs": rows}, indent=2))
    with (output / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    if video and demo is not None:
        print("Encoding demo ...", flush=True)
        render_demo(output, *demo, fps=fps, frame_interval=frame_interval)
    build_report(output, rows, flights, metadata)
    print(f"Report: {(output / 'index.html').resolve()}", flush=True)
    return rows
