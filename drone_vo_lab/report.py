"""Standalone experiment report, scientific plots, and a recorded sensor replay."""

import json
import re
from pathlib import Path

import cv2
import imageio.v2 as imageio
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .benchmark import CASES, MODES, perturb
from .geometry import inverse

BACKGROUND = (13, 19, 27)
TRUTH = (199, 209, 218)
BASELINE = (93, 174, 255)
CHECKED = (222, 237, 139)


def summary_plot(output: Path, rows: list[dict]):
    scenes = list(dict.fromkeys(row["scene"] for row in rows))
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "text.color": "#d9e1eb",
            "axes.labelcolor": "#a4b1c0",
            "xtick.color": "#a4b1c0",
            "ytick.color": "#a4b1c0",
            "axes.edgecolor": "#344052",
            "figure.facecolor": "#0d131b",
            "axes.facecolor": "#0d131b",
            "savefig.facecolor": "#0d131b",
        }
    )
    fig, axes = plt.subplots(len(scenes), 2, figsize=(12, 4 * len(scenes)), squeeze=False)
    x = np.arange(len(CASES))
    for row_index, scene in enumerate(scenes):
        for col, (metric, ylabel) in enumerate(
            [
                ("position_rmse_m", "Position RMSE / meters"),
                ("tracking_coverage", "Accepted updates / %"),
            ]
        ):
            ax = axes[row_index, col]
            for mode_index, mode in enumerate(MODES):
                groups = [
                    [
                        r[metric]
                        for r in rows
                        if r["scene"] == scene and r["case"] == case and r["estimator"] == mode
                    ]
                    for case in CASES
                ]
                scale = 100 if metric == "tracking_coverage" else 1
                means = np.array([np.mean(group) for group in groups]) * scale
                stds = np.array([np.std(group, ddof=0) for group in groups]) * scale
                ax.bar(
                    x + (mode_index - 0.5) * 0.34,
                    means,
                    0.32,
                    label=mode,
                    color=["#5daeff", "#deed8b"][mode_index],
                    yerr=stds,
                    capsize=3,
                    error_kw={"ecolor": "#d9e1eb", "elinewidth": 1},
                )
            ax.set_xticks(x, ["Clean", "Blur", "Frame drop", "Blackout"])
            ax.set_ylabel(ylabel)
            ax.set_title(scene.replace("_", " ").upper(), loc="left", color="#e7edf5", pad=15)
            ax.grid(axis="y", alpha=0.13)
            ax.set_axisbelow(True)
            ax.spines[["top", "right"]].set_visible(False)
            if metric == "tracking_coverage":
                ax.set_ylim(0, 110)
            if row_index == 0 and col == 0:
                ax.legend(frameon=False, labelcolor="#d9e1eb")
    fig.suptitle("Drone VO Lab / paired sensor stress tests", x=0.08, ha="left", fontsize=18)
    fig.text(
        0.08,
        0.015,
        "Mean ± population standard deviation across seeds. Held poses remain in RMSE. "
        "No fitted trajectory alignment.",
        color="#a4b1c0",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(output / "summary.png", dpi=150)
    plt.close(fig)


def _text(frame, text, xy, scale=0.5, color=TRUTH, thickness=1):
    cv2.putText(frame, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def render_demo(output: Path, sequence, results: dict, fps: float, frame_interval: int):
    truth = sequence.poses[:, :3, 3]
    relative_truth = inverse(sequence.poses[0])[None] @ sequence.poses
    gif_frames = []
    count = 0
    with imageio.get_writer(
        output / "demo.mp4",
        fps=fps,
        codec="libx264",
        quality=8,
        macro_block_size=1,
        ffmpeg_log_level="error",
    ) as writer:
        for case in CASES:
            indices, images = perturb(sequence, case, frame_interval=frame_interval)
            modes = {mode: results[(case, mode)][0] for mode in MODES}
            world = {
                mode: (sequence.poses[0][None] @ np.array([e.pose for e in modes[mode]]))[:, :3, 3]
                for mode in MODES
            }

            def project(points):
                # Shared fixed bounds keep scale comparable across all stress conditions.
                return np.column_stack(
                    [980 + points[:, 0] * 49, 560 - (points[:, 1] + 5) * 39]
                ).astype(np.int32)

            for frame_index in range(len(sequence.rgb)):
                selected = max(0, int(np.searchsorted(indices, frame_index, side="right") - 1))
                image = images[selected].copy()
                estimate = modes["checked"][selected]
                color = CHECKED if estimate.accepted else (255, 139, 112)
                for before, after in zip(
                    estimate.previous_pixels[::2], estimate.pixels[::2], strict=True
                ):
                    cv2.line(
                        image,
                        tuple(np.rint(before).astype(int)),
                        tuple(np.rint(after).astype(int)),
                        color,
                        1,
                        cv2.LINE_AA,
                    )
                    cv2.circle(image, tuple(np.rint(after).astype(int)), 2, color, -1, cv2.LINE_AA)
                canvas = np.full((720, 1280, 3), BACKGROUND, dtype=np.uint8)
                _text(canvas, "DRONE VO LAB", (32, 38), 0.72, CHECKED, 2)
                _text(canvas, "RGB-D ODOMETRY / CONTROLLED SENSOR FAILURES", (272, 36), 0.52)
                cv2.line(canvas, (32, 59), (1248, 59), (45, 57, 73), 1)
                _text(canvas, case.replace("_", " ").upper(), (32, 94), 0.70, (239, 244, 249), 2)
                _text(
                    canvas,
                    f"T+{sequence.timestamps[indices[selected]]:05.2f}s   |   {estimate.inliers} inliers   |   {estimate.reason}",
                    (32, 636),
                    0.59,
                    color,
                )
                canvas[120:600, 32:672] = cv2.resize(
                    image, (640, 480), interpolation=cv2.INTER_NEAREST
                )
                _text(canvas, "ONBOARD RGB / CHECKED TRACKS", (32, 116), 0.40)
                _text(canvas, "TRAJECTORY / TOP VIEW", (724, 116), 0.40)
                for x in range(-4, 5):
                    line = project(np.array([[x, -5, 0], [x, 5, 0]]))
                    cv2.line(canvas, tuple(line[0]), tuple(line[1]), (30, 40, 52), 1)
                for y in range(-5, 6):
                    line = project(np.array([[-4, y, 0], [4, y, 0]]))
                    cv2.line(canvas, tuple(line[0]), tuple(line[1]), (30, 40, 52), 1)
                cv2.polylines(canvas, [project(truth)], False, (63, 73, 85), 2, cv2.LINE_AA)
                gt_index = indices[selected]
                cv2.polylines(
                    canvas, [project(truth[: gt_index + 1])], False, TRUTH, 2, cv2.LINE_AA
                )
                for mode, line_color in [("baseline", BASELINE), ("checked", CHECKED)]:
                    points = project(world[mode][: selected + 1])
                    cv2.polylines(canvas, [points], False, line_color, 2, cv2.LINE_AA)
                    cv2.circle(canvas, tuple(points[-1]), 5, line_color, -1, cv2.LINE_AA)
                    error = np.linalg.norm(
                        modes[mode][selected].pose[:3, 3] - relative_truth[gt_index, :3, 3]
                    )
                    y = 651 if mode == "baseline" else 680
                    _text(
                        canvas,
                        f"{mode.upper():8}  position error {error:.3f} m",
                        (724, y),
                        0.55,
                        line_color,
                    )
                _text(canvas, CASES[case], (32, 676), 0.41)
                _text(
                    canvas,
                    "Scripted camera motion. Ideal depth. No pose fitting or truth resets.",
                    (32, 701),
                    0.39,
                    (132, 148, 166),
                )
                writer.append_data(canvas)
                if count == 30:
                    imageio.imwrite(output / "preview.png", canvas)
                if case == "clean" and frame_index % 3 == 0:
                    gif_frames.append(cv2.resize(canvas, (768, 432), interpolation=cv2.INTER_AREA))
                count += 1
    imageio.mimsave(output / "demo.gif", gif_frames, duration=3 / fps * 1000, loop=0)


def build_report(output: Path, rows: list[dict], flights: list[dict], metadata: dict):
    summary_plot(output, rows)
    payload = json.dumps({"runs": rows, "flights": flights, "metadata": metadata}, allow_nan=False)
    template = Path(__file__).with_name("report.html").read_text()
    if not metadata.get("video_rendered", True):
        template = re.sub(r'<section id="sensor-replay">.*?</section>', "", template, flags=re.S)
        template = re.sub(r'<a href="demo.mp4">.*?</a>', "", template, flags=re.S)
        template = template.replace(
            '<div class="two">', '<div class="two" style="grid-template-columns:1fr">'
        )
    (output / "index.html").write_text(template.replace("__EXPERIMENT_DATA__", payload))
