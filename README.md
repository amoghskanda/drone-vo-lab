# Drone VO Lab

**What happens to a drone's visual odometry when its camera blurs, drops frames, or goes dark?**

A small RGB-D visual-odometry benchmark built with MuJoCo and OpenCV. Generate a repeatable aerial camera trajectory, estimate its motion from sensor frames, and compare the estimate against ground truth under controlled corruptions. Runs locally on a CPU. No model weights or external scene downloads.

![Recorded sensor replay with feature tracks and estimated trajectories](artifacts/demo.gif)

[Watch the MP4](artifacts/demo.mp4) · [Experiment report](artifacts/index.html) · [Raw metrics](artifacts/results.csv) · [Methodology](docs/methodology.md)

The camera motion is prescribed. This project evaluates perception; it does not implement autonomous flight, drone dynamics, or a path planner.

## Run it

Python 3.11 or newer:

```bash
git clone https://github.com/amoghskanda/drone-vo-lab.git
cd drone-vo-lab
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
drone-vo benchmark --quick --output runs/demo
open runs/demo/index.html
```

The command above opens the report in your browser. Without `--output`, the benchmark creates a timestamped folder under `runs/` and prints the report path. The report has an orbitable 3D trajectory replay, a time scrubber, scene/seed/condition selectors, metrics, and a recorded RGB replay. It uses local files and does not need a server or a network connection.

`requirements-lock.txt` records the exact dependencies used for the committed run. To use them, install with `pip install -r requirements-lock.txt` before the editable install.

Set the frame-drop interval with `--frame-interval`. For example, `--frame-interval 2` keeps frames 0, 2, 4, and so on:

```bash
drone-vo benchmark --quick --frame-interval 2
```

The interval must be positive and leave at least two frames. The selected interval is recorded in the output metadata.

Reproduce the full experiment:

```bash
drone-vo benchmark --output runs/full --scenes warehouse low_texture --seeds 0 1 2
```

This captures six 180-frame sequences at 320 × 240 and 15 Hz, then evaluates two estimators under four conditions, for 48 estimator runs. Each complete camera trajectory covers about 15 meters. By default, the frame-drop case keeps every fourth observation with its original timestamp.

On a Linux machine without a display, configure an available MuJoCo rendering backend before running. The CI workflow uses software OSMesa:

```bash
sudo apt-get install libosmesa6
MUJOCO_GL=osmesa drone-vo benchmark --quick --no-video
```

On macOS, offscreen capture runs with normal `python`; the project does not launch a passive MuJoCo viewer or an OpenCV GUI window. A graphical login session is still required by the default GLFW backend.

## The experiment

Both configurations use Shi-Tomasi features, pyramidal Lucas-Kanade optical flow, previous-frame metric depth, RANSAC PnP, and pose refinement. Features are detected afresh for each image pair.

- **Baseline:** forward flow and RANSAC PnP.
- **Checked:** the same pipeline, plus forward/backward flow consistency and minimum inlier-support checks.

There is no claim that the checked variant always wins. It rejects some unsupported estimates at the cost of additional CPU work and potentially missed motion.

| Condition | Change to the sensor stream |
| --- | --- |
| Clean | Original synchronized RGB-D |
| Motion blur | Horizontal 21-pixel box filter on RGB |
| Frame drop | Keep every Nth synchronized RGB-D pair; N defaults to 4 |
| Blackout | Replace RGB with black during 40–55% of the trajectory |

The warehouse and low-texture scenes share the same layout. The latter removes checkerboard flooring and wall markers and reduces box color variation. Random seeds change scene appearance and camera path phase. These are procedural test scenes, not independent real-world datasets.

## Evaluation without hidden pose corrections

- The estimator receives RGB, depth, and intrinsics. It never receives simulator geometry, camera poses, or trajectory commands.
- Estimated and true trajectories share the initial camera frame. There is no best-fit trajectory alignment, scale correction, or ground-truth reset.
- Failed updates hold the previous pose. Those held poses stay in position RMSE, rotation RMSE, and final error.
- Tracking coverage counts accepted motion updates and excludes initialization. It accompanies error so an estimator cannot appear better simply by rejecting difficult frames.
- Runtime measures estimator-only wall time. Rendering, corruption, plotting, and encoding are excluded. OpenCV uses one thread.

The generated report saves environment versions, parameters, all per-frame estimated transforms, update status, feature counts, and ground-truth camera poses. See [methodology](docs/methodology.md) for conventions and limitations.

## Results

The committed experiment contains **48 runs across two scenes and three seeds**, recorded with the default frame interval of 4 on an Apple M5 with OpenCV restricted to one thread. Values below are the mean of per-run position RMSE over three seeds.

| Scene and condition | Baseline RMSE | Checked RMSE | Baseline / checked coverage |
| --- | ---: | ---: | ---: |
| Warehouse, clean | 0.163 m | 0.182 m | 100% / 100% |
| Warehouse, blackout | 0.720 m | 0.732 m | 84.4% / 84.4% |
| Low texture, clean | 1.539 m | 1.404 m | 100% / 95.5% |
| Low texture, frame drop | 5.822 m | 1.156 m | 100% / 80.3% |

In the low-texture frame-drop condition, the checked estimator rejects more updates and avoids large drift seen in two baseline runs. The clean warehouse favors the simpler baseline. Local tracking resumes after a blackout, but missed displacement remains in the accumulated trajectory.

These are small synthetic experiments, not a general accuracy claim. [All conditions and individual seeds](artifacts/results.csv) are retained, including runs where checks increase error. See [findings](docs/findings.md) for the per-seed tradeoff.

The `artifacts/` directory includes a 48-second H.264 replay, animated preview, offline interactive report, metrics, and every per-frame pose. GitHub displays HTML as source; clone or download the repository and open `artifacts/index.html` locally to use the report.

![Position error and tracking coverage](artifacts/summary.png)

## Structure

```text
drone_vo_lab/
  scene.py       Original procedural scene and calibrated RGB-D capture
  odometry.py    Sensor-only motion estimation and explicit failure handling
  geometry.py    Camera conventions, SE(3), and evaluation
  benchmark.py   Paired corruptions, seeded runs, CSV/JSON logging
  report.py      Scientific figures and encoded sensor replay
  report.html    Offline interactive report template
  cli.py         One-command benchmark
tests/           Geometry, PnP outliers, failure handling, rendering integration
artifacts/       Reproducible run, video, charts, and trajectories
```

## Checks

```bash
pytest
RUN_RENDER_TESTS=1 pytest
ruff check .
```

The rendered integration test checks the entire RGB-D-to-pose chain against a known camera flight. The numeric tests cover pose direction, metric depth, outlier rejection, held-pose failure handling, and initial-frame evaluation.

## Limits

This is classical RGB-D visual odometry. It has no learned model, IMU fusion, loop closure, bundle adjustment, or relocalization. Simulator depth is ideal; blur is a post-render filter rather than a physical exposure model. Tracking can resume after a blackout while the global trajectory retains the motion lost during that blackout.

Results characterize these synthetic sequences and the recorded software environment. They do not establish real-flight accuracy or onboard runtime.

## References

- [MuJoCo Python renderer](https://mujoco.readthedocs.io/en/stable/python.html)
- [OpenCV pyramidal Lucas-Kanade](https://docs.opencv.org/4.x/dc/d6b/group__video__track.html)
- [OpenCV PnP, RANSAC, and pose refinement](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)

Original scene, benchmark, and estimator integration. No external meshes, datasets, or trained models are bundled. MIT license.
