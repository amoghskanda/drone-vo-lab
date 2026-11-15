# Visual Odometry Lab

**How accurately can a camera estimate its movement when images blur, frames go missing, or the view goes dark?**

This project estimates a camera's movement through simulated indoor environments, then compares the estimated path with the true path. It tests whether extra reliability checks reduce localization errors when camera observations become less useful.

Built with **Python, OpenCV, and MuJoCo**. Runs locally on a CPU, without pretrained models or external scene downloads.

![Camera replay with tracked features and estimated paths](artifacts/demo.gif)

[Results chart](artifacts/summary.png) · [Detailed findings](docs/findings.md) · [Raw results](artifacts/results.csv)

## What problem does it explore?

Robots, drones, and autonomous vehicles need to estimate their movement. **Visual odometry** does this using successive camera images. Small mistakes can accumulate, causing the estimated path to drift away from the actual path.

This project asks a practical question: **when a motion estimate looks unreliable, does rejecting it cause less error than accepting it?**

It provides a controlled way to investigate that tradeoff before testing with real sensors. The camera follows a scripted path; this project evaluates motion estimation rather than controlling a flying drone.

## How it works

1. **Create an indoor scene.** MuJoCo renders a warehouse or a version with fewer distinctive visual details.
2. **Move a virtual camera.** Capture color images and depth images, which record distance along the camera's viewing axis at each pixel. Together, these are called **RGB-D** observations.
3. **Make observations harder to use.** Produce clean, blurred, dropped-frame, and blackout versions of the same recording.
4. **Estimate movement.** OpenCV tracks image features, such as corners. Their depth gives their 3D positions, which are used to calculate camera translation and rotation. Position and orientation together are called a **pose**. Successive poses form a trajectory.
5. **Compare against the true path.** The simulator knows the actual camera poses, called **ground truth**. These are used for evaluation, never supplied to the motion estimator.

Both estimator configurations receive exactly the same observations for each condition.

| Condition | What changes |
| --- | --- |
| Clean | Original color and depth images |
| Motion blur | Apply a horizontal 21-pixel blur to color images |
| Dropped frames | Keep every fourth color-and-depth pair by default |
| Blackout | Replace color images with black during 40–55% of the recording |

Depth remains ideal in these experiments. A random seed makes the scene variations and camera path repeatable.

## What is being compared?

The **baseline** tracks features and estimates movement using RANSAC PnP, a method that finds camera motion while rejecting feature matches that disagree with it.

The **checked** version uses the same pipeline with two additional safeguards:

- Track each feature forward into the next image and backward again. Keep the match only if it returns within 0.8 pixels of its starting point.
- Accept a motion estimate only when at least 12 feature matches support it and those matches account for at least 35% of the correspondences supplied to pose estimation.

These checks filter feature matches and accept or reject **motion estimates**. They do not assign a blur score to an entire frame.

If an update fails, either estimator holds its last pose and uses the current image as the reference for the next update. Rejection can prevent a large incorrect movement from entering the trajectory, but it also loses real movement. That missed displacement is not recovered later.

## What did the experiment show?

The recorded experiment contains **48 runs**: two scenes × three seeds × four conditions × two estimators. Each source recording contains 180 frames at 15 frames per second.

The table reports **position RMSE**, a measure of the distance between estimated and true positions over a recording. Lower is better; larger mistakes contribute more strongly. Each value is the mean of three runs with different seeds.

| Scene and condition | Baseline error | Checked error |
| --- | ---: | ---: |
| Warehouse, clean | 0.163 m | 0.182 m |
| Warehouse, blackout | 0.720 m | 0.732 m |
| Low texture, clean | 1.539 m | 1.404 m |
| Low texture, dropped frames | 5.822 m | 1.156 m |

**The checks reduced mean position RMSE by about 80% in the low-texture, dropped-frame condition.** This is a comparison with the baseline in that specific simulated condition, not an improvement across every test.

In that condition, the checked estimator accepted 80.3% of motion updates, compared with 100% for the baseline. This percentage is called **tracking coverage**. Held positions after rejected updates still count toward the error, so rejection does not hide difficult timestamps from evaluation. The estimated path is compared from the initial camera frame without fitting it to the true path afterward.

The simpler baseline performed better in the clean warehouse. The extra checks also take more processing time. Both accuracy and coverage matter when deciding whether a safeguard helps.

![Position error and tracking coverage across conditions](artifacts/summary.png)

See the [findings](docs/findings.md) for individual-seed results and the [methodology](docs/methodology.md) for equations, algorithm settings, timing, and evaluation details.

## Run it locally

Use Python 3.11 or newer. On macOS or Linux:

```bash
git clone https://github.com/amoghskanda/drone-vo-lab.git
cd drone-vo-lab
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
drone-vo benchmark --quick --output runs/demo
```

The quick run uses one scene, one seed, and 90 frames, testing both estimators under all four conditions. It is a smaller experiment than the 48-run results above.

Open `runs/demo/index.html` in your browser to explore the trajectories and results. On macOS:

```bash
open runs/demo/index.html
```

The report opens locally without a web server. The same output folder contains a generated `demo.mp4`, `demo.gif`, results in CSV and JSON, and per-frame trajectory logs. Add `--no-video` to skip MP4 and GIF generation while keeping the report and metrics.

The GIF preview and results chart above can be viewed directly on GitHub. The interactive HTML report and MP4 are generated locally and excluded from Git tracking.

### Reproduce the full experiment

```bash
drone-vo benchmark --output runs/full --scenes warehouse low_texture --seeds 0 1 2
```

The committed results use the default frame-drop interval of 4. To try a different interval:

```bash
drone-vo benchmark --quick --frame-interval 2 --output runs/interval-2
```

This keeps frames 0, 2, 4, and so on in the dropped-frame condition, preserving their original timestamps. The interval must be a positive integer that leaves at least two observations. Without `--output`, runs are saved in a timestamped folder under `runs/`.

<details>
<summary>Installation and rendering notes</summary>

`requirements-lock.txt` records the dependency versions used for the committed results. To use those versions, install with `pip install -r requirements-lock.txt` before the editable install.

On a Linux machine without a display, install and select a supported rendering backend. The CI workflow uses software OSMesa:

```bash
sudo apt-get install libosmesa6
MUJOCO_GL=osmesa drone-vo benchmark --quick --no-video
```

On macOS, run with normal Python in a graphical login session. The default renderer requires that session even though the project does not open a simulator window.

</details>

## Read the code

Follow this order to trace a run from the command line to the report:

| File | Purpose |
| --- | --- |
| [cli.py](drone_vo_lab/cli.py) | Read command-line options and start a run |
| [benchmark.py](drone_vo_lab/benchmark.py) | Apply image degradations, run comparisons, and save results |
| [scene.py](drone_vo_lab/scene.py) | Build environments and capture images, depth, and true poses |
| [odometry.py](drone_vo_lab/odometry.py) | Track features and estimate camera movement |
| [geometry.py](drone_vo_lab/geometry.py) | Handle camera geometry, pose transforms, and error calculations |
| [report.py](drone_vo_lab/report.py) | Generate charts, replay videos, and the local HTML report |

Run the checks with:

```bash
pytest
ruff check .
```

To include the rendering integration test, which requires a working graphics backend:

```bash
RUN_RENDER_TESTS=1 pytest
```

## Scope and limitations

This is classical RGB-D visual odometry evaluated on small synthetic scenes. The current implementation has no learned model, inertial sensor fusion, or mechanism to revisit known places and correct accumulated drift. Tracking can resume after a blackout while the trajectory retains an offset from missed movement.

Depth has no added sensor noise, blur is an image filter, and camera movement is scripted. Results establish behavior on these recordings; real-flight accuracy and onboard performance remain untested. The two safeguards are evaluated together, so their individual contributions have not been isolated.

Original procedural scenes and estimator integration. No external meshes, datasets, or trained models are bundled. [MIT license](LICENSE).
