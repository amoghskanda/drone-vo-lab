# Methodology

## Sensor and evaluation boundary

`scene.capture()` renders synchronized RGB and axial depth from a moving MuJoCo camera. It exports timestamps, intrinsic calibration, and the actual camera transform from MuJoCo after `mj_forward()`. The camera follows a deterministic translation, yaw, pitch, and roll sequence. Its motion is prescribed, without a drone controller or a physics-based flight simulation.

`RGBDOdometry` accepts only the intrinsic calibration and the RGB-D observations. It does not import MuJoCo. Ground-truth transforms remain in the capture and evaluation code.

The clean, blurred, dropped-frame, and blackout variants all derive from the same sequence for each scene and seed. Each estimator sees exactly the same observations for a given condition. OpenCV uses a single thread and a reset RNG seed for each estimator run. Renderer and library versions can still affect numerical results.

## Coordinates and depth

`T_a_b` transforms points expressed in frame b into frame a. Homogeneous points are column vectors.

The estimator uses OpenCV camera axes: X right, Y down, Z forward. MuJoCo camera axes are X right, Y up, Z backward. Right-multiplying the MuJoCo camera rotation by `diag(1, -1, -1)` gives the OpenCV camera orientation in world coordinates.

The pinhole camera has vertical field of view 65 degrees and square pixels:

```text
fx = fy = height / (2 tan(fovy / 2))
cx = (width - 1) / 2
cy = (height - 1) / 2
```

Depth is axial Z in meters. For pixel `(u, v)` and depth `z`:

```text
point_previous = z * inverse(K) * [u, v, 1]
```

PnP estimates `T_current_previous` by matching those previous-camera 3D points with tracked current-image pixels. The accumulated camera pose is:

```text
T_first_current = T_first_previous * inverse(T_current_previous)
```

Using the inverse in this composition is essential. The PnP transform moves points between camera coordinates; the reported trajectory describes camera motion.

## Estimators

Both modes detect up to 500 Shi-Tomasi corners in the previous RGB image, use pyramidal Lucas-Kanade to track them, reject out-of-frame tracks, sample the previous depth image at rounded pixel locations, and reject nonfinite depth and depth outside 0.15–25 meters. No current-frame depth consistency check is applied.

RANSAC PnP uses EPNP, 150 iterations, 2.5-pixel reprojection threshold, and 0.999 confidence. The pose is refined with Levenberg-Marquardt using the selected inliers. At least six inliers are required for the baseline.

The checked variant additionally rejects forward/backward flow discrepancies of 0.8 pixels or more. It accepts a pose only when at least 12 inliers remain and their fraction of depth-valid tracks is at least 0.35. This variant bundles both checks; it is not an isolated ablation of each check.

If an update fails, the estimator holds its last pose and advances its image/depth reference to the current frame. It does not integrate the missed displacement later or reset to truth. It can therefore resume local tracking while retaining a global offset.

## Corruptions

- Clean uses original synchronized frames.
- Motion blur convolves RGB with a horizontal 21-pixel uniform kernel. Depth stays ideal. This approximates image degradation and is not physically simulated motion blur.
- Frame drop keeps indices `0, 4, 8, ...` for RGB, depth, timestamps, and corresponding evaluation poses. At 15 Hz this gives 3.75 Hz observations. If the final source frame is not retained, its pose is not evaluated.
- Blackout replaces RGB images in the integer frame interval `[floor(0.40*N), floor(0.55*N))` with zero. Depth remains available, but the feature-based estimator cannot use depth alone.

## Metrics

The true camera trajectory is expressed relative to its first camera frame before comparison. This removes the arbitrary initial coordinate system only. There is no trajectory-fitted SE(3) or Sim(3) alignment, no scale fitting, and no per-frame normalization.

Position RMSE uses every observed timestamp, including held estimates after failures. Rotation RMSE uses the geodesic rotation angle from the trace of `R_truth.T @ R_estimate`. Final error is the positional error at the last observed timestamp. Path length is the sum of distances between the observed ground-truth positions, so the dropped-frame path length is sampled more sparsely.

Coverage is accepted motion updates divided by attempted updates, excluding the initialization frame. A rejection is not equivalent to a confirmed incorrect pose; it is a conservative validity decision. Report coverage and trajectory error together.

Per-update wall time includes grayscale conversion, feature detection, tracking, depth backprojection, pose estimation, and state update. It excludes image acquisition/rendering, corruption generation, evaluation, video encoding, and reporting. The initialization step is excluded from timing summaries. P95 uses NumPy's default percentile interpolation. The bar chart aggregates per-run metrics using the arithmetic mean and population standard deviation across seeds for each scene, condition, and estimator.

## Outputs

`results.json` records settings, software versions, methodology, and all run metrics. `results.csv` gives the same metrics in tabular form. Each trajectory CSV has original frame index, timestamp, validity flag, track/inlier counts, timing, reason, and the full `T_first_current` matrix. Each ground-truth JSON contains camera calibration, world camera transforms, and timestamps.

The offline report embeds the trajectories in JSON. Its 3D viewer transforms estimates into the world frame using the initial true camera transform for display only. The recorded video shows the first scene and seed, with the same fixed top-view scale across conditions.

## Validation and remaining limitations

Tests verify metric backprojection, rigid-transform inversion, camera axes, PnP motion direction with injected correspondence outliers, failure handling, and error accounting without fitted alignment. The rendering integration test checks the whole camera/depth/flow/PnP/pose-composition chain against a prescribed flight.

The benchmark does not model rolling shutter, sensor exposure, lens distortion, asynchronous RGB/depth capture, IMU noise, depth noise, dynamic objects, or real drone dynamics. The procedural warehouse is intentionally small and visually repetitive. Real-world generalization remains untested.
