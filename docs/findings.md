# Findings from the recorded experiment

The experiment used two procedural scenes, three seeds, four sensor conditions, and two estimator configurations. Each source flight had 180 RGB-D frames at 320 × 240 and 15 Hz. The camera traveled about 15 meters. Motion was scripted; depth had no added sensor noise. See `artifacts/results.json` for the exact environment and `artifacts/trajectories/` for the full pose logs.

## Rejecting an update can be better than integrating it

The clearest difference appears when low texture combines with dropping three out of every four frames:

| Seed | Baseline position RMSE | Checked position RMSE | Baseline / checked coverage |
| --- | ---: | ---: | ---: |
| 0 | 11.466 m | 1.294 m | 100% / 72.7% |
| 1 | 5.210 m | 1.507 m | 100% / 79.5% |
| 2 | 0.789 m | 0.668 m | 100% / 88.6% |

The baseline accepted every motion update despite large accumulated error in the first two seeds. The checked variant rejected more updates and had lower error in all three seeds. Its mean position RMSE was 1.156 m, compared with 5.822 m for the baseline. Its mean accepted-update coverage was 80.3%.

Rejected increments hold the previous pose and still count toward RMSE. The result therefore does not come from removing difficult timestamps from the evaluation. However, rejection also loses real displacement. It is not recovery or relocalization, and residual error remains substantial.

## Additional checks are not a universal improvement

For clean warehouse flights, the baseline averaged 0.163 m position RMSE and the checked variant averaged 0.182 m. Both accepted every update. The checked pipeline also required more CPU time. For the recorded runs, mean per-run median processing time in this condition was 1.50 ms for the baseline and 2.32 ms for the checked variant.

Under low-texture blur, the checked variant improved seed 0 but worsened seeds 1 and 2. The aggregate mean alone hides that variation. Both individual seeds and mean/standard-deviation plots are included in the report.

## Tracking resumption does not restore lost displacement

The blackout condition removes RGB information for 15% of the flight. After images return, the estimator can redetect features and resume frame-to-frame motion estimation. It cannot recover displacement from the unavailable increments, so the accumulated path retains an offset.

This is expected for adjacent-frame odometry without inertial propagation, loop closure, or relocalization. The benchmark exposes that distinction through held poses, validity flags, and trajectory error.

## Limits of the findings

Three seeds and simple synthetic scenes cannot establish real-world robustness. The checked variant combines forward/backward flow filtering with inlier-support gates; this run does not isolate their individual contributions. Rendering and OpenCV versions can affect numerical results. The data demonstrates specific failure cases and tradeoffs, not a state-of-the-art estimator.
