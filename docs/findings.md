# Findings from the recorded experiment

The experiment used two procedural scenes, three seeds, four sensor conditions, and three estimator configurations, for 72 runs. Each source flight had 180 RGB-D frames at 320 × 240 and 15 Hz. The camera traveled about 15 meters. Motion was scripted; depth had no added sensor noise. See `artifacts/results.json` for the exact environment and `artifacts/trajectories/` for the full pose logs.

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

For clean warehouse flights, the baseline averaged 0.163 m position RMSE and the checked variant averaged 0.182 m. Both accepted every update. The checked pipeline also required more CPU time. For the recorded runs, mean per-run median processing time in this condition was 1.48 ms for the baseline and 2.25 ms for the checked variant.

Under low-texture blur, the checked variant improved seed 0 but worsened seeds 1 and 2. The aggregate mean alone hides that variation. Both individual seeds and mean/standard-deviation plots are included in the report.

## Recovery can bridge a gap, but can also accept a large wrong motion

Recovery retains its reference after rejected updates, uses the same consistency/support checks as checked, and re-anchors after 30 consecutive rejections. It does not receive true poses or revise earlier held estimates.

Warehouse blackout results show both outcomes:

| Seed | Checked RMSE | Recovery RMSE | Recovery events | Re-anchors |
| --- | ---: | ---: | ---: | ---: |
| 0 | 0.739 m | 9.845 m | 1 | 0 |
| 1 | 0.720 m | 9.860 m | 1 | 0 |
| 2 | 0.738 m | 0.297 m | 1 | 0 |

All three runs accept a gap-spanning estimate at source frame 99. Seed 2 improves, but seeds 0 and 1 accept movements of about 7.2 meters and rotations of about 171 degrees. Those incorrect estimates produce large subsequent drift despite passing the existing consistency and inlier-support gates. The event label `recovered` records algorithm acceptance, not ground-truth correctness. This exposes a limitation of retained-reference recovery with the current checks.

For the low-texture blackout condition, recovery's mean RMSE is 1.588 m, compared with 1.725 m for checked. However, all three recovery runs record zero accepted recovery events and two re-anchors each. The lower error in those runs must not be described as successful displacement recovery; it reflects different reference and rejection behavior. Other low-texture conditions favor checked over recovery on average.

Baseline and checked resume local tracking after a blackout without recovering missed displacement. Recovery can bridge a gap when matching remains reliable, but it is experimental and is not consistently better. The short synthetic image-shift test verifies successful pose composition across a gap; it does not establish reliable recovery in these larger scenes.

## Limits of the findings

Three seeds and simple synthetic scenes cannot establish real-world robustness. The checked variant combines forward/backward flow filtering with inlier-support gates; this run does not isolate their individual contributions. Rendering and OpenCV versions can affect numerical results. The data demonstrates specific failure cases and tradeoffs, not a state-of-the-art estimator.
