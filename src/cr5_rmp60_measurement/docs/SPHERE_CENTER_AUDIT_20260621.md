# Standard Sphere Centre Audit (2026-06-21)

## Accepted base-frame centre

```text
C_s = [-401.897176, 126.488486, 89.571120] mm
```

Source: `data/2026.6.16/yneg_near_fourth_refit_20260616.json`.

Machine-readable authority: `config/calibration_registry.json`. Data status and
physical-stylus identity are summarized in `data/CALIBRATION_DATA_INDEX.md`.

The fit uses 10 mixed-orientation contacts, has rank 6, condition 7.655,
RMS residual 0.055429 mm, and maximum residual 0.103729 mm. Re-running the
absolute fitter from its five source CSV files reproduces every reported value.

## Independent agreement

| Fit | Centre delta from accepted centre | Notes |
|---|---:|---|
| 10-point distance-only, 2026-06-10 | 0.021 mm | Does not use approach vectors |
| 6-point normal absolute, 2026-06-10 | 0.085 mm | Rank 6, condition 12.90 |
| 7-point incremental absolute, 2026-06-16 | 0.128 mm | RMS 0.057 mm |
| 8-point incremental absolute, 2026-06-16 | 0.108 mm | RMS 0.054 mm |
| 9-point incremental absolute, 2026-06-16 | 0.036 mm | RMS 0.055 mm |
| 10-point incremental absolute, 2026-06-16 | 0.000 mm | Accepted result |

The 2026-06-01 relative four-direction result with centre Z near 139.5 mm is
not an absolute centre estimate. Its own method warning states that sphere
centre and branch offsets are gauge-coupled because the rows use nearly the
same flange orientation.

## 2026-06-21 early trigger

The early transition trigger does not indicate a moved standard sphere. The
original manual four-branch workflow projected each contact along an assumed
normal before averaging offsets. Manual jog directions are not reliable
contact normals, so the resulting `y_neg` seed had 7.27 mm sample spread.

Fitting the three original `y_neg` trigger poses plus the early transition
trigger with the accepted sphere centre fixed gives:

```text
p_y_neg = [-30.593184, -22.282973, 174.806427] mm
rows = 4, rank = 3, DOF = 1, condition = 1.787
RMS radial residual = 0.088924 mm
maximum radial residual = 0.129844 mm
```

This result is stored in
`data/2026.6.21/y_neg_fixed_center_distance_fit_20260621.json`.

This four-contact result was subsequently used only as the coarse seed for a
real-robot automatic calibration. Five guarded contacts passed MoveIt using
the live `/joint_states_robot` state and produced the accepted offset:

```text
p_y_neg = [-30.650710, -22.242713, 174.850003] mm
rows = 5, rejected = 0
RMS residual = 0.019445 mm
maximum residual = 0.030452 mm
```

Canonical result:
`data/2026.6.21/y_neg_auto_calibrated_fit_20260621.json`.

## Identity warning

The fully calibrated reference ruby offset has XY direction angle +142.55
degrees. The physical ruby called `y_neg` in the later 12-contact collection
has angle -143.93 degrees. They differ by about 73.5 degrees and are adjacent
physical styli. The reference fit supplies the sphere centre only; its local
ruby offset must not be assigned to the later branch by matching the historical
text label.
