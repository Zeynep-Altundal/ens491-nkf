# Interface contract

Status: DRAFT, not frozen. Changes after the freeze require an ADR, a migration note
and a contract test.

## Sequence

| Field | Shape / type | Unit | Note |
|---|---|---|---|
| `id` | str | - | Drive or flight identifier |
| `platform` | `ugv` or `uav` | - | |
| `t_s` | [N] float64 | s | Strictly increasing |
| `gyro_rad_s` | [N, 3] | rad/s | Frame: OPEN |
| `accel_m_s2` | [N, 3] | m/s^2 | Frame and gravity sign: OPEN |
| `reference_pose` | - | m, rotation | Evaluation only |
| `reference_mask` | [N] bool | - | Valid reference samples |
| `calibration` | - | - | Sensor-to-body extrinsics |
| `source_hash` | str | - | SHA-256 of the source files |
| `observation_stream` | - | - | Position observations outside outages |
| `availability_mask` | - | - | False during an outage |
| `split_id` | str | - | train / validation / test_locked |

## Filter

```
Filter.step(imu, dt, observations_available_now)
    -> state (p, v, R, b_g, b_a), covariance [15, 15], diagnostics
```

Ground truth is never passed to `Filter.step`.

## EvaluationResult

`run_id, sequence_id, outage_id, metric, value, unit, reference_dof, status,
failure_reason, provenance`

## Conventions to fix before the freeze

| Item | Decision |
|---|---|
| Quaternion ordering | OPEN |
| World and body frames | OPEN |
| Gravity sign | OPEN |
| Time origin | OPEN |
| Covariance error-state ordering | OPEN |
