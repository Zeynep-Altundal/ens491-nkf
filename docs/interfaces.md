# Interface and convention contract

Status: DRAFT. Not frozen. No implementation may rely on this document until the
freeze is recorded in an ADR. After the freeze, any change requires an ADR, a
migration note and a contract test.

Evidence base: KITTI OXTS audit in `docs/data_manifest.md` (commit `32eadaa`).
The EuRoC audit is NOT RECORDED; every UAV-specific statement below is PROPOSED.

## 1. Frames

| Frame | Definition | Status |
|---|---|---|
| World `w` | Local tangent plane, ENU: x East, y North, z Up. Origin at the first valid reference sample of the sequence. | Agreed |
| Body `b` | The frame in which the inertial measurements are expressed. | Agreed |
| Body, UGV (KITTI) | x front, y left, z top, as described by the dataset for `ax ay az`, `wx wy wz`. Treated as identical to the vehicle frame. | PROPOSED; mounting rotation and lever arm OPEN |
| Body, UAV (EuRoC) | IMU sensor frame of the dataset. Not assumed to be front-left-up. | PROPOSED; audit NOT RECORDED |

Geodetic reference positions are converted to `w` by WGS-84 geodetic to ECEF to ENU
about the first valid reference sample. Status: PROPOSED. The KITTI development kit
uses a scaled Mercator projection instead; the difference over the sequence length is
to be quantified before the freeze.

## 2. Nominal state

| Symbol | Meaning | Shape | Unit |
|---|---|---|---|
| `p` | Position of `b` origin in `w` | [3] | m |
| `v` | Velocity of `b` origin in `w` | [3] | m/s |
| `q` | Orientation, Hamilton quaternion, order `[w, x, y, z]`, unit norm | [4] | - |
| `b_g` | Gyroscope bias, in `b` | [3] | rad/s |
| `b_a` | Accelerometer bias, in `b` | [3] | m/s^2 |

`R = R(q)` is the rotation matrix `R_wb`: it maps a vector expressed in `b` to `w`.

## 3. Inertial measurement model

```
w_m = w_true + b_g + n_g          angular rate, rad/s, in b
f_m = f_true + b_a + n_a          specific force, m/s^2, in b
```

The accelerometer measures specific force: gravity is not removed (audit: mean
`az` = +9.798 m/s^2 at near-level attitude, mean norm 9.8119 m/s^2).

```
g_w   = [0, 0, -9.81] m/s^2       configuration field; value PROPOSED
p_dot = v
v_dot = R (f_m - b_a - n_a) + g_w
R_dot = R [w_m - b_g - n_g]x
b_g_dot = n_bg
b_a_dot = n_ba
```

`[u]x` is the skew-symmetric matrix of `u`. Earth rotation and Coriolis terms are
neglected. This is a stated modelling assumption and a limitation, not a finding.

## 4. Discrete nominal propagation (reference implementation)

For sample `k` with measured `dt_k = t_{k+1} - t_k`:

```
w   = w_m[k] - b_g
a   = R_k (f_m[k] - b_a) + g_w
R_{k+1} = R_k Exp(w dt_k)
v_{k+1} = v_k + a dt_k
p_{k+1} = p_k + v_k dt_k + 0.5 a dt_k^2
```

`Exp` is the SO(3) exponential map. The measurement at `k` is held constant over the
interval. A different integration scheme is a configuration field and requires an ADR
after the freeze.

## 5. Error state

Order and indices: `dx = [dp (0:3), dv (3:6), dtheta (6:9), db_g (9:12), db_a (12:15)]`.
The covariance `P` is [15, 15] in this order.

```
p_true   = p_hat + dp
v_true   = v_hat + dv
R_true   = R_hat Exp([dtheta]x)       right-multiplicative; dtheta expressed in b
b_g_true = b_g_hat + db_g             bias error = true minus nominal
b_a_true = b_a_hat + db_a
```

First-order error dynamics, consistent with the definitions above:

```
dp_dot     = dv
dv_dot     = -R_hat [f_m - b_a_hat]x dtheta - R_hat db_a - R_hat n_a
dtheta_dot = -[w_m - b_g_hat]x dtheta - db_g - n_g
db_g_dot   = n_bg
db_a_dot   = n_ba
```

Continuous noise: `n_g` (rad/s/sqrt(Hz)), `n_a` (m/s^2/sqrt(Hz)), `n_bg`
(rad/s^2/sqrt(Hz)), `n_ba` (m/s^3/sqrt(Hz)). Numerical values: OPEN.

Covariance propagation: `P <- Phi P Phi^T + Q_d`, with `Phi = I + F dt_k` and
`Q_d = G Q_c G^T dt_k` as the reference discretisation, where `F` and `G` are the
Jacobians of the dynamics above. A higher-order discretisation is a configuration
field.

## 6. Measurement update

```
y  = z - h(x_hat)
H  = dh/d(dx) evaluated at dx = 0
S  = H P H^T + R_meas
K  = P H^T S^-1
dx_hat = K y
P  <- (I - K H) P (I - K H)^T + K R_meas K^T        Joseph form
```

Position observation: `z = p + n`, `H = [I3, 0, 0, 0, 0]`, unit m, expressed in `w`.
Motion-constraint pseudo-measurements use the same update; their models are defined
in the theory note, not in this contract.

## 7. Injection and reset

```
p_hat   <- p_hat + dp_hat
v_hat   <- v_hat + dv_hat
q_hat   <- q_hat (x) q{dtheta_hat}      quaternion product, increment on the right
q_hat   <- q_hat / |q_hat|
b_g_hat <- b_g_hat + db_g_hat
b_a_hat <- b_a_hat + db_a_hat

dx <- 0
P  <- G_r P G_r^T,   G_r = blockdiag(I6, I3 - 0.5 [dtheta_hat]x, I6)
```

`q{dtheta}` is the unit quaternion of the rotation vector `dtheta`.

## 8. Data objects

### 8.1 Inertial input (allowlist)

Only these fields may reach the filter, the network, preprocessing or feature code.

| Field | Shape | Unit | KITTI source |
|---|---|---|---|
| `t_s` | [N] float64 | s | `timestamps.txt` |
| `gyro_rad_s` | [N, 3] float64 | rad/s | `wx wy wz` |
| `accel_m_s2` | [N, 3] float64 | m/s^2 | `ax ay az` |

Rejected KITTI fields: `af al au`, `wf wl wu` (rotated with the OXTS attitude
solution; audit: `af - ax` = 0.1059 m/s^2 against g x pitch = 0.1086 m/s^2).

### 8.2 Reference (evaluation only)

Held in a separate object that is never passed to the filter or the network.

| Field | Shape | Unit |
|---|---|---|
| `t_s` | [M] float64 | s |
| `p_w` | [M, 3] | m |
| `v_w` | [M, 3] | m/s |
| `q_wb` | [M, 4] | - |
| `valid` | [M] bool | - |
| `quality` | dataset-specific | - |

### 8.3 Observation stream

Position observations made available to the filter outside outages, with an
availability mask. For the UAV track they are synthesised from the reference and are
reported as simulated absolute-position updates, never as GNSS.

### 8.4 Sequence metadata

`id`, `platform` (`ugv` or `uav`), `calibration`, `source_hash` (SHA-256),
`split_id` (`train`, `validation`, `test_locked`).

### 8.5 Filter

```
Filter.step(gyro_rad_s, accel_m_s2, dt_s, observations_available_now)
    -> state (p, v, q, b_g, b_a), covariance [15, 15], diagnostics
```

The signature has no reference argument.

### 8.6 EvaluationResult

`run_id, sequence_id, outage_id, metric, value, unit, reference_dof, status,
failure_reason, provenance`

## 9. Time validation and invalid samples

- `t_s` is float64 seconds from the first sample of the sequence.
- `dt_k` is the measured timestamp difference. A constant rate is never assumed.
- A non-increasing timestamp rejects the sequence. It is not repaired.
- A non-finite inertial sample is rejected and counted. It is never interpolated
  silently.
- A gap `dt_k > dt_max` is flagged and counted in diagnostics. `dt_max` and the
  handling of outage windows that contain a gap: OPEN.

Audit reference for KITTI drive `2011_09_30_drive_0016`: 2967 samples, `dt` 9.658 to
10.342 ms, no non-increasing timestamp, no gap above 15 ms.

## 10. Initialisation and leakage safeguards

- Oracle-initialised baseline: `p`, `v`, `q` at `t_0` are taken from the reference.
  After `t_0` the reference changes the state only through the declared observation
  stream. Every result produced this way carries the label `init: oracle`.
- An oracle-initialised result is not evidence of operational GNSS-denied navigation.
- Initial biases are set to zero. This is an initial estimate, not a statement that
  the true biases are zero.
- Initial covariance `P_0`: OPEN.
- Operational initialisation (no reference at `t_0`) is outside this version of the
  contract.
- The allowlist of section 8.1 is enforced in the loader and covered by a contract
  test (planned: `tests/fast/test_no_gt_leakage.py`).

## 11. Reference orientation, KITTI

PROPOSED mapping: `R_wb = Rz(yaw) Ry(pitch) Rx(roll)` with the dataset definitions
(yaw 0 = east, counter-clockwise; pitch positive = front down; roll positive = left
side up).

Numerical check on drive `2011_09_30_drive_0016`, all 2967 samples
(`docs/data_manifest.md`):

- VERIFIED: sign and axis of roll and pitch (residual 0.012 m/s^2 rms against 0.25
  and 0.85 m/s^2 for the sign-flipped variants); sign of yaw (0.078 m/s rms against
  27.9 m/s).
- NOT OBSERVABLE on this drive: the order of the roll and pitch rotations.
- OPEN: unexplained residuals of the levelled-frame comparison.

The comparison of `yaw` with `atan2(vn, ve)` checks course over ground, not heading,
and is not a verification.

## 12. Open decisions

| Item | Status |
|---|---|
| KITTI OXTS mounting rotation and lever arm | OPEN |
| Whether KITTI inertial channels are raw or corrected by the unit's internal filter | OPEN |
| Meaning and time intervals of `navstat` and mode codes | OPEN |
| Geodetic to ENU method (ECEF-ENU against scaled Mercator) | PROPOSED |
| EuRoC body frame, reference frame and time alignment | OPEN |
| Value of `g_w` magnitude | PROPOSED 9.81 |
| Noise densities and `P_0` | OPEN |
| `dt_max` and gap handling in outage windows | OPEN |
| Order of roll and pitch rotations in the KITTI reference orientation | OPEN: not observable on the audited drive; to be confirmed from the dataset development kit and a drive with larger attitude angles |
| Residuals of the levelled-frame check (0.012 m/s^2, 0.078 m/s rms) | OPEN |
| Second KITTI drive with turns and stops | OPEN; does not block the freeze |
