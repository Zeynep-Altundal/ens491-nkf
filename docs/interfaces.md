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
about the first valid reference sample. Status: PROPOSED. pykitti uses a scaled
Mercator projection instead; the difference over the sequence length is to be
quantified before the first baseline evaluation.

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
neglected. This is a stated modelling assumption for short-duration local navigation
and a limitation, not a finding.

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

### 5.1 Definition

Order and indices: `dx = [dp (0:3), dv (3:6), dtheta (6:9), db_g (9:12), db_a (12:15)]`.
The covariance `P` is [15, 15] in this order.

```
p_true   = p_hat + dp
v_true   = v_hat + dv
R_true   = R_hat Exp([dtheta]x)       right-multiplicative; dtheta expressed in b
b_g_true = b_g_hat + db_g             bias error = true minus nominal
b_a_true = b_a_hat + db_a
```

### 5.2 Error dynamics

With `f_hat = f_m - b_a_hat` and `w_hat = w_m - b_g_hat`, to first order:

```
dp_dot     = dv
dv_dot     = -R_hat [f_hat]x dtheta - R_hat db_a - R_hat n_a
dtheta_dot = -[w_hat]x dtheta - db_g - n_g
db_g_dot   = n_bg
db_a_dot   = n_ba
```

### 5.3 Jacobians

`dx_dot = F dx + G n`, with the noise vector in the fixed order
`n = [n_g (0:3), n_a (3:6), n_bg (6:9), n_ba (9:12)]`.

`F` is [15, 15]. Rows and columns follow the error-state order. `0` and `I` are 3x3.

```
         dp    dv    dtheta            db_g   db_a
dp     [ 0     I     0                 0      0      ]
dv     [ 0     0    -R_hat [f_hat]x    0     -R_hat  ]
dtheta [ 0     0    -[w_hat]x         -I      0      ]
db_g   [ 0     0     0                 0      0      ]
db_a   [ 0     0     0                 0      0      ]
```

`G` is [15, 12]. Columns follow the noise-vector order.

```
         n_g   n_a      n_bg   n_ba
dp     [ 0     0        0      0  ]
dv     [ 0    -R_hat    0      0  ]
dtheta [-I     0        0      0  ]
db_g   [ 0     0        I      0  ]
db_a   [ 0     0        0      I  ]
```

### 5.4 Noise convention

`n` is zero-mean white noise with `E[n(t) n(s)^T] = Q_c delta(t - s)` and

```
Q_c = blockdiag(sigma_g^2 I3, sigma_a^2 I3, sigma_bg^2 I3, sigma_ba^2 I3)     [12, 12]
```

Configuration stores noise densities `sigma` (square roots of the power spectral
densities), never per-sample standard deviations and never the densities squared.

| Config field | Symbol | Meaning | Unit of `sigma` | Unit of the `Q_c` entry |
|---|---|---|---|---|
| `noise.gyro_density` | `sigma_g` | Gyroscope white noise | rad/s/sqrt(Hz) | (rad/s)^2/Hz |
| `noise.accel_density` | `sigma_a` | Accelerometer white noise | m/s^2/sqrt(Hz) | (m/s^2)^2/Hz |
| `noise.gyro_bias_rw` | `sigma_bg` | Gyroscope bias random walk | rad/s^2/sqrt(Hz) | (rad/s^2)^2/Hz |
| `noise.accel_bias_rw` | `sigma_ba` | Accelerometer bias random walk | m/s^3/sqrt(Hz) | (m/s^3)^2/Hz |

Each density is one scalar per sensor (isotropic) in this version. Validation: finite
and strictly positive. Numerical values per platform: OPEN; selected on training and
validation data only.

### 5.5 Covariance propagation

```
Phi = I + F dt_k
Q_d = G Q_c G^T dt_k
P  <- Phi P Phi^T + Q_d
```

This first-order discretisation is the reference. Under it, the white-noise terms
correspond to a per-sample standard deviation `sigma / sqrt(dt_k)` and the bias
random-walk terms to a per-step increment standard deviation `sigma_b sqrt(dt_k)`.
`P` is symmetrised after each propagation. A higher-order discretisation is a
configuration field.

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

Order of operations for every accepted observation: (1) Joseph update of `P` and
computation of `dx_hat`; (2) injection; (3) covariance reset. Steps 2 and 3 use the
`dx_hat` of step 1.

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

`q{dtheta}` is the unit quaternion of the rotation vector `dtheta`. `dtheta_hat` in
`G_r` is the correction that was injected in step 2. `G_r` is a first-order
approximation in `dtheta_hat`.

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
Reference-only KITTI fields (section 8.2) are refused as inertial input in the same
way: a request for a rejected or reference-only field is an error, not a silent drop.

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

### 8.3 Observation

One observation is one record with all of the following fields.

| Field | Type | Meaning |
|---|---|---|
| `t_s` | float64 | Time of validity, in the time base of the sequence |
| `kind` | enum | `position` or `constraint:<name>` |
| `z` | [m] float64 | Measured value |
| `unit` | str | Unit of `z` |
| `frame` | str | Frame in which `z` is expressed (`w` or `b`) |
| `R_meas` | [m, m] float64 | Measurement covariance; symmetric positive definite |
| `source` | enum | `simulated_from_reference`, `dataset_sensor` or `pseudo_measurement` |
| `valid` | bool | False if the record must not be used |
| `provenance` | record | Generator name, configuration hash and random seed |

Rules:

- The filter receives `observations`: the list of valid observations whose `t_s`
  lies in `(t_k, t_{k+1}]` and that are available under the outage mask. An empty
  list means no update.
- An observation is applied after the state has been propagated to `t_{k+1}`. This
  is an approximation: the measurement is treated as if it had been taken at
  `t_{k+1}`. The offset `t_{k+1} - t_s` is non-negative, smaller than one inertial
  sample period, and logged for every observation in diagnostics. A sensitivity check of the results to
  this offset is PLANNED before the first baseline evaluation.
- A simulated position update is produced only by a declared generator that reads
  the reference and adds seeded zero-mean noise. Its `source` is
  `simulated_from_reference`, its `R_meas` is the covariance of the generator noise,
  and it is unavailable during an outage. For the UAV track it is reported as a
  simulated absolute-position update, never as GNSS.
- A pseudo-measurement (`pseudo_measurement`) carries no value derived from the
  reference.
- The generator is the only path by which reference data can reach the filter after
  initialisation. A contract test verifies that every `simulated_from_reference`
  observation carries the provenance of the declared generator and that no other
  code path constructs one.
- Updates are reported per `source`. The run manifest holds separate counts for
  `simulated_from_reference`, `dataset_sensor` and `pseudo_measurement` updates,
  separate from the `init` label.

### 8.4 Initial state

Initialisation is not an observation. It is a separate record used once at `t_0`:
`p`, `v`, `q`, `b_g`, `b_a`, `P_0`, and `init` with the value `oracle` or
`operational`. The record and its `init` label are stored in the run manifest.

### 8.5 Sequence metadata

`id`, `platform` (`ugv` or `uav`), `calibration`, `source_hash` (SHA-256),
`split_id` (`train`, `validation`, `test_locked`).

### 8.6 Filter

```
Filter.init(initial_state)
Filter.step(gyro_rad_s, accel_m_s2, dt_s, observations)
    -> state (p, v, q, b_g, b_a), covariance [15, 15], diagnostics
```

Neither call has a reference argument.

### 8.7 EvaluationResult

`run_id, sequence_id, outage_id, metric, value, unit, reference_dof, status,
failure_reason, provenance`

## 9. Time validation and invalid samples

- `t_s` is float64 seconds from the first sample of the sequence.
- `dt_k` is the measured timestamp difference. A constant rate is never assumed.
- `dt_nominal` is the median of `dt_k` over the sequence.
- `dt_max = dt_max_factor x dt_nominal`. `dt_max_factor` is a configuration field,
  default 5, valid range (1, 20]. Default value: PROPOSED.

| Condition | Action |
|---|---|
| `dt_k <= 0` | The sequence is rejected. It is not repaired. |
| `0 < dt_k <= dt_max` | Propagation with the measured `dt_k`. |
| `dt_k > dt_max` | Gap. The filter does not integrate across it. The sequence is split into segments at the gap; each segment is an independent run with its own initial state (section 10). No interpolation. |
| Non-finite inertial sample | Treated as a gap at that sample; counted. Never interpolated silently. |

Every gap is counted in diagnostics and listed in the run manifest. An outage window
must lie entirely inside one segment; otherwise it is reported as N/A with the reason
`timestamp_gap`.

A segment is run only if both conditions hold:

- its duration is at least `pre_outage_init_s` plus the shortest outage duration
  configured for the platform (with the planned values: 15 s for the UGV, 10 s for
  the UAV);
- a valid initial state (section 8.4) can be formed at its first sample. For an
  oracle-initialised run this requires a valid reference sample within one inertial
  sample period of the segment start.

A segment that fails a condition is not run. It is reported as N/A with the reason
`segment_too_short` or `no_initial_state`. No segment is started silently.

Audit reference for KITTI drive `2011_09_30_drive_0016`: 2967 samples, `dt` 9.658 to
10.342 ms, no non-increasing timestamp, no gap above 15 ms.

## 10. Initialisation and leakage safeguards

- Oracle-initialised baseline: `p`, `v`, `q` at `t_0` are taken from the reference.
  After `t_0` the reference reaches the filter only through simulated position
  updates (section 8.3). Every result produced this way carries the label
  `init: oracle`.
- An oracle-initialised result is not evidence of operational GNSS-denied navigation.
- Initial biases are set to zero. This is an initial estimate, not a statement that
  the true biases are zero.
- Operational initialisation (no reference at `t_0`) is outside this version of the
  contract.
- The allowlist of section 8.1 is enforced in the loader.

### 10.1 Initial covariance

`P_0` is diagonal, in the error-state order of section 5.1:

```
P_0  = blockdiag(D(s_p), D(s_v), D(s_theta), D(s_bg), D(s_ba))
D(s) = s^2 I3                        if s is a scalar
D(s) = diag(s_x^2, s_y^2, s_z^2)     if s is a length-3 vector
```

| Config field | Symbol | Unit | Valid range |
|---|---|---|---|
| `p0.pos_std_m` | `s_p` | m | (0, 100] |
| `p0.vel_std_m_s` | `s_v` | m/s | (0, 10] |
| `p0.att_std_rad` | `s_theta` | rad | (0, pi] |
| `p0.gyro_bias_std_rad_s` | `s_bg` | rad/s | (0, 0.1] |
| `p0.accel_bias_std_m_s2` | `s_ba` | m/s^2 | (0, 1] |

Each field is a scalar or a length-3 vector. Validation: finite, strictly positive,
inside the valid range; the assembled `P_0` is symmetric positive definite. A value
outside the range is a configuration error, not a warning. The ranges apply to each
component. They are provisional configuration checks (PROPOSED): neither tuned values
nor physical limits. Numerical values per dataset: OPEN; selected on
training and validation data only and recorded in the run manifest.

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
  The order `Rz Ry Rx` is documented from the pykitti source (commit `de33723`).
  pykitti is not claimed to be identical to the official development kit.
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
| Noise density values per platform | OPEN |
| `P_0` values per dataset | OPEN |
| `dt_max_factor` default (5), minimum segment rule and `P_0` configuration checks | PROPOSED |
| Sensitivity of results to the delayed-observation approximation | PLANNED |
| Order of roll and pitch rotations in the KITTI reference orientation | Documented from source: pykitti `pose_from_oxts_packet` computes `R = Rz Ry Rx` (`pykitti/utils.py`, commit `de33723`). Not observable numerically on the audited drive. The official development kit file was not inspected. |
| Residuals of the levelled-frame check (0.012 m/s^2, 0.078 m/s rms) | OPEN |
| Second KITTI drive with turns and stops | OPEN; does not block the freeze |

## 13. Contract tests

Status: IMPLEMENTED in `tests/fast/` (four files). The freeze requires them to pass.
Tolerances and their sensitivity to step and perturbation size are reported in
`docs/notes/contract_test_tolerances.md`.

| Test | Checks | File |
|---|---|---|
| Quaternion convention | Hamilton product, `[w, x, y, z]` order, `R(q)` maps `b` to `w`; zero-norm and non-finite quaternions rejected | `test_conventions.py`, `test_contract_edges.py` |
| Exponential map | `Exp` and its inverse agree from 0 to `pi - 1e-10` rad; small-angle behaviour | `test_conventions.py`, `test_contract_edges.py` |
| Error definition | `R_true = R_hat Exp([dtheta]x)` recovered by injection on the right, not on the left | `test_conventions.py`, `test_contract_oracles.py` |
| Jacobians | `F` and `G` of section 5.3 against an analytic block oracle and against finite differences of an independent model, at zero and at small non-zero error states | `test_conventions.py`, `test_contract_oracles.py` |
| Reset | `G_r` against a finite-difference reset from an independent SO(3) reference at small non-zero rotations; `P` stays symmetric positive definite | `test_conventions.py`, `test_contract_oracles.py` |
| Joseph update | `P` symmetric positive definite after update, including ill-conditioned covariances and a suboptimal gain | `test_conventions.py`, `test_contract_edges.py` |
| Noise convention | `Q_d` units and scaling with `dt_k` | `test_conventions.py` |
| Time validation | Rejection, gap splitting, segment-duration boundary, missing initial state and N/A outage windows of section 9 | `test_data_contract.py`, `test_contract_edges.py` |
| Initial covariance | Block order, range and positive-definiteness validation of section 10.1 | `test_conventions.py`, `test_contract_edges.py` |
| Leakage | Only allowlisted fields reach the filter; rejected and reference-only fields are refused; the reference object is never passed | `test_data_contract.py`, `test_contract_edges.py` |
| Observation provenance and timing | `simulated_from_reference` observations come only from the declared generator; per-source update counts; offset `t_{k+1} - t_s` in `[0, dt_k)` | `test_data_contract.py`, `test_contract_edges.py` |

The tests use synthetic values only. They verify the conventions and their
implementation in `nkf/filters/conventions.py` and `nkf/data/sequence.py`; they are not
evidence about any dataset or about filter performance. `propagate_nominal` is a single
step used by the Jacobian tests; it is not a strapdown implementation over data.
