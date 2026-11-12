# Error-state Kalman filter: theory note

Status: DRAFT for review. Written 12 November 2026. Related issue: #2.

This note collects, in one place, the model behind the filter. Sections 1 to 6 restate
and derive what `docs/interfaces.md` fixes (ADR-0001); where the two differ, the
contract is authoritative. Section 7 adds the measurement models for the motion
constraints, which the contract leaves to this note. Section 9 states which equations
are checked by automated tests and which are not.

## 1. Setting

The filter estimates position, velocity and orientation of a body from an inertial
sensor. It corrects the estimate whenever an observation is available: a position
update outside an outage, or a motion constraint treated as a pseudo-measurement.
Without observations the estimate drifts; the constraints are what limit the drift
during an outage.

## 2. Notation

| Symbol | Meaning | Unit |
|---|---|---|
| `w`, `b` | World frame (right-handed, z up) and body frame (inertial sensor frame) | - |
| `p`, `v` | Position and velocity of the body origin, expressed in `w` | m, m/s |
| `q`, `R` | Orientation: Hamilton quaternion `[w, x, y, z]` and `R = R(q) = R_wb`, body to world | - |
| `b_g`, `b_a` | Gyroscope and accelerometer bias, expressed in `b` | rad/s, m/s^2 |
| `w_m`, `f_m` | Measured angular rate and specific force, expressed in `b` | rad/s, m/s^2 |
| `g_w` | Gravity, `[0, 0, -9.81]` | m/s^2 |
| `[u]x` | Skew-symmetric matrix of `u`: `[u]x a = u x a` | - |
| `Exp`, `Log` | SO(3) exponential map and its inverse | - |
| `x_hat` | Nominal (estimated) value of `x` | - |

## 3. Inertial navigation model

The gyroscope and the accelerometer measure the true quantities plus a slowly varying
bias and white noise:

```
w_m = w + b_g + n_g
f_m = f + b_a + n_a
```

The accelerometer measures specific force, that is, acceleration minus gravity. A
body at rest and level therefore reads `[0, 0, +9.81]`. The kinematics are

```
p_dot   = v
v_dot   = R (f_m - b_a - n_a) + g_w
R_dot   = R [w_m - b_g - n_g]x
b_g_dot = n_bg
b_a_dot = n_ba
```

Assumptions: the world frame is treated as inertial (Earth rotation and Coriolis
terms neglected) and gravity is constant. Both are acceptable for runs of tens of
seconds to a few minutes over a few hundred metres, and are limitations beyond that.

Discrete nominal propagation holds the measurement of sample `k` over the measured
interval `dt_k`:

```
w = w_m[k] - b_g_hat          a = R_k (f_m[k] - b_a_hat) + g_w
R_{k+1} = R_k Exp(w dt_k)
v_{k+1} = v_k + a dt_k
p_{k+1} = p_k + v_k dt_k + 0.5 a dt_k^2
```

This scheme has an error of first order in `dt`. On a synthetic 300 m circle with
exact measurements it gives 0.31 m at 100 Hz and 0.16 m at 200 Hz.

## 4. Error state

The filter does not estimate the state directly. It keeps a nominal state and
estimates the small error between truth and nominal:

```
p_true   = p_hat + dp
v_true   = v_hat + dv
R_true   = R_hat Exp([dtheta]x)        orientation error on the right, expressed in b
b_g_true = b_g_hat + db_g
b_a_true = b_a_hat + db_a
```

`dx = [dp, dv, dtheta, db_g, db_a]` has 15 components. The orientation error is a
3-vector, so the covariance has no redundant dimension, unlike a 4-component
quaternion error.

### 4.1 Derivation of the error dynamics

Write `f_hat = f_m - b_a_hat` and `w_hat = w_m - b_g_hat`. The nominal state follows
the kinematics with zero noise.

Velocity. With `R_true = R_hat (I + [dtheta]x)` to first order,

```
v_true_dot = R_hat (I + [dtheta]x)(f_hat - db_a - n_a) + g_w
dv_dot     = v_true_dot - v_hat_dot
           = R_hat [dtheta]x f_hat - R_hat db_a - R_hat n_a      (second-order terms dropped)
           = -R_hat [f_hat]x dtheta - R_hat db_a - R_hat n_a
```

using `[a]x b = -[b]x a`.

Orientation. Differentiating `R_true = R_hat Exp([dtheta]x)` and equating with
`R_true [w_hat - db_g - n_g]x` gives, to first order,

```
dtheta_dot = -[w_hat]x dtheta - db_g - n_g
```

Position and biases follow directly: `dp_dot = dv`, `db_g_dot = n_bg`,
`db_a_dot = n_ba`.

### 4.2 Matrix form

`dx_dot = F dx + G n` with `n = [n_g, n_a, n_bg, n_ba]`:

```
         dp    dv    dtheta            db_g   db_a
dp     [ 0     I     0                 0      0      ]
dv     [ 0     0    -R_hat [f_hat]x    0     -R_hat  ]
dtheta [ 0     0    -[w_hat]x         -I      0      ]
db_g   [ 0     0     0                 0      0      ]
db_a   [ 0     0     0                 0      0      ]

         n_g   n_a      n_bg   n_ba
dp     [ 0     0        0      0  ]
dv     [ 0    -R_hat    0      0  ]
dtheta [-I     0        0      0  ]
db_g   [ 0     0        I      0  ]
db_a   [ 0     0        0      I  ]
```

Reading the matrix: the block `-R_hat [f_hat]x` is how an orientation error turns the
measured specific force (mostly the reaction to gravity) into a velocity error. This
coupling is why tilt errors and accelerometer biases dominate inertial drift.

## 5. Noise and discretisation

`n` is white with `E[n(t) n(s)^T] = Q_c delta(t - s)` and
`Q_c = blockdiag(sigma_g^2 I, sigma_a^2 I, sigma_bg^2 I, sigma_ba^2 I)`. The
configuration stores the densities `sigma` in rad/s/sqrt(Hz), m/s^2/sqrt(Hz),
rad/s^2/sqrt(Hz) and m/s^3/sqrt(Hz). Over one step,

```
Phi = I + F dt_k            Q_d = G Q_c G^T dt_k            P <- Phi P Phi^T + Q_d
```

The EuRoC dataset states its densities (`docs/data_manifest.md`). KITTI does not;
values for KITTI are OPEN and will be chosen on training and validation data only.

## 6. Update, injection and reset

For an observation `z = h(x) + n_z` with covariance `R_z`:

```
y = z - h(x_hat)            H = dh/d(dx) at dx = 0
S = H P H^T + R_z           K = P H^T S^-1          dx_hat = K y
P <- (I - K H) P (I - K H)^T + K R_z K^T            (Joseph form)
```

The Joseph form keeps `P` symmetric and positive definite for any gain, at the cost of
a few more products. The error estimate is then injected into the nominal state,

```
p_hat <- p_hat + dp_hat        v_hat <- v_hat + dv_hat
q_hat <- q_hat (x) q{dtheta_hat}        (increment on the right, then normalise)
b_g_hat <- b_g_hat + db_g_hat   b_a_hat <- b_a_hat + db_a_hat
```

and the error is reset to zero. Because the orientation error is defined relative to
the nominal orientation, changing the nominal changes the error coordinates. To first
order the new error is `dtheta_new = (I - 0.5 [dtheta_hat]x)(dtheta - dtheta_hat)`,
so

```
P <- G_r P G_r^T,      G_r = blockdiag(I6, I3 - 0.5 [dtheta_hat]x, I6)
```

## 7. Measurement models

Each model gives the predicted measurement `h(x)`, the observed value `z`, the Jacobian
`H` with respect to the error state and the conditions under which the model holds.

### 7.1 Position update

```
z = p + n                h(x) = p                H = [I3, 0, 0, 0, 0]
```

Expressed in `w`, in metres. Available only outside an outage. In this project the
values come from a declared generator that adds seeded noise to the reference; they
are reported as simulated position updates.

### 7.2 Non-holonomic constraint (ground vehicle)

A wheeled vehicle that does not slip moves along its forward axis. Its velocity
expressed in the body frame, `v_b = R^T v`, has no lateral and no vertical component.
With `S = [[0, 1, 0], [0, 0, 1]]` selecting those two components (body y to the left,
z up):

```
z = [0, 0]^T                         h(x) = S R^T v
```

Jacobian. With the error definitions of section 4,

```
v_b_true = Exp(-[dtheta]x) R_hat^T (v_hat + dv)
         = v_b_hat + R_hat^T dv + [v_b_hat]x dtheta          (first order)
```

so

```
H_NHC = S [ 0,  R_hat^T,  [v_b_hat]x,  0,  0 ]          y = -S v_b_hat
```

The measurement covariance is `R_NHC = diag(s_lat^2, s_vert^2)` in (m/s)^2. Values:
OPEN.

Validity. The model assumes:

- no side slip: violated in fast turns, on wet or loose ground, and during hard
  braking;
- no vertical body velocity: violated by suspension motion and on rough ground;
- the body frame is aligned with the vehicle and the velocity refers to a point on
  the rear axle. The mounting rotation and the lever arm of the KITTI unit are OPEN
  in the contract. With a lever arm `r` from that point to the sensor, the constraint
  applies to `v_b - [w_hat]x r` instead of `v_b`; the term is neglected here.

When the constraint is applied while it does not hold, it pulls the estimate towards a
wrong value with unwarranted confidence. This is the failure the project targets: a
network is to estimate, from recent inertial data only, how reliable the constraint
is, and the filter is to enlarge `R_NHC` or skip the update accordingly. That part is
planned and not implemented.

### 7.3 Zero-velocity update (ground vehicle)

When the vehicle stands still, its velocity is zero:

```
z = [0, 0, 0]^T          h(x) = v          H_ZUPT = [ 0, I3, 0, 0, 0 ]          y = -v_hat
```

with `R_ZUPT = s_zupt^2 I3` in (m/s)^2. Values: OPEN.

Validity. The update is correct only while the vehicle is actually stationary, so it
depends on a detector. The detector must be causal and must use the inertial data
only: typically the variance of the specific-force norm and the angular-rate norm over
a short window, compared with thresholds. Thresholds and window length: OPEN.

Two failure cases matter. A false detection during motion (for example at constant
speed on a smooth road, where the accelerometer shows little variation) forces the
velocity to zero and corrupts the state. A missed detection only loses a correction.
The detector is therefore tuned to avoid false detections first.

### 7.4 Aerial vehicle

No general motion constraint holds for a multirotor. Conditional priors for hover or
small vertical velocity are a candidate, to be accepted only where an inertial-only
detector is defensible. No model is written here; the decision is scheduled for W06,
with an unconstrained aerial filter as the fallback.

## 8. What can and cannot be observed

The statements below are standard results for inertial navigation and are expected to
hold here. They have not been verified by an analysis in this project.

- With no observation, every error grows; position error grows fastest.
- A position update makes position and velocity observable, and orientation and
  biases gradually, depending on the motion.
- The non-holonomic constraint observes two velocity components in the body frame. It
  slows the drift during an outage but does not make absolute position or heading
  observable.
- A zero-velocity update observes velocity. At standstill, a horizontal accelerometer
  bias and a tilt error produce the same measurement and cannot be separated; heading
  remains unobservable.

## 9. Verification status

| Item | Checked by | Status |
|---|---|---|
| Quaternion and rotation conventions | `tests/fast/test_conventions.py`, `test_contract_oracles.py` | Automated |
| `F`, `G` (section 4.2) | Analytic oracle and finite differences of an independent model | Automated |
| Discretisation and linearisation error orders | `docs/notes/contract_test_tolerances.md` | Automated |
| Joseph update, injection, reset (section 6) | Independent SO(3) reference, ill-conditioned covariances | Automated |
| Conventions on real data | KITTI strapdown run; EuRoC pre-audit | One drive, one flight |
| `H_NHC` (section 7.2) | Finite differences, one state, largest difference 9e-10; a sign change in the orientation block is detected | Checked once by hand; no automated test, no implementation |
| `H_ZUPT` (section 7.3) | Linear in the state | No implementation |
| Stationary detector | - | Not designed |
| Filter with constraints on data | - | Not run |

## 10. Open items

- Values of `R_NHC`, `R_ZUPT`, the detector thresholds and the KITTI noise densities.
- Mounting rotation and lever arm of the KITTI inertial unit.
- Whether the KITTI inertial channels are raw or corrected by the unit.
- Integration scheme: the zero-order hold is the frozen baseline; a midpoint scheme
  needs its own decision record and a paired comparison.
- Aerial constraints (section 7.4).
- Review of this note by the data and classical-filter owner.
