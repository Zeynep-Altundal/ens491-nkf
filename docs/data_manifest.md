# Data manifest

Machine-readable tables for the audits below are in `data/manifests/` (`sources.csv`,
`fields.csv`, `timebase.csv`, `reference_quality.csv`). They hold metadata only. The
text in this document is the audit record; the tables are checked against the loader and
the allowlists by `tests/fast/test_data_manifests.py`.

## KITTI raw, drive 2011_09_30_drive_0016 (extract)

Status: read-only audit of one drive. Not a split decision. Audited 12 November 2026.
Related issue: #10.

### Source

| Item | Value |
|---|---|
| Base URL | https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data |
| Drive archive | `2011_09_30_drive_0016_extract.zip`, 1 724 245 728 bytes |
| SHA-256 | `f610151241c027be8d29e9b55876021673e4431790f5a74b94bbdd5bb1686434` |
| Calibration archive | `2011_09_30_calib.zip`, 4 073 bytes |
| SHA-256 | `2ed01563b616baec14f25659671bd9a77810bd87752d709d184fe304ff7258e3` |
| Local storage | Outside the repository; only `oxts/` extracted |

The unsynchronised `extract` variant is used because it carries OXTS samples at the
native rate. The `sync` variant was not downloaded.

### Time base

| Quantity | Value |
|---|---|
| Samples | 2967 |
| Duration | 29.659 s |
| dt mean / std | 0.01000 s / 0.000081 s |
| dt min / max | 0.009658 s / 0.010342 s |
| Non-increasing timestamps | 0 |
| Gaps above 15 ms | 0 |

### Field classification (30 fields per sample, all samples)

| Fields | Content | Use |
|---|---|---|
| `ax ay az`, `wx wy wz` | Acceleration and angular rate in the vehicle frame (x front, y left, z top); m/s^2, rad/s | Admitted as inertial input |
| `af al au`, `wf wl wu` | The same quantities expressed in an earth-surface-parallel frame | Not admitted: depends on the OXTS attitude solution |
| `lat lon alt`, `roll pitch yaw`, `vn ve vf vl vu` | OXTS navigation solution | Reference and simulated position updates only; never a network or filter input during an outage |
| `pos_accuracy vel_accuracy navstat numsats posmode velmode orimode` | Quality indicators | Reference-quality audit only |

`dataformat.txt` labels the third acceleration field `ay` (it is `az`) and describes
`pos_accuracy` as a velocity accuracy. Fields are read by column order.

### Consistency checks

| Check | Observed | Expected |
|---|---|---|
| Mean accelerometer norm | 9.8119 m/s^2 | Local gravity magnitude |
| Mean `az` | +9.798 m/s^2 | Specific force: +g on the upward axis at near-level attitude |
| Mean `ay` | +0.4165 m/s^2 | g sin(mean roll) = 0.4175 m/s^2 (mean roll 0.04256 rad) |
| Mean `ax` | -0.0158 m/s^2 | Mean forward acceleration +0.111 minus g sin(mean pitch) 0.123 = -0.012 m/s^2 |
| `af - ax`, first sample | 0.1059 m/s^2 | g x pitch = 0.1086 m/s^2: `af` is attitude-derived |
| Heading, first sample | atan2(vn, ve) = 1.684 rad | `yaw` = 1.685 rad (0 = east, counter-clockwise) |

The accelerometer channels report specific force (gravity not removed).

### Motion content

Forward speed 12.41 to 15.71 m/s; `wz` within -0.015 to +0.021 rad/s. The drive is
close to straight, with no stop and no significant turn. It does not exercise
zero-velocity updates or cornering.

### Reference quality

| Indicator | Observed values |
|---|---|
| `pos_accuracy` | 0.100 to 0.155 m |
| `vel_accuracy` | 0.016 to 0.037 m/s |
| `navstat` | 4 |
| `numsats` | 5 to 11 |
| `posmode` | 4, 5, 6 |
| `velmode` | 0, 4, 5, 6 |
| `orimode` | 0, 1, 6 |

### Open items

- OPEN: meaning of the `navstat` and mode codes, and the time intervals in which the
  modes change.
- OPEN: whether the acceleration and angular-rate channels are raw sensor outputs or
  corrected by the unit's internal navigation filter. Input and reference originate
  from the same OXTS unit; this coupling limits independent ground-truth claims.
- OPEN: lever arm and mounting rotation between the OXTS unit and the vehicle frame.
- NOT RECORDED: EuRoC audit; any second KITTI drive.

### Numerical check of the reference orientation (12 November 2026)

Body measurements rotated by roll and pitch are compared with the levelled-frame
fields over all 2967 samples. The levelled-frame fields are used for this check only;
they remain rejected as inputs.

| Variant | Specific force residual, rms / max [m/s^2] | Angular rate residual, rms / max [rad/s] |
|---|---|---|
| `Ry(pitch) Rx(roll)` | 0.01233 / 0.05219 | 0.00002 / 0.00015 |
| `Rx(roll) Ry(pitch)` | 0.01235 / 0.05227 | 0.00002 / 0.00016 |
| `Ry(-pitch) Rx(roll)` | 0.25051 / 0.34253 | 0.00048 / 0.00264 |
| `Ry(pitch) Rx(-roll)` | 0.85301 / 1.52638 | 0.00128 / 0.00550 |

| Yaw variant, `[ve, vn]` against `Rz(yaw) [vf, vl]` | Residual, rms / max [m/s] |
|---|---|
| `Rz(+yaw)` | 0.07753 / 0.23016 |
| `Rz(-yaw)` | 27.92687 / 31.21452 |

Findings:

- VERIFIED: sign and axis of roll and pitch; sign of yaw.
- NOT OBSERVABLE on this drive: the order of the roll and pitch rotations. Mean roll
  0.043 rad and mean pitch 0.013 rad make the two orders differ by less than the
  residual floor.
- OPEN: the specific-force residual (0.012 m/s^2 rms) and the velocity residual
  (0.078 m/s rms) are not explained by the rotation model. The angular-rate residual
  is at numerical level.

## EuRoC MAV, flight V1_01_easy (ASL format)

Status: read-only pre-audit of one training flight. Not a split decision. No filter
was run. The locked test flight `V1_03` was not opened. Audited 12 November 2026.
Related issue: #1.

### Source

| Item | Value |
|---|---|
| Landing page | https://doi.org/10.3929/ethz-b-000690084 (ETH Research Collection) |
| Licence | In Copyright - Non-Commercial Use Permitted |
| Archive | `vicon_room1.zip`, 6 042 263 426 bytes |
| SHA-256 | `fe73c27be6dc8ac00493b78b750d36b144daf49eea7fdf3163e934527c1b5297` |
| Member used | `vicon_room1/V1_01_easy/V1_01_easy.zip`, 1 149 702 102 bytes |
| SHA-256 | `a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa` |
| Extracted | `mav0/imu0/`, `mav0/vicon0/`, `mav0/state_groundtruth_estimate0/`, `mav0/body.yaml` |
| Local storage | Outside the repository; cameras and point cloud not extracted |

### Fields

| Stream | Columns | Use |
|---|---|---|
| `imu0/data.csv` | `timestamp [ns]`, `w_RS_S_{x,y,z} [rad s^-1]`, `a_RS_S_{x,y,z} [m s^-2]` | Inertial input |
| `state_groundtruth_estimate0/data.csv` | `timestamp`, `p_RS_R [m]`, `q_RS [w, x, y, z]`, `v_RS_R [m s^-1]`, `b_w_RS_S [rad s^-1]`, `b_a_RS_S [m s^-2]` | Reference only, including the two bias estimates |
| `vicon0/data.csv` | `timestamp [ns]`, `p_RS_R [m]`, `q_RS [w, x, y, z]` | Not used; raw motion-capture pose with a non-identity extrinsic |

Metadata from `imu0/sensor.yaml`: ADIS16448, 200 Hz, sensor-to-body transform equal to
the identity (the body frame is the inertial sensor frame). Noise parameters stated by
the dataset: gyroscope noise density 1.6968e-4 rad/s/sqrt(Hz), gyroscope random walk
1.9393e-5 rad/s^2/sqrt(Hz), accelerometer noise density 2.0e-3 m/s^2/sqrt(Hz),
accelerometer random walk 3.0e-3 m/s^3/sqrt(Hz).

Metadata from `state_groundtruth_estimate0/sensor.yaml`: the reference is the
non-linear least-squares batch solution over the motion-capture pose and the inertial
measurements, including time-offset estimation; identity extrinsic.

### Numerical checks

Reproduced by `scripts/audit_euroc.py --mav0 <flight>/mav0`. Each reference sample is
paired with the nearest inertial sample.

| Check | Result |
|---|---|
| Inertial stream | 29 120 samples, 145.595 s; dt mean 5.0000 ms, std 0.0001 ms, min 4.9999 ms, max 5.0002 ms; 0 non-increasing; 0 gaps |
| Reference stream | 28 712 samples, 143.555 s; dt mean 5.0000 ms, std 0.0002 ms, min 4.9997 ms, max 5.0002 ms; 0 non-increasing; 0 gaps |
| Overlap | Reference starts 1.04 s after and ends 1.00 s before the inertial stream |
| Time-stamp pairing | Offset to the nearest inertial sample within +-256 ns, mean absolute 59 ns |
| Mean accelerometer norm | 9.8012 m/s^2 (bias not removed) |
| Reference quaternion norm | Largest deviation from 1: 2.1e-5 |
| Mean of `R(q) (a - b_a)` | [0.0032, -0.0121, 9.8080] m/s^2 |
| Mean of `R(q)^T (a - b_a)` | [-1.6714, -1.4639, 5.5266] m/s^2 |
| Gyroscope against reference orientation, 6000 steps, rms angle per step | Right-multiplied, sample k held: 2.67e-4 rad; left-multiplied: 2.86e-3 rad |
| Same, other choices of the rate | Sample k+1 held: 2.66e-4 rad; mean of samples k and k+1: 1.37e-4 rad |
| Change of the gyroscope reading between samples | rms 0.0914 rad/s |
| Reference bias estimates, first sample | Gyroscope [-0.0022, 0.0207, 0.0764] rad/s; accelerometer [-0.0125, 0.5477, 0.0691] m/s^2; nearly constant over the flight |

### Findings

- VERIFIED: the reference quaternion is scalar-first and maps the body frame to the
  reference frame (`R_wb`); the reference frame is gravity-aligned with z up; the
  accelerometer measures specific force and the gravity magnitude is 9.81 m/s^2 within
  0.002 m/s^2.
- VERIFIED: integrating the gyroscope with the increment on the right agrees with the
  reference orientation about ten times better than with the increment on the left.
- VERIFIED: units, rate, monotonic time stamps, no gaps; the body frame is the inertial
  sensor frame and is not front-left-up (first sample: specific force mainly along the
  sensor x axis).
- The reference frame is the motion-capture frame of the dataset. It has no geographic
  heading and is not ENU. A contract change is proposed in ADR-0002.

### Limitations

- The pairing within 256 ns shows that the two streams use one time-stamp
  representation. It does not show that the physical time offset between the inertial
  sensor and the reference is zero; the reference solution itself includes an estimated
  time offset.
- The reference is not independent of the inertial input: it is a batch solution that
  uses the same inertial measurements.
- The reference bias estimates are reference-only fields. Used in a run, they make it
  a diagnostic, not an independent result.
- An initial state from the reference exists only from 1.04 s after the first inertial
  sample.
- The per-step gyroscope residual is larger than the stated white noise explains. It
  halves when two consecutive samples are averaged, which is consistent with
  high-frequency content in the angular rate; this does not establish its cause and
  does not show that another integration scheme reduces trajectory error.
- One flight only. OPEN: all other flights; `P_0` values.
