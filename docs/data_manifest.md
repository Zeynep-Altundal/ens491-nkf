# Data manifest

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
