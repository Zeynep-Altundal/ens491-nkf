# ADR-0002: Platform-specific realisation of the world frame

Status: PROPOSED. Not accepted. The aerial track is not activated.
Date proposed: 12 November 2026.
Related: ADR-0001, issue #1, `docs/data_manifest.md` (EuRoC `V1_01_easy` pre-audit).

ADR-0001 stays as accepted. This record changes the contract only from its own
acceptance onwards.

## Context

ADR-0001 fixed the world frame as local ENU with the origin at the first valid
reference sample. That holds for KITTI, whose reference is geodetic. The EuRoC
pre-audit shows that the EuRoC reference is expressed in the motion-capture frame of
the dataset: gravity-aligned with z up (mean of `R(q)(a - b_a)` = [0.0032, -0.0121,
9.8080] m/s^2) but without a geographic heading. The filter mathematics of the
contract does not need a geographic frame; it needs one consistent, gravity-aligned
world frame for measurements, state and reference.

## Decision

Section 1 of `docs/interfaces.md` defines the world frame in two layers.

| Layer | Definition |
|---|---|
| Shared (all platforms) | Right-handed, fixed to the ground, z up along the local vertical; `g_w = [0, 0, -9.81]` m/s^2; `R(q) = R_wb` maps body to world |
| UGV, KITTI | Local ENU (x East, y North, z Up), origin at the first valid reference sample. Unchanged from ADR-0001. |
| UAV, EuRoC | The reference frame of the dataset (`R` in `p_RS_R`, `q_RS`, `v_RS_R`): gravity-aligned, z up, heading not geographic. No conversion is applied. |

Further statements for the aerial track:

- The body frame is the inertial sensor frame of the dataset (identity extrinsic). It
  is not front-left-up.
- Inertial input: `timestamp`, `w_RS_S`, `a_RS_S` of `imu0`. Reference-only: every
  column of `state_groundtruth_estimate0`, including the bias estimates `b_w_RS_S` and
  `b_a_RS_S`, and every column of `vicon0`.
- A run starts at the first valid reference sample inside the inertial stream, not at
  the first inertial sample.
- Initial noise densities are the values stated by the dataset (`imu0/sensor.yaml`);
  they are starting values, not tuned values.

## Planned first aerial baselines (not authorised by this record)

| Label | Definition |
|---|---|
| `oracle-init / zero-bias / no-update` | Oracle position, velocity and orientation at the first valid reference sample; zero initial bias estimate; no measurement update; zero-order hold |
| `reference-bias diagnostic` | As above, with the initial bias estimates of the reference. Diagnostic only: never used for learning, model selection or as an independent result, and reported separately |

## Migration note

No change for the ground-vehicle track: local ENU satisfies the shared definition, and
the frozen statements, code and tests of ADR-0001 are unaffected. Code that assumes a
geographic heading of the world frame must state that it applies to the ground-vehicle
track only.

## Required before acceptance

| Item | Status |
|---|---|
| EuRoC pre-audit recorded and reproducible (`scripts/audit_euroc.py`) | Recorded in `docs/data_manifest.md` |
| EuRoC loader with the allowlist and a separate reference object | NOT RECORDED |
| Frame contract tests: gravity direction in the reference frame on a fixture; reference-only columns refused; run start at the first valid reference sample | NOT RECORDED |
| Review of this record | NOT RECORDED |

## Not decided here

- The integration scheme. The frozen zero-order hold stays the baseline. A midpoint or
  trapezoidal scheme needs its own ADR and a paired comparison on the same data.
- `P_0` values and any tuned noise values.
- Flights other than `V1_01_easy`; the split.

## Consequences

On acceptance: section 1 of the contract is replaced by the two-layer definition, the
EuRoC-specific statements above become part of the contract, and the aerial track may
be implemented. Until then no filter is run on EuRoC data.

## Approval

| Role | Decision | Date |
|---|---|---|
| Research design owner | NOT RECORDED | |
| Data/classical owner | NOT RECORDED | |
| Neural/infrastructure owner | NOT RECORDED | |
