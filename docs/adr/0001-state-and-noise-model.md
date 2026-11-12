# ADR-0001: State, noise model and interface contract

Status: PROPOSED. Not accepted. The interface contract is not frozen.
Date proposed: 12 November 2026.
Related: issue #8, pull request #18, issues #16, #17, #19.

## Context

The project needs one mathematical and data contract shared by the classical
reference filter, the differentiable filter, the data loaders and the evaluation code,
for a ground-vehicle track (KITTI raw, OXTS) and an aerial track (EuRoC MAV). Without
a fixed convention for frames, quaternion order, error definition, noise units and
reference handling, results from different modules cannot be compared and a sign or
frame error can stay hidden behind plausible plots.

## Decision

`docs/interfaces.md` is the contract. On acceptance of this ADR the following become
fixed; any later change requires a new ADR, a migration note and a contract test.

| Topic | Decision | Contract section |
|---|---|---|
| World frame | Local ENU, origin at the first valid reference sample | 1 |
| Body frame | The frame in which the inertial measurements are expressed; not assumed front-left-up | 1 |
| Nominal state | `p`, `v`, `q`, `b_g`, `b_a`; Hamilton quaternion `[w, x, y, z]`; `R(q) = R_wb` | 2 |
| Inertial model | Specific force; `v_dot = R (f_m - b_a) + g_w`; Earth rotation and Coriolis neglected | 3 |
| Nominal propagation | Measurement held constant over the measured `dt_k` | 4 |
| Error state | 15 dimensions, order `[dp, dv, dtheta, db_g, db_a]`; right-multiplicative orientation error; bias error true minus nominal | 5.1 |
| Jacobians and noise | `F` [15, 15], `G` [15, 12], noise order `[n_g, n_a, n_bg, n_ba]`; configuration stores noise densities | 5.3, 5.4 |
| Covariance propagation | `Phi = I + F dt`, `Q_d = G Q_c G^T dt`, symmetrised | 5.5 |
| Update | Joseph form, then injection on the right, then first-order reset | 6, 7 |
| Inertial input | Allowlist: time, angular rate, specific force | 8.1 |
| Reference | Separate evaluation-only object; reaches the filter only through the declared generator of simulated position updates | 8.2, 8.3 |
| Observation | Record with time, kind, value, unit, frame, covariance, source, validity, provenance | 8.3 |
| Initial state | Separate record with an `init` label (`oracle` or `operational`) | 8.4, 10 |
| Time | float64 seconds; measured `dt_k`; rejection of non-increasing time; gap splitting at `dt_max`; no interpolation | 9 |
| Initial covariance | Diagonal blocks in error-state order with validated configuration fields | 10.1 |

## Options considered

| Choice | Alternative | Reason for the decision |
|---|---|---|
| Right-multiplicative (body-frame) orientation error | Left-multiplicative (world-frame) error | Matches body-frame inertial measurements and bias states; one convention for both platforms |
| Body frame defined as the inertial measurement frame | Front-left-up assumed for both platforms | The aerial dataset is not assumed front-left-up; avoids a hidden frame conversion |
| Noise densities in configuration | Per-sample standard deviations | Independent of the sampling rate (100 Hz and 200 Hz in the two datasets) |
| Split the sequence at a timestamp gap | Integrate across the gap; interpolate | No silent repair; unsupported outage windows are reported as N/A |
| First-order transition and reset | Higher-order forms | Sufficient as the reference; the error order is measured and asserted by tests |

## Not fixed by this ADR

The following stay OPEN or PROPOSED in section 12 of the contract and are closed by
later records: noise density values per platform; `P_0` values per dataset; the
`dt_max_factor` default and the `P_0` configuration checks; the `g_w` magnitude; the
geodetic-to-ENU method and its difference from the pykitti projection; KITTI OXTS
mounting rotation and lever arm; whether the KITTI inertial channels are raw or
corrected by the unit; the unexplained residuals of the levelled-frame check; all
EuRoC-specific frame, unit and time-alignment statements.

## Evidence

| Item | Reference |
|---|---|
| Contract text | `docs/interfaces.md` at commit `c09252b` (head of pull request #18) |
| KITTI OXTS audit | `docs/data_manifest.md`, commit `32eadaa`; orientation check, commit `72948df` |
| Reference implementation | `nkf/filters/conventions.py`, `nkf/data/sequence.py` |
| Contract tests | `tests/fast/`, 152 tests, all passing on `c09252b` |
| CI | `pr-lint`, `pr-tests`, `pr-title` successful on `c09252b` |
| Tolerances | `docs/notes/contract_test_tolerances.md` |

Accepted tolerances are those stated in the tolerance note, for the stated step and
perturbation sizes only. The relative tolerance of the positive-definiteness check is
a numerical acceptance criterion, not a proof.

## Scope limits

- All tests use synthetic values. No filter has been run on any dataset.
- `propagate_nominal` is a single step used for verification. It is not a strapdown
  implementation over data.
- Orientation corrections above 0.075 rad are outside the tested range of the
  first-order reset.
- The KITTI evidence comes from one near-straight drive without stops
  (`2011_09_30_drive_0016`).

## Risks

| Risk | Record |
|---|---|
| The repository is temporarily public | #16 |
| Branch protection is applied with administrator bypass | #17 |
| Commit `32eadaa` was pushed without review | #19 |
| The independent oracle does not call the production helpers but has the same author as the production code | Human review, gate G2 |
| EuRoC statements in the contract are unverified | Gate G1 |

## Acceptance gates

| Gate | Requirement | Status |
|---|---|---|
| G1 | Pre-audit of one EuRoC training flight (`V1_01_easy`): archive integrity, sensor metadata, time stamps, units, initialisation feasibility. The locked test flight is not opened. No filter is run. | NOT RECORDED |
| G2 | Independent human review of the contract, code, tests and scope by a team member; substantive comments resolved | NOT RECORDED |
| G3 | This record | PROPOSED |
| Final check | Clean working tree, green CI on the final commit, and the same commit referenced here | NOT RECORDED |

## Consequences

On acceptance: the conventions above are fixed; the strapdown and filter
implementations may start and must pass the contract tests; a convention change needs
a new ADR. Until acceptance: no implementation may rely on the contract, pull request
#18 is not merged, and no filter is run on real data.

## Approval

| Role | Decision | Date |
|---|---|---|
| Lead | NOT RECORDED | |
| Data/classical owner | NOT RECORDED | |
| Neural/infrastructure owner | NOT RECORDED | |
| Final frozen commit | NOT RECORDED | |
