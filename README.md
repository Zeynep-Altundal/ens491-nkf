# ens491-nkf

Cross-platform, constraint-aware neural Kalman filtering for GNSS-denied navigation.
ENS491 senior project, Sabanci University. Phase I: 9 November 2026 - 1 February 2027.

## Question

Can a small network learn when motion constraints are reliable and reduce error-state
Kalman filter drift during GNSS outages, while the filter stays model-based and
well calibrated?

## Scope

| Track | Dataset | Position updates | Constraints |
|---|---|---|---|
| UGV | KITTI raw (OXTS) | Removed during simulated outages | Non-holonomic, zero-velocity |
| UAV | EuRoC MAV | Synthesised from ground truth; reported as simulated absolute-position update outages, never as GNSS | Flight-regime (conditional) |

Models are trained separately per platform in Phase I. No hardware and no flights.

## Status

Status is reported only from accepted evidence. See `docs/advisor_dashboard.md`
and `docs/research_status.md`. Initial state: NOT RECORDED.

## Quick start

```bash
conda env create -f environment.yml
conda activate nkf
make lint
make test
```

## Repository map

| Path | Content |
|---|---|
| `nkf/` | Python package: data, baselines, filters, models, train, eval, infra, figures |
| `configs/` | Platform, method, outage and split configuration |
| `tests/` | `fast/` runs on every pull request; `slow/` needs real data |
| `docs/` | Problem statement, interface contract, decisions, weekly reports, milestones |
| `paper/` | Claim registry and manuscript sources |
| `results/` | Run registry and generated exports; large payloads stay outside Git |
| `scripts/` | Repository set-up and reproduction entry points |

## Working rules

See `CONTRIBUTING.md`. Every reported number traces to a configuration file and a run id.
