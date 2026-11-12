# Contributing

## Branches

- `main`: release promotions from `dev` only.
- `dev`: integration branch; pull requests only.
- Work branches: `feat/ISSUE-area-summary`, `fix/ISSUE-area-summary`,
  `exp/ISSUE-hypothesis`, `docs/ISSUE-topic`, `release/vX.Y.Z`.

One issue has one accountable owner. An author does not approve their own pull request.

## Commits

Conventional Commits: `type(scope): imperative summary`.
Types: `feat`, `fix`, `data`, `exp`, `docs`, `test`, `chore`, `refactor`, `ci`, `build`, `perf`.
Keep commits small and single-purpose; reference the issue (`Refs: #12`).
A breaking interface change uses `!` and a migration note.

Examples:

```
feat(ugv): add stationary-gated NHC update
fix(euroc): correct sensor-to-body rotation direction
data(splits): freeze drive-level validation manifest
docs(interfaces): document covariance error ordering
```

## Pull requests

Pull request titles follow the same Conventional Commit format (checked in CI).
Fill in every field of the pull request template. A screenshot of a curve alone is
not evidence: link the run id, the resolved configuration and the generating command.

## Weekly rhythm (Europe/Istanbul)

| When | Checkpoint |
|---|---|
| Monday 10:00 | Scope freeze: issues, hours, dependencies, artifact paths |
| Wednesday 16:00 | Integration on `dev`: synthetic end-to-end path runs |
| Friday 16:00 | Real-data smoke on both platforms plus week-specific evidence |
| Sunday 18:00 | Weekly report, dashboard, release manifest |

## Scientific discipline

- No tuning on the locked test split. Selection uses training and validation data only.
- Ground truth is carried in evaluation objects, never in the online filter call.
- Failed runs stay in the registry and in every denominator.
- Unknown values are recorded as OPEN or NOT RECORDED, never estimated.
- Decisions are recorded in `docs/adr/`; test evidence in `docs/weekly/` and `results/`.

## Documentation

Every package directory has a README or module docstring stating its purpose and its
contract. Repository documents use role names (lead, data/classical owner,
neural/infrastructure owner), not personal names.
