#!/usr/bin/env bash
# Opens the W01 work packages and the added sprint items. Run once, after
# scripts/bootstrap_github.sh. Assignees are optional environment variables.
# Usage: DATA_OWNER=handle NEURAL_OWNER=handle scripts/create_sprint1_issues.sh [owner/repo]
set -euo pipefail
REPO="${1:-Zeynep-Altundal/ens491-nkf}"
LEAD="${LEAD:-@me}"
DATA_OWNER="${DATA_OWNER:-}"
NEURAL_OWNER="${NEURAL_OWNER:-}"
M1="M1 - Theory and baselines"
START=$(date +%s)
echo "Estimated duration: about 1 minute."

mk() { # title, assignee, labels, body
  local args=(--repo "$REPO" --title "$1" --label "$3" --milestone "$M1" --body "$4")
  [ -n "$2" ] && args+=(--assignee "$2")
  gh issue create "${args[@]}" | tail -1
}

mk "[W01][SHARED] Audit one KITTI drive and two EuRoC flights" "$DATA_OWNER" \
  "type:feature,platform:shared,area:data,priority:P1-required" \
  "Planned 10 h. Output: data/manifests/*.csv, docs/data_manifest.md. Acceptance: timestamps, channels, frames and reference quality inventoried."
mk "[W01][SHARED] Write strapdown, ESKF, NHC, ZUPT and 6-DoF theory note" "$DATA_OWNER" \
  "type:docs,platform:shared,area:filter,priority:P1-required" \
  "Planned 8 h. Output: docs/theory.pdf, ADR-0001. Acceptance: state and noise model agreed with the lead; equations reviewed."
mk "[W01][SHARED] Minimal CPU data-to-metrics fixture" "$DATA_OWNER" \
  "type:feature,platform:shared,area:eval,priority:P1-required" \
  "Planned 6 h. Output: tests/fast/test_units.py, causal-input checklist."
mk "[W01][SHARED] Pinned environment, CPU container and PR CI" "$NEURAL_OWNER" \
  "type:infra,platform:shared,area:ci,priority:P1-required" \
  "Planned 10 h. Output: environment.yml, docker/Dockerfile.cpu. Acceptance: imports verified on all three machines."
mk "[W01][SHARED] Synthetic UGV/UAV sequences, config schema, run registry, SQLite schema" "$NEURAL_OWNER" \
  "type:feature,platform:shared,area:data,priority:P1-required" \
  "Planned 8 h. Output: nkf/data/synthetic.py, results/schema.sql. Acceptance: two synthetic run bundles with schema-valid manifests."
mk "[W01][SHARED] Interface review, early integration PR, literature matrix" "$NEURAL_OWNER" \
  "type:docs,platform:shared,area:paper,priority:P1-required" \
  "Planned 6 h. Output: docs/related_work.md, pipeline smoke log."
mk "[W01][SHARED] Problem statement, hypotheses H-A/H-B/H-C, source-to-claim map" "$LEAD" \
  "type:docs,platform:shared,area:paper,priority:P1-required" \
  "Planned 10 h. Output: docs/problem_statement.md, paper/claims.csv in planned state."
mk "[W01][SHARED] Repository rules, labels, templates, board; freeze API contract" "$LEAD" \
  "type:infra,platform:shared,area:ci,priority:P1-required" \
  "Planned 8 h. Output: docs/interfaces.md, protection-check evidence, assigned issues."
mk "[W01][SHARED] Scope note, dashboard and W01 report" "$LEAD" \
  "type:docs,platform:shared,area:paper,priority:P1-required" \
  "Planned 6 h. Output: docs/advisor_dashboard.md, docs/weekly/W01.md."
mk "[W02][UGV] Inspect real IMU data on one KITTI drive" "$LEAD" \
  "type:feature,platform:ugv,area:data,priority:P1-required" \
  "Planned 3 h. Output: inspection script, entry in docs/data_manifest.md. Acceptance: monotonic time checked; admitted OXTS fields and frames documented."
mk "[W02][UGV] Strapdown IMU integration on one KITTI drive" "$LEAD" \
  "type:feature,platform:ugv,area:filter,priority:P1-required" \
  "Planned 4 h. Output: nkf/baselines/strapdown.py, run config. Acceptance: synthetic unit test passes; coordinated with the baseline owner."
mk "[W02][UGV] First dead-reckoning figure against reference" "$LEAD" \
  "type:experiment,platform:ugv,area:eval,priority:P1-required,needs-evidence" \
  "Planned 3 h. Output: figure and run manifest. Acceptance: regenerates from a clean checkout; no performance claim attached."
mk "[W02][UAV] Dead-reckoning figure on one EuRoC flight" "$LEAD" \
  "type:experiment,platform:uav,area:eval,priority:P2-stretch" \
  "Planned 3 h. Stretch. Acceptance: sensor-to-body frame verified. Moves to W03 if the data audit is not ready."
mk "[W02][SHARED] Short technical progress report" "$LEAD" \
  "type:docs,platform:shared,area:paper,priority:P1-required,needs-advisor" \
  "Planned 4 h. Content: problem, data, working system, first figures, next technical step. Acceptance: team-approved; every figure traces to a run id."

echo "Elapsed: $(( $(date +%s) - START )) s"
