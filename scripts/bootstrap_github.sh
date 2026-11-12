#!/usr/bin/env bash
# Loads the label vocabulary and the three milestones. Safe to re-run.
# Usage: scripts/bootstrap_github.sh [owner/repo]
set -euo pipefail
REPO="${1:-Zeynep-Altundal/ens491-nkf}"
START=$(date +%s)
echo "Estimated duration: about 1 minute."

echo "Labels:"
gh label create "type:feature" --color "1d76db" --force --repo "$REPO" >/dev/null && echo "  label type:feature"
gh label create "type:bug" --color "d73a4a" --force --repo "$REPO" >/dev/null && echo "  label type:bug"
gh label create "type:experiment" --color "5319e7" --force --repo "$REPO" >/dev/null && echo "  label type:experiment"
gh label create "type:docs" --color "0075ca" --force --repo "$REPO" >/dev/null && echo "  label type:docs"
gh label create "type:decision" --color "fbca04" --force --repo "$REPO" >/dev/null && echo "  label type:decision"
gh label create "type:infra" --color "6f42c1" --force --repo "$REPO" >/dev/null && echo "  label type:infra"
gh label create "platform:ugv" --color "0e8a16" --force --repo "$REPO" >/dev/null && echo "  label platform:ugv"
gh label create "platform:uav" --color "006b75" --force --repo "$REPO" >/dev/null && echo "  label platform:uav"
gh label create "platform:shared" --color "bfdadc" --force --repo "$REPO" >/dev/null && echo "  label platform:shared"
gh label create "paper:A" --color "c5def5" --force --repo "$REPO" >/dev/null && echo "  label paper:A"
gh label create "paper:B" --color "c5def5" --force --repo "$REPO" >/dev/null && echo "  label paper:B"
gh label create "paper:C" --color "c5def5" --force --repo "$REPO" >/dev/null && echo "  label paper:C"
gh label create "method:R1" --color "d4c5f9" --force --repo "$REPO" >/dev/null && echo "  label method:R1"
gh label create "method:R2" --color "d4c5f9" --force --repo "$REPO" >/dev/null && echo "  label method:R2"
gh label create "method:R3" --color "d4c5f9" --force --repo "$REPO" >/dev/null && echo "  label method:R3"
gh label create "priority:P0-blocker" --color "b60205" --force --repo "$REPO" >/dev/null && echo "  label priority:P0-blocker"
gh label create "priority:P1-required" --color "d93f0b" --force --repo "$REPO" >/dev/null && echo "  label priority:P1-required"
gh label create "priority:P2-stretch" --color "fef2c0" --force --repo "$REPO" >/dev/null && echo "  label priority:P2-stretch"
gh label create "milestone:M1" --color "ededed" --force --repo "$REPO" >/dev/null && echo "  label milestone:M1"
gh label create "milestone:M2" --color "ededed" --force --repo "$REPO" >/dev/null && echo "  label milestone:M2"
gh label create "milestone:M3" --color "ededed" --force --repo "$REPO" >/dev/null && echo "  label milestone:M3"
gh label create "blocked" --color "000000" --force --repo "$REPO" >/dev/null && echo "  label blocked"
gh label create "needs-evidence" --color "e99695" --force --repo "$REPO" >/dev/null && echo "  label needs-evidence"
gh label create "needs-advisor" --color "f9d0c4" --force --repo "$REPO" >/dev/null && echo "  label needs-advisor"
gh label create "area:data" --color "bfd4f2" --force --repo "$REPO" >/dev/null && echo "  label area:data"
gh label create "area:filter" --color "bfd4f2" --force --repo "$REPO" >/dev/null && echo "  label area:filter"
gh label create "area:training" --color "bfd4f2" --force --repo "$REPO" >/dev/null && echo "  label area:training"
gh label create "area:eval" --color "bfd4f2" --force --repo "$REPO" >/dev/null && echo "  label area:eval"
gh label create "area:ci" --color "bfd4f2" --force --repo "$REPO" >/dev/null && echo "  label area:ci"
gh label create "area:paper" --color "bfd4f2" --force --repo "$REPO" >/dev/null && echo "  label area:paper"

echo "Milestones:"
mk() {
  gh api "repos/$REPO/milestones" -f title="$1" -f due_on="$2" -f description="$3" >/dev/null 2>&1 \
    && echo "  created $1" || echo "  exists or failed: $1"
}
mk "M1 - Theory and baselines" "2026-11-29T20:59:59Z" "Data pipelines, strapdown and ESKF baselines, v0.1.0"
mk "M2 - Method and evidence" "2026-12-27T20:59:59Z" "Learned-covariance models, ablations, stability study, v0.2.0"
mk "M3 - Reproducible study and handover" "2027-02-01T20:59:59Z" "Robustness and generalisation results, v1.0.0"

echo "Elapsed: $(( $(date +%s) - START )) s"
