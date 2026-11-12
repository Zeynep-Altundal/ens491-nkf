#!/usr/bin/env bash
# Applies branch protection to dev and main. Run only after the skeleton is merged
# and all collaborators have joined: with protection on, the owner cannot push directly.
# Usage: scripts/protect_branches.sh [owner/repo]
set -uo pipefail
REPO="${1:-Zeynep-Altundal/ens491-nkf}"
START=$(date +%s)
echo "Estimated duration: under 1 minute."

protect() {
  local branch="$1" approvals="$2"
  if gh api -X PUT "repos/$REPO/branches/$branch/protection" --input - >/dev/null <<JSON
{
  "required_status_checks": {"strict": true, "contexts": ["pr-lint", "pr-tests"]},
  "enforce_admins": true,
  "required_pull_request_reviews": {
    "dismiss_stale_reviews": true,
    "required_approving_review_count": $approvals
  },
  "restrictions": null,
  "required_conversation_resolution": true,
  "allow_force_pushes": false,
  "allow_deletions": false
}
JSON
  then
    echo "  $branch: protection applied ($approvals approval(s))"
  else
    echo "  $branch: NOT APPLIED. Record this limitation in docs/weekly/W01.md."
  fi
}

protect dev 1
protect main 2
echo "Elapsed: $(( $(date +%s) - START )) s"
