"""eval.metrics: W01 placeholder position metrics (no alignment). Owner: Betül (review)."""

import numpy as np


def position_metrics(p_est, p_ref, t, outage_s=None):
    err = np.linalg.norm(p_est - p_ref, axis=1)
    rows = [
        ("ate_rmse", "value", float(np.sqrt(np.mean(err**2))), "m", "full"),
        ("endpoint_error", "value", float(err[-1]), "m", "full"),
    ]
    if outage_s is not None:
        i0 = int(np.searchsorted(t, outage_s[0]))
        i1 = int(np.searchsorted(t, outage_s[1])) - 1
        drift = float(err[i1] - err[i0])
        rows.append(
            ("outage_drift", "value", drift, "m", f"outage_{outage_s[0]:g}_{outage_s[1]:g}")
        )
    return rows
