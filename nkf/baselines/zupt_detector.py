"""Causal stationary detector for the zero-velocity update, docs/theory.md section 7.3.

Status: PROPOSED (docs/adr/0003-stationary-detector.md). Inertial input only. The decision at
sample k uses samples k - window + 1 ... k and nothing later. Thresholds are arguments;
values per dataset are OPEN and are selected on training and validation data only.
"""

from __future__ import annotations

import numpy as np


def detect_stationary(gyro_rad_s, accel_m_s2, window, gyro_thresh_rad_s, accel_std_thresh_m_s2):
    """Return a [N] bool array; True where the vehicle is declared stationary.

    Stationary at k if, over the last `window` samples, the mean gyroscope norm is below
    `gyro_thresh_rad_s` and the standard deviation of the specific-force norm is below
    `accel_std_thresh_m_s2`. The first `window - 1` samples are never stationary.
    """
    gyro = np.asarray(gyro_rad_s, dtype=np.float64)
    accel = np.asarray(accel_m_s2, dtype=np.float64)
    if gyro.ndim != 2 or gyro.shape[1] != 3 or accel.shape != gyro.shape:
        raise ValueError("gyro and accel must both be [N, 3]")
    if not (isinstance(window, int) and window >= 2):
        raise ValueError("window must be an integer >= 2")
    if not (gyro_thresh_rad_s > 0.0 and accel_std_thresh_m_s2 > 0.0):
        raise ValueError("thresholds must be strictly positive")
    n = gyro.shape[0]
    g = np.linalg.norm(gyro, axis=1)
    a = np.linalg.norm(accel, axis=1)
    out = np.zeros(n, dtype=bool)
    if n < window:
        return out

    # Trailing (causal) window sums via cumulative sums.
    def trailing_mean(x):
        cs = np.concatenate([[0.0], np.cumsum(x)])
        return (cs[window:] - cs[:-window]) / window

    g_mean = trailing_mean(g)
    a_mean = trailing_mean(a)
    a_var = np.maximum(trailing_mean(a * a) - a_mean**2, 0.0)
    ok = (g_mean < gyro_thresh_rad_s) & (np.sqrt(a_var) < accel_std_thresh_m_s2)
    out[window - 1 :] = ok
    return out
