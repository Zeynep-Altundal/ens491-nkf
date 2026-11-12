"""Strapdown inertial dead reckoning with the nominal model of docs/interfaces.md, section 4.

No measurement update and no bias estimation: the initial biases are kept constant.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from nkf.data.sequence import split_into_segments
from nkf.filters import conventions as c


@dataclass(frozen=True)
class InitialState:
    """Initial-state record of section 8.4."""

    p: np.ndarray
    v: np.ndarray
    q: np.ndarray
    b_g: np.ndarray
    b_a: np.ndarray
    init: str

    def __post_init__(self):
        if self.init not in ("oracle", "operational"):
            raise ValueError("init must be 'oracle' or 'operational'")


def oracle_initial_state(reference, index=0):
    """Initial state taken from the reference at one sample, biases set to zero (section 10)."""
    if not reference.valid[index] or reference.v_w is None or reference.q_wb is None:
        raise ValueError("no valid reference sample for an oracle initial state")
    zero = np.zeros(3)
    p, v, q = reference.p_w[index], reference.v_w[index], reference.q_wb[index]
    return InitialState(np.array(p), np.array(v), np.array(q), zero, zero, "oracle")


def run_strapdown(imu, initial_state, dt_max_factor=5.0, g_w=c.GRAVITY_W):
    """Integrate one gap-free sequence. Returns positions, velocities and quaternions per sample.

    A sequence with a timestamp gap is refused: each segment is an independent run with
    its own initial state (section 9).
    """
    segments, gaps = split_into_segments(imu.t_s, imu.gyro_rad_s, imu.accel_m_s2, dt_max_factor)
    if gaps or len(segments) != 1:
        raise ValueError(f"sequence has {gaps} gap(s); run each segment separately")
    n = len(imu.t_s)
    p_out, v_out, q_out = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 4))
    state = (
        initial_state.p,
        initial_state.v,
        initial_state.q,
        initial_state.b_g,
        initial_state.b_a,
    )
    p_out[0], v_out[0], q_out[0] = state[0], state[1], state[2]
    dt = np.diff(imu.t_s)
    for k in range(n - 1):
        state = c.propagate_nominal(*state, imu.gyro_rad_s[k], imu.accel_m_s2[k], float(dt[k]), g_w)
        p_out[k + 1], v_out[k + 1], q_out[k + 1] = state[0], state[1], state[2]
    return p_out, v_out, q_out


def drift_metrics(t_s, p_est, p_ref):
    """Position error against the reference. Descriptive values only."""
    err = p_est - p_ref
    norm = np.linalg.norm(err, axis=1)
    path = float(np.sum(np.linalg.norm(np.diff(p_ref, axis=0), axis=1)))
    return {
        "duration_s": float(t_s[-1] - t_s[0]),
        "reference_path_length_m": path,
        "final_position_error_m": float(norm[-1]),
        "max_position_error_m": float(norm.max()),
        "final_error_enu_m": [float(x) for x in err[-1]],
        "final_error_percent_of_path": float(100.0 * norm[-1] / path) if path > 0 else None,
    }
