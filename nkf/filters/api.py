"""filters.api: Filter.step contract plus a dummy strapdown implementation for W01 tests.

Contract: docs/interfaces.md. The dummy ignores observations and is NOT an estimator.

Interface (to be frozen in docs/interfaces.md):
    filter_step(state, imu, dt, obs_now) -> state, covariance[15,15], diagnostics
state = {"p": (3,), "v": (3,), "R": (3,3) body->world, "bg": (3,), "ba": (3,)}
covariance error ordering: [dp, dv, dtheta, dbg, dba]
"""

import numpy as np
from scipy.spatial.transform import Rotation

GRAVITY_W = np.array([0.0, 0.0, -9.81])


def init_state(p, v, q_wb_wxyz):
    return {
        "p": np.array(p, dtype=float),
        "v": np.array(v, dtype=float),
        "R": Rotation.from_quat(q_wb_wxyz, scalar_first=True).as_matrix(),
        "bg": np.zeros(3),
        "ba": np.zeros(3),
        "P": np.eye(15) * 1e-4,
    }


def filter_step(state, imu, dt, obs_now=None):
    gyro, accel = imu
    R = state["R"]
    a_w = R @ (accel - state["ba"]) + GRAVITY_W
    p = state["p"] + state["v"] * dt + 0.5 * a_w * dt * dt
    v = state["v"] + a_w * dt
    R = R @ Rotation.from_rotvec((gyro - state["bg"]) * dt).as_matrix()
    P = state["P"] + np.eye(15) * 1e-6 * dt
    P = 0.5 * (P + P.T)
    new_state = {"p": p, "v": v, "R": R, "bg": state["bg"], "ba": state["ba"], "P": P}
    diag = {"used_observation": False}
    return new_state, P, diag


def run_filter(seq):
    """Run filter_step over a whole sequence. Reference is used ONLY for oracle initialisation."""
    ref = seq["reference_pose"]
    state = init_state(ref["p_w"][0], ref["v_w"][0], ref["q_wb_wxyz"][0])
    t = seq["t_s"]
    n = len(t)
    p_est = np.zeros((n, 3))
    p_est[0] = state["p"]
    for k in range(1, n):
        dt = t[k] - t[k - 1]
        state, _P, _ = filter_step(state, (seq["gyro_rad_s"][k - 1], seq["accel_m_s2"][k - 1]), dt)
        p_est[k] = state["p"]
    return {"p_w": p_est, "P_final": state["P"]}
