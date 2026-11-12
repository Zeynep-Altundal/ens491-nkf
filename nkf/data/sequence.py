"""Sequence contract: required keys and boundary validation. Contract: docs/interfaces.md."""

import numpy as np

SEQUENCE_KEYS = [
    "id",
    "platform",
    "t_s",
    "gyro_rad_s",
    "accel_m_s2",
    "reference_pose",
    "reference_mask",
    "calibration",
    "source_hash",
    "observation_stream",
    "availability_mask",
    "split_id",
]


def validate_sequence(seq):
    """Raise ValueError on any contract violation; return True otherwise."""
    missing = [k for k in SEQUENCE_KEYS if k not in seq]
    if missing:
        raise ValueError(f"missing keys: {missing}")
    if seq["platform"] not in ("ugv", "uav"):
        raise ValueError(f"bad platform {seq['platform']}")
    t = seq["t_s"]
    n = len(t)
    if t.ndim != 1 or n < 2:
        raise ValueError("t_s must be 1-D with >= 2 samples")
    if not np.all(np.diff(t) > 0):
        raise ValueError("t_s is not strictly increasing")
    for key in ("gyro_rad_s", "accel_m_s2"):
        if seq[key].shape != (n, 3):
            raise ValueError(f"{key} shape {seq[key].shape} != ({n}, 3)")
        if not np.all(np.isfinite(seq[key])):
            raise ValueError(f"{key} has non-finite values")
    ref = seq["reference_pose"]
    if ref["p_w"].shape != (n, 3) or ref["q_wb_wxyz"].shape != (n, 4):
        raise ValueError("reference_pose shapes do not match t_s")
    if not np.allclose(np.linalg.norm(ref["q_wb_wxyz"], axis=1), 1.0, atol=1e-9):
        raise ValueError("reference quaternions are not unit norm")
    if seq["reference_mask"].shape != (n,):
        raise ValueError("reference_mask shape does not match t_s")
    obs = seq["observation_stream"]
    if seq["availability_mask"].shape != obs["t_s"].shape:
        raise ValueError("availability_mask must match observation_stream t_s")
    if obs["p_w"].shape != (len(obs["t_s"]), 3):
        raise ValueError("observation p_w shape mismatch")
    return True
