"""Measurement models of the ground-vehicle motion constraints, docs/theory.md section 7.

Each function returns the innovation y = z - h(x_hat) and the Jacobian H with respect to
the 15-component error state of docs/interfaces.md. Detection of when a constraint may be
applied, and the measurement covariances, are outside this module.
"""

from __future__ import annotations

import numpy as np

from nkf.filters import conventions as c

# Rows select the lateral (y) and vertical (z) velocity components in the vehicle frame.
LATERAL_VERTICAL = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


def nhc_measurement(v_w, q, rot_vehicle_from_body=None):
    """Non-holonomic constraint: zero lateral and vertical velocity in the vehicle frame.

    `rot_vehicle_from_body` maps the body (inertial sensor) frame to the vehicle frame
    (x forward, y left, z up). The default is the identity: the two frames are assumed
    aligned, which is an unverified modelling assumption. The lever arm between the
    vehicle reference point and the sensor is neglected.
    """
    rot_vb = np.eye(3) if rot_vehicle_from_body is None else np.asarray(rot_vehicle_from_body)
    rot_hat = c.quat_to_rot(q)
    v_b = rot_hat.T @ np.asarray(v_w, dtype=np.float64)
    select = LATERAL_VERTICAL @ rot_vb
    h = np.zeros((2, 15))
    h[:, c.DV] = select @ rot_hat.T
    h[:, c.DTH] = select @ c.skew(v_b)
    return -select @ v_b, h


def zupt_measurement(v_w):
    """Zero-velocity update: the velocity is zero while the vehicle is stationary."""
    h = np.zeros((3, 15))
    h[:, c.DV] = np.eye(3)
    return -np.asarray(v_w, dtype=np.float64), h
