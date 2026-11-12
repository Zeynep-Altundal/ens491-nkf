"""Reference implementation of the conventions in docs/interfaces.md (NumPy, float64).

Scope: single-step mathematical building blocks only. No dataset loop, no experiment.
Section numbers refer to docs/interfaces.md.
"""

from __future__ import annotations

import math

import numpy as np

GRAVITY_W = np.array([0.0, 0.0, -9.81])  # section 3; value PROPOSED

# Error-state layout, section 5.1.
DP, DV, DTH, DBG, DBA = (slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15))
# Noise-vector layout, section 5.3.
NG, NA, NBG, NBA = (slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12))

# Provisional configuration checks for the initial covariance, section 10.1.
P0_RANGES = {
    "pos_std_m": 100.0,
    "vel_std_m_s": 10.0,
    "att_std_rad": math.pi,
    "gyro_bias_std_rad_s": 0.1,
    "accel_bias_std_m_s2": 1.0,
}


def _finite_vector(u, size, name):
    """Validated float64 vector. Non-finite input is an error, never repaired."""
    v = np.asarray(u, dtype=np.float64)
    if v.shape != (size,) or not np.all(np.isfinite(v)):
        raise ValueError(f"{name} must be a finite vector of length {size}")
    return v


def _unit_quaternion(q):
    """Validated unit quaternion [w, x, y, z]. A zero or non-finite quaternion is an error."""
    v = _finite_vector(q, 4, "quaternion")
    norm = float(np.linalg.norm(v))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return v / norm


def skew(u):
    """Skew-symmetric matrix [u]x."""
    x, y, z = (float(c) for c in u)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def so3_exp(rotvec):
    """SO(3) exponential map of a rotation vector."""
    v = _finite_vector(rotvec, 3, "rotation vector")
    angle = float(np.linalg.norm(v))
    k = skew(v)
    if angle < 1e-8:
        return np.eye(3) + k + 0.5 * k @ k
    a = math.sin(angle) / angle
    b = (1.0 - math.cos(angle)) / (angle * angle)
    return np.eye(3) + a * k + b * k @ k


def so3_log(rot):
    """Rotation vector of a rotation matrix, angle in [0, pi].

    The angle uses atan2 and stays accurate near 0 and near pi. Near pi the axis is
    taken from the symmetric part of the matrix; at exactly pi the sign of the axis
    is not defined by the rotation and either sign is returned.
    """
    r = np.asarray(rot, dtype=np.float64)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)):
        raise ValueError("rotation matrix must be a finite 3x3 array")
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    sin_angle = 0.5 * float(np.linalg.norm(vee))
    cos_angle = 0.5 * (float(np.trace(r)) - 1.0)
    angle = math.atan2(sin_angle, cos_angle)
    if sin_angle < 1e-8 and cos_angle > 0.0:
        return 0.5 * vee
    if cos_angle > 0.0 or sin_angle > 1e-6:
        return angle / (2.0 * sin_angle) * vee
    outer = (0.5 * (r + r.T) - cos_angle * np.eye(3)) / (1.0 - cos_angle)
    i = int(np.argmax(np.diag(outer)))
    axis = outer[:, i] / math.sqrt(outer[i, i])
    if float(axis @ vee) < 0.0:
        axis = -axis
    return angle * axis


def quat_mul(q1, q2):
    """Hamilton product, order [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_from_rotvec(rotvec):
    """Unit quaternion q{v} of a rotation vector."""
    v = _finite_vector(rotvec, 3, "rotation vector")
    angle = float(np.linalg.norm(v))
    if angle < 1e-8:
        q = np.concatenate(([1.0], 0.5 * v))
        return q / np.linalg.norm(q)
    axis = v / angle
    return np.concatenate(([math.cos(angle / 2.0)], math.sin(angle / 2.0) * axis))


def quat_to_rot(q):
    """Rotation matrix R_wb of a unit quaternion: maps body vectors to the world frame."""
    w, x, y, z = _unit_quaternion(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def propagate_nominal(p, v, q, b_g, b_a, gyro, accel, dt, g_w=GRAVITY_W):
    """One nominal step with the measurement held constant, section 4."""
    if not (math.isfinite(dt) and dt > 0.0):
        raise ValueError("dt must be finite and strictly positive")
    rot = quat_to_rot(q)
    w = _finite_vector(gyro, 3, "gyro") - b_g
    a = rot @ (_finite_vector(accel, 3, "accel") - b_a) + g_w
    q_next = quat_mul(q, quat_from_rotvec(w * dt))
    q_next = q_next / np.linalg.norm(q_next)
    v_next = v + a * dt
    p_next = p + v * dt + 0.5 * a * dt * dt
    return p_next, v_next, q_next, np.array(b_g, dtype=np.float64), np.array(b_a, dtype=np.float64)


def error_jacobians(rot_hat, f_hat, w_hat):
    """Continuous-time Jacobians F [15, 15] and G [15, 12], section 5.3."""
    f = np.zeros((15, 15))
    f[DP, DV] = np.eye(3)
    f[DV, DTH] = -rot_hat @ skew(f_hat)
    f[DV, DBA] = -rot_hat
    f[DTH, DTH] = -skew(w_hat)
    f[DTH, DBG] = -np.eye(3)
    g = np.zeros((15, 12))
    g[DV, NA] = -rot_hat
    g[DTH, NG] = -np.eye(3)
    g[DBG, NBG] = np.eye(3)
    g[DBA, NBA] = np.eye(3)
    return f, g


def continuous_noise_cov(gyro_density, accel_density, gyro_bias_rw, accel_bias_rw):
    """Q_c [12, 12] from noise densities, section 5.4."""
    sigmas = (gyro_density, accel_density, gyro_bias_rw, accel_bias_rw)
    for s in sigmas:
        if not (isinstance(s, (int, float)) and math.isfinite(s) and s > 0.0):
            raise ValueError("noise densities must be finite and strictly positive")
    return np.diag(np.repeat(np.square(sigmas), 3))


def discretise(f, g, q_c, dt):
    """First-order discretisation, section 5.5."""
    return np.eye(15) + f * dt, g @ q_c @ g.T * dt


def propagate_cov(p, phi, q_d):
    """Covariance propagation with symmetrisation, section 5.5."""
    p_next = phi @ p @ phi.T + q_d
    return 0.5 * (p_next + p_next.T)


def joseph_update(p, h, r_meas, y):
    """Joseph-form update, section 6. Returns the error estimate and the covariance."""
    s = h @ p @ h.T + r_meas
    k = p @ h.T @ np.linalg.inv(s)
    i_kh = np.eye(p.shape[0]) - k @ h
    p_next = i_kh @ p @ i_kh.T + k @ r_meas @ k.T
    return k @ y, 0.5 * (p_next + p_next.T)


def inject(p, v, q, b_g, b_a, dx):
    """Injection of the error estimate with the increment on the right, section 7."""
    q_next = quat_mul(q, quat_from_rotvec(dx[DTH]))
    return p + dx[DP], v + dx[DV], q_next / np.linalg.norm(q_next), b_g + dx[DBG], b_a + dx[DBA]


def reset_jacobian(dtheta):
    """First-order reset Jacobian G_r, section 7."""
    g_r = np.eye(15)
    g_r[DTH, DTH] = np.eye(3) - 0.5 * skew(dtheta)
    return g_r


def reset_cov(p, dtheta):
    """Covariance reset after injection, section 7."""
    g_r = reset_jacobian(dtheta)
    p_next = g_r @ p @ g_r.T
    return 0.5 * (p_next + p_next.T)


def initial_covariance(config):
    """P_0 [15, 15] from the configuration fields of section 10.1."""
    blocks = []
    for key, upper in P0_RANGES.items():
        if key not in config:
            raise ValueError(f"missing initial-covariance field: {key}")
        std = np.atleast_1d(np.asarray(config[key], dtype=np.float64))
        if std.shape == (1,):
            std = np.repeat(std, 3)
        if std.shape != (3,):
            raise ValueError(f"{key} must be a scalar or a length-3 vector")
        if not np.all(np.isfinite(std)) or np.any(std <= 0.0) or np.any(std > upper):
            raise ValueError(f"{key} outside the valid range (0, {upper}]")
        blocks.append(np.square(std))
    return np.diag(np.concatenate(blocks))
