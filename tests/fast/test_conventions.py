"""Contract tests for the mathematical conventions of docs/interfaces.md, sections 2-7 and 10.1."""

import math

import numpy as np
import pytest

from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

RNG = np.random.default_rng(0)


def _state():
    q = c.quat_from_rotvec(np.array([0.3, -0.2, 0.9]))
    return (
        np.array([1.0, -2.0, 0.5]),
        np.array([3.0, 0.4, -0.1]),
        q,
        np.array([0.01, -0.02, 0.005]),
        np.array([0.05, 0.02, -0.03]),
    )


GYRO = np.array([0.2, -0.1, 0.4])
ACCEL = np.array([0.6, -0.3, 9.7])


def _boxminus(true, nom):
    rot_err = c.quat_to_rot(nom[2]).T @ c.quat_to_rot(true[2])
    return np.concatenate(
        [true[0] - nom[0], true[1] - nom[1], c.so3_log(rot_err), true[3] - nom[3], true[4] - nom[4]]
    )


def _true_step(state, dt, noise=None):
    n = np.zeros(12) if noise is None else noise
    p, v, q, b_g, b_a = c.propagate_nominal(*state, GYRO - n[c.NG], ACCEL - n[c.NA], dt)
    return p, v, q, b_g + n[c.NBG] * dt, b_a + n[c.NBA] * dt


def _error_after(dx0, dt, noise=None):
    nom = _state()
    return _boxminus(_true_step(c.inject(*nom, dx0), dt, noise), _true_step(nom, dt))


def _jacobians():
    p, v, q, b_g, b_a = _state()
    return c.error_jacobians(c.quat_to_rot(q), ACCEL - b_a, GYRO - b_g)


# --- 1. Quaternion convention -------------------------------------------------------------


def test_quaternion_is_hamilton_scalar_first():
    i, j, k = np.eye(4)[1], np.eye(4)[2], np.eye(4)[3]
    assert np.allclose(c.quat_mul(i, j), k)
    assert np.allclose(c.quat_mul(j, i), -k)
    assert np.allclose(c.quat_to_rot(np.array([1.0, 0.0, 0.0, 0.0])), np.eye(3))


def test_rotation_maps_body_to_world():
    q = c.quat_from_rotvec(np.array([0.0, 0.0, math.pi / 2]))
    assert np.allclose(c.quat_to_rot(q) @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)


def test_quaternion_product_matches_rotation_product():
    q1 = c.quat_from_rotvec(RNG.normal(size=3))
    q2 = c.quat_from_rotvec(RNG.normal(size=3))
    assert np.allclose(c.quat_to_rot(c.quat_mul(q1, q2)), c.quat_to_rot(q1) @ c.quat_to_rot(q2))


# --- 2. Exponential map -------------------------------------------------------------------


@pytest.mark.parametrize("scale", [1e-10, 1e-4, 0.1, 1.0, 2.5])
def test_exp_log_round_trip(scale):
    v = scale * np.array([0.3, -0.5, 0.8])
    rot = c.so3_exp(v)
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-12)
    assert math.isclose(np.linalg.det(rot), 1.0, abs_tol=1e-12)
    assert np.allclose(c.so3_log(rot), v, atol=1e-9)
    assert np.allclose(c.quat_to_rot(c.quat_from_rotvec(v)), rot, atol=1e-12)


def test_exp_small_angle():
    v = np.array([1e-6, -2e-6, 3e-6])
    assert np.allclose(c.so3_exp(v), np.eye(3) + c.skew(v), atol=1e-10)


# --- 3. Error definition ------------------------------------------------------------------


def test_orientation_error_is_right_multiplicative():
    nom = _state()
    dx = np.zeros(15)
    dx[c.DTH] = [0.02, -0.01, 0.03]
    injected = c.inject(*nom, dx)
    expected = c.quat_to_rot(nom[2]) @ c.so3_exp(dx[c.DTH])
    assert np.allclose(c.quat_to_rot(injected[2]), expected, atol=1e-12)
    assert np.allclose(_boxminus(injected, nom), dx, atol=1e-12)


def test_bias_error_is_true_minus_nominal():
    nom = _state()
    dx = np.zeros(15)
    dx[c.DBG], dx[c.DBA] = [1e-3, 2e-3, -1e-3], [-2e-2, 1e-2, 3e-2]
    injected = c.inject(*nom, dx)
    assert np.allclose(injected[3] - nom[3], dx[c.DBG])
    assert np.allclose(injected[4] - nom[4], dx[c.DBA])


# --- 4. Jacobians -------------------------------------------------------------------------


def test_f_matches_finite_differences_at_zero_error():
    dt, eps = 1e-3, 1e-6
    f, _ = _jacobians()
    phi_num = np.zeros((15, 15))
    for i in range(15):
        e = np.zeros(15)
        e[i] = eps
        phi_num[:, i] = (_error_after(e, dt) - _error_after(-e, dt)) / (2 * eps)
    assert np.allclose(phi_num, np.eye(15) + f * dt, atol=2e-5)


def test_f_predicts_small_nonzero_error():
    dt = 1e-3
    f, _ = _jacobians()
    dx0 = 1e-3 * RNG.normal(size=15)
    dx0[c.DTH] = [0.01, -0.006, 0.008]
    predicted = (np.eye(15) + f * dt) @ dx0
    assert np.allclose(_error_after(dx0, dt), predicted, atol=1e-5)
    wrong = np.eye(15) + f * dt
    wrong[c.DV, c.DTH] *= -1.0
    assert not np.allclose(_error_after(dx0, dt), wrong @ dx0, atol=1e-5)


def test_g_matches_finite_differences():
    dt, eps = 1e-3, 1e-4
    _, g = _jacobians()
    g_num = np.zeros((15, 12))
    for i in range(12):
        n = np.zeros(12)
        n[i] = eps
        g_num[:, i] = (_error_after(np.zeros(15), dt, n) - _error_after(np.zeros(15), dt, -n)) / (
            2 * eps
        )
    assert np.allclose(g_num, g * dt, atol=1e-6)


# --- 5. Reset -----------------------------------------------------------------------------


@pytest.mark.parametrize("dtheta", [[0.03, -0.02, 0.01], [-0.01, 0.04, 0.02]])
def test_reset_jacobian_matches_finite_differences(dtheta):
    a, eps = np.array(dtheta), 1e-6
    rot_a_t = c.so3_exp(a).T

    def new_error(delta):
        return c.so3_log(rot_a_t @ c.so3_exp(delta))

    num = np.zeros((3, 3))
    for i in range(3):
        e = np.zeros(3)
        e[i] = eps
        num[:, i] = (new_error(a + e) - new_error(a - e)) / (2 * eps)
    g_r = c.reset_jacobian(a)
    assert np.allclose(num, g_r[c.DTH, c.DTH], atol=5e-4)
    assert not np.allclose(num, np.eye(3) + 0.5 * c.skew(a), atol=5e-4)
    assert np.allclose(np.delete(np.delete(g_r, range(6, 9), 0), range(6, 9), 1), np.eye(12))


def test_reset_keeps_covariance_symmetric_positive_definite():
    m = RNG.normal(size=(15, 15))
    p = m @ m.T + 1e-3 * np.eye(15)
    p_reset = c.reset_cov(p, np.array([0.03, -0.02, 0.01]))
    assert np.allclose(p_reset, p_reset.T)
    assert np.all(np.linalg.eigvalsh(p_reset) > 0.0)


# --- 6. Joseph update ---------------------------------------------------------------------


def test_joseph_update_is_symmetric_positive_definite_and_optimal():
    m = RNG.normal(size=(15, 15))
    p = m @ m.T + 1e-3 * np.eye(15)
    h = np.zeros((3, 15))
    h[:, c.DP] = np.eye(3)
    r_meas = 0.25 * np.eye(3)
    y = np.array([0.4, -0.2, 0.1])
    dx, p_next = c.joseph_update(p, h, r_meas, y)
    assert np.allclose(p_next, p_next.T)
    assert np.all(np.linalg.eigvalsh(p_next) > 0.0)
    k = p @ h.T @ np.linalg.inv(h @ p @ h.T + r_meas)
    assert np.allclose(dx, k @ y)
    assert np.allclose(p_next, (np.eye(15) - k @ h) @ p, atol=1e-9)
    assert np.all(np.diag(p_next)[c.DP] < np.diag(p)[c.DP])


# --- 7. Noise convention ------------------------------------------------------------------


def test_noise_covariance_layout_and_scaling():
    q_c = c.continuous_noise_cov(1e-3, 2e-2, 1e-5, 3e-4)
    assert q_c.shape == (12, 12)
    assert np.allclose(np.diag(q_c), np.repeat([1e-6, 4e-4, 1e-10, 9e-8], 3))
    f, g = _jacobians()
    _, q1 = c.discretise(f, g, q_c, 0.01)
    _, q2 = c.discretise(f, g, q_c, 0.02)
    assert np.allclose(q2, 2.0 * q1)
    assert np.allclose(q1[c.DTH, c.DTH], 1e-6 * 0.01 * np.eye(3))
    assert np.allclose(q1[c.DV, c.DV], 4e-4 * 0.01 * np.eye(3))
    assert np.allclose(q1[c.DBG, c.DBG], 1e-10 * 0.01 * np.eye(3))
    assert np.allclose(q1[c.DBA, c.DBA], 9e-8 * 0.01 * np.eye(3))
    assert np.allclose(q1[c.DP, c.DP], 0.0)


@pytest.mark.parametrize("bad", [0.0, -1e-3, float("nan"), float("inf")])
def test_noise_density_validation(bad):
    with pytest.raises(ValueError):
        c.continuous_noise_cov(bad, 1e-2, 1e-5, 1e-4)


def test_covariance_propagation_stays_symmetric_positive_definite():
    f, g = _jacobians()
    phi, q_d = c.discretise(f, g, c.continuous_noise_cov(1e-3, 2e-2, 1e-5, 3e-4), 0.01)
    p = 0.01 * np.eye(15)
    for _ in range(200):
        p = c.propagate_cov(p, phi, q_d)
    assert np.allclose(p, p.T)
    assert np.all(np.linalg.eigvalsh(p) > 0.0)


# --- 9. Initial covariance ----------------------------------------------------------------

P0_CONFIG = {
    "pos_std_m": 0.1,
    "vel_std_m_s": 0.05,
    "att_std_rad": [0.01, 0.01, 0.05],
    "gyro_bias_std_rad_s": 1e-3,
    "accel_bias_std_m_s2": 0.05,
}


def test_initial_covariance_scalar_and_vector_fields():
    p0 = c.initial_covariance(P0_CONFIG)
    assert p0.shape == (15, 15)
    assert np.allclose(np.diag(p0)[c.DP], 0.01)
    assert np.allclose(np.diag(p0)[c.DTH], [1e-4, 1e-4, 2.5e-3])
    assert np.allclose(p0, np.diag(np.diag(p0)))
    assert np.all(np.linalg.eigvalsh(p0) > 0.0)


@pytest.mark.parametrize(
    "key,value",
    [
        ("pos_std_m", 0.0),
        ("pos_std_m", 101.0),
        ("att_std_rad", 4.0),
        ("gyro_bias_std_rad_s", -1e-3),
        ("accel_bias_std_m_s2", float("nan")),
        ("vel_std_m_s", [0.1, 0.1]),
    ],
)
def test_initial_covariance_validation(key, value):
    with pytest.raises(ValueError):
        c.initial_covariance({**P0_CONFIG, key: value})


def test_initial_covariance_missing_field():
    config = dict(P0_CONFIG)
    del config["vel_std_m_s"]
    with pytest.raises(ValueError):
        c.initial_covariance(config)
