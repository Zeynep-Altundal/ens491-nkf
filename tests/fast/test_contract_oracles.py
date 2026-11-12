"""Independent oracles for the conventions of docs/interfaces.md.

The reference quantities in this file are computed without the helper functions of
nkf.filters.conventions: rotations come from the quaternion sandwich product and from
matrix power series, cross-product matrices from numpy.cross, and block positions are
written as explicit integer indices.
"""

import numpy as np
import pytest

from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

G_W = np.array([0.0, 0.0, -9.81])
Q = np.array([0.82, 0.15, -0.10, 0.54])
Q = Q / np.sqrt(Q @ Q)
P0, V0 = np.array([1.0, -2.0, 0.5]), np.array([3.0, 0.4, -0.1])
BG, BA = np.array([0.01, -0.02, 0.005]), np.array([0.05, 0.02, -0.03])
GYRO, ACCEL = np.array([0.2, -0.1, 0.4]), np.array([0.6, -0.3, 9.7])


def ind_cross_matrix(a):
    return np.column_stack([np.cross(a, e) for e in np.eye(3)])


def ind_rot_from_quat(q):
    w, u = q[0], q[1:]
    return np.column_stack(
        [e + 2 * w * np.cross(u, e) + 2 * np.cross(u, np.cross(u, e)) for e in np.eye(3)]
    )


def ind_expm(rotvec, terms=40):
    k = ind_cross_matrix(rotvec)
    out, term = np.eye(3), np.eye(3)
    for n in range(1, terms):
        term = term @ k / n
        out = out + term
    return out


def ind_logm(rot, terms=80):
    x = rot - np.eye(3)
    out, power = np.zeros((3, 3)), np.eye(3)
    for n in range(1, terms):
        power = power @ x
        out = out + (-1) ** (n + 1) * power / n
    return np.array([out[2, 1] - out[1, 2], out[0, 2] - out[2, 0], out[1, 0] - out[0, 1]]) / 2


def ind_step(p, v, rot, b_g, b_a, dt):
    a = rot @ (ACCEL - b_a) + G_W
    return p + v * dt + 0.5 * a * dt * dt, v + a * dt, rot @ ind_expm((GYRO - b_g) * dt), b_g, b_a


def ind_perturb(dx):
    return (
        P0 + dx[0:3],
        V0 + dx[3:6],
        ind_rot_from_quat(Q) @ ind_expm(dx[6:9]),
        BG + dx[9:12],
        BA + dx[12:15],
    )


def ind_error_after(dx, dt):
    true = ind_step(*ind_perturb(dx), dt)
    nom = ind_step(*ind_perturb(np.zeros(15)), dt)
    return np.concatenate(
        [
            true[0] - nom[0],
            true[1] - nom[1],
            ind_logm(nom[2].T @ true[2]),
            true[3] - nom[3],
            true[4] - nom[4],
        ]
    )


def ind_phi(dt, eps=1e-6):
    cols = [
        (ind_error_after(e, dt) - ind_error_after(-e, dt)) / (2 * eps) for e in eps * np.eye(15)
    ]
    return np.column_stack(cols)


def production_f_g():
    return c.error_jacobians(c.quat_to_rot(Q), ACCEL - BA, GYRO - BG)


# --- Analytic block oracle ----------------------------------------------------------------


def test_production_rotation_matches_independent_rotation():
    assert np.allclose(c.quat_to_rot(Q), ind_rot_from_quat(Q), atol=1e-14)
    v = np.array([0.4, -0.7, 0.2])
    assert np.allclose(c.so3_exp(v), ind_expm(v), atol=1e-13)
    assert np.allclose(c.skew(v), ind_cross_matrix(v), atol=0.0)


def test_f_blocks_against_analytic_oracle():
    rot = ind_rot_from_quat(Q)
    expected = np.zeros((15, 15))
    expected[0:3, 3:6] = np.eye(3)
    expected[3:6, 6:9] = -rot @ ind_cross_matrix(ACCEL - BA)
    expected[3:6, 12:15] = -rot
    expected[6:9, 6:9] = -ind_cross_matrix(GYRO - BG)
    expected[6:9, 9:12] = -np.eye(3)
    f, _ = production_f_g()
    assert np.allclose(f, expected, atol=1e-13)


def test_g_blocks_against_analytic_oracle():
    expected = np.zeros((15, 12))
    expected[3:6, 3:6] = -ind_rot_from_quat(Q)
    expected[6:9, 0:3] = -np.eye(3)
    expected[9:12, 6:9] = np.eye(3)
    expected[12:15, 9:12] = np.eye(3)
    _, g = production_f_g()
    assert np.allclose(g, expected, atol=1e-13)


# --- Independent finite-difference oracle -------------------------------------------------


def test_f_against_independent_model():
    dt = 1e-3
    f, _ = production_f_g()
    assert np.allclose(ind_phi(dt), np.eye(15) + f * dt, atol=2e-5)


def test_sign_errors_in_f_are_detected_by_independent_model():
    dt = 1e-3
    f, _ = production_f_g()
    phi = ind_phi(dt)
    for rows, cols in [((3, 6), (6, 9)), ((3, 6), (12, 15)), ((6, 9), (6, 9)), ((6, 9), (9, 12))]:
        wrong = f.copy()
        wrong[rows[0] : rows[1], cols[0] : cols[1]] *= -1.0
        assert not np.allclose(phi, np.eye(15) + wrong * dt, atol=2e-5), (rows, cols)


def test_production_step_matches_independent_step():
    dt = 0.01
    p, v, q, _, _ = c.propagate_nominal(P0, V0, Q, BG, BA, GYRO, ACCEL, dt)
    p_i, v_i, rot_i, _, _ = ind_step(P0, V0, ind_rot_from_quat(Q), BG, BA, dt)
    assert np.allclose(p, p_i, atol=1e-13) and np.allclose(v, v_i, atol=1e-13)
    assert np.allclose(c.quat_to_rot(q), rot_i, atol=1e-13)


# --- Injection and reset against an independent SO(3) reference ---------------------------


def test_injection_is_on_the_right_not_on_the_left():
    a = np.array([0.03, -0.02, 0.01])
    dx = np.zeros(15)
    dx[6:9] = a
    _, _, q_new, _, _ = c.inject(P0, V0, Q, BG, BA, dx)
    rot = ind_rot_from_quat(Q)
    assert np.allclose(c.quat_to_rot(q_new), rot @ ind_expm(a), atol=1e-13)
    assert not np.allclose(c.quat_to_rot(q_new), ind_expm(a) @ rot, atol=1e-6)


def ind_reset_jacobian(a, eps=1e-6):
    exp_a_t = ind_expm(a).T
    cols = []
    for e in eps * np.eye(3):
        cols.append(
            (ind_logm(exp_a_t @ ind_expm(a + e)) - ind_logm(exp_a_t @ ind_expm(a - e))) / (2 * eps)
        )
    return np.column_stack(cols)


@pytest.mark.parametrize("a", [[0.03, -0.02, 0.01], [-0.01, 0.04, 0.02], [0.002, 0.001, -0.003]])
def test_reset_jacobian_against_independent_reference(a):
    a = np.array(a)
    num = ind_reset_jacobian(a)
    g_r = c.reset_jacobian(a)
    bound = float(a @ a)  # first-order formula: error of second order in |a|
    assert np.max(np.abs(num - g_r[6:9, 6:9])) < bound
    assert np.max(np.abs(num - (np.eye(3) + 0.5 * ind_cross_matrix(a)))) > 10 * bound


def test_error_is_zero_after_injecting_the_true_error():
    a = np.array([0.03, -0.02, 0.01])
    assert np.allclose(ind_logm(ind_expm(a).T @ ind_expm(a)), 0.0, atol=1e-14)


# --- Sensitivity of the tolerances to step and perturbation size --------------------------


def test_discretisation_error_is_second_order_in_dt():
    f, _ = production_f_g()
    err = [np.max(np.abs(ind_phi(dt) - (np.eye(15) + f * dt))) for dt in (4e-3, 2e-3, 1e-3)]
    assert 3.5 < err[0] / err[1] < 4.5 and 3.5 < err[1] / err[2] < 4.5
    assert err[2] < 2e-5 / 2


def test_linearisation_error_is_second_order_in_the_error_state():
    dt = 1e-3
    phi = ind_phi(dt)
    base = np.zeros(15)
    base[6:9] = [0.01, -0.006, 0.008]
    err = [
        np.max(np.abs(ind_error_after(s * base, dt) - phi @ (s * base))) for s in (4.0, 2.0, 1.0)
    ]
    assert 3.5 < err[0] / err[1] < 4.5 and 3.5 < err[1] / err[2] < 4.5


def test_reset_error_is_second_order_in_the_correction():
    base = np.array([0.03, -0.02, 0.01])
    err = []
    for s in (2.0, 1.0, 0.5):
        a = s * base
        err.append(np.max(np.abs(ind_reset_jacobian(a) - c.reset_jacobian(a)[6:9, 6:9])))
    assert 3.5 < err[0] / err[1] < 4.5 and 3.5 < err[1] / err[2] < 4.5
