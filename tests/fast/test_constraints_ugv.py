"""Tests for the ground-vehicle constraint models of docs/theory.md, section 7."""

import numpy as np
import pytest

from nkf.baselines.constraints_ugv import LATERAL_VERTICAL, nhc_measurement, zupt_measurement
from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

MOUNTING = c.so3_exp(np.array([0.02, -0.03, 0.1]))


def _states():
    """Deterministic states with non-trivial orientation and velocity."""
    rng = np.random.default_rng(7)
    out = []
    for _ in range(6):
        q = c.quat_from_rotvec(rng.uniform(-1.5, 1.5, 3))
        v = rng.uniform(-12.0, 12.0, 3)
        out.append((v, q))
    return out


def _h_numeric(v, q, rot_vb, eps=1e-6):
    zero = np.zeros(3)

    def predicted(dx):
        _, v_new, q_new, _, _ = c.inject(zero, v, q, zero, zero, dx)
        innovation, _ = nhc_measurement(v_new, q_new, rot_vb)
        return -innovation

    cols = [(predicted(e) - predicted(-e)) / (2 * eps) for e in eps * np.eye(15)]
    return np.column_stack(cols)


@pytest.mark.parametrize("rot_vb", [None, MOUNTING])
@pytest.mark.parametrize("state", _states())
def test_nhc_jacobian_matches_finite_differences(state, rot_vb):
    v, q = state
    _, h = nhc_measurement(v, q, rot_vb)
    numeric = _h_numeric(v, q, rot_vb)
    assert h.shape == (2, 15)
    assert np.allclose(h, numeric, atol=1e-7)
    wrong = h.copy()
    wrong[:, c.DTH] *= -1.0
    assert not np.allclose(wrong, numeric, atol=1e-3)


def test_nhc_jacobian_blocks_outside_velocity_and_orientation_are_zero():
    v, q = _states()[0]
    _, h = nhc_measurement(v, q)
    assert np.all(h[:, c.DP] == 0.0) and np.all(h[:, c.DBG] == 0.0) and np.all(h[:, c.DBA] == 0.0)


@pytest.mark.parametrize("state", _states())
def test_nhc_innovation_is_zero_for_pure_forward_motion(state):
    _, q = state
    v_w = c.quat_to_rot(q) @ np.array([9.0, 0.0, 0.0])
    innovation, _ = nhc_measurement(v_w, q)
    assert np.allclose(innovation, 0.0, atol=1e-12)


def test_nhc_innovation_reports_lateral_and_vertical_velocity():
    _, q = _states()[1]
    v_b = np.array([9.0, 0.4, -0.2])
    innovation, _ = nhc_measurement(c.quat_to_rot(q) @ v_b, q)
    assert np.allclose(innovation, [-0.4, 0.2], atol=1e-12)


def test_nhc_uses_the_vehicle_frame_when_a_mounting_rotation_is_given():
    _, q = _states()[2]
    v_vehicle = np.array([9.0, 0.0, 0.0])
    v_w = c.quat_to_rot(q) @ (MOUNTING.T @ v_vehicle)
    aligned, _ = nhc_measurement(v_w, q, MOUNTING)
    assumed_aligned, _ = nhc_measurement(v_w, q)
    assert np.allclose(aligned, 0.0, atol=1e-12)
    assert not np.allclose(assumed_aligned, 0.0, atol=1e-3)


def test_selection_matrix_picks_lateral_and_vertical():
    assert np.array_equal(LATERAL_VERTICAL @ np.array([1.0, 2.0, 3.0]), [2.0, 3.0])


def test_zupt_model():
    v = np.array([0.3, -0.1, 0.05])
    innovation, h = zupt_measurement(v)
    assert np.array_equal(innovation, -v)
    expected = np.zeros((3, 15))
    expected[:, 3:6] = np.eye(3)
    assert np.array_equal(h, expected)
