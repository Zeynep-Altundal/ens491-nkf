"""Edge-case and negative tests for docs/interfaces.md: numerics, covariance, time, leakage."""

import math

import numpy as np
import pytest

from nkf.data import sequence as s
from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

RNG = np.random.default_rng(1)
IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])


def is_spd(p, rtol=1e-12):
    """Symmetric and positive definite up to a tolerance relative to the largest eigenvalue."""
    eig = np.linalg.eigvalsh(0.5 * (p + p.T))
    return bool(np.allclose(p, p.T, rtol=0.0, atol=rtol * eig[-1]) and eig[0] > -rtol * eig[-1])


# --- Quaternion and rotation numerics -----------------------------------------------------


def test_non_unit_quaternion_is_normalised():
    q = np.array([2.0, 0.0, 0.0, 2.0])
    rot = c.quat_to_rot(q)
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-14)
    assert np.allclose(rot, c.quat_to_rot(q / np.linalg.norm(q)), atol=1e-14)


@pytest.mark.parametrize(
    "bad",
    [
        [0.0, 0.0, 0.0, 0.0],
        [float("nan"), 0.0, 0.0, 1.0],
        [float("inf"), 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ],
)
def test_invalid_quaternion_is_rejected(bad):
    with pytest.raises(ValueError):
        c.quat_to_rot(np.array(bad))


@pytest.mark.parametrize("bad", [[float("nan"), 0.0, 0.0], [0.0, float("inf"), 0.0], [0.0, 0.0]])
def test_invalid_rotation_vector_is_rejected(bad):
    with pytest.raises(ValueError):
        c.so3_exp(np.array(bad))
    with pytest.raises(ValueError):
        c.quat_from_rotvec(np.array(bad))


def test_invalid_rotation_matrix_is_rejected():
    with pytest.raises(ValueError):
        c.so3_log(np.full((3, 3), np.nan))
    with pytest.raises(ValueError):
        c.so3_log(np.eye(4))


def test_quaternion_norm_does_not_drift_under_repeated_injection():
    state = (np.zeros(3), np.zeros(3), IDENTITY_Q, np.zeros(3), np.zeros(3))
    dx = np.zeros(15)
    dx[c.DTH] = [1e-3, -2e-3, 1.5e-3]
    for _ in range(10000):
        state = c.inject(*state, dx)
    assert abs(np.linalg.norm(state[2]) - 1.0) < 1e-12


@pytest.mark.parametrize("angle", [0.0, 1e-12, 1e-9, 1e-6])
def test_log_small_angle(angle):
    v = angle * np.array([0.6, 0.0, -0.8])
    assert np.allclose(c.so3_log(c.so3_exp(v)), v, atol=1e-15)


@pytest.mark.parametrize("gap", [1e-2, 1e-4, 1e-6, 1e-8, 1e-10])
def test_log_near_pi(gap):
    axis = np.array([0.6, 0.0, -0.8])
    v = (math.pi - gap) * axis
    assert np.allclose(c.so3_log(c.so3_exp(v)), v, atol=1e-7)


def test_log_at_pi_returns_an_equivalent_rotation():
    rot = c.so3_exp(math.pi * np.array([0.0, 0.6, 0.8]))
    v = c.so3_log(rot)
    assert math.isclose(np.linalg.norm(v), math.pi, abs_tol=1e-7)
    assert np.allclose(c.so3_exp(v), rot, atol=1e-7)


@pytest.mark.parametrize("dt", [0.0, -0.01, float("nan"), float("inf")])
def test_nominal_step_rejects_invalid_dt(dt):
    with pytest.raises(ValueError):
        c.propagate_nominal(
            np.zeros(3),
            np.zeros(3),
            IDENTITY_Q,
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            dt,
        )


def test_nominal_step_rejects_non_finite_measurement():
    bad = np.array([0.0, float("nan"), 0.0])
    with pytest.raises(ValueError):
        c.propagate_nominal(
            np.zeros(3), np.zeros(3), IDENTITY_Q, np.zeros(3), np.zeros(3), bad, np.zeros(3), 0.01
        )


def test_stationary_level_body_does_not_accelerate():
    f_rest = np.array([0.0, 0.0, 9.81])
    p, v, _, _, _ = c.propagate_nominal(
        np.zeros(3), np.zeros(3), IDENTITY_Q, np.zeros(3), np.zeros(3), np.zeros(3), f_rest, 0.01
    )
    assert np.allclose(p, 0.0, atol=1e-15) and np.allclose(v, 0.0, atol=1e-15)


# --- Covariance robustness ----------------------------------------------------------------


@pytest.mark.parametrize("log_cond", [6, 9, 12])
def test_joseph_update_with_ill_conditioned_covariance(log_cond):
    u, _ = np.linalg.qr(RNG.normal(size=(15, 15)))
    p = u @ np.diag(np.logspace(-log_cond / 2, log_cond / 2, 15)) @ u.T
    p = 0.5 * (p + p.T)
    h = np.zeros((3, 15))
    h[:, c.DP] = np.eye(3)
    _, p_next = c.joseph_update(p, h, 1e-6 * np.eye(3), np.array([0.1, -0.2, 0.3]))
    assert is_spd(p_next, rtol=1e-9)
    assert np.trace(p_next) <= np.trace(p) * (1.0 + 1e-9)


def test_joseph_update_stays_valid_with_a_suboptimal_gain():
    m = RNG.normal(size=(15, 15))
    p = m @ m.T + 1e-3 * np.eye(15)
    h = np.zeros((3, 15))
    h[:, c.DP] = np.eye(3)
    r_meas = 0.25 * np.eye(3)
    k = 0.3 * p @ h.T @ np.linalg.inv(h @ p @ h.T + r_meas)
    i_kh = np.eye(15) - k @ h
    assert is_spd(i_kh @ p @ i_kh.T + k @ r_meas @ k.T)


def test_initial_covariance_block_order_and_units():
    p0 = c.initial_covariance(
        {
            "pos_std_m": 2.0,
            "vel_std_m_s": 0.5,
            "att_std_rad": 0.1,
            "gyro_bias_std_rad_s": 0.01,
            "accel_bias_std_m_s2": 0.2,
        }
    )
    assert np.allclose(np.diag(p0)[0:3], 4.0)
    assert np.allclose(np.diag(p0)[3:6], 0.25)
    assert np.allclose(np.diag(p0)[6:9], 0.01)
    assert np.allclose(np.diag(p0)[9:12], 1e-4)
    assert np.allclose(np.diag(p0)[12:15], 0.04)
    assert is_spd(p0)


def test_spd_helper_rejects_invalid_matrices():
    assert not is_spd(np.diag([1.0, -1e-6, 1.0]))
    assert not is_spd(np.array([[1.0, 0.1], [0.0, 1.0]]))


# --- Segment boundaries and timestamps ----------------------------------------------------


def test_minimum_segment_duration_from_configuration():
    assert s.min_segment_duration(5, [10, 30, 60, 120, "full"]) == 15.0
    assert s.min_segment_duration(5, [5, 10, 20, 40]) == 10.0
    with pytest.raises(ValueError):
        s.min_segment_duration(5, ["full"])
    with pytest.raises(ValueError):
        s.min_segment_duration(-1, [10])


def test_segment_duration_boundary():
    minimum = 15.0
    assert s.segment_status(s.Segment(0, 10, minimum), minimum, True) == "ok"
    assert (
        s.segment_status(s.Segment(0, 10, math.nextafter(minimum, 0.0)), minimum, True)
        == "segment_too_short"
    )
    assert s.segment_status(s.Segment(0, 10, minimum), minimum, False) == "no_initial_state"
    assert s.segment_status(s.Segment(0, 10, 1.0), minimum, False) == "segment_too_short"


def _imu(n=400, rate=200.0):
    t = np.arange(n) / rate
    return t, np.zeros((n, 3)), np.zeros((n, 3))


def test_repeated_timestamp_rejects_sequence():
    t, gyro, accel = _imu()
    t[100] = t[99]
    with pytest.raises(ValueError):
        s.split_into_segments(t, gyro, accel)


def test_non_finite_first_and_last_samples():
    t, gyro, accel = _imu()
    gyro[0, 0] = np.inf
    accel[-1, 2] = np.nan
    segments, gaps = s.split_into_segments(t, gyro, accel)
    assert gaps == 2
    assert [(seg.start, seg.stop) for seg in segments] == [(1, 399)]


def test_gap_exactly_at_dt_max_is_not_a_gap():
    t = np.arange(400, dtype=np.float64)
    gyro, accel = np.zeros((400, 3)), np.zeros((400, 3))
    t[200:] += 4.0  # one interval of exactly 5 x the nominal interval of 1 s
    assert s.split_into_segments(t, gyro, accel)[1] == 0
    t[200:] += 0.5
    assert s.split_into_segments(t, gyro, accel)[1] == 1


def test_too_few_samples_are_rejected():
    with pytest.raises(ValueError):
        s.split_into_segments(np.array([0.0]), np.zeros((1, 3)), np.zeros((1, 3)))


# --- Leakage: forbidden fields ------------------------------------------------------------


def test_kitti_inertial_selection():
    names = ["ax", "ay", "az", "wx", "wy", "wz"]
    assert s.select_kitti_inertial(names) == ["wx", "wy", "wz", "ax", "ay", "az"]


@pytest.mark.parametrize(
    "forbidden", ["af", "wu", "lat", "yaw", "vn", "vf", "pos_accuracy", "posmode"]
)
def test_forbidden_kitti_field_is_refused(forbidden):
    with pytest.raises(ValueError):
        s.select_kitti_inertial(["ax", "ay", "az", forbidden])


def test_unknown_field_is_refused():
    with pytest.raises(ValueError):
        s.select_kitti_inertial(["ax", "temperature"])


def test_kitti_field_lists_are_disjoint_and_cover_all_30_fields():
    admitted = {n for names in s.KITTI_ADMITTED.values() for n in names}
    rejected, reference = set(s.KITTI_REJECTED), set(s.KITTI_REFERENCE_ONLY)
    assert admitted.isdisjoint(rejected) and admitted.isdisjoint(reference)
    assert rejected.isdisjoint(reference)
    assert len(admitted | rejected | reference) == 30


def test_imu_sequence_refuses_extra_fields():
    with pytest.raises(TypeError):
        s.ImuSequence(np.zeros(2), np.zeros((2, 3)), np.zeros((2, 3)), p_w=np.zeros((2, 3)))


# --- Observation timing -------------------------------------------------------------------


def _obs(t, valid=True):
    return s.Observation(
        t, "constraint:nhc", np.zeros(2), "m/s", "b", np.eye(2), "pseudo_measurement", valid
    )


def test_observation_assignment_and_offset_sign():
    t = np.array([0.0, 0.01, 0.02, 0.03])
    assigned, unassigned = s.assign_observations(
        t,
        [
            _obs(0.004),
            _obs(0.01),
            _obs(0.0101),
            _obs(0.03),
            _obs(0.0),
            _obs(0.031),
            _obs(0.02, False),
        ],
    )
    assert sorted(assigned) == [0, 1, 2]
    assert [round(off, 6) for _, off in assigned[0]] == [0.006, 0.0]
    assert [round(off, 6) for _, off in assigned[1]] == [0.0099]
    assert [round(off, 6) for _, off in assigned[2]] == [0.0]
    assert [o.t_s for o in unassigned] == [0.0, 0.031]
    dt = np.diff(t)
    for k, items in assigned.items():
        assert all(0.0 <= off < dt[k] for _, off in items)
