"""Unit, frame and causality checks of the data-to-metrics path (issue #3).

Each check uses a case with a known analytic answer, so a unit or frame mistake (degrees
for radians, g for m/s^2, milliseconds for seconds, a body/world swap) changes the result.
Synthetic and fixture data only.
"""

import inspect

import numpy as np
import pytest

from nkf.baselines.strapdown import oracle_initial_state, run_strapdown
from nkf.data.sequence import ImuSequence
from nkf.data.synthetic import make_synthetic
from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

G = 9.81
I3 = np.eye(3)


def _identity_state():
    return np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), np.zeros(3)


def _propagate(state, gyro, accel, dt, steps):
    for _ in range(steps):
        state = c.propagate_nominal(*state, np.asarray(gyro), np.asarray(accel), dt)
    return state


def check_inertial_units(imu):
    """Plausibility screen for the inertial input; returns a list of problems (empty if none).

    This catches unit slips, not small errors: time in milliseconds, specific force in g,
    and angular rates that are not plausible in rad/s.
    """
    problems = []
    t = np.asarray(imu.t_s)
    dt = np.diff(t)
    if t[0] != 0.0:
        problems.append("t_s does not start at 0")
    if not np.all(dt > 0.0):
        problems.append("t_s is not strictly increasing")
    elif not (1e-3 <= float(np.median(dt)) <= 0.05):
        problems.append(f"median dt {np.median(dt):g} s is outside 1 ms - 50 ms (ms or s mix-up?)")
    a = float(np.median(np.linalg.norm(imu.accel_m_s2, axis=1)))
    if not (7.5 <= a <= 12.0):
        problems.append(f"median specific-force norm {a:g} is not near 9.81 (g or m/s^2?)")
    w = float(np.max(np.linalg.norm(imu.gyro_rad_s, axis=1)))
    if w > 35.0:
        problems.append(f"peak angular rate {w:g} is above 35 rad/s (deg/s mixed in?)")
    return problems


def test_specific_force_at_rest_means_no_acceleration():
    p, v, *_ = _propagate(_identity_state(), np.zeros(3), [0.0, 0.0, G], 0.01, 100)
    assert np.allclose(p, 0.0, atol=1e-12) and np.allclose(v, 0.0, atol=1e-12)


def test_free_fall_reads_zero_specific_force():
    _, v, *_ = _propagate(_identity_state(), np.zeros(3), np.zeros(3), 0.01, 100)
    assert np.allclose(v, [0.0, 0.0, -G * 1.0], atol=1e-9)


def test_acceleration_is_in_m_s2_and_integrates_to_metres():
    # 1 m/s^2 forward for 2 s from rest: v = 2 m/s, p = 2 m.
    p, v, *_ = _propagate(_identity_state(), np.zeros(3), [1.0, 0.0, G], 0.01, 200)
    assert np.allclose(v, [2.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(p, [2.0, 0.0, 0.0], atol=1e-9)


def test_gyro_is_in_rad_per_s():
    # pi/2 rad/s for 1 s turns the body x axis onto the world y axis.
    _, _, q, *_ = _propagate(_identity_state(), [0.0, 0.0, np.pi / 2], [0.0, 0.0, G], 0.01, 100)
    assert np.allclose(c.quat_to_rot(q) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-9)
    # A rate given in deg/s (90 per second) but read as rad/s gives a different attitude.
    _, _, q_deg, *_ = _propagate(_identity_state(), [0.0, 0.0, 90.0], [0.0, 0.0, G], 0.01, 100)
    assert not np.allclose(c.quat_to_rot(q_deg) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-3)


def test_specific_force_is_rotated_from_body_to_world():
    q = c.quat_from_rotvec(np.array([0.0, 0.0, np.pi / 2]))  # body x points along world y
    state = (np.zeros(3), np.zeros(3), q, np.zeros(3), np.zeros(3))
    _, v, *_ = _propagate(state, np.zeros(3), [1.0, 0.0, G], 0.01, 100)
    assert np.allclose(v, [0.0, 1.0, 0.0], atol=1e-9)


def test_noise_densities_scale_with_the_square_root_of_dt():
    # Section 5.4: per-sample std = density / sqrt(dt); velocity-increment variance = density^2 dt.
    density, dt = 0.02 * np.sqrt(0.01), 0.01
    q_c = c.continuous_noise_cov(1e-4, density, 1e-6, 1e-5)
    f, g = c.error_jacobians(I3, np.array([0.0, 0.0, G]), np.zeros(3))
    _, q_d = c.discretise(f, g, q_c, dt)
    assert np.allclose(np.diag(q_d)[c.DV], density**2 * dt)
    assert np.isclose(density / np.sqrt(dt), 0.02)  # per-sample accelerometer std
    _, q_d2 = c.discretise(f, g, q_c, 4.0 * dt)
    assert np.allclose(np.diag(q_d2)[c.DV], 4.0 * np.diag(q_d)[c.DV])


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_synthetic_inputs_pass_the_unit_screen(platform):
    seq = make_synthetic(platform, seed=0, duration_s=20.0)
    assert check_inertial_units(seq.imu) == []


def test_unit_screen_catches_common_slips():
    seq = make_synthetic("ugv", seed=0, duration_s=10.0)
    imu = seq.imu
    in_g = ImuSequence(imu.t_s, imu.gyro_rad_s, imu.accel_m_s2 / G)
    in_ms = ImuSequence(imu.t_s * 1000.0, imu.gyro_rad_s, imu.accel_m_s2)
    in_deg = ImuSequence(imu.t_s, imu.gyro_rad_s * 400.0, imu.accel_m_s2)
    assert any("specific-force" in p for p in check_inertial_units(in_g))
    assert any("dt" in p for p in check_inertial_units(in_ms))
    assert any("angular rate" in p for p in check_inertial_units(in_deg))


def test_strapdown_estimate_up_to_k_does_not_depend_on_later_samples():
    seq = make_synthetic("ugv", seed=0, duration_s=20.0)
    imu = seq.imu
    k = 1000
    cut = ImuSequence(imu.t_s[: k + 1], imu.gyro_rad_s[: k + 1], imu.accel_m_s2[: k + 1])
    init = oracle_initial_state(seq.reference)
    full = run_strapdown(imu, init)
    short = run_strapdown(cut, init)
    for a, b in zip(full, short, strict=True):
        assert np.array_equal(a[: k + 1], b)
    # Changing only the future must not change the past.
    gyro = imu.gyro_rad_s.copy()
    gyro[k + 1 :] += 5.0
    altered = ImuSequence(imu.t_s, gyro, imu.accel_m_s2)
    other = run_strapdown(altered, init)
    for a, b in zip(full, other, strict=True):
        assert np.array_equal(a[: k + 1], b[: k + 1])


def test_online_runners_do_not_accept_a_reference():
    for runner in (run_strapdown,):
        assert not any("ref" in name for name in inspect.signature(runner).parameters)
