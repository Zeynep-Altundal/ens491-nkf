"""Tests for the KITTI OXTS loader and the strapdown baseline. Synthetic and fixture data only."""

import math

import numpy as np
import pytest

from nkf.baselines.strapdown import (
    InitialState,
    drift_metrics,
    oracle_initial_state,
    run_strapdown,
)
from nkf.data import kitti
from nkf.data.sequence import ImuSequence, Reference
from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

# First record and first two timestamps of KITTI raw 2011_09_30_drive_0016 (extract),
# as recorded in docs/data_manifest.md. The second record is a copy with one field changed.
RECORD = (
    "49.033553228861 8.3950119439646 112.42118835449 0.039873 0.011068 1.6849703267949 "
    "12.328776905945 -1.4021179049548 12.408244909094 0.012518613059099 -0.089126925780574 "
    "0.20436507080925 0.22432866387161 9.8937524923296 0.31022243808138 -0.1762380633938 "
    "9.8920628130061 -0.010454464465486 0.0063655089020684 -0.0016335449466135 "
    "-0.010468392829279 0.0064273704473409 -0.0012684682420449 0.10816653826392 "
    "0.017029386365926 4 7 5 5 6"
)
STAMPS = "2011-09-30 11:50:39.904535903\n2011-09-30 11:50:39.914552544\n"


@pytest.fixture
def oxts_dir(tmp_path):
    (tmp_path / "data").mkdir()
    (tmp_path / "timestamps.txt").write_text(STAMPS)
    (tmp_path / "data" / "0000000000.txt").write_text(RECORD)
    fields = RECORD.split()
    fields[2] = "113.42118835449"  # altitude + 1 m
    (tmp_path / "data" / "0000000001.txt").write_text(" ".join(fields))
    return tmp_path


def test_loader_maps_allowlisted_columns(oxts_dir):
    imu, reference, meta = kitti.load_kitti_oxts(oxts_dir)
    assert np.allclose(
        imu.gyro_rad_s[0], [-0.010454464465486, 0.0063655089020684, -0.0016335449466135]
    )
    assert np.allclose(imu.accel_m_s2[0], [0.20436507080925, 0.22432866387161, 9.8937524923296])
    assert imu.t_s[0] == 0.0
    assert math.isclose(imu.t_s[1], 0.010016641, abs_tol=1e-12)
    assert meta["samples"] == 2 and meta["numsats_min"] == 7 and meta["posmode_values"] == [5]
    assert reference.p_w.shape == (2, 3)


def test_loader_reference_frame(oxts_dir):
    _, reference, _ = kitti.load_kitti_oxts(oxts_dir)
    assert np.allclose(reference.p_w[0], 0.0, atol=1e-9)
    assert np.allclose(reference.p_w[1], [0.0, 0.0, 1.0], atol=1e-6)
    assert np.allclose(reference.v_w[0], [-1.4021179049548, 12.328776905945, -0.089126925780574])
    roll, pitch, yaw = 0.039873, 0.011068, 1.6849703267949
    forward = c.quat_to_rot(reference.q_wb[0]) @ np.array([1.0, 0.0, 0.0])
    expected = [math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)]
    assert np.allclose(forward, expected, atol=1e-12)
    up = c.quat_to_rot(reference.q_wb[0]).T @ np.array([0.0, 0.0, 1.0])
    assert math.isclose(up[1], math.sin(roll) * math.cos(pitch), abs_tol=1e-12)


def test_loader_rejects_inconsistent_drive(oxts_dir):
    (oxts_dir / "data" / "0000000001.txt").unlink()
    with pytest.raises(ValueError):
        kitti.load_kitti_oxts(oxts_dir)


def test_geodetic_to_enu_against_radii_of_curvature():
    lat0, lon0, alt0 = 49.0, 8.4, 110.0
    phi = math.radians(lat0)
    w = math.sqrt(1.0 - kitti.WGS84_E2 * math.sin(phi) ** 2)
    meridian = kitti.WGS84_A * (1.0 - kitti.WGS84_E2) / w**3
    normal = kitti.WGS84_A / w
    d = 1e-5  # degrees
    north = kitti.geodetic_to_enu(lat0 + d, lon0, alt0, (lat0, lon0, alt0))
    east = kitti.geodetic_to_enu(lat0, lon0 + d, alt0, (lat0, lon0, alt0))
    up = kitti.geodetic_to_enu(lat0, lon0, alt0 + 2.0, (lat0, lon0, alt0))
    assert np.allclose(north, [0.0, math.radians(d) * (meridian + alt0), 0.0], atol=1e-5)
    assert np.allclose(
        east, [math.radians(d) * (normal + alt0) * math.cos(phi), 0.0, 0.0], atol=1e-5
    )
    assert np.allclose(up, [0.0, 0.0, 2.0], atol=1e-9)


def _circle(rate_hz, duration_s=30.0, speed=10.0, yaw_rate=0.2):
    """Level circular motion with exact body-frame measurements and reference."""
    n = int(duration_s * rate_hz) + 1
    t = np.arange(n) / rate_hz
    yaw = yaw_rate * t
    radius = speed / yaw_rate
    p = np.column_stack([radius * np.sin(yaw), radius * (1.0 - np.cos(yaw)), np.zeros(n)])
    v = np.column_stack([speed * np.cos(yaw), speed * np.sin(yaw), np.zeros(n)])
    q = np.array([c.quat_from_rotvec(np.array([0.0, 0.0, a])) for a in yaw])
    gyro = np.tile([0.0, 0.0, yaw_rate], (n, 1))
    accel = np.tile([0.0, speed * yaw_rate, 9.81], (n, 1))
    return ImuSequence(t, gyro, accel), Reference(t, p, np.ones(n, dtype=bool), v, q)


def test_strapdown_tracks_a_circle_and_error_is_first_order_in_dt():
    errors = []
    for rate in (100.0, 200.0):
        imu, ref = _circle(rate)
        p, v, q = run_strapdown(imu, oracle_initial_state(ref))
        metrics = drift_metrics(imu.t_s, p, ref.p_w)
        errors.append(metrics["final_position_error_m"])
        assert math.isclose(metrics["reference_path_length_m"], 300.0, rel_tol=1e-3)
        assert np.allclose(np.linalg.norm(q, axis=1), 1.0, atol=1e-12)
        assert np.allclose(p[:, 2], 0.0, atol=1e-9) and np.allclose(v[:, 2], 0.0, atol=1e-9)
    assert errors[0] < 0.5
    assert 1.8 < errors[0] / errors[1] < 2.2


def test_strapdown_stationary_stays_at_rest():
    n = 1001
    t = np.arange(n) / 100.0
    imu = ImuSequence(t, np.zeros((n, 3)), np.tile([0.0, 0.0, 9.81], (n, 1)))
    init = InitialState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), np.zeros(3), "oracle"
    )
    p, v, _ = run_strapdown(imu, init)
    assert np.allclose(p, 0.0, atol=1e-9) and np.allclose(v, 0.0, atol=1e-9)


def test_uncompensated_accelerometer_bias_gives_quadratic_drift():
    n = 1001
    t = np.arange(n) / 100.0
    bias = 0.05
    imu = ImuSequence(t, np.zeros((n, 3)), np.tile([bias, 0.0, 9.81], (n, 1)))
    init = InitialState(
        np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3), np.zeros(3), "oracle"
    )
    p, _, _ = run_strapdown(imu, init)
    assert math.isclose(p[-1, 0], 0.5 * bias * 10.0**2, rel_tol=1e-9)


def test_strapdown_refuses_a_sequence_with_a_gap():
    imu, ref = _circle(100.0, duration_s=5.0)
    t = imu.t_s.copy()
    t[200:] += 0.5
    with pytest.raises(ValueError):
        run_strapdown(ImuSequence(t, imu.gyro_rad_s, imu.accel_m_s2), oracle_initial_state(ref))


def test_oracle_initial_state_requires_a_valid_reference():
    imu, ref = _circle(100.0, duration_s=1.0)
    invalid = Reference(ref.t_s, ref.p_w, np.zeros(len(ref.t_s), dtype=bool), ref.v_w, ref.q_wb)
    with pytest.raises(ValueError):
        oracle_initial_state(invalid)
    with pytest.raises(ValueError):
        InitialState(
            np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3), "guess"
        )
    assert oracle_initial_state(ref).init == "oracle"
