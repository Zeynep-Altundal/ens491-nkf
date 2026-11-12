"""Contract tests for the data objects and time validation of docs/interfaces.md, sections 8-9."""

import dataclasses
import inspect

import numpy as np
import pytest

from nkf.data import sequence as s
from nkf.filters import conventions as c

pytestmark = pytest.mark.fast


def _imu(n=1000, rate=100.0):
    t = np.arange(n) / rate
    return t, np.zeros((n, 3)), np.tile([0.0, 0.0, 9.81], (n, 1))


# --- 8. Time validation -------------------------------------------------------------------


def test_regular_sequence_is_one_segment():
    t, gyro, accel = _imu()
    segments, gaps = s.split_into_segments(t, gyro, accel)
    assert gaps == 0
    assert [(seg.start, seg.stop) for seg in segments] == [(0, 1000)]


@pytest.mark.parametrize("bad_index,bad_value", [(500, None), (500, float("nan"))])
def test_non_increasing_or_non_finite_time_rejects_sequence(bad_index, bad_value):
    t, gyro, accel = _imu()
    t[bad_index] = t[bad_index - 1] if bad_value is None else bad_value
    with pytest.raises(ValueError):
        s.split_into_segments(t, gyro, accel)


def test_gap_splits_sequence_without_integration_across_it():
    t, gyro, accel = _imu()
    t[600:] += 0.2
    segments, gaps = s.split_into_segments(t, gyro, accel)
    assert gaps == 1
    assert [(seg.start, seg.stop) for seg in segments] == [(0, 600), (600, 1000)]
    assert s.outage_window_status(100, 400, segments) == "ok"
    assert s.outage_window_status(550, 650, segments) == "timestamp_gap"


def test_jitter_below_dt_max_is_not_a_gap():
    t, gyro, accel = _imu()
    t[600:] += 0.03
    _, gaps = s.split_into_segments(t, gyro, accel)
    assert gaps == 0


def test_non_finite_sample_is_a_counted_gap():
    t, gyro, accel = _imu()
    accel[300, 1] = np.nan
    segments, gaps = s.split_into_segments(t, gyro, accel)
    assert gaps == 1
    assert [(seg.start, seg.stop) for seg in segments] == [(0, 300), (301, 1000)]


def test_segment_run_decision():
    t, gyro, accel = _imu()
    t[600:] += 0.2
    segments, _ = s.split_into_segments(t, gyro, accel)
    assert s.segment_status(segments[0], 5.0, True) == "ok"
    assert s.segment_status(segments[1], 5.0, True) == "segment_too_short"
    assert s.segment_status(segments[0], 5.0, False) == "no_initial_state"


@pytest.mark.parametrize("factor", [1.0, 0.5, 21.0])
def test_dt_max_factor_range(factor):
    t, gyro, accel = _imu()
    with pytest.raises(ValueError):
        s.split_into_segments(t, gyro, accel, dt_max_factor=factor)


# --- 10. Leakage --------------------------------------------------------------------------


def test_imu_sequence_carries_only_allowlisted_fields():
    assert tuple(f.name for f in dataclasses.fields(s.ImuSequence)) == s.IMU_ALLOWLIST
    admitted = {name for names in s.KITTI_ADMITTED.values() for name in names}
    assert admitted == {"wx", "wy", "wz", "ax", "ay", "az"}
    assert admitted.isdisjoint(s.KITTI_REJECTED)


def test_filter_building_blocks_take_no_reference_argument():
    for name in ("propagate_nominal", "joseph_update", "inject", "reset_cov"):
        params = inspect.signature(getattr(c, name)).parameters
        assert not any("ref" in p or "truth" in p or p == "gt" for p in params), name


# --- 11. Observation provenance -----------------------------------------------------------


def _reference(n=500, rate=100.0):
    t = np.arange(n) / rate
    p = np.column_stack([t, np.zeros(n), np.zeros(n)])
    valid = np.ones(n, dtype=bool)
    valid[200:260] = False
    return s.Reference(t, p, valid)


def test_generator_output_carries_provenance():
    obs = s.simulate_position_updates(_reference(), 1.0, 0.5, seed=7, config_hash="abc")
    assert len(obs) == 4
    for o in obs:
        assert o.source == "simulated_from_reference" and o.kind == "position"
        assert o.unit == "m" and o.frame == "w"
        assert o.provenance == {"generator": s.POSITION_GENERATOR, "config_hash": "abc", "seed": 7}
        assert np.allclose(o.r_meas, 0.25 * np.eye(3))
    again = s.simulate_position_updates(_reference(), 1.0, 0.5, seed=7, config_hash="abc")
    assert all(np.array_equal(a.z, b.z) for a, b in zip(obs, again, strict=True))


def test_generator_skips_invalid_reference_samples():
    obs = s.simulate_position_updates(_reference(), 1.0, 0.5, seed=7, config_hash="abc")
    assert all(not (2.0 <= o.t_s < 2.6) for o in obs)


@pytest.mark.parametrize(
    "provenance",
    [
        {},
        {"generator": "other", "config_hash": "abc", "seed": 1},
        {"generator": s.POSITION_GENERATOR},
    ],
)
def test_simulated_observation_requires_declared_generator(provenance):
    with pytest.raises(ValueError):
        s.Observation(
            0.0,
            "position",
            np.zeros(3),
            "m",
            "w",
            np.eye(3),
            "simulated_from_reference",
            True,
            provenance,
        )


def test_observation_validation():
    with pytest.raises(ValueError):
        s.Observation(0.0, "position", np.zeros(3), "m", "w", np.eye(3), "unknown_source")
    with pytest.raises(ValueError):
        s.Observation(0.0, "position", np.zeros(3), "m", "w", -np.eye(3), "dataset_sensor")
    with pytest.raises(ValueError):
        s.Observation(0.0, "position", np.zeros(3), "m", "w", np.eye(2), "dataset_sensor")


def test_updates_are_counted_per_source():
    sim = s.simulate_position_updates(_reference(), 1.0, 0.5, seed=7, config_hash="abc")
    pseudo = s.Observation(
        0.1, "constraint:nhc", np.zeros(2), "m/s", "b", np.eye(2), "pseudo_measurement"
    )
    unused = s.Observation(
        0.2, "constraint:nhc", np.zeros(2), "m/s", "b", np.eye(2), "pseudo_measurement", False
    )
    counts = s.count_updates_by_source([*sim, pseudo, unused])
    assert counts == {"simulated_from_reference": 4, "dataset_sensor": 0, "pseudo_measurement": 1}
