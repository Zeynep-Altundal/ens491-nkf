"""Tests for the classical ESKF and the stationary detector (synthetic data only).

These are implementation checks, not evidence about any dataset. Noise densities, constraint
covariances and detector thresholds below are test values matched to the synthetic generator,
not tuned or recommended values.
"""

import inspect

import numpy as np
import pytest

from nkf.baselines.eskf import (
    NHC,
    POSITION,
    ZUPT,
    Eskf,
    NoiseDensities,
    constraint_observation,
    run_eskf,
)
from nkf.baselines.strapdown import oracle_initial_state, run_strapdown
from nkf.baselines.zupt_detector import detect_stationary
from nkf.data.sequence import Observation
from nkf.data.synthetic import make_synthetic
from nkf.filters import conventions as c

pytestmark = pytest.mark.fast

DT = 0.01  # synthetic UGV inertial period
# Per-sample noise std of the generator (0.002 rad/s, 0.02 m/s^2) converted to densities.
NOISE = NoiseDensities(0.002 * np.sqrt(DT), 0.02 * np.sqrt(DT), 1e-5, 1e-4)
P0 = c.initial_covariance(
    {
        "pos_std_m": 0.1,
        "vel_std_m_s": 0.1,
        "att_std_rad": 0.01,
        "gyro_bias_std_rad_s": 0.003,
        "accel_bias_std_m_s2": 0.06,
    }
)
NHC_STD = 0.1  # m/s, test value
ZUPT_STD = 0.01  # m/s, test value


@pytest.fixture(scope="module")
def ugv():
    seq = make_synthetic("ugv", seed=0)
    return seq, oracle_initial_state(seq.reference)


def _err(p, ref):
    return np.linalg.norm(p - ref.p_w, axis=1)


def test_api_has_no_reference_argument():
    for fn in (Eskf.init, Eskf.step):
        names = list(inspect.signature(fn).parameters)
        assert not any("ref" in n or "truth" in n for n in names)
    assert list(inspect.signature(Eskf.step).parameters)[1:] == [
        "gyro_rad_s",
        "accel_m_s2",
        "dt_s",
        "observations",
    ]


def test_without_observations_the_nominal_state_equals_strapdown(ugv):
    seq, init = ugv
    imu = seq.imu
    out = run_eskf(imu, init, P0, NOISE)
    p_sd, v_sd, q_sd = run_strapdown(imu, init)
    assert np.allclose(out["p"], p_sd, atol=1e-9)
    assert np.allclose(out["v"], v_sd, atol=1e-9)
    assert np.allclose(out["q"], q_sd, atol=1e-12)


def test_covariance_stays_symmetric_positive_definite_and_grows_without_updates():
    seq = make_synthetic("ugv", seed=1, duration_s=5.0)
    filt = Eskf(NOISE)
    filt.init(oracle_initial_state(seq.reference), P0)
    traces = []
    for k in range(len(seq.imu.t_s) - 1):
        _, cov, _ = filt.step(seq.imu.gyro_rad_s[k], seq.imu.accel_m_s2[k], DT, [])
        traces.append(np.trace(cov[c.DP, c.DP]))
    assert np.allclose(cov, cov.T)
    assert np.all(np.linalg.eigvalsh(cov) > 0.0)
    assert np.all(np.diff(traces) >= -1e-12)


def test_position_updates_bound_the_error(ugv):
    seq, init = ugv
    ref = seq.reference
    p_sd, _, _ = run_strapdown(seq.imu, init)
    out = run_eskf(seq.imu, init, P0, NOISE, seq.observations)
    rmse = lambda e: float(np.sqrt(np.mean(e**2)))  # noqa: E731
    applied = len(seq.observations) - out["unassigned_observations"]
    assert out["update_counts"][POSITION] == applied > 0
    assert out["diverged_at"] is None
    assert rmse(_err(out["p"], ref)) < 0.1 * rmse(_err(p_sd, ref))
    offsets = out["observation_offsets_s"]
    assert np.all((offsets >= 0.0) & (offsets < DT))


def test_nhc_reduces_the_error_at_the_end_of_the_outage(ugv):
    seq, init = ugv
    ref = seq.reference
    i_end = int(np.searchsorted(seq.imu.t_s, seq.outage_s[1])) - 1
    plain = run_eskf(seq.imu, init, P0, NOISE, seq.observations)
    nhc = run_eskf(seq.imu, init, P0, NOISE, seq.observations, nhc_std_m_s=NHC_STD)
    assert nhc["update_counts"][NHC] == len(seq.imu.t_s) - 1
    assert _err(nhc["p"], ref)[i_end] < _err(plain["p"], ref)[i_end]


def test_detector_finds_the_stop_and_is_causal(ugv):
    seq, _ = ugv
    t, gyro, accel = seq.imu.t_s, seq.imu.gyro_rad_s, seq.imu.accel_m_s2
    # The synthetic vehicle stops between about 40 s and 50 s. A 2 s window is used because
    # the synthetic signals carry no engine or road vibration; this is not a tuned value.
    args = (200, 0.01, 0.05)
    st = detect_stationary(gyro, accel, *args)
    assert st[(t > 42.0) & (t < 48.0)].mean() > 0.95
    assert st[(t > 5.0) & (t < 37.0)].mean() < 0.01
    # Changing future samples must not change past decisions.
    k = 3000
    g2, a2 = gyro.copy(), accel.copy()
    g2[k:] += 1.0
    a2[k:] *= 3.0
    assert np.array_equal(detect_stationary(g2, a2, *args)[:k], st[:k])


def test_zupt_holds_the_velocity_near_zero_during_the_stop(ugv):
    seq = make_synthetic("ugv", seed=0, duration_s=55.0)
    init = oracle_initial_state(seq.reference)
    t = seq.imu.t_s
    st = detect_stationary(seq.imu.gyro_rad_s, seq.imu.accel_m_s2, 200, 0.01, 0.05)
    stop = (t > 42.0) & (t < 48.0)
    with_zupt = run_eskf(seq.imu, init, P0, NOISE, [], NHC_STD, ZUPT_STD, stationary=st)
    without = run_eskf(seq.imu, init, P0, NOISE, [], NHC_STD)
    speed = lambda r: np.linalg.norm(r["v"][stop], axis=1).mean()  # noqa: E731
    assert with_zupt["update_counts"][ZUPT] > 0
    assert speed(with_zupt) < 0.01
    assert speed(with_zupt) < speed(without)


def test_constraint_with_wrong_source_is_refused(ugv):
    seq, init = ugv
    filt = Eskf(NOISE)
    filt.init(init, P0)
    bad = Observation(
        0.01, NHC, np.zeros(2), "m/s", "b", np.eye(2), "simulated_from_reference",
        True, {"generator": "simulated_position_updates", "config_hash": "x", "seed": 0},
    )  # fmt: skip
    with pytest.raises(ValueError):
        filt.step(seq.imu.gyro_rad_s[0], seq.imu.accel_m_s2[0], DT, [bad])


def test_constraint_observation_records():
    nhc = constraint_observation(NHC, 1.0, 0.1)
    zupt = constraint_observation(ZUPT, 1.0, 0.05)
    assert nhc.z.shape == (2,) and zupt.z.shape == (3,)
    assert nhc.source == zupt.source == "pseudo_measurement"
    with pytest.raises(ValueError):
        constraint_observation(POSITION, 1.0, 0.1)
    with pytest.raises(ValueError):
        constraint_observation(NHC, 1.0, 0.0)


def test_step_requires_init():
    with pytest.raises(RuntimeError):
        Eskf(NOISE).step(np.zeros(3), np.array([0.0, 0.0, 9.81]), DT, [])
