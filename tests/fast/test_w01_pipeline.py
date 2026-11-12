"""W01 fast tests: synthetic sequences, conventions, provenance and synthetic pipeline smoke."""

import json
import sqlite3

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from nkf.data.sequence import POSITION_GENERATOR, ImuSequence, Reference, split_into_segments
from nkf.data.synthetic import make_synthetic
from nkf.infra import provenance
from nkf.infra.run import load_config, run_synthetic

pytestmark = pytest.mark.fast


def _rotation(q_wxyz):
    """scipy expects [x, y, z, w]; the contract order is [w, x, y, z]."""
    return Rotation.from_quat(np.roll(q_wxyz, -1))


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_sequence_contract(platform):
    seq = make_synthetic(platform, seed=0)
    assert isinstance(seq.imu, ImuSequence) and isinstance(seq.reference, Reference)
    n = len(seq.imu.t_s)
    assert seq.imu.gyro_rad_s.shape == (n, 3) and seq.imu.accel_m_s2.shape == (n, 3)
    assert seq.reference.p_w.shape == (n, 3) and seq.reference.q_wb.shape == (n, 4)
    assert np.allclose(np.linalg.norm(seq.reference.q_wb, axis=1), 1.0, atol=1e-9)
    segments, gaps = split_into_segments(seq.imu.t_s, seq.imu.gyro_rad_s, seq.imu.accel_m_s2)
    assert gaps == 0 and len(segments) == 1


def test_bad_time_rejected():
    seq = make_synthetic("ugv", seed=0, duration_s=5.0)
    with pytest.raises(ValueError):
        split_into_segments(seq.imu.t_s[::-1].copy(), seq.imu.gyro_rad_s, seq.imu.accel_m_s2)


def test_stationary_specific_force_points_up():
    # UGV stops between ~40 and ~48 s; level and stationary -> accel ~ [0, 0, +9.81]
    seq = make_synthetic("ugv", seed=0, cfg={"accel_noise_std": 0.0, "accel_bias_std": 0.0})
    idx = (seq.imu.t_s > 42.0) & (seq.imu.t_s < 47.0)
    assert np.allclose(seq.imu.accel_m_s2[idx].mean(axis=0), [0.0, 0.0, 9.81], atol=1e-3)


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_gyro_integrates_to_reference_attitude(platform):
    seq = make_synthetic(
        platform, seed=0, duration_s=10.0, cfg={"gyro_noise_std": 0.0, "gyro_bias_std": 0.0}
    )
    q = seq.reference.q_wb
    rot = _rotation(q[0])
    dt = np.diff(seq.imu.t_s)
    for k in range(len(dt)):
        rot = rot * Rotation.from_rotvec(seq.imu.gyro_rad_s[k] * dt[k])
    err = (rot.inv() * _rotation(q[-1])).magnitude()
    assert err < 1e-6


def test_seed_determinism():
    a = make_synthetic("uav", seed=3, duration_s=5.0)
    b = make_synthetic("uav", seed=3, duration_s=5.0)
    assert a.source_hash == b.source_hash
    assert all(
        np.array_equal(x.z, y.z) for x, y in zip(a.observations, b.observations, strict=True)
    )


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_observations_come_from_the_generator_and_are_hidden_in_the_outage(platform):
    seq = make_synthetic(platform, seed=0)
    start, stop = seq.outage_s
    assert seq.hidden_observations > 0
    assert all(not (start <= o.t_s < stop) for o in seq.observations)
    for o in seq.observations:
        assert o.source == "simulated_from_reference"
        assert o.provenance["generator"] == POSITION_GENERATOR


def test_run_id_format():
    rid = provenance.make_run_id("ugv", "strapdown", 0, "a" * 64, "b" * 40)
    assert provenance.RUN_ID_RE.match(rid)


def test_config_hash_is_order_independent():
    assert provenance.config_hash({"a": 1, "b": 2}) == provenance.config_hash({"b": 2, "a": 1})


def test_pipeline_writes_bundle_db_and_registry(tmp_path):
    registry = tmp_path / "registry.csv"
    registry.write_text("run_id,status,artifact_uri,sha256\n")
    for platform in ("ugv", "uav"):
        cfg = load_config(
            "configs/base.yaml", "configs/experiments/smoke_synthetic.yaml", platform, 0
        )
        cfg["results_dir"] = str(tmp_path / "runs")
        cfg["db_path"] = str(tmp_path / "results.sqlite")
        cfg["registry"] = str(registry)
        run_synthetic(cfg)

    con = sqlite3.connect(tmp_path / "results.sqlite")
    status = con.execute("SELECT platform, status FROM runs ORDER BY platform").fetchall()
    assert status == [("uav", "completed"), ("ugv", "completed")]
    assert con.execute("SELECT COUNT(*) FROM metrics").fetchone()[0] == 6
    bundles = list((tmp_path / "runs").iterdir())
    assert len(bundles) == 2
    for b in bundles:
        assert {p.name for p in b.iterdir()} == {
            "manifest.json",
            "metrics.csv",
            "resolved_config.yaml",
            "trajectory.npz",
        }
        manifest = json.loads((b / "manifest.json").read_text())
        assert manifest["init"] == "oracle"
        assert sum(manifest["updates_by_source"].values()) == 0
        assert manifest["observations"]["hidden_in_outage"] > 0
    lines = registry.read_text().strip().splitlines()
    assert lines[0] == "run_id,status,artifact_uri,sha256"
    assert len(lines) == 3
