"""W01 fast tests: Sequence contract, conventions, provenance and synthetic pipeline smoke."""

import sqlite3

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from nkf.data.sequence import validate_sequence
from nkf.data.synthetic import make_synthetic
from nkf.infra import provenance
from nkf.infra.run import load_config, run_synthetic

pytestmark = pytest.mark.fast


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_sequence_contract(platform):
    assert validate_sequence(make_synthetic(platform, seed=0))


def test_bad_time_rejected():
    seq = make_synthetic("ugv", seed=0, duration_s=5.0)
    seq["t_s"] = seq["t_s"][::-1].copy()
    with pytest.raises(ValueError):
        validate_sequence(seq)


def test_stationary_specific_force_points_up():
    # UGV stops between ~40 and ~48 s; level and stationary -> accel ~ [0, 0, +9.81]
    seq = make_synthetic("ugv", seed=0, cfg={"accel_noise_std": 0.0, "accel_bias_std": 0.0})
    idx = (seq["t_s"] > 42.0) & (seq["t_s"] < 47.0)
    assert np.allclose(seq["accel_m_s2"][idx].mean(axis=0), [0.0, 0.0, 9.81], atol=1e-3)


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_gyro_integrates_to_reference_attitude(platform):
    seq = make_synthetic(
        platform, seed=0, duration_s=10.0, cfg={"gyro_noise_std": 0.0, "gyro_bias_std": 0.0}
    )
    q = seq["reference_pose"]["q_wb_wxyz"]
    R = Rotation.from_quat(q[0], scalar_first=True)
    dt = np.diff(seq["t_s"])
    for k in range(len(dt)):
        R = R * Rotation.from_rotvec(seq["gyro_rad_s"][k] * dt[k])
    err = (R.inv() * Rotation.from_quat(q[-1], scalar_first=True)).magnitude()
    assert err < 1e-6


def test_seed_determinism():
    a = make_synthetic("uav", seed=3, duration_s=5.0)
    b = make_synthetic("uav", seed=3, duration_s=5.0)
    assert a["source_hash"] == b["source_hash"]


def test_run_id_format():
    rid = provenance.make_run_id("ugv", "strapdown_dummy", 0, "a" * 64, "b" * 40)
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
    lines = registry.read_text().strip().splitlines()
    assert lines[0] == "run_id,status,artifact_uri,sha256"
    assert len(lines) == 3
