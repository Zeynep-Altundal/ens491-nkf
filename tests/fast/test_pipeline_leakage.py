"""Leakage checks of the synthetic pipeline: reference independence, outage, provenance."""

import inspect
import json

import numpy as np
import pytest

from nkf.baselines.strapdown import oracle_initial_state, run_strapdown
from nkf.data.sequence import POSITION_GENERATOR, Reference
from nkf.data.synthetic import make_synthetic
from nkf.infra.run import load_config, run_synthetic

pytestmark = pytest.mark.fast


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_estimate_does_not_depend_on_the_evaluation_reference(platform):
    seq = make_synthetic(platform, seed=0, duration_s=20.0)
    ref = seq.reference
    baseline = run_strapdown(seq.imu, oracle_initial_state(ref))

    rng = np.random.default_rng(1)
    p_bad, v_bad, q_bad = ref.p_w.copy(), ref.v_w.copy(), ref.q_wb.copy()
    p_bad[1:] += rng.normal(0.0, 50.0, p_bad[1:].shape)
    v_bad[1:] += rng.normal(0.0, 5.0, v_bad[1:].shape)
    q_bad[1:] = q_bad[1:][::-1]
    corrupted = Reference(ref.t_s, p_bad, ref.valid, v_bad, q_bad)

    again = run_strapdown(seq.imu, oracle_initial_state(corrupted))
    for a, b in zip(baseline, again, strict=True):
        assert np.array_equal(a, b)


def test_strapdown_takes_only_the_inertial_input_and_the_initial_state():
    assert list(inspect.signature(run_strapdown).parameters) == [
        "imu",
        "initial_state",
        "dt_max_factor",
        "g_w",
    ]


@pytest.mark.parametrize("platform", ["ugv", "uav"])
def test_outage_window_is_half_open(platform):
    seq = make_synthetic(platform, seed=0, duration_s=30.0, cfg={"outage_s": [10.0, 20.0]})
    times = {round(o.t_s, 6) for o in seq.observations}
    assert 9.0 in times and 20.0 in times
    assert 10.0 not in times and 19.0 not in times
    assert seq.hidden_observations == 10
    assert all(not (10.0 <= t < 20.0) for t in times)


def test_manifest_traces_the_simulated_observations(tmp_path):
    cfg = load_config("configs/base.yaml", "configs/experiments/smoke_synthetic.yaml", "ugv", 3)
    cfg["results_dir"] = str(tmp_path / "runs")
    cfg["db_path"] = str(tmp_path / "results.sqlite")
    registry = tmp_path / "registry.csv"
    registry.write_text("run_id,status,artifact_uri,sha256\n")
    cfg["registry"] = str(registry)
    run_synthetic(cfg)
    bundle = next((tmp_path / "runs").iterdir())
    manifest = json.loads((bundle / "manifest.json").read_text())
    provenance = manifest["observations"]["provenance"]
    assert provenance["generator"] == POSITION_GENERATOR
    assert provenance["seed"] == 3
    assert provenance["config_hash"] == manifest["method"]["config_hash"]
    assert manifest["observations"]["used_by_estimator"] == 0
    assert manifest["data"]["source_hash"]
