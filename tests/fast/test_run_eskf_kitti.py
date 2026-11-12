"""Smoke test of scripts/run_eskf_kitti.py on a synthetic stationary OXTS drive."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

pytestmark = pytest.mark.fast

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "run_eskf_kitti.py"


def _load_script():
    spec = importlib.util.spec_from_file_location("run_eskf_kitti", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _fake_drive(root, n=1200, hz=100):
    """A vehicle at rest: gravity-only accelerometer, zero gyro, zero velocity."""
    (root / "data").mkdir()
    stamps = []
    for k in range(n):
        ns = int(round(k * 1e9 / hz))
        stamps.append(f"2011-09-30 11:50:{ns // 10**9:02d}.{ns % 10**9:09d}")
        fields = [0.0] * 30
        fields[0], fields[1], fields[2] = 49.0, 8.4, 112.0  # lat lon alt
        fields[13] = 9.81  # az: specific force
        fields[14 + 2] = 9.81  # au
        fields[23], fields[24] = 0.1, 0.02
        fields[25:30] = [4, 7, 5, 5, 6]
        (root / "data" / f"{k:010d}.txt").write_text(" ".join(str(x) for x in fields))
    (root / "timestamps.txt").write_text("\n".join(stamps) + "\n")


@pytest.mark.parametrize("variant", ["nhc", "nhc_zupt"])
def test_script_runs_and_writes_manifest(tmp_path, variant):
    script = _load_script()
    oxts = tmp_path / "drive" / "oxts"
    oxts.mkdir(parents=True)
    _fake_drive(oxts)
    args = script.build_parser().parse_args(
        [
            "--oxts-dir", str(oxts),
            "--out", str(tmp_path / "runs"),
            "--variant", variant,
            "--det-window", "100",
        ]
    )  # fmt: skip
    imu, reference, meta, segments, gaps, initial, out, stationary, obs, config = script.run(args)
    assert out["diverged_at"] is None and np.all(np.isfinite(out["p"]))
    assert len(segments) == 1 and gaps == 0
    assert np.linalg.norm(out["p"][-1] - reference.p_w[-1]) < 5.0
    if variant == "nhc_zupt":
        assert stationary.sum() > 0 and out["update_counts"]["constraint:zupt"] > 0
    assert out["update_counts"]["constraint:nhc"] == len(imu.t_s) - 1
    json.dumps(config)


def test_outage_requires_position_rate(tmp_path):
    script = _load_script()
    oxts = tmp_path / "oxts"
    oxts.mkdir()
    _fake_drive(oxts, n=300)
    args = script.build_parser().parse_args(
        ["--oxts-dir", str(oxts), "--outage", "1", "2", "--det-window", "50"]
    )
    with pytest.raises(SystemExit):
        script.run(args)
