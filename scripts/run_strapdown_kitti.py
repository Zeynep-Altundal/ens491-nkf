#!/usr/bin/env python3
"""Strapdown dead reckoning on one KITTI raw drive, compared with the OXTS reference.

Oracle-initialised baseline (docs/interfaces.md, section 10): position, velocity and
orientation at the first sample come from the reference; biases are zero; no update is
applied afterwards. The output is a descriptive first check of the data path and the
conventions. It is not a performance result.

Usage:
    python scripts/run_strapdown_kitti.py --oxts-dir <drive>/oxts --out results/runs
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from nkf.baselines.strapdown import (
    drift_metrics,
    oracle_initial_state,
    run_strapdown,
    state_diagnostics,
)
from nkf.data.kitti import load_kitti_oxts
from nkf.data.sequence import split_into_segments


def _git(*args):
    try:
        return subprocess.run(
            ["git", *args], capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _hash_inputs(oxts_dir):
    digest = hashlib.sha256()
    for path in [oxts_dir / "timestamps.txt", *sorted((oxts_dir / "data").glob("*.txt"))]:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--oxts-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("results/runs"))
    parser.add_argument("--sequence-id", default=None)
    args = parser.parse_args()

    started = time.time()
    imu, reference, meta = load_kitti_oxts(args.oxts_dir)
    segments, gaps = split_into_segments(imu.t_s, imu.gyro_rad_s, imu.accel_m_s2)
    initial = oracle_initial_state(reference)
    p, v, q = run_strapdown(imu, initial)
    metrics = drift_metrics(imu.t_s, p, reference.p_w)
    diagnostics = state_diagnostics(imu.t_s, p, v, q)
    if not diagnostics["all_finite"]:
        raise SystemExit("STOP: non-finite state in the estimated trajectory")

    sha = _git("rev-parse", "HEAD")
    dirty = _git("status", "--porcelain") not in ("", "unknown")
    sequence_id = args.sequence_id or args.oxts_dir.resolve().parent.name
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    run_id = f"{stamp}_ugv_strapdown_s00_{sha[:8]}"
    out = args.out / run_id
    out.mkdir(parents=True, exist_ok=True)

    err = p - reference.p_w
    err_3d = np.linalg.norm(err, axis=1)
    err_h = np.linalg.norm(err[:, :2], axis=1)
    table = np.column_stack([imu.t_s, p, reference.p_w, err_3d, err_h, err[:, 2]])
    header = (
        "t_s,est_e_m,est_n_m,est_u_m,ref_e_m,ref_n_m,ref_u_m,"
        "error_3d_m,error_horizontal_m,error_vertical_m"
    )
    np.savetxt(out / "trajectory.csv", table, delimiter=",", header=header, comments="", fmt="%.9f")

    manifest = {
        "run_id": run_id,
        "purpose": "first data-path and convention check; not a performance result",
        "method": "strapdown",
        "integration": "zero-order hold on sample k (contract section 4)",
        "inertial_fields": ["wx", "wy", "wz", "ax", "ay", "az"],
        "reference_use": "initial state at the first sample and evaluation only",
        "platform": "ugv",
        "sequence_id": sequence_id,
        "init": initial.init,
        "updates_by_source": {
            "simulated_from_reference": 0,
            "dataset_sensor": 0,
            "pseudo_measurement": 0,
        },
        "bias_estimation": False,
        "gravity_w": [0.0, 0.0, -9.81],
        "code_sha": sha,
        "dirty_tree": dirty,
        "input_sha256": _hash_inputs(args.oxts_dir),
        "segments": len(segments),
        "timestamp_gaps": gaps,
        "dataset": meta,
        "metrics": metrics,
        "diagnostics": diagnostics,
        "limitations": [
            "single drive, close to straight, no stop: no statement on turns or stops",
            "oracle initial state: not evidence of operational GNSS-denied navigation",
            "initial biases set to zero and not estimated",
            "inertial input and reference originate from the same OXTS unit",
            "zero-order-hold integration has an error of first order in the step",
        ],
        "elapsed_s": round(time.time() - started, 3),
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (left, right) = plt.subplots(1, 2, figsize=(12, 5.2))
        left.plot(reference.p_w[:, 0], reference.p_w[:, 1], label="OXTS reference")
        left.plot(p[:, 0], p[:, 1], "--", label="Strapdown")
        left.set_xlabel("East [m]")
        left.set_ylabel("North [m]")
        left.set_aspect("equal", adjustable="datalim")
        left.grid(True, alpha=0.3)
        left.legend()
        left.set_title("A. Horizontal trajectory, local ENU")
        right.plot(imu.t_s, err_3d, label="3D")
        right.plot(imu.t_s, err_h, ":", label="Horizontal")
        right.set_xlabel("Elapsed time [s]")
        right.set_ylabel("Position error [m]")
        right.grid(True, alpha=0.3)
        right.legend()
        right.set_title("B. Position error against the reference")
        fig.suptitle(
            f"KITTI raw {sequence_id}: strapdown dead reckoning\n"
            f"oracle initial state, zero initial bias estimate, no measurement update, "
            f"zero-order hold; {metrics['duration_s']:.2f} s, {metrics['samples']} samples",
            fontsize=10,
        )
        fig.tight_layout()
        fig.savefig(out / "strapdown.png", dpi=150)
        figure = str(out / "strapdown.png")
    except ImportError:
        figure = "not produced: matplotlib is not installed"

    print(
        json.dumps(
            {"run_id": run_id, "out": str(out), "figure": figure, **metrics, **diagnostics},
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
