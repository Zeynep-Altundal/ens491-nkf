#!/usr/bin/env python3
"""Classical ESKF with NHC / ZUPT on one KITTI raw drive, compared with the OXTS reference.

Oracle-initialised (docs/interfaces.md, section 10). Optional simulated position updates come
only from the declared generator and are hidden during an optional outage window. All noise
values, constraint standard deviations and detector thresholds are command-line arguments; the
defaults are UNTUNED placeholder starting points with no recorded source (OPEN). The output is
a first run on real data, not a performance result.

Usage:
    python scripts/run_eskf_kitti.py --oxts-dir <drive>/oxts --variant nhc_zupt --out results/runs
    python scripts/run_eskf_kitti.py --oxts-dir <drive>/oxts --variant nhc \
        --pos-rate-hz 1 --outage 60 120
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

from nkf.baselines.eskf import NoiseDensities, run_eskf
from nkf.baselines.strapdown import (
    drift_metrics,
    oracle_initial_state,
    run_strapdown,
    state_diagnostics,
)
from nkf.baselines.zupt_detector import detect_stationary
from nkf.data.kitti import load_kitti_oxts
from nkf.data.sequence import simulate_position_updates, split_into_segments

VARIANTS = ("nhc", "nhc_zupt")


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


def build_parser():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--oxts-dir", type=Path, required=True)
    p.add_argument("--out", type=Path, default=Path("results/runs"))
    p.add_argument("--sequence-id", default=None)
    p.add_argument("--variant", choices=VARIANTS, default="nhc")
    # Noise densities (sigma, interfaces 5.4). UNTUNED placeholders.
    p.add_argument("--gyro-density", type=float, default=1e-4)
    p.add_argument("--accel-density", type=float, default=1e-3)
    p.add_argument("--gyro-bias-rw", type=float, default=1e-6)
    p.add_argument("--accel-bias-rw", type=float, default=1e-5)
    # Initial standard deviations for P0. UNTUNED placeholders.
    p.add_argument("--p0-pos-m", type=float, default=0.1)
    p.add_argument("--p0-vel-m-s", type=float, default=0.1)
    p.add_argument("--p0-angle-rad", type=float, default=0.01)
    p.add_argument("--p0-gyro-bias", type=float, default=1e-3)
    p.add_argument("--p0-accel-bias", type=float, default=1e-2)
    # Constraints and detector. UNTUNED placeholders.
    p.add_argument("--nhc-std-m-s", type=float, default=0.1)
    p.add_argument("--zupt-std-m-s", type=float, default=0.05)
    p.add_argument("--det-window", type=int, default=200)
    p.add_argument("--det-gyro-thresh", type=float, default=0.01)
    p.add_argument("--det-accel-std-thresh", type=float, default=0.05)
    # Optional simulated position updates with an outage window.
    p.add_argument("--pos-rate-hz", type=float, default=None)
    p.add_argument("--pos-noise-m", type=float, default=1.0)
    p.add_argument("--outage", type=float, nargs=2, metavar=("START_S", "END_S"), default=None)
    p.add_argument("--seed", type=int, default=0)
    return p


def p0_matrix(args):
    std = np.concatenate(
        [
            np.full(3, args.p0_pos_m),
            np.full(3, args.p0_vel_m_s),
            np.full(3, args.p0_angle_rad),
            np.full(3, args.p0_gyro_bias),
            np.full(3, args.p0_accel_bias),
        ]
    )
    return np.diag(std**2)


def config_dict(args):
    keys = (
        "variant gyro_density accel_density gyro_bias_rw accel_bias_rw p0_pos_m p0_vel_m_s "
        "p0_angle_rad p0_gyro_bias p0_accel_bias nhc_std_m_s zupt_std_m_s det_window "
        "det_gyro_thresh det_accel_std_thresh pos_rate_hz pos_noise_m outage seed"
    ).split()
    return {k: getattr(args, k) for k in keys}


def run(args):
    imu, reference, meta = load_kitti_oxts(args.oxts_dir)
    segments, gaps = split_into_segments(imu.t_s, imu.gyro_rad_s, imu.accel_m_s2)
    initial = oracle_initial_state(reference)
    config = config_dict(args)
    config_hash = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:16]

    observations = []
    if args.pos_rate_hz is not None:
        observations = simulate_position_updates(
            reference, args.pos_rate_hz, args.pos_noise_m, args.seed, config_hash
        )
        if args.outage is not None:
            lo, hi = args.outage
            observations = [o for o in observations if not (lo <= o.t_s <= hi)]
    elif args.outage is not None:
        raise SystemExit("--outage needs --pos-rate-hz (an outage hides position updates)")

    stationary = None
    zupt_std = None
    if args.variant == "nhc_zupt":
        stationary = detect_stationary(
            imu.gyro_rad_s,
            imu.accel_m_s2,
            args.det_window,
            args.det_gyro_thresh,
            args.det_accel_std_thresh,
        )
        zupt_std = args.zupt_std_m_s
    noise = NoiseDensities(
        args.gyro_density, args.accel_density, args.gyro_bias_rw, args.accel_bias_rw
    )
    out = run_eskf(
        imu,
        initial,
        p0_matrix(args),
        noise,
        observations=observations,
        nhc_std_m_s=args.nhc_std_m_s,
        zupt_std_m_s=zupt_std,
        stationary=stationary,
    )
    return imu, reference, meta, segments, gaps, initial, out, stationary, observations, config


def main():
    args = build_parser().parse_args()
    started = time.time()
    imu, reference, meta, segments, gaps, initial, out, stationary, observations, config = run(args)
    config_hash = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:16]

    # Strapdown with the same initial state, for a same-run comparison.
    p_sd, _, _ = run_strapdown(imu, initial)
    p, v, q = out["p"], out["v"], out["q"]
    metrics = drift_metrics(imu.t_s, p, reference.p_w)
    metrics_strapdown = drift_metrics(imu.t_s, p_sd, reference.p_w)
    diagnostics = state_diagnostics(imu.t_s, p, v, q)

    nis = out["nis"]
    nis_summary = {}
    for kind in sorted({k for _, k, _ in nis}):
        values = np.array([x for _, kk, x in nis if kk == kind], dtype=np.float64)
        nis_summary[kind] = {
            "count": int(values.size),
            "median": float(np.median(values)),
            "mean": float(np.mean(values)),
        }

    sha = _git("rev-parse", "HEAD")
    dirty = _git("status", "--porcelain") not in ("", "unknown")
    sequence_id = args.sequence_id or args.oxts_dir.resolve().parent.name
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    run_id = f"{stamp}_ugv_eskf_{args.variant}_s{args.seed:02d}_{sha[:8]}"
    out_dir = args.out / run_id
    out_dir.mkdir(parents=True, exist_ok=True)

    err = p - reference.p_w
    err_3d = np.linalg.norm(err, axis=1)
    table = np.column_stack([imu.t_s, p, reference.p_w, err_3d])
    header = "t_s,est_e_m,est_n_m,est_u_m,ref_e_m,ref_n_m,ref_u_m,error_3d_m"
    np.savetxt(
        out_dir / "trajectory.csv", table, delimiter=",", header=header, comments="", fmt="%.9f"
    )

    manifest = {
        "run_id": run_id,
        "purpose": "first ESKF run on a real drive; noise and thresholds are untuned placeholders",
        "method": f"eskf_{args.variant}",
        "platform": "ugv",
        "sequence_id": sequence_id,
        "init": initial.init,
        "config": config,
        "config_hash": config_hash,
        "config_status": "UNTUNED placeholder starting values; no recorded source (OPEN)",
        "updates_by_kind": out["update_counts"],
        "simulated_position_updates": len(observations),
        "unassigned_observations": out["unassigned_observations"],
        "stationary_samples": None if stationary is None else int(stationary.sum()),
        "diverged_at": out["diverged_at"],
        "nis": nis_summary,
        "code_sha": sha,
        "dirty_tree": dirty,
        "input_sha256": _hash_inputs(args.oxts_dir),
        "segments": len(segments),
        "timestamp_gaps": gaps,
        "dataset": meta,
        "metrics_eskf": metrics,
        "metrics_strapdown_same_init": metrics_strapdown,
        "diagnostics": diagnostics,
        "limitations": [
            "noise densities, P0, constraint standard deviations and detector thresholds are OPEN",
            "oracle initial state: not evidence of operational GNSS-denied navigation",
            "inertial input and reference originate from the same OXTS unit",
            "R_vb = I and neglected lever arm are unverified for KITTI",
            "a single drive: no statement on generalisation",
        ],
        "elapsed_s": round(time.time() - started, 3),
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"run_id": run_id, "out": str(out_dir), **manifest["metrics_eskf"]}, indent=2))
    return out_dir


if __name__ == "__main__":
    main()
