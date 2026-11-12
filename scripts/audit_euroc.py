#!/usr/bin/env python3
"""Read-only pre-audit of one EuRoC MAV flight in ASL format.

Checks the time stamps of the inertial and reference streams, their overlap, the
direction of gravity in the reference frame and the consistency of the gyroscope with
the reference orientation. No filter is run and nothing is written.

Usage:
    python scripts/audit_euroc.py --mav0 <flight>/mav0
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

from nkf.filters import conventions as c


def _load(path):
    stamps = np.loadtxt(path, delimiter=",", skiprows=1, usecols=0, dtype=np.int64)
    values = np.loadtxt(path, delimiter=",", skiprows=1)[:, 1:]
    return stamps, values


def _timing(t_ns):
    dt = np.diff(t_ns) / 1e6
    return {
        "samples": int(len(t_ns)),
        "duration_s": round(float((t_ns[-1] - t_ns[0]) / 1e9), 3),
        "dt_ms_mean": round(float(dt.mean()), 4),
        "dt_ms_std": round(float(dt.std()), 4),
        "dt_ms_min": round(float(dt.min()), 4),
        "dt_ms_max": round(float(dt.max()), 4),
        "non_increasing": int((dt <= 0).sum()),
        "gaps_above_5x_median": int((dt > 5 * np.median(dt)).sum()),
    }


def _yaml_numbers(path):
    out = {}
    for key in (
        "rate_hz",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
        "accelerometer_noise_density",
        "accelerometer_random_walk",
    ):
        match = re.search(rf"^{key}:\s*([0-9.eE+-]+)", path.read_text(), flags=re.M)
        out[key] = float(match.group(1)) if match else None
    return out


def _rms(values):
    return float(np.sqrt(np.mean(np.square(values))))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--mav0", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=6000)
    args = parser.parse_args()

    t_imu, imu = _load(args.mav0 / "imu0" / "data.csv")
    t_ref, ref = _load(args.mav0 / "state_groundtruth_estimate0" / "data.csv")

    idx = np.clip(np.searchsorted(t_imu, t_ref), 1, len(t_imu) - 1)
    near = np.where(np.abs(t_imu[idx] - t_ref) < np.abs(t_imu[idx - 1] - t_ref), idx, idx - 1)
    offset = (t_ref - t_imu[near]).astype(np.float64)

    gyro, accel = imu[near, 0:3], imu[near, 3:6]
    quat, b_g, b_a = ref[:, 3:7], ref[:, 10:13], ref[:, 13:16]
    rot = np.array([c.quat_to_rot(q) for q in quat])
    force = accel - b_a

    dt = np.diff(t_ref) / 1e9
    n = min(args.steps, len(dt) - 1)

    def residual(rate, left=False):
        out = []
        for k in range(n):
            inc = c.so3_exp(rate(k) * dt[k])
            pred = inc @ rot[k] if left else rot[k] @ inc
            out.append(np.linalg.norm(c.so3_log(pred.T @ rot[k + 1])))
        return _rms(out)

    report = {
        "imu_sensor": _yaml_numbers(args.mav0 / "imu0" / "sensor.yaml"),
        "imu_timing": _timing(t_imu),
        "reference_timing": _timing(t_ref),
        "reference_start_after_imu_start_s": round(float((t_ref[0] - t_imu[0]) / 1e9), 4),
        "reference_end_before_imu_end_s": round(float((t_imu[-1] - t_ref[-1]) / 1e9), 4),
        "nearest_imu_offset_ns": {
            "min": float(offset.min()),
            "max": float(offset.max()),
            "abs_mean": round(float(np.abs(offset).mean()), 2),
        },
        "accel_norm_mean_m_s2": round(float(np.linalg.norm(imu[:, 3:6], axis=1).mean()), 4),
        "reference_quaternion_norm_deviation_max": float(
            np.abs(np.linalg.norm(quat, axis=1) - 1).max()
        ),
        "reference_gyro_bias_first_last": [b_g[0].tolist(), b_g[-1].tolist()],
        "reference_accel_bias_first_last": [b_a[0].tolist(), b_a[-1].tolist()],
        "mean_specific_force_in_reference_frame_R": np.round(
            np.einsum("nij,nj->ni", rot, force).mean(0), 4
        ).tolist(),
        "mean_specific_force_with_R_transposed": np.round(
            np.einsum("nji,nj->ni", rot, force).mean(0), 4
        ).tolist(),
        "gyro_residual_rad_per_step": {
            "steps": n,
            "right_multiplied_hold_k": residual(lambda k: gyro[k] - b_g[k]),
            "right_multiplied_hold_k_plus_1": residual(lambda k: gyro[k + 1] - b_g[k]),
            "right_multiplied_mean_of_k_and_k_plus_1": residual(
                lambda k: 0.5 * (gyro[k] + gyro[k + 1]) - b_g[k]
            ),
            "left_multiplied_hold_k": residual(lambda k: gyro[k] - b_g[k], left=True),
        },
        "gyro_change_per_step_rms_rad_s": round(
            _rms(np.linalg.norm(np.diff(gyro[:n], axis=0), axis=1)), 4
        ),
    }
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
