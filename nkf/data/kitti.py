"""KITTI raw OXTS loader (unsynchronised `extract` variant), docs/interfaces.md sections 8 and 11.

The inertial input and the reference are returned as two separate objects. Only the
allowlisted fields enter the inertial input; the reference is for evaluation, for the
oracle initial state and for the declared generator of simulated position updates.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from nkf.data.sequence import ImuSequence, Reference, select_kitti_inertial
from nkf.filters import conventions as c

# Column order of the OXTS records. The dataset format file mislabels column 13 (`az`);
# fields are read by position.
OXTS_FIELDS = (
    "lat", "lon", "alt", "roll", "pitch", "yaw", "vn", "ve", "vf", "vl", "vu",
    "ax", "ay", "az", "af", "al", "au", "wx", "wy", "wz", "wf", "wl", "wu",
    "pos_accuracy", "vel_accuracy", "navstat", "numsats", "posmode", "velmode", "orimode",
)  # fmt: skip
_COL = {name: i for i, name in enumerate(OXTS_FIELDS)}

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def _timestamp_ns(line):
    """Integer nanoseconds from a 'YYYY-MM-DD hh:mm:ss.fffffffff' line (treated as UTC)."""
    date, clock = line.split()
    hms, _, frac = clock.partition(".")
    stamp = datetime.strptime(f"{date} {hms}", "%Y-%m-%d %H:%M:%S").replace(tzinfo=UTC)
    seconds = int(stamp.timestamp())
    return seconds * 1_000_000_000 + int(frac.ljust(9, "0")[:9])


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * np.sin(lat) ** 2)
    return np.stack(
        [
            (n + alt_m) * np.cos(lat) * np.cos(lon),
            (n + alt_m) * np.cos(lat) * np.sin(lon),
            (n * (1.0 - WGS84_E2) + alt_m) * np.sin(lat),
        ],
        axis=-1,
    )


def geodetic_to_enu(lat_deg, lon_deg, alt_m, origin):
    """Local ENU coordinates about `origin` = (lat_deg, lon_deg, alt_m), WGS-84 via ECEF."""
    lat0, lon0 = math.radians(origin[0]), math.radians(origin[1])
    d = geodetic_to_ecef(lat_deg, lon_deg, alt_m) - geodetic_to_ecef(*origin)
    sl, cl, so, co = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    rot = np.array([[-so, co, 0.0], [-sl * co, -sl * so, cl], [cl * co, cl * so, sl]])
    return d @ rot.T


def quat_from_kitti_rpy(roll, pitch, yaw):
    """q_wb for R_wb = Rz(yaw) Ry(pitch) Rx(roll), section 11 (PROPOSED mapping)."""
    qz = c.quat_from_rotvec(np.array([0.0, 0.0, yaw]))
    qy = c.quat_from_rotvec(np.array([0.0, pitch, 0.0]))
    qx = c.quat_from_rotvec(np.array([roll, 0.0, 0.0]))
    return c.quat_mul(qz, c.quat_mul(qy, qx))


def load_kitti_oxts(oxts_dir):
    """Read one drive. Returns (ImuSequence, Reference, metadata)."""
    oxts_dir = Path(oxts_dir)
    stamps = [ln for ln in (oxts_dir / "timestamps.txt").read_text().splitlines() if ln.strip()]
    files = sorted((oxts_dir / "data").glob("*.txt"))
    if len(files) != len(stamps) or len(files) < 2:
        raise ValueError("timestamp and record counts differ or fewer than two samples")
    raw = np.array([[float(x) for x in f.read_text().split()] for f in files])
    if raw.shape[1] != len(OXTS_FIELDS):
        raise ValueError(f"expected {len(OXTS_FIELDS)} fields per record, found {raw.shape[1]}")
    ns = np.array([_timestamp_ns(ln) for ln in stamps], dtype=np.int64)
    t_s = (ns - ns[0]) / 1e9

    admitted = select_kitti_inertial(["wx", "wy", "wz", "ax", "ay", "az"])
    gyro = raw[:, [_COL[n] for n in admitted[:3]]]
    accel = raw[:, [_COL[n] for n in admitted[3:]]]
    imu = ImuSequence(t_s, gyro, accel)

    origin = (raw[0, _COL["lat"]], raw[0, _COL["lon"]], raw[0, _COL["alt"]])
    p_w = geodetic_to_enu(raw[:, _COL["lat"]], raw[:, _COL["lon"]], raw[:, _COL["alt"]], origin)
    v_w = raw[:, [_COL["ve"], _COL["vn"], _COL["vu"]]]
    q_wb = np.array(
        [quat_from_kitti_rpy(r[_COL["roll"]], r[_COL["pitch"]], r[_COL["yaw"]]) for r in raw]
    )
    reference = Reference(t_s, p_w, np.ones(len(t_s), dtype=bool), v_w, q_wb)
    metadata = {
        "dataset": "kitti_raw_oxts_extract",
        "samples": int(len(t_s)),
        "origin_lat_lon_alt": [float(x) for x in origin],
        "numsats_min": int(raw[:, _COL["numsats"]].min()),
        "posmode_values": sorted({int(x) for x in raw[:, _COL["posmode"]]}),
        "pos_accuracy_max_m": float(raw[:, _COL["pos_accuracy"]].max()),
    }
    return imu, reference, metadata
