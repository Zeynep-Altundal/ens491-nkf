"""Data objects and time validation of docs/interfaces.md, sections 8 and 9."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Section 8.1: the only fields that may reach the filter or the network.
IMU_ALLOWLIST = ("t_s", "gyro_rad_s", "accel_m_s2")
KITTI_ADMITTED = {"gyro_rad_s": ("wx", "wy", "wz"), "accel_m_s2": ("ax", "ay", "az")}
KITTI_REJECTED = ("af", "al", "au", "wf", "wl", "wu")

SOURCES = ("simulated_from_reference", "dataset_sensor", "pseudo_measurement")
POSITION_GENERATOR = "simulated_position_updates"


@dataclass(frozen=True)
class ImuSequence:
    """Inertial input. Carries the allowlisted fields and nothing else."""

    t_s: np.ndarray
    gyro_rad_s: np.ndarray
    accel_m_s2: np.ndarray


@dataclass(frozen=True)
class Reference:
    """Evaluation-only reference. Never passed to the filter or the network."""

    t_s: np.ndarray
    p_w: np.ndarray
    valid: np.ndarray
    v_w: np.ndarray | None = None
    q_wb: np.ndarray | None = None


@dataclass(frozen=True)
class Observation:
    """One observation record, section 8.3."""

    t_s: float
    kind: str
    z: np.ndarray
    unit: str
    frame: str
    r_meas: np.ndarray
    source: str
    valid: bool = True
    provenance: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.source not in SOURCES:
            raise ValueError(f"unknown observation source: {self.source}")
        r = np.asarray(self.r_meas, dtype=np.float64)
        m = np.asarray(self.z).shape[0]
        if r.shape != (m, m) or not np.allclose(r, r.T) or np.any(np.linalg.eigvalsh(r) <= 0.0):
            raise ValueError("r_meas must be symmetric positive definite and match z")
        if self.source == "simulated_from_reference":
            required = {"generator", "config_hash", "seed"}
            if not required <= set(self.provenance):
                raise ValueError("a simulated observation requires generator provenance")
            if self.provenance["generator"] != POSITION_GENERATOR:
                raise ValueError("a simulated observation must come from the declared generator")


def simulate_position_updates(reference, rate_hz, noise_std_m, seed, config_hash):
    """Declared generator of simulated position updates, section 8.3.

    The only path by which reference data reaches the filter after initialisation.
    """
    if not (rate_hz > 0.0 and noise_std_m > 0.0):
        raise ValueError("rate and noise standard deviation must be strictly positive")
    rng = np.random.default_rng(seed)
    r_meas = np.eye(3) * noise_std_m**2
    provenance = {"generator": POSITION_GENERATOR, "config_hash": config_hash, "seed": seed}
    out = []
    next_time = float(reference.t_s[0])
    for t, p, ok in zip(reference.t_s, reference.p_w, reference.valid, strict=True):
        if t + 1e-12 < next_time:
            continue
        next_time = float(t) + 1.0 / rate_hz
        if not ok:
            continue
        z = np.asarray(p, dtype=np.float64) + rng.normal(0.0, noise_std_m, 3)
        out.append(
            Observation(
                float(t),
                "position",
                z,
                "m",
                "w",
                r_meas,
                "simulated_from_reference",
                True,
                provenance,
            )
        )
    return out


def count_updates_by_source(observations):
    """Per-source update counts for the run manifest, section 8.3."""
    counts = dict.fromkeys(SOURCES, 0)
    for obs in observations:
        if obs.valid:
            counts[obs.source] += 1
    return counts


@dataclass(frozen=True)
class Segment:
    """Half-open sample range [start, stop) without a timestamp gap."""

    start: int
    stop: int
    duration_s: float


def split_into_segments(t_s, gyro_rad_s, accel_m_s2, dt_max_factor=5.0):
    """Time validation and gap splitting, section 9. Returns (segments, gap_count)."""
    t = np.asarray(t_s, dtype=np.float64)
    if not 1.0 < dt_max_factor <= 20.0:
        raise ValueError("dt_max_factor must lie in (1, 20]")
    if t.ndim != 1 or t.size < 2:
        raise ValueError("at least two samples are required")
    dt = np.diff(t)
    if not np.all(np.isfinite(t)) or np.any(dt <= 0.0):
        raise ValueError("timestamps must be finite and strictly increasing")
    dt_max = dt_max_factor * float(np.median(dt))
    finite = np.all(np.isfinite(gyro_rad_s), axis=1) & np.all(np.isfinite(accel_m_s2), axis=1)
    segments, gaps, start = [], 0, None
    for i in range(t.size):
        if not finite[i]:
            if start is not None:
                segments.append(Segment(start, i, float(t[i - 1] - t[start])))
            start, gaps = None, gaps + 1
            continue
        if start is None:
            start = i
        elif t[i] - t[i - 1] > dt_max:
            segments.append(Segment(start, i, float(t[i - 1] - t[start])))
            start, gaps = i, gaps + 1
    if start is not None:
        segments.append(Segment(start, t.size, float(t[-1] - t[start])))
    return segments, gaps


def segment_status(segment, min_duration_s, has_initial_state):
    """Run decision for one segment, section 9."""
    if segment.duration_s < min_duration_s:
        return "segment_too_short"
    if not has_initial_state:
        return "no_initial_state"
    return "ok"


def outage_window_status(start_index, stop_index, segments):
    """An outage window must lie entirely inside one segment, section 9."""
    for seg in segments:
        if seg.start <= start_index and stop_index <= seg.stop:
            return "ok"
    return "timestamp_gap"
