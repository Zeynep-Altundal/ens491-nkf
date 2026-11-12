"""Synthetic UGV / UAV sequences for W01 pipeline tests.

Conventions follow docs/interfaces.md (ADR-0001):
  - World frame: right-handed, z up. Gravity vector g_w = [0, 0, -9.81] m/s^2.
  - Attitude: rotation body->world R_wb. Quaternions are Hamilton, scalar-first [w, x, y, z].
  - accel_m_s2 = specific force in body frame = R_wb^T (a_w - g_w).
    Stationary and level -> [0, 0, +9.81].
  - gyro_rad_s = body-frame angular rate.
  - Time origin: t_s[0] = 0.0, seconds, strictly increasing.
  - The inertial input and the reference are returned as separate objects (sections 8.1,
    8.2). The reference is for evaluation, for the oracle initial state and for the declared
    generator of simulated position updates; it is never passed to a filter.
"""

import hashlib
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from nkf.data.sequence import ImuSequence, Reference, simulate_position_updates

GRAVITY_W = np.array([0.0, 0.0, -9.81])


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def ugv_trajectory(t):
    """Planar ground vehicle: drive, stop (ZUPT segment 40-50 s), drive with turns."""
    stop = 1.0 - smoothstep((t - 38.0) / 2.0) + smoothstep((t - 50.0) / 2.0)
    speed = 8.0 * smoothstep(t / 5.0) * np.clip(stop, 0.0, 1.0)  # m/s
    yaw_rate = 0.15 * np.sin(2 * np.pi * t / 30.0) * (speed > 0.1)  # rad/s, only turns while moving
    dt = np.diff(t, prepend=t[0])
    yaw = np.cumsum(yaw_rate * dt)
    v_w = np.stack([speed * np.cos(yaw), speed * np.sin(yaw), np.zeros_like(t)], axis=1)
    p_w = np.cumsum(v_w * dt[:, None], axis=0)
    R_wb = Rotation.from_euler("z", yaw[:, None])
    return p_w, R_wb


def uav_trajectory(t):
    """Quadrotor-like: 5 s hover, then a smooth figure-8 at ~1.5 m altitude.

    Attitude follows the thrust direction (no drag), plus a slow yaw oscillation.
    """
    ramp = smoothstep((t - 5.0) / 5.0)
    w = 2 * np.pi / 20.0
    p_w = np.stack(
        [
            2.0 * np.sin(w * t) * ramp,
            1.5 * np.sin(2 * w * t) * ramp,
            1.5 + 0.3 * np.sin(0.5 * w * t) * ramp,
        ],
        axis=1,
    )
    return p_w


def uav_attitude(t, a_w):
    thrust = a_w - GRAVITY_W
    z_b = thrust / np.linalg.norm(thrust, axis=1, keepdims=True)
    yaw = 0.3 * np.sin(2 * np.pi * t / 25.0)
    x_c = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], axis=1)
    y_b = np.cross(z_b, x_c)
    y_b /= np.linalg.norm(y_b, axis=1, keepdims=True)
    x_b = np.cross(y_b, z_b)
    return Rotation.from_matrix(np.stack([x_b, y_b, z_b], axis=2))


def body_rates(R_wb, dt):
    """omega_b such that R_{k+1} = R_k exp(omega_b dt)."""
    rel = R_wb[:-1].inv() * R_wb[1:]
    omega = rel.as_rotvec() / dt
    return np.vstack([omega, omega[-1:]])


def hash_arrays(*arrays):
    h = hashlib.sha256()
    for a in arrays:
        h.update(np.ascontiguousarray(a).tobytes())
    return h.hexdigest()


@dataclass(frozen=True)
class SyntheticSequence:
    """One synthetic sequence in the object layout of docs/interfaces.md, section 8."""

    id: str
    platform: str
    imu: ImuSequence
    reference: Reference
    observations: list
    outage_s: tuple
    hidden_observations: int
    truth: dict
    source_hash: str
    split_id: str


def make_synthetic(platform, seed=0, duration_s=None, imu_hz=None, cfg=None):
    """Build one synthetic sequence for platform 'ugv' or 'uav'."""
    cfg = cfg or {}
    rng = np.random.default_rng(seed)
    if platform == "ugv":
        duration_s = duration_s or 120.0
        imu_hz = imu_hz or 100.0
    elif platform == "uav":
        duration_s = duration_s or 60.0
        imu_hz = imu_hz or 200.0
    else:
        raise ValueError(f"unknown platform {platform}")

    dt = 1.0 / imu_hz
    t = np.arange(0.0, duration_s, dt)

    if platform == "ugv":
        p_w, R_wb = ugv_trajectory(t)
        v_w = np.gradient(p_w, t, axis=0)
        a_w = np.gradient(v_w, t, axis=0)
    else:
        p_w = uav_trajectory(t)
        v_w = np.gradient(p_w, t, axis=0)
        a_w = np.gradient(v_w, t, axis=0)
        R_wb = uav_attitude(t, a_w)

    gyro_true = body_rates(R_wb, dt)
    accel_true = R_wb.inv().apply(a_w - GRAVITY_W)

    gyro_std = cfg.get("gyro_noise_std", 0.002)  # rad/s
    accel_std = cfg.get("accel_noise_std", 0.02)  # m/s^2
    gyro_bias = rng.normal(0.0, cfg.get("gyro_bias_std", 0.001), 3)
    accel_bias = rng.normal(0.0, cfg.get("accel_bias_std", 0.02), 3)
    gyro = gyro_true + gyro_bias + rng.normal(0.0, gyro_std, gyro_true.shape)
    accel = accel_true + accel_bias + rng.normal(0.0, accel_std, accel_true.shape)

    # scipy returns [x, y, z, w]; the contract order is [w, x, y, z].
    q_wb = np.roll(R_wb.as_quat(), 1, axis=1)
    imu = ImuSequence(t, gyro, accel)
    reference = Reference(t, p_w, np.ones(len(t), dtype=bool), v_w, q_wb)

    # 1 Hz simulated position updates from the declared generator; hidden during the outage.
    obs_std = cfg.get("obs_pos_std_m", 0.5)
    outage = tuple(cfg.get("outage_s", [60.0, 90.0] if platform == "ugv" else [30.0, 40.0]))
    generated = simulate_position_updates(
        reference, 1.0, obs_std, seed=seed, config_hash=cfg.get("config_hash", "synthetic")
    )
    observations = [o for o in generated if not (outage[0] <= o.t_s < outage[1])]

    return SyntheticSequence(
        id=f"synthetic_{platform}_s{seed:02d}",
        platform=platform,
        imu=imu,
        reference=reference,
        observations=observations,
        outage_s=outage,
        hidden_observations=len(generated) - len(observations),
        truth={
            "imu_hz": imu_hz,
            "gyro_bias_true": gyro_bias,
            "accel_bias_true": accel_bias,
            "gyro_noise_std": gyro_std,
            "accel_noise_std": accel_std,
        },
        source_hash=hash_arrays(t, gyro, accel, p_w),
        split_id="synthetic",
    )
