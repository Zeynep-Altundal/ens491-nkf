"""Classical error-state Kalman filter (ESKF), docs/interfaces.md sections 4-8 and
docs/theory.md sections 4-7.

The filter assembles the frozen building blocks of `nkf.filters.conventions`; it adds no
new mathematics. Measurement models: position (interfaces section 6) and the ground-vehicle
pseudo-measurements of `nkf.baselines.constraints_ugv` (theory section 7).

Scope and limits:
- No noise value, constraint covariance or detector threshold is chosen here. They are
  arguments; values per dataset are OPEN (interfaces section 12).
- No outlier gating. Every valid observation passed to `step` is applied.
- The filter never receives the reference (interfaces sections 8.2, 8.6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from nkf.baselines.constraints_ugv import nhc_measurement, zupt_measurement
from nkf.data.sequence import Observation, assign_observations, split_into_segments
from nkf.filters import conventions as c

POSITION = "position"
NHC = "constraint:nhc"
ZUPT = "constraint:zupt"
KINDS = (POSITION, NHC, ZUPT)


@dataclass(frozen=True)
class NoiseDensities:
    """Noise densities of interfaces section 5.4 (sigma, not sigma squared)."""

    gyro_density: float
    accel_density: float
    gyro_bias_rw: float
    accel_bias_rw: float

    def q_c(self):
        return c.continuous_noise_cov(
            self.gyro_density, self.accel_density, self.gyro_bias_rw, self.accel_bias_rw
        )


class Eskf:
    """ESKF with the API of interfaces section 8.6.

    `init(initial_state, p0)` sets the state once; `step(gyro, accel, dt_s, observations)`
    propagates over one inertial interval and then applies the observations assigned to it.
    """

    def __init__(self, noise, g_w=c.GRAVITY_W, rot_vehicle_from_body=None):
        self._q_c = noise.q_c()
        self._g_w = np.asarray(g_w, dtype=np.float64)
        self._rot_vb = rot_vehicle_from_body
        self._x = None
        self._p = None

    def init(self, initial_state, p0):
        p0 = np.array(p0, dtype=np.float64)
        if p0.shape != (15, 15) or not np.allclose(p0, p0.T):
            raise ValueError("p0 must be a symmetric [15, 15] matrix")
        if np.any(np.linalg.eigvalsh(p0) <= 0.0):
            raise ValueError("p0 must be positive definite")
        s = initial_state
        q = np.asarray(s.q, dtype=np.float64)
        self._x = (
            np.array(s.p, dtype=np.float64),
            np.array(s.v, dtype=np.float64),
            q / np.linalg.norm(q),
            np.array(s.b_g, dtype=np.float64),
            np.array(s.b_a, dtype=np.float64),
        )
        self._p = p0

    @property
    def state(self):
        return tuple(np.array(a) for a in self._x)

    @property
    def covariance(self):
        return np.array(self._p)

    def step(self, gyro_rad_s, accel_m_s2, dt_s, observations=()):
        """One interval (t_k, t_{k+1}]: propagation, then the observations in order.

        Returns (state, covariance, diagnostics). `diagnostics["updates"]` holds one record
        per applied observation: kind, source, innovation and normalised innovation squared.
        """
        if self._x is None:
            raise RuntimeError("init must be called before step")
        if not (math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError("dt_s must be finite and strictly positive")
        p, v, q, b_g, b_a = self._x
        rot = c.quat_to_rot(q)
        f_hat = np.asarray(accel_m_s2, dtype=np.float64) - b_a
        w_hat = np.asarray(gyro_rad_s, dtype=np.float64) - b_g
        f_mat, g_mat = c.error_jacobians(rot, f_hat, w_hat)
        phi, q_d = c.discretise(f_mat, g_mat, self._q_c, dt_s)
        self._x = c.propagate_nominal(p, v, q, b_g, b_a, gyro_rad_s, accel_m_s2, dt_s, self._g_w)
        self._p = c.propagate_cov(self._p, phi, q_d)

        updates = [self._update(obs) for obs in observations if obs.valid]
        diagnostics = {"updates": updates, "all_finite": self._finite()}
        return self.state, self.covariance, diagnostics

    def _measurement(self, obs):
        p, v, q, _, _ = self._x
        z = np.asarray(obs.z, dtype=np.float64)
        if obs.kind == POSITION:
            if obs.frame != "w" or z.shape != (3,):
                raise ValueError("a position observation is a [3] vector in frame w")
            h = np.zeros((3, 15))
            h[:, c.DP] = np.eye(3)
            return z - p, h
        if obs.source != "pseudo_measurement":
            raise ValueError("a motion constraint must have source 'pseudo_measurement'")
        if obs.kind == NHC:
            y0, h = nhc_measurement(v, q, self._rot_vb)
        elif obs.kind == ZUPT:
            y0, h = zupt_measurement(v)
        else:
            raise ValueError(f"unknown observation kind: {obs.kind}")
        if z.shape != y0.shape:
            raise ValueError(f"{obs.kind} expects z of shape {y0.shape}")
        # The models return y for z = 0; for a general z, y = z - h(x) = z + y0.
        return z + y0, h

    def _update(self, obs):
        y, h = self._measurement(obs)
        r_meas = np.asarray(obs.r_meas, dtype=np.float64)
        s = h @ self._p @ h.T + r_meas
        nis = float(y @ np.linalg.solve(s, y))
        dx, p_upd = c.joseph_update(self._p, h, r_meas, y)
        self._x = c.inject(*self._x, dx)
        self._p = c.reset_cov(p_upd, dx[c.DTH])
        return {"kind": obs.kind, "source": obs.source, "innovation": y, "nis": nis}

    def _finite(self):
        return bool(all(np.all(np.isfinite(a)) for a in self._x) and np.all(np.isfinite(self._p)))


def constraint_observation(kind, t_s, std_m_s):
    """Pseudo-measurement record for a motion constraint (no value from the reference)."""
    if kind == NHC:
        m = 2
    elif kind == ZUPT:
        m = 3
    else:
        raise ValueError(f"not a motion constraint: {kind}")
    if not (math.isfinite(std_m_s) and std_m_s > 0.0):
        raise ValueError("constraint standard deviation must be finite and strictly positive")
    return Observation(
        float(t_s),
        kind,
        np.zeros(m),
        "m/s",
        "b",
        np.eye(m) * std_m_s**2,
        "pseudo_measurement",
        True,
        {"generator": "constraint_policy"},
    )


def run_eskf(
    imu,
    initial_state,
    p0,
    noise,
    observations=(),
    nhc_std_m_s=None,
    zupt_std_m_s=None,
    stationary=None,
    dt_max_factor=5.0,
    rot_vehicle_from_body=None,
):
    """Run the ESKF over one gap-free sequence.

    `observations`: position records, already filtered by the outage mask.
    `nhc_std_m_s`: if given, an NHC pseudo-measurement at every inertial step.
    `zupt_std_m_s` and `stationary` ([N] bool from a causal detector): a ZUPT
    pseudo-measurement at step k when `stationary[k + 1]` is true.
    Returns a dict with per-sample p, v, q, the diagonal of P, NIS records and counts.
    """
    segments, gaps = split_into_segments(imu.t_s, imu.gyro_rad_s, imu.accel_m_s2, dt_max_factor)
    if gaps or len(segments) != 1:
        raise ValueError(f"sequence has {gaps} gap(s); run each segment separately")
    if zupt_std_m_s is not None and stationary is None:
        raise ValueError("ZUPT needs the output of a stationary detector")
    t = imu.t_s
    n = len(t)
    assigned, unassigned = assign_observations(t, observations)
    filt = Eskf(noise, rot_vehicle_from_body=rot_vehicle_from_body)
    filt.init(initial_state, p0)
    p_out, v_out, q_out = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros((n, 4))
    p_diag = np.zeros((n, 15))
    p_out[0], v_out[0], q_out[0] = filt.state[0], filt.state[1], filt.state[2]
    p_diag[0] = np.diag(filt.covariance)
    nis, offsets = [], []
    counts = {kind: 0 for kind in KINDS}
    diverged_at = None
    dt = np.diff(t)
    for k in range(n - 1):
        obs_k = []
        for obs, offset in assigned.get(k, []):
            obs_k.append(obs)
            offsets.append(offset)
        if nhc_std_m_s is not None:
            obs_k.append(constraint_observation(NHC, t[k + 1], nhc_std_m_s))
        if zupt_std_m_s is not None and stationary[k + 1]:
            obs_k.append(constraint_observation(ZUPT, t[k + 1], zupt_std_m_s))
        state, cov, diag = filt.step(imu.gyro_rad_s[k], imu.accel_m_s2[k], float(dt[k]), obs_k)
        for u in diag["updates"]:
            counts[u["kind"]] += 1
            nis.append((k + 1, u["kind"], u["nis"]))
        p_out[k + 1], v_out[k + 1], q_out[k + 1] = state[0], state[1], state[2]
        p_diag[k + 1] = np.diag(cov)
        if not diag["all_finite"]:
            diverged_at = k + 1
            p_out[k + 1 :] = np.nan
            v_out[k + 1 :] = np.nan
            q_out[k + 1 :] = np.nan
            p_diag[k + 1 :] = np.nan
            break
    return {
        "p": p_out,
        "v": v_out,
        "q": q_out,
        "p_diag": p_diag,
        "nis": nis,
        "update_counts": counts,
        "observation_offsets_s": np.asarray(offsets),
        "unassigned_observations": len(unassigned),
        "diverged_at": diverged_at,
    }
