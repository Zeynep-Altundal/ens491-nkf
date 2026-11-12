# ADR-0003: Causal stationary detector for the zero-velocity update

Status: PROPOSED. Not accepted.
Date proposed: 12 November 2026.
Related: `docs/theory.md` section 7.3, `nkf/baselines/zupt_detector.py`, `nkf/baselines/eskf.py`.

## Context

The zero-velocity update (ZUPT) of theory section 7.3 is applied only while the vehicle is
stationary. The theory note leaves the detector undesigned. A detector that reads the
reference velocity would leak ground truth into the filter (interfaces sections 8.2, 10).

## Options

1. Reference speed below a threshold. Rejected: leakage.
2. Inertial-only, trailing window: mean gyroscope norm and standard deviation of the
   specific-force norm below thresholds. Simple, causal, no extra sensor.
3. Generalised likelihood ratio test (SHOE-type, used in foot-mounted navigation). More
   principled, needs noise values that are still OPEN.
4. Learned detector (for example an LSTM on IMU windows). Belongs to the neural track.

## Decision (proposed)

Option 2 as the classical baseline. The decision at sample `k` uses samples
`k - window + 1 ... k` only. Window length and both thresholds are arguments; values per
dataset are OPEN and are selected on training and validation data only. The first
`window - 1` samples are never declared stationary.

## Consequences

- Known failure mode: constant velocity on a straight, smooth path gives low gyroscope and
  specific-force variation and can be declared stationary. On the synthetic UGV sequence,
  which has no vibration, a 0.5 s window produced false detections at about 4 % of the moving
  samples; a 2 s window produced none. Real vehicles add vibration while moving, which works in
  the detector's favour, but this is NOT VERIFIED on KITTI.
- A false ZUPT at speed corrupts the velocity and, through the coupling noted in theory
  section 8, the tilt and accelerometer-bias estimates.
- The detection delay equals at most one window length at the start of each stop.

## Evidence

Synthetic only: `tests/fast/test_eskf.py` (stop found, no false detections on the moving
part with the test window, causality). No real-data evidence.
