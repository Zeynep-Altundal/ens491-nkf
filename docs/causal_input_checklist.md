# Causal-input checklist

Status: DRAFT (issue #3), for review by the lead. Use it for every pull request that adds or
changes an online filter, a stationary detector, a network input or an evaluation script.
It restates `docs/interfaces.md` sections 8 to 10 as yes/no questions and names the test that
checks each one. An answer of "no" or "not checked" goes into the pull request description.

| # | Question | How it is checked |
|---|---|---|
| 1 | Does the online call take only the allowlisted inertial fields and the declared observations? No `Reference`, no ground-truth array, no levelled-frame OXTS field (`af al au wf wl wu`). | Signature checks in `test_units.py` and `test_eskf.py`; `test_data_contract.py`, `test_contract_edges.py` |
| 2 | Does the estimate at sample k stay the same if every later sample is changed or removed? | Truncation and perturbation tests in `test_units.py` (strapdown) and `test_eskf.py` (ESKF) |
| 3 | Are all windows, detectors and features trailing (samples k-w+1 to k)? No centred window, no smoothing over future samples, no backward pass. | Causality test of the detector in `test_eskf.py`; review |
| 4 | Are normalisation constants and tuned values fitted on training data only, never on the validation or locked test sequence they are applied to? | Config and run manifest; review |
| 5 | Does the initial state carry its label (`oracle` or `operational`), and does an `operational` start avoid the reference? | `InitialState.init`; run manifest |
| 6 | Do position updates come only from the declared generator, with the outage window half-open and the hidden updates counted? | `test_pipeline_leakage.py` |
| 7 | Are constraint updates (NHC, ZUPT) marked `pseudo_measurement` and free of reference values? | `test_constraints_ugv.py`, `test_eskf.py` |
| 8 | Are time stamps in seconds from zero, specific force in m/s^2, angular rate in rad/s? | `check_inertial_units` in `test_units.py` |
| 9 | Does the run manifest record the init label, updates by source, code SHA and input hash? | `test_w01_pipeline.py`; review |
| 10 | Is the locked test split untouched (no tuning, no repeated looks)? | Review; split manifest |

Not covered by any test yet: item 4 (fitting statistics) and item 10 (locked split), which
need the split manifests of W02. The ESKF truncation test (item 2) is added with the ESKF
pull request.
