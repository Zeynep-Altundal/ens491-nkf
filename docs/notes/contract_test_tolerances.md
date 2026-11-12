# Contract-test tolerances

Status: measured on synthetic values only, 12 November 2026. No dataset is involved.
Scope: the tolerances used in `tests/fast/test_conventions.py` and
`tests/fast/test_contract_oracles.py`. The reference quantities are computed by an
independent model (matrix power series, `numpy.cross`, explicit indices) that does not
call the helper functions of `nkf/filters/conventions.py`.

The tolerances are not universal. Each one is tied to the step or perturbation size
stated below; the error orders are asserted by the tests so that a tolerance cannot be
met by a change of scale alone.

## 1. Discretisation: finite-difference transition against `I + F dt`

| `dt` [s] | Largest absolute difference |
|---|---|
| 4e-3 | 7.15e-05 |
| 2e-3 | 1.79e-05 |
| 1e-3 | 4.47e-06 |
| 5e-4 | 1.12e-06 |

The difference falls by a factor of 4.0 per halving of `dt`: second order, as expected
for a first-order transition matrix. Test tolerance at `dt` = 1e-3: 2e-5 (margin 4.5).
A sign error in any coupling block of `F` changes the transition by an amount of first
order in `dt` and is detected (asserted for four blocks).

## 2. Linearisation: error after one step against the linear prediction

Orientation error scaled along a fixed direction, `dt` = 1e-3.

| Orientation error norm [rad] | Largest absolute residual |
|---|---|
| 0.0566 | 7.94e-06 |
| 0.0283 | 1.99e-06 |
| 0.0141 | 4.99e-07 |
| 0.0071 | 1.25e-07 |

Factor 4.0 per halving: second order in the error state. Test tolerance at 0.014 rad:
1e-5.

## 3. Reset Jacobian: `I - 0.5 [a]x` against the finite-difference reset

| Correction norm [rad] | Difference, contract formula | Difference, opposite sign |
|---|---|---|
| 0.0748 | 8.66e-04 | 6.01e-02 |
| 0.0374 | 2.17e-04 | 3.00e-02 |
| 0.0187 | 5.42e-05 | 1.50e-02 |
| 0.0094 | 1.35e-05 | 7.50e-03 |

The contract formula has an error of second order in the correction (factor 4.0 per
halving); the opposite sign has an error of first order (factor 2.0). The test bound is
the squared correction norm; the opposite sign must exceed ten times that bound.

## 4. Limits

- The first-order reset and the first-order transition are approximations. Their error
  grows with the size of the correction and of the step; the tables give the measured
  size for the stated ranges only.
- Orientation corrections above the tested range (0.075 rad) are not covered.
- The logarithm map is tested up to `pi - 1e-10` rad; at exactly `pi` the sign of the
  axis is not defined and only the rotation is compared.
- The symmetric-positive-definite check of the Joseph update with ill-conditioned
  covariances uses a tolerance of 1e-9 relative to the largest eigenvalue. It is a
  numerical acceptance criterion in float64, not a proof of positive definiteness.
