# Related work matrix (W01, issue #6)

Selected work that H-A, H-B, H-C or the UGV/UAV scope needs as a baseline, a method template
or a closest prior result. Bibliographic data checked on 12 Nov 2026 against the sources listed
at the end.

Arm legend: **cov** learned covariance, **gate** learned constraint validity,
**gain** learned Kalman gain, **meas** learned measurement, **free** filter-free,
**train** filter training / consistency.

## 1. Constraint reliability on ground vehicles (Paper A, H-A)

| # | Work | Arm | What is learned, where it enters the filter | Data / platform | Why it matters here | Code |
|---|---|---|---|---|---|---|
| A1 | Brossard, Barrau, Bonnabel. **AI-IMU Dead-Reckoning.** IEEE T-IV 5(4):585-595, 2020 | cov | CNN adapts the pseudo-measurement noise of an IEKF from the IMU window | KITTI, IMU only | Required baseline. Official repo reports 1.10% translational error on KITTI odometry sequences | github.com/mbrossar/ai-imu-dr |
| A2 | Brossard, Barrau, Bonnabel. **RINS-W: Robust Inertial Navigation System on Wheels.** IROS 2019, pp. 2068-2075 | gate | Deep-network zero-velocity detector; IEKF applies zero velocity, no lateral slip and no vertical velocity as pseudo-measurements | Public car dataset, IMU of moderate precision | The validity half of R1. Reports about 20 m final error over a 21 km, >1 h drive | github.com/mbrossar/RINS-W |
| A3 | Feng, Wang, Li, Shen. **SINDy-Based Motion-Dependent Nonholonomic Constraint Covariance Adaptation for GNSS-Denied MEMS/INS Vehicle Navigation.** Micromachines 17(10):1187, 2026 | cov (sparse rule) | Sparse law from causal IMU windows to the lateral/vertical NHC covariance; InEKF remains the only estimator | Vehicle sequences 0027, 0033, 0034, complete GNSS denial | Closest prior result to R1. 3-D RMSE vs fixed NHC: 4.135 to 1.283 m (0027), 14.456 to 4.293 m (0033), 6.735 to 3.929 m (0034). Only baseline is fixed NHC | Not stated |
| A4 | Tang, Niu, Zhang, Li, Liu. **OdoNet: Untethered Speed Aiding for Vehicle Navigation Without Hardware Wheeled Odometer.** arXiv:2109.03091, 2021 | meas | 1-D CNN pseudo-odometer (forward speed) from a single IMU, fused in GNSS/INS | Land vehicle | Forward-speed counterpart of NHC. Positioning error reduced by about 68% with the pseudo-odometer vs about 74% with a hardware odometer | Not stated |

## 2. Learned gain and the gain-vs-noise question (required comparison)

| # | Work | Arm | What is learned | Data | Why it matters here | Code |
|---|---|---|---|---|---|---|
| G1 | Revach, Shlezinger, Ni, López Escoriza, van Sloun, Eldar. **KalmanNet: Neural Network Aided Kalman Filtering for Partially Known Dynamics.** IEEE TSP 70:1532ff, 2022 | gain | RNN outputs the Kalman gain inside the KF flow | Synthetic and non-linear systems | Required baseline (advisor). No covariance output by design | github.com/KalmanNet/KalmanNet_TSP |
| G2 | Choi, Park, Shlezinger, Eldar, Lee. **Split-KalmanNet: A Robust Model-Based Deep Learning Approach for State Estimation.** IEEE T-VT 72(9):12326ff, 2023 | gain | Separate networks for the prior-covariance and innovation-covariance factors of the gain | Simulation, model mismatch | More robust gain variant; candidate for the simple learned-gain row | Not checked |
| G3 | Hanlon, García-Fernández, Peng. **A Comparison Between Kalman-MLE and KalmanNet for State Estimation with Unknown Noise Parameters.** IEEE MFI 2024, Pilsen | cov (MLE) vs gain | Q and R by maximum likelihood during training, then a standard KF | Synthetic systems | Direct evidence on "learn the noise or learn the gain" | Not stated |
| G4 | Song, Mei, Xu, Fu, Bu. **Practical Implementation of KalmanNet for Accurate Data Fusion in Integrated Navigation.** IEEE SPL, 2024 | gain | KalmanNet in multi-sensor integrated navigation | Real integrated-navigation data | KalmanNet on real navigation data; reference for our KalmanNet adapter and its training setup | Available |

## 3. Training through the filter and consistency (Paper B, H-B)

| # | Work | Arm | Idea | Data | Why it matters here | Code |
|---|---|---|---|---|---|---|
| T1 | Haarnoja, Ajay, Levine, Abbeel. **Backprop KF: Learning Discriminative Deterministic State Estimators.** NeurIPS 2016 | train | Train the network end to end through a differentiable KF | Visual tracking / odometry | Origin of the differentiable-filter training used for the torch ESKF | Not checked |
| T2 | Kloss, Martius, Bohg. **How to Train Your Differentiable Filter.** Autonomous Robots 45(4):561-578, 2021 | train | Compares differentiable filter variants and training losses; practical guidance for training DFs | Robotic state-estimation tasks | Design reference for the R2 loss and training setup | Available |
| T3 | Mortada, Falcon, Kahil, Clavaud, Michel. **Recursive KalmanNet.** arXiv:2506.11639, 2025 | gain + train | Propagates the error covariance with the Joseph form and trains with Gaussian NLL | Synthetic, non-Gaussian noise | A learned-gain model that still reports a consistent covariance; reference for B's consistency loss | Available |
| T4 | Chen, Heckman, Julier, Ahmed. **Weak in the NEES?: Auto-Tuning Kalman Filters with Bayesian Optimization.** FUSION 2018, Cambridge | train | Tunes Q/R with NEES/NIS consistency as the objective | Simulation | Precedent for NEES/NIS as a tuning target; same venue as Paper B | Not stated |

## 4. Learned measurement and filter-free inertial odometry (context)

| # | Work | Arm | What is learned | Data | Why it matters here | Code |
|---|---|---|---|---|---|---|
| M1 | Liu, Caruso, Ilg, Dong, Mourikis, Daniilidis, Kumar, Engel. **TLIO: Tight Learned Inertial Odometry.** IEEE RA-L, 2020 | meas | Network displacement + covariance as an EKF measurement | Head-mounted pedestrian IMU | Template for a learned covariance used inside an EKF; contrast with constraint reliability | Available |
| M2 | Chen et al. **IONet** (AAAI 2018); Herath, Yan, Furukawa. **RoNIN** (ICRA 2020) | free | Network regresses motion directly, no filter | Pedestrian | Context: why we keep a model-based filter | Available |

## 5. UAV (platform 2)

| # | Work | Arm | What is learned | Data | Why it matters here | Code |
|---|---|---|---|---|---|---|
| U1 | Qiu, Xu, Chen, Zhao, Geng, Scherer. **AirIO: Learning Inertial Odometry with Enhanced IMU Feature Observability.** IEEE RA-L, 2025 | meas | Body-frame IMU representation + attitude encoding; EKF fusion | Blackbird, EuRoC, Pegasus | Open learned baseline on EuRoC; argues against transforming IMU data to the world frame for UAVs | air-io.github.io |
| U2 | Cioffi, Bauersfeld, Kaufmann, Scaramuzza. **Learned Inertial Odometry for Autonomous Drone Racing.** IEEE RA-L, 2023 | meas | TCN on IMU + thrust, coupled with a model-based filter | Racing quadrotor, up to 70 km/h | UAV analogue of physics filter + learned module; needs thrust, which EuRoC lacks | github.com/uzh-rpg/learned_inertial_model_odometry |
| U3 | Azhari, Lee, Han, Park, Shim. **VeloBins.** arXiv:2608.29720, 2026 | meas | Velocity as a distribution over bins; uncertainty from the same distribution; EKF fusion | AI-IO, NanoBench, TII-RATM, NeuroBEM | Reports EKF NEES on UAV: median velocity NEES 3.08 vs ideal 2.37, 73% of updates inside 95% bounds; regression baseline 61% and over-confident. Supports H-B on UAV | Promised on acceptance |

## 6. Gaps this project addresses

- No work found that compares learned covariance, learned gate and learned gain in the same ESKF, on the same KITTI drives, across outage lengths (A1, A2, A3, G1 each use their own setup).
- A3 is the closest prior result for NHC covariance adaptation but uses a sparse rule, no learned baseline and no consistency metric.
- UAV learned inertial work (U1-U3) is indoor, motion-capture referenced and short; only U3 reports NEES.
- Wording rule for the papers: "we did not find", not "first".

## 7. Considered and not included

LiDAR-inertial work (ALIVE-LIO, AKF-LIO, KISS-IMU): different sensor setup. AUV/DVL learned-Q
work (A-KIT, ProNet): different platform, adapts Q rather than constraint R. Pedestrian ZUPT and
foot-mounted work (Wagstaff and Kelly, GNIO): different motion model. Conformal and
protection-level papers: only if the FUSION scope expands. Embedded/size papers (TinyOdom,
EVC-Mamba): only if a model-size axis is added.

## Sources

- AI-IMU: arXiv:1904.06064; github.com/mbrossar/ai-imu-dr; DOI 10.1109/TIV.2020.2980758
- RINS-W: arXiv:1903.02210; dblp conf/iros/BrossardBB19
- SINDy-NHC: mdpi.com/2072-666X/17/10/1187 (DOI 10.3390/mi17101187)
- OdoNet: arXiv:2109.03091
- KalmanNet: arXiv:2107.10043
- Split-KalmanNet: arXiv:2210.09636; DOI 10.1109/TVT.2023.3270353
- Kalman-MLE vs KalmanNet: livrepository.liverpool.ac.uk/3184477
- Practical KalmanNet: DOI 10.1109/LSP.2024.3431443
- Kloss et al.: arXiv:2012.14313; DOI 10.1007/s10514-021-09990-9
- Recursive KalmanNet: arXiv:2506.11639
- Weak in the NEES?: arXiv:1807.08855
- TLIO: arXiv:2007.01867; DOI 10.1109/LRA.2020.3007421
- AirIO: arXiv:2501.15659
- Learned IO for drone racing: arXiv:2210.15287; DOI 10.1109/LRA.2023.3252342
- VeloBins: arXiv:2608.29720
