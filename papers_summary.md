# Papers Summary: Methods for Improving FAST-LIO2 Robustness

This document summarizes all reviewed papers organized by the failure modes they address (as defined in [[failure_modes]]). For each paper, the core method is described concisely, and applicability to **IEKF/FAST-LIO2** and **solid-state LiDARs (Livox Mid-360)** is rated. For detailed analysis, follow the Obsidian links to individual paper notes.

---

## Applicability Legend

| Symbol | IEKF/FAST-LIO2 Applicability |
|--------|------|
| ★★★ | Directly integrable with minimal changes |
| ★★☆ | Conceptually applicable, requires moderate adaptation |
| ★☆☆ | Requires significant redesign or incompatible architecture |

| Symbol | Livox Mid-360 Applicability |
|--------|------|
| ◆◆◆ | Designed for or fully compatible with solid-state / Livox |
| ◆◆☆ | Works but requires adaptation for non-repetitive patterns |
| ◆☆☆ | Relies on spinning LiDAR assumptions (scan lines, uniform density) |

---

## Failure Mode 1: Geometric Degeneracy

Geometric degeneracy occurs when the spatial distribution of point-to-plane correspondences fails to constrain one or more pose DoFs. The information matrix $A = H_{\text{pose}}^T R^{-1} H_{\text{pose}}$ becomes rank-deficient along directions where all surface normals are (near-)parallel (e.g., corridors, tunnels, silos). This was the **primary failure mode** observed across Cameroon, Silo, Senegal, and Valis datasets.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[D2-LIO]] | Eigenvalue analysis on **separate** translation/rotation blocks of the information matrix; directional Kalman gain attenuation along degenerate eigenvectors; also includes motion-aware outlier filter | ★★★ | ◆◆◆ |
| [[LODESTAR]] | **Schmidt-Kalman filter** partitions states into "solve" (well-conditioned) and "consider" (degenerate) sets. Updates only solve states while maintaining covariance consistency. Also enhances IMU exploitation during degeneracy | ★★★ | ◆◆◆ |
| [[Selective Kalman Filter]] | Per-direction observability analysis across sensors; selectively fuses only informative directions from each sensor source | ★★★ | ◆◆◆ |
| [[RELEAD]] | Constrained IEKF updates with state update bounds; degradation-aware processing with adaptive correspondence thresholds. Directly extends FAST-LIO2 | ★★★ | ◆◆◆ |
| [[I2EKF-LO]] | Dual-iteration EKF: outer loop refines process noise based on innovation statistics, enabling **continuous** (non-binary) degeneracy handling | ★★★ | ◆◆◆ |
| [[Geometrically Stable Sampling]] | Greedy point subset selection maximizing information matrix conditioning via normal diversity; prevents degeneracy at the sampling stage | ★★★ | ◆◆◆ |
| [[Informed Constrained Aligned]] | Field comparison of three strategies: informed weighting, constrained updates, and solution remapping. Recommends adaptive eigenvalue **gap**-based thresholds over absolute thresholds | ★★★ | ◆◆◆ |
| [[AdaLIO]] | Continuous adaptive weights on optimization cost function based on per-direction information content | ★★☆ | ◆◆☆ |
| [[LP-ICP and X-ICP]] | Localizability quantification via Fisher Information eigenvalues; restricts solution to well-conditioned subspace | ★★☆ | ◆◆◆ |
| [[GenZ-ICP]] | Learned adaptive per-point weighting based on geometric informativeness | ★☆☆ | ◆◆☆ |
| [[Probabilistic Degeneracy Detection]] | Hessian eigenvalue analysis for degeneracy characterization; solution remapping away from degenerate subspaces | ★★☆ | ◆◆◆ |
| [[Real-time Degeneracy Sensing and Compensation]] | Real-time eigenvalue-based detection with constrained optimization preventing updates along degenerate directions | ★★☆ | ◆◆◆ |
| [[On Degeneracy of Optimization-based State Estimation]] | Foundational theory: rigorous necessary/sufficient conditions for degeneracy in point-to-plane registration | ★★★ | ◆◆◆ |
| [[Thesis - Degeneracy-Aware LiDAR Odometry]] | Comprehensive thesis unifying detection, mitigation, and evaluation. Recommends eigenvalue gap detection with layered mitigation | ★★★ | ◆◆◆ |
| [[Principled ICP Covariance Modelling]] | Principled covariance estimation for ICP using regularized Hessian inverse; caps covariance along degenerate directions | ★☆☆ | ◆◆◆ |
| [[DAMM-LOAM]] | Multi-metric optimization (point-to-plane + point-to-point + GICP) with metric switching based on conditioning | ★☆☆ | ◆☆☆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**D²-LIO**, **LODESTAR**, and **Selective Kalman Filter** are the three most directly applicable methods. D²-LIO is already tested on FAST-LIO2 and handles the translation/rotation scale mismatch (FM10). LODESTAR's Schmidt-Kalman approach is native to filter architectures and avoids the binary threshold problem. The Selective Kalman Filter elegantly handles multi-sensor directional fusion. **RELEAD** directly extends FAST-LIO2's ESIKF with constrained updates. **I2EKF-LO** provides continuous degeneracy handling through adaptive process noise, avoiding hard thresholds entirely.

---

## Failure Mode 2: Insufficient / Low Number of Effective Features

When the effective feature count drops below ~100 (vs. typical ~800–1000), the information matrix loses rank regardless of spatial distribution. Causes include narrow FoV, LiDAR occlusion, sparse environments, and aggressive motion. Observed prominently in Cameroon, Silo, Drift_GTC, and Valis.

### Intensity-Based Feature Augmentation

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[IGE-LIO]] | Computes **3D intensity gradients** via kNN neighborhood fitting; adds intensity gradient residuals alongside geometric residuals. Scan-pattern agnostic | ★★★ | ◆◆◆ |
| [[COIN-LIO]] | Augments point-to-plane with intensity-based **photometric residuals** via spherical projection. Provides constraints orthogonal to geometry | ★★☆ | ◆☆☆ |
| [[Enhanced LiDAR-inertial SLAM Adaptive Intensity]] | Adaptive intensity feature selection based on gradient magnitude, consistency, range, and incidence angle quality metrics | ★★★ | ◆◆☆ |
| [[Intensity Enhanced Solid-State-LiDAR SLAM]] | Calibrates intensity for range/angle; uses 4D distance metric (position + intensity) for enhanced correspondence. Designed for solid-state LiDARs | ★★☆ | ◆◆◆ |
| [[Intensity-Enhanced LIO Gradient Flow Sampling]] | Selects points along intensity gradient flow lines to maximize intensity information | ★☆☆ | ◆☆☆ |

### Geometric Feature Augmentation

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[Line-LIO]] | PCA-based line feature extraction; adds **point-to-line residuals** alongside point-to-plane; quality-driven keyframe selection prevents map corruption | ★★★ | ◆◆◆ |
| [[NV-LIO]] | Adds **normal vector residuals** alongside point-to-plane; normal vectors constrain rotation without requiring point positions | ★★★ | ◆◆◆ |
| [[VE-LIOM]] | Three feature types (planes, edges, general) with type-specific residuals and adaptive quality weighting | ★★★ | ◆◆☆ |
| [[Feature Assessment and Enhanced Vertical Constraint]] | Assesses feature spatial distribution; adds explicit **ground plane and gravity constraints** as pseudo-measurements when observability is weak | ★★★ | ◆◆◆ |

### General Feature Methods

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[Methods for Feature Detection in Point Clouds]] | Survey of PCA, curvature, Harris 3D, ISS keypoints for unstructured point clouds | ★★☆ | ◆◆◆ |
| [[Extracting General-Purpose Features from LiDAR]] | FPFH, spin images, PCA eigenvalue features for correspondence quality assessment | ★★☆ | ◆◆◆ |
| [[SE-LIO]] | Semantic segmentation to weight features by class reliability; cylindrical tree trunk models | ★☆☆ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**IGE-LIO** is the top choice: its 3D kNN-based intensity gradient approach is scan-pattern agnostic and integrates directly as additional IEKF measurements. **Line-LIO** adds complementary line features using PCA (no scan lines needed) with quality-gated keyframe insertion. **NV-LIO** adds normal vector residuals that specifically improve rotational constraints. **Feature Assessment and Enhanced Vertical Constraint** provides ground plane pseudo-measurements that directly constrain roll/pitch via gravity alignment. Avoid COIN-LIO for Livox due to its spherical projection dependency.

---

## Failure Mode 3: Attitude Mis-Estimation and Gravity Vector Corruption

A corrupted rotation estimate $R$ causes phantom lateral/vertical acceleration via $a_{\text{world}} = R(a_m - b_a)$, creating a feedback loop: $R$ error → phantom acceleration → position drift → further $R$ error. Yaw is fundamentally unobservable from IMU alone. Observed in Valis (severe) and Drift_GTC (jittery compensated acceleration).

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[LIO with Uncertainty IEKF and Ground Constraint]] | Per-point heteroscedastic noise model (function of range, incidence angle, intensity); explicit **ground plane and attitude constraints** as pseudo-measurements | ★★★ | ◆◆◆ |
| [[Equivariant Filter for LIO]] | Exploits SE(3) symmetry structure for linearization errors **independent of state magnitude**; better attitude consistency than standard IEKF | ★★☆ | ◆◆◆ |
| [[I2EKF-LO]] | Adaptive process noise on gyroscope states during aggressive rotation prevents attitude overconfidence | ★★★ | ◆◆◆ |
| [[AKF-LIO]] | Online estimation of process/measurement covariances via innovation statistics; prevents over-trusting biased IMU propagation | ★★★ | ◆◆☆ |
| [[Selective Kalman Filter]] | Prevents corrupted LiDAR updates from polluting attitude estimates by per-direction sensor selection | ★★★ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**LIO with Uncertainty IEKF and Ground Constraint** is the most directly impactful: ground plane constraints enforce gravity alignment as pseudo-measurements within the IEKF, directly preventing roll/pitch corruption. **I2EKF-LO** prevents attitude overconfidence through adaptive process noise. **AKF-LIO** online covariance adaptation prevents the filter from over-trusting a drifting IMU. The **Equivariant Filter** offers fundamental improvements to attitude consistency but requires substantial IEKF reformulation.

---

## Failure Mode 4: Dynamic Objects and Ghosting

FAST-LIO2 assumes a static world. Moving objects create ghost points in the ikd-Tree map that produce incorrect correspondences for future registrations. Observed in Cameroon (operator appearing as ghosts in final scan).

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[RF-LIO]] | Scan-to-map consistency detection; removes dynamic objects **before** registration; retroactive map cleaning of ghost trails | ★★★ | ◆◆☆ |
| [[TRLO]] | Multi-hypothesis **tracking** of dynamic objects with constant-velocity models; removes tracked points; retroactive map cleaning | ★★★ | ◆◆☆ |
| [[Online Dynamic Point Separation and Removal]] | Visibility conflict and free-space violation detection; multi-frame confirmation with false-positive recovery of erroneously removed static points. Built on FAST-LIO | ★★★ | ◆◆☆ |
| [[SE-LIO]] | Semantic class-level filtering of known dynamic categories (people, vehicles) | ★☆☆ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**Online Dynamic Point Separation and Removal** is built directly on FAST-LIO and includes false-positive recovery (important since aggressive removal can discard valid static structure). **RF-LIO** offers tight LIO coupling with scan-to-map consistency detection. **TRLO** provides the most temporally consistent detection via tracking but adds computational overhead. All methods have moderate Livox compatibility due to the non-repetitive pattern complicating temporal association.

---

## Failure Mode 5: Scan-to-Map Correspondence Errors (Outliers)

FAST-LIO2's gating (kNN distance < 5 m², heuristic residual score) is coarse and not motion-aware. No robust kernel is used in the IEKF measurement update. Bad correspondences are amplified along degenerate directions and disproportionately affect low-feature-count scenarios. Observed across all datasets.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[D2-LIO]] | **Motion-aware adaptive outlier filter**: per-point thresholds based on current velocity and scan geometry. Tested on FAST-LIO2 with measurable improvement on Cameroon | ★★★ | ◆◆◆ |
| [[RELEAD]] | Constrained IEKF updates with explicit outlier handling; residual-based adaptive thresholds in the ESIKF front end | ★★★ | ◆◆◆ |
| [[KISS-ICP]] | Motion-adaptive correspondence thresholds scaled by velocity and covariance; truncated least squares | ★☆☆ | ◆◆◆ |
| [[Kinematic-Aware Robust LIO-W]] | Welsch **M-estimation** for adaptive outlier down-weighting; integrates wheel odometry and non-holonomic constraints | ★★★ | ◆◆◆ |
| [[GenZ-ICP]] | Learned per-point weights based on local geometry and degeneracy contribution | ★☆☆ | ◆◆☆ |
| [[Geometrically Stable Sampling]] | Selects geometrically stable point subsets, implicitly avoiding outlier-prone correspondences | ★★★ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**D²-LIO's motion-aware outlier filter** is already tested on FAST-LIO2 and showed measurable improvement on the Cameroon dataset. **RELEAD** adds constrained state updates with outlier rejection directly in the ESIKF pipeline. The **Welsch M-estimation** from Kinematic-Aware Robust LIO-W can replace FAST-LIO2's unweighted least squares in the measurement update — the concept of iteratively reweighted residuals is the key missing piece. **KISS-ICP's** adaptive threshold concept is simple and effective, though it requires adaptation from ICP to IEKF.

---

## Failure Mode 6: Motion Distortion / Deskew Errors

The Livox Mid-360 accumulates points over ~100 ms. FAST-LIO2 compensates via IMU-propagated trajectory, but biased/noisy IMU or time misalignment produces systematic deskew errors. At 2 rad/s rotation, even 1% gyroscope bias error produces ~2 cm displacement per point at 10 m range. Observed in Cameroon, Silo, and Drift_GTC.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[AC-LIO]] | **Asymptotic compensation** with dual motion models (IMU-based and constant-velocity) that adaptively blend based on motion quality; intra-frame temporal smoothing | ★★★ | ◆◆◆ |
| [[De-Skewing with GP Regression]] | Gaussian Process regression over IMU measurements for smooth, **uncertainty-aware** deskewing with per-point confidence propagation | ★★★ | ◆◆◆ |
| [[Dense Forest Mapping with Iterated MDC]] | **Re-deskews** point cloud at each IEKF iteration using updated pose estimates (iterated motion distortion correction) | ★★★ | ◆◆◆ |
| [[HCTO]] | Hybrid: standard IEKF normally + optional continuous-time B-spline refinement for critical high-motion frames | ★★★ | ◆◆◆ |
| [[IKS and Robocentric Voxels]] | Iterated Kalman Smoother uses both past and future IMU data within scan window for improved deskewing; robocentric voxels for numerical stability | ★★☆ | ◆◆◆ |
| [[Eigen Is All You Need]] | Full continuous-time B-spline trajectory; eliminates discrete deskewing entirely | ★☆☆ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**AC-LIO** is the most practical: it replaces the single deskewing model with adaptive blending using quality metrics computed from IEKF residuals — no architectural redesign needed. **De-Skewing with GP Regression** provides uncertainty-aware deskewing that propagates per-point confidence into the measurement noise covariance $R$, addressing the missing gap of deskew uncertainty quantification. **Dense Forest Mapping with Iterated MDC** adds iterative re-deskewing within IEKF iterations — conceptually simple and directly addresses the gap of not re-deskewing after each iteration. **HCTO's** hybrid approach is attractive for selective refinement of only the worst frames.

---

## Failure Mode 7: IMU Quality Limitations (Low-Cost MEMS)

The ICM40609 has limited gyroscope bias stability (~several °/hr) and can sustain dead-reckoning for only ~3 seconds. FAST-LIO2 uses inflated covariance parameters (`acc_cov: 0.1`, `gyr_cov: 0.1`) rather than calibrated values. Reducing to calibrated values causes immediate divergence. This failure mode amplifies all other failure modes during LiDAR degradation.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[Robust LIO Without Sensor-Specific Modeling]] | **Online IMU noise estimation** (per-axis gyro/accel noise, bias random walk) via innovation-based adaptation; eliminates sensor-specific calibration | ★★★ | ◆◆◆ |
| [[I2EKF-LO]] | Dynamically adjusts process noise from IMU based on current state estimation quality; compensates for IMU limitations adaptively | ★★★ | ◆◆◆ |
| [[LIO-EKF]] | Processes each LiDAR point sequentially at ~200 kHz, providing corrections at every IMU step; reduces dead-reckoning intervals to near-zero | ★★☆ | ◆◆◆ |
| [[AKF-LIO]] | Online tuning of process/measurement covariances; addresses the inflated default parameter problem | ★★★ | ◆◆☆ |
| [[LODESTAR]] | Careful IMU data exploitation during LiDAR degradation; extracts maximum information from limited IMU quality | ★★★ | ◆◆◆ |
| [[Selective Kalman Filter]] | Per-direction sensor trust prevents unilateral IMU reliance when certain axes are drifting | ★★★ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**Robust LIO Without Sensor-Specific Modeling** directly solves the covariance tuning problem by estimating IMU noise online — eliminating the need for the inflated parameters that FAST-LIO2 currently uses. **I2EKF-LO** dynamically scales process noise so the filter trusts IMU less when estimation quality is poor. **LIO-EKF** is architecturally radical (per-point updates) but eliminates the dead-reckoning problem entirely by providing LiDAR corrections at IMU rate. **AKF-LIO** provides a more conventional approach to online covariance adaptation.

---

## Failure Mode 8: Map-History Effects and Initialization Sensitivity

Errors baked into the ikd-Tree map during early trajectory propagate into all future registrations. The Cameroon dataset demonstrated this: running 0–400 s fails at ~200 s, but running 100–400 s succeeds entirely. This is the **most poorly covered** failure mode.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[Dynamic Initialization for LiDAR-Inertial SLAM]] | Multi-frame LiDAR-IMU joint optimization for **robust initialization** without stationary startup or specific motion excitation | ★★★ | ◆◆◆ |
| [[SLAM2REF]] | Dual registration against online and reference maps; quality-weighted fusion; drift correction via reference comparison | ★★★ | ◆◆◆ |
| [[Line-LIO]] | **Quality-driven keyframe selection** — only high-quality frames contribute to the map, preventing poorly-registered scans from corrupting it | ★★★ | ◆◆◆ |
| [[Faster-LIO]] | Incremental voxel hash (iVox) replaces ikd-Tree; different map update dynamics; O(1) insertion/lookup | ★★★ | ◆◆◆ |
| [[AS-LIO]] | Sliding window limits map age/staleness by controlling which scans contribute to the active map | ★★★ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**Dynamic Initialization** directly solves the initialization sensitivity problem. **Line-LIO's quality-driven keyframe selection** is the most impactful map quality measure: preventing poorly-conditioned scans from entering the map stops error propagation at the source. **SLAM2REF** provides long-term map correction via reference maps. Key gaps remain: no method provides **map rollback/reset** capability or **online map corruption detection** for FAST-LIO2.

---

## Failure Mode 9: Solid-State LiDAR Specific Challenges

The Livox Mid-360 has a non-repetitive rosette scanning pattern — no scan lines, irregular angular sampling, and spatially varying point density. Single scans have lower spatial coverage than mechanical LiDARs, and consecutive scans see different parts of the scene.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[SR-LIO++]] | **Sweep reconstruction** accumulates multiple non-repetitive scans with motion compensation to build spatially-complete sweeps; density-uniform downsampling | ★★★ | ◆◆◆ |
| [[LOAM-Livox]] | Adapts LOAM for Livox with kNN-based edge/plane detection (no scan lines); multi-scan accumulation for FoV coverage | ★☆☆ | ◆◆◆ |
| [[Towards High-Performance SS-LIO]] | Comprehensive solid-state LIO: adaptive scan accumulation, density-aware downsampling, multi-strategy matching, gravity-aligned initialization | ★★☆ | ◆◆◆ |
| [[Intensity Enhanced Solid-State-LiDAR SLAM]] | Intensity calibration and 4D distance metric specifically designed for solid-state LiDARs | ★★☆ | ◆◆◆ |
| [[SE-LIO]] | Semantic LIO designed for solid-state LiDARs in tree-rich environments | ★☆☆ | ◆◆◆ |
| [[AS-LIO]] | FOV-aware adaptive sliding window handling aggressive FoV variation during transitions between constrained/open spaces | ★★★ | ◆◆◆ |
| [[CTE-MLO]] | Localizability-aware point sampling ensuring geometric diversity regardless of scan pattern | ★☆☆ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**SR-LIO++** is the most directly applicable: its sweep reconstruction provides FAST-LIO2 with denser, more spatially complete scans as a preprocessing step. **AS-LIO's** adaptive sliding window handles the FoV variation problem that causes feature drops in narrow spaces. The density-aware downsampling concepts from **Towards High-Performance SS-LIO** should replace FAST-LIO2's uniform voxel grid, which wastes features in sparse regions and over-represents dense regions.

---

## Failure Mode 10: Scale Mismatch Between Translation and Rotation

The 6×6 information matrix $H^T R^{-1} H$ mixes translation Jacobian columns (units of meters) with rotation Jacobian columns (units of radians × point distance). At typical ranges (5–20 m), rotation columns dominate, masking translation degeneracy in the combined eigenvalue spectrum.

| Paper | Core Method | IEKF | Livox |
|-------|-------------|------|-------|
| [[D2-LIO]] | **Separate 3×3 eigenvalue analysis** for translation and rotation blocks; independent directional attenuation per block | ★★★ | ◆◆◆ |

### Most Applicable for FAST-LIO2 + Livox Mid-360

**D²-LIO** directly and adequately solves this problem. No additional methods needed — the block-separated analysis is practical and already validated on FAST-LIO2.

---

## Cross-Cutting Methods (Address Multiple Failure Modes)

Several papers address multiple failure modes simultaneously and deserve special attention for a holistic improvement strategy:

| Paper | Failure Modes Addressed | IEKF | Livox | Notes |
|-------|------------------------|------|-------|-------|
| [[D2-LIO]] | FM1, FM5, FM10 | ★★★ | ◆◆◆ | Already tested on FAST-LIO2; best starting point |
| [[LODESTAR]] | FM1, FM7 | ★★★ | ◆◆◆ | Schmidt-Kalman is native to filter architectures |
| [[I2EKF-LO]] | FM1, FM3, FM7 | ★★★ | ◆◆◆ | Dual-iteration EKF with continuous adaptation |
| [[RELEAD]] | FM1, FM5 | ★★★ | ◆◆◆ | Direct FAST-LIO2 extension with constrained IEKF |
| [[Selective Kalman Filter]] | FM1, FM3, FM7 | ★★★ | ◆◆◆ | Elegant per-direction multi-sensor fusion |
| [[AKF-LIO]] | FM3, FM7 | ★★★ | ◆◆☆ | Online covariance adaptation |
| [[Line-LIO]] | FM2, FM8 | ★★★ | ◆◆◆ | Line features + quality keyframes |
| [[SE-LIO]] | FM2, FM4, FM9 | ★☆☆ | ◆◆◆ | Semantic approach; high overhead |

---

## Recommended Priority Ordering for Implementation

Based on coverage analysis, applicability ratings, and the observed failure mode severity:

### Tier 1: Highest Impact, Directly Integrable
1. **[[D2-LIO]]** — Degeneracy detection + directional attenuation + outlier filter (FM1, FM5, FM10)
2. **[[IGE-LIO]]** — 3D intensity gradient features for scan-pattern-agnostic feature augmentation (FM2)
3. **[[Robust LIO Without Sensor-Specific Modeling]]** — Online IMU noise estimation (FM7)
4. **[[LIO with Uncertainty IEKF and Ground Constraint]]** — Ground constraints + heteroscedastic noise (FM3)
5. **[[Dynamic Initialization for LiDAR-Inertial SLAM]]** — Robust initialization (FM8)

### Tier 2: High Impact, Moderate Integration Effort
6. **[[AC-LIO]]** — Adaptive deskewing (FM6)
7. **[[LODESTAR]]** — Schmidt-Kalman for principled degeneracy handling (FM1, FM7)
8. **[[Line-LIO]]** — Line features + quality keyframes (FM2, FM8)
9. **[[SR-LIO++]]** — Sweep reconstruction for Livox (FM9)
10. **[[RF-LIO]]** or **[[TRLO]]** — Dynamic object removal (FM4)

### Tier 3: Valuable but Requires Significant Adaptation
11. **[[I2EKF-LO]]** — Dual-iteration EKF (FM1, FM3, FM7)
12. **[[Equivariant Filter for LIO]]** — Improved attitude consistency (FM3)
13. **[[De-Skewing with GP Regression]]** — Uncertainty-aware deskewing (FM6)
14. **[[LIO-EKF]]** — Per-point sequential updates (FM7)

---

## Coverage Gaps Still Remaining

| Gap | Description | Relevant FMs |
|-----|-------------|-------------|
| Cross-coupling in IEKF covariance | How translation degeneracy leaks into rotation estimates through off-diagonal covariance blocks | FM1, FM3 |
| Map corruption detection | Online detection of distorted map regions for rollback or re-registration | FM8 |
| Map point aging/expiry | Temporal decay mechanism for ikd-Tree to remove stale ghost points | FM4, FM8 |
| Adaptive downsampling for non-uniform density | Density-aware voxel grid preserving features in sparse regions | FM2, FM9 |
| Dead-reckoning budget estimation | Predicting how long IMU can sustain acceptable drift; triggering protective actions | FM7 |
| Robust kernel in IEKF | Huber/Cauchy M-estimation within the IEKF measurement update (partially addressed by Kinematic-Aware Robust LIO-W) | FM5 |
