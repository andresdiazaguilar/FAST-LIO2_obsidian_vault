# Papers Map: Failure Modes → Literature Coverage

This document maps each failure mode from `failure_modes.md` to existing papers in the collection, identifies gaps, and suggests additional papers.

---

# Failure Mode 1: Geometric Degeneracy

## Existing Papers

- **D²-LIO** (`Degeneracy-aware/`)
  - Core idea: Detects directional degeneracy via eigenvalue analysis of the information matrix, separated into translation and rotation blocks. Applies directional regularization by attenuating the Kalman gain along degenerate eigenvectors.
  - Strengths: Directly applicable to IEKF; explicitly handles the translation/rotation scale mismatch (FM10); tested on FAST-LIO2 codebase.
  - Limitations: Fixed eigenvalue threshold; regularization discards LiDAR information entirely along degenerate directions rather than down-weighting; may over-attenuate during mild degeneracy.

- **Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization** (`Degeneracy-aware/`)
  - Core idea: Uses a probabilistic framework (condition number / eigenvalue ratios) to detect degeneracy in point-to-plane ICP. Proposes solution remapping to project the solution away from degenerate subspaces.
  - Strengths: Rigorous mathematical formulation; generalizable to any point-to-plane system.
  - Limitations: Designed for optimization-based (not filter-based) registration; solution remapping doesn't directly translate to IEKF Kalman gain modification.

- **AdaLIO** (`Degeneracy-aware/`)
  - Core idea: Adaptive LIO for degenerate indoor environments. Adjusts optimization weights based on per-direction information content.
  - Strengths: Tested specifically in degenerate indoor scenarios (corridors, rooms).
  - Limitations: Optimization-based (factor-graph), not IEKF; requires adaptation for filter-based systems.

- **DAMM-LOAM** (`Degeneracy-aware/`)
  - Core idea: Degeneracy-aware multi-metric LiDAR odometry using point-to-plane, point-to-point, and GICP residuals simultaneously. Switches metric based on degeneracy level.
  - Strengths: Multi-metric approach provides redundancy when one metric degenerates. Also addresses FM2 (features).
  - Limitations: LOAM-based (scan-line features), not directly compatible with solid-state LiDAR or IEKF pipeline.

- **LODESTAR** (`Degeneracy-aware/`)
  - Core idea: Degeneracy-aware LIO using an adaptive Schmidt-Kalman filter. When LiDAR is degenerate, "considers" (rather than updates) degenerate state components, plus exploits IMU data more carefully.
  - Strengths: Schmidt-Kalman approach is native to Kalman filter frameworks; directly applicable to IEKF conceptually. Data exploitation component addresses FM7.
  - Limitations: Computational overhead of the Schmidt-Kalman extension; threshold selection for degeneracy detection.

- **Selective Kalman Filter** (`Degeneracy-aware/`)
  - Core idea: Selectively fuses multi-sensor information based on which sensor directions are informative. When LiDAR is degenerate in a direction, it selectively trusts IMU for that direction.
  - Strengths: Directly applicable to Kalman filter architectures; elegant selective fusion.
  - Limitations: Requires reliable degeneracy detection; performance degrades if IMU also drifts in the selected direction.

- **A Real-time Degeneracy Sensing and Compensation Method** (`Degeneracy-aware/`)
  - Core idea: Real-time eigenvalue-based degeneracy detection with compensation via constrained optimization that prevents updates along degenerate directions.
  - Strengths: Real-time capable; proven in LiDAR SLAM.
  - Limitations: Compensation strategy (constrained optimization) may need adaptation for IEKF.

- **GenZ-ICP** (`Degeneracy-aware/`)
  - Core idea: Generalizable, degeneracy-robust LiDAR odometry using adaptive per-point weighting. Learns to weight correspondences based on their geometric informativeness.
  - Strengths: Adaptive weighting handles degeneracy gracefully; also addresses FM5 (outliers).
  - Limitations: Learning-based component requires training; may not generalize to all solid-state LiDAR patterns.

- **LP-ICP / X-ICP** (`Degeneracy-aware/`)
  - Core idea: Localizability-aware point cloud registration. Quantifies per-direction localizability and constrains registration to well-conditioned directions only.
  - Strengths: Principled localizability metric; tested in extreme environments (EELS mission).
  - Limitations: ICP-based, requires adaptation for IEKF pipeline.

- **On Degeneracy of Optimization-based State Estimation Problems** (`Degeneracy-aware/`)
  - Core idea: Theoretical foundation paper analyzing when/why optimization-based state estimation degenerates. Provides formal conditions for degeneracy.
  - Strengths: Foundational theory applicable to all systems.
  - Limitations: Theoretical — no direct implementation guidance for IEKF.

- **Geometrically Stable Sampling for the ICP Algorithm** (`Degeneracy-aware/`)
  - Core idea: Selects geometrically stable point subsets for registration to avoid ill-conditioning.
  - Strengths: Prevention-oriented (avoids degeneracy rather than detecting/compensating).
  - Limitations: Point selection heuristics may not transfer well to solid-state scan patterns.

- **Principled ICP Covariance Modelling (EELS)** (`Degeneracy-aware/`)
  - Core idea: Principled covariance estimation for ICP in perceptually degraded environments. Models registration uncertainty accounting for degeneracy.
  - Strengths: Provides accurate uncertainty estimates that downstream filters can use.
  - Limitations: Covariance modelling for ICP, not directly for IEKF point-to-plane.

- **Informed, Constrained, Aligned** (`Degeneracy-aware/`)
  - Core idea: Field analysis of degeneracy-aware point cloud registration strategies. Compares informed (eigenvalue-aware), constrained (direction-locked), and aligned (subspace projection) approaches.
  - Strengths: Comprehensive comparison of mitigation strategies; practical guidance.
  - Limitations: Evaluation-focused, not a novel method.

- **Thesis: Degeneracy-Aware LiDAR Odometry and Mapping** (`Degeneracy-aware/`)
  - Core idea: Comprehensive thesis covering degeneracy detection, mitigation, and evaluation across multiple environments.
  - Strengths: Thorough treatment with practical insights.
  - Limitations: May overlap with individual papers above.

## Missing Gaps

- **Continuous/gradual degeneracy handling**: Most methods use binary or threshold-based degeneracy detection. Smooth, continuous attenuation as a function of eigenvalue magnitude is underexplored for IEKF.
- **Cross-coupling between degenerate and non-degenerate directions**: When some directions collapse, errors can leak into well-conditioned directions through the IEKF covariance structure. No paper explicitly addresses this for FAST-LIO2.
- **Solid-state LiDAR-specific degeneracy patterns**: Degeneracy analysis tailored to non-repetitive scan patterns (Livox) is absent.

## Additional Suggested Papers

- **I2EKF-LO: A Dual-Iteration Extended Kalman Filter Based LiDAR Odometry** (Yu et al., 2024, IROS)
  - Why: Proposes a dual-iteration EKF that dynamically adjusts process noise based on current state quality. Directly addresses IEKF limitations during degeneracy.
  - Gap filled: Continuous (non-binary) degeneracy handling within EKF framework.

- **RELEAD: Resilient Localization with Enhanced LiDAR Odometry in Adverse Environments** (Chen et al., 2024, ICRA)
  - Why: Solves constrained ESIKF updates in the front end, incorporating robustness to degraded environments. Directly extends FAST-LIO2 architecture.
  - Gap filled: Constrained state updates during degeneracy within the ESIKF/IEKF pipeline.

---

# Failure Mode 2: Insufficient / Low Number of Effective Features

## Existing Papers

- **COIN-LIO** (`Intensity features/`)
  - Core idea: Augments geometric point-to-plane residuals with intensity-based photometric residuals via spherical projection. Complementary features provide constraints when geometry alone is insufficient.
  - Strengths: Directly adds measurement rows to $H$, improving rank even when geometric features are sparse.
  - Limitations: Spherical projection produces unstable pixel correspondences with Livox non-repetitive scan patterns. Intensity calibration required. Not directly compatible with Mid-360.

- **IGE-LIO** (`Intensity features/`)
  - Core idea: Intensity gradient enhanced LIO. Uses intensity gradients as additional features for registration.
  - Strengths: Gradient-based approach may be more compatible with irregular scan patterns than image projection.
  - Limitations: Intensity gradient reliability depends on surface material and range; performance on solid-state LiDARs needs validation.

- **Enhanced LiDAR-inertial SLAM with Adaptive Intensity Feature Extraction** (`Intensity features/`)
  - Core idea: Adaptive intensity feature extraction that selects intensity features based on their reliability and informational content.
  - Strengths: Adaptive selection avoids unreliable intensity features.
  - Limitations: Applicability to Livox-specific intensity characteristics unclear.

- **Intensity Enhanced for Solid-State-LiDAR in SLAM** (`Intensity features/`)
  - Core idea: Specifically designed intensity enhancement for solid-state LiDAR SLAM.
  - Strengths: Directly addresses solid-state LiDAR intensity usage (FM9).
  - Limitations: May be specific to certain solid-state sensors.

- **Intensity-Enhanced LiDAR-Inertial Odometry with Gradient Flow Sampling** (`Intensity features/`)
  - Core idea: Uses gradient flow sampling to extract robust intensity features for LIO.
  - Strengths: Gradient flow is scan-pattern agnostic.
  - Limitations: Computational overhead of gradient flow computation.

- **NV-LIO** (`Other features/`)
  - Core idea: Uses normal vectors as features for LIO, providing additional constraints beyond point-to-plane distances.
  - Strengths: Normal vector matching adds rotational constraints; tested in multi-floor environments.
  - Limitations: Normal estimation quality depends on local point density, which varies with solid-state scan patterns.

- **VE-LIOM** (`Other features/`)
  - Core idea: Versatile and efficient LIO with multiple feature types (planes, edges, general).
  - Strengths: Multi-feature approach provides robustness when any single feature type is sparse.
  - Limitations: Edge detection assumes scan-line structure (incompatible with Livox).

- **SE-LIO** (`Semantic-aware/`)
  - Core idea: Semantic-enhanced LIO for solid-state LiDARs in tree-rich environments. Uses semantic labels to weight or filter correspondences.
  - Strengths: Specifically designed for solid-state LiDAR; semantic features persist even when geometric features are weak.
  - Limitations: Requires semantic segmentation network; limited to environments with learnable semantic categories.

- **DAMM-LOAM** (`Degeneracy-aware/`)
  - Core idea: Multi-metric approach (point-to-plane + point-to-point + GICP) provides more features per scan.
  - Strengths: Multiple registration metrics inherently increase effective feature count.
  - Limitations: LOAM-based; scan-line assumption.

- **Methods for Feature Detection in Point Clouds** (`Other features/`)
  - Core idea: Survey/methods for detecting features (corners, edges, planes) in unstructured point clouds.
  - Strengths: Techniques applicable to Livox point clouds without scan-line assumptions.
  - Limitations: General-purpose; not LIO-specific.

- **Extracting General-Purpose Features from LiDAR Data** (`Other features/`)
  - Core idea: Methods for general-purpose LiDAR feature extraction.
  - Strengths: Broad feature set.
  - Limitations: Not targeted at real-time odometry.

## Missing Gaps

- **Scan-pattern-agnostic feature extraction for Livox**: No paper provides a complete feature extraction pipeline for non-repetitive solid-state scans integrated into IEKF.
- **Adaptive feature count monitoring with fallback**: No method monitors effective feature count in real-time and triggers a fallback strategy (e.g., accumulating multiple scans, reducing downsampling resolution).
- **PCA-based edge features for unstructured point clouds in IEKF**: Edge features could supplement planes, but extraction without scan lines is not well addressed.

## Additional Suggested Papers

- **Line-LIO: High-Precision Line Feature Integration and Quality-Driven Keyframe Optimization for LiDAR-Inertial Odometry** (Ding et al., 2026, Meas. Sci. Technol.)
  - Why: Integrates line features alongside planes, with quality-driven keyframe selection. Directly applicable to LIO.
  - Gap filled: Additional feature types (lines) for IEKF-based systems when planar features are sparse.

- **Feature Assessment and Enhanced Vertical Constraint LiDAR Odometry** (Li et al., 2025, IEEE TIM)
  - Why: Explicitly assesses feature quality per scan and applies vertical constraints to improve robustness when features are weak.
  - Gap filled: Feature quality assessment with adaptive constraint strategies.

---

# Failure Mode 3: Attitude Mis-Estimation and Gravity Vector Corruption

## Existing Papers

- **AKF-LIO** (`Degeneracy-aware/`)
  - Core idea: Adaptive Kalman filter that tunes process and measurement covariances online. This indirectly helps attitude estimation by adjusting trust between IMU and LiDAR dynamically.
  - Strengths: Directly applicable to IEKF; online adaptation prevents over-trusting biased IMU propagation.
  - Limitations: Adaptation heuristics may be slow to react during rapid attitude divergence.

- **LODESTAR** (`Degeneracy-aware/`)
  - Core idea: Data exploitation component more carefully uses IMU accelerometer data during LiDAR degeneracy, which helps maintain gravity vector consistency.
  - Strengths: Explicitly considers IMU data quality during degeneracy.
  - Limitations: Primarily focused on degeneracy, not attitude estimation per se.

- **Selective Kalman Filter** (`Degeneracy-aware/`)
  - Core idea: Selectively trusts different sensor directions, which can prevent corrupted LiDAR updates from polluting attitude estimates.
  - Strengths: Prevents bad LiDAR updates from corrupting attitude.
  - Limitations: Does not directly improve IMU-based attitude propagation.

## Missing Gaps

- **No paper directly addresses gravity vector corruption in IEKF-based LIO**: The feedback loop where $R$ error → phantom acceleration → position drift → further $R$ error is not explicitly tackled.
- **Attitude-specific observability analysis for FAST-LIO2**: No paper analyzes which motion profiles maintain roll/pitch/yaw observability for the FAST-LIO2 state with the Livox Mid-360.
- **IMU bias estimation improvement during LiDAR degradation**: Gyroscope bias estimation quality degrades exactly when it matters most (during LiDAR outages).
- **Gravity constraint enforcement**: Explicit gravity magnitude/direction constraints within the IEKF are not explored.

## Additional Suggested Papers

- **Equivariant Filter for Tightly Coupled LiDAR-Inertial Odometry** (Tao et al., 2025, ICRA)
  - Why: Equivariant filters on Lie groups provide inherently better attitude consistency than standard IEKF. The imperfect-IEKF formulation better preserves the $SO(3)$ structure.
  - Gap filled: Improved attitude estimation consistency through geometric filter design.

- **LiDAR-Inertial Odometry with Uncertainty IEKF and Ground Constraint** (Luo & Cao, 2024)
  - Why: Adds explicit ground plane constraint to the IEKF, which directly constrains roll and pitch through gravity-aligned surface normal. Reduces attitude drift.
  - Gap filled: Direct gravity/attitude constraint enforcement in IEKF.

- **I2EKF-LO** (Yu et al., 2024, IROS)
  - Why: Dual-iteration EKF dynamically adjusts process noise. During aggressive rotation, higher process noise on gyroscope prevents attitude overconfidence.
  - Gap filled: Adaptive process noise for attitude states.

---

# Failure Mode 4: Dynamic Objects and Ghosting

## Existing Papers

- **RF-LIO** (`Dynamic objects/`)
  - Core idea: Removal-first tightly-coupled LIO. Detects and removes dynamic objects before scan-to-map registration, preventing ghost points from entering the map.
  - Strengths: Directly prevents map corruption; tightly-coupled LIO framework.
  - Limitations: Dynamic object detection relies on multi-scan consistency checks, which may miss slow-moving objects or fail with sparse solid-state LiDAR scans.

- **SE-LIO** (`Semantic-aware/`)
  - Core idea: Semantic labels can identify known dynamic classes (people, vehicles) for exclusion.
  - Strengths: Class-level filtering is robust for known categories.
  - Limitations: Requires semantic network inference; limited to trained categories; computational overhead.

## Missing Gaps

- **Lightweight dynamic point detection for IEKF pipeline**: RF-LIO's detection may be too heavy for real-time FAST-LIO2. A lightweight heuristic (e.g., map-consistency check at the point level) is needed.
- **Map point aging/expiry**: The ikd-Tree never forgets — ghost points persist indefinitely. No paper proposes a temporal decay mechanism for FAST-LIO2's map.
- **Operator body filtering for hand-held systems**: Specific to the use case; no paper addresses static blind-zone masking for hand-held LiDAR.

## Additional Suggested Papers

- **TRLO: An Efficient LiDAR Odometry with 3D Dynamic Object Tracking and Removal** (Jia et al., 2025, IEEE Trans.)
  - Why: Efficient dynamic object tracking and removal for LiDAR odometry. Lower computational overhead than RF-LIO.
  - Gap filled: Lightweight dynamic object removal compatible with real-time LIO.

- **An Online Dynamic Point Separation and Removal SLAM Framework** (Zhu et al., 2025, AJSE)
  - Why: Online separation of dynamic vs static points based on FAST-LIO, with recovery of erroneously removed static points.
  - Gap filled: Directly built on FAST-LIO; dynamic/static separation with false-positive recovery.

---

# Failure Mode 5: Scan-to-Map Correspondence Errors (Outliers)

## Existing Papers

- **D²-LIO** (`Degeneracy-aware/`)
  - Core idea: Includes a motion-aware adaptive outlier filter that sets per-point thresholds based on current velocity and scan geometry. Tested on FAST-LIO2 with measurable improvement on Cameroon dataset.
  - Strengths: Directly addresses Gate 2 limitations; motion-aware; integrated with FAST-LIO2.
  - Limitations: Still heuristic-based; no robust kernel in the IEKF update itself.

- **GenZ-ICP** (`Degeneracy-aware/`)
  - Core idea: Adaptive per-point weighting based on geometric informativeness. Down-weights unreliable correspondences.
  - Strengths: Soft weighting is more principled than hard gating.
  - Limitations: Learning-based; requires adaptation for IEKF.

- **Geometrically Stable Sampling** (`Degeneracy-aware/`)
  - Core idea: Selects point subsets that are geometrically stable for registration, implicitly avoiding outlier-prone correspondences.
  - Strengths: Preventive approach.
  - Limitations: Not a per-correspondence outlier rejection method.

## Missing Gaps

- **Robust cost functions (M-estimation / Huber kernel) in IEKF measurement update**: FAST-LIO2 uses unweighted least squares in the IEKF. Integrating a Huber or Cauchy kernel into the IEKF measurement update is unexplored in the collected papers.
- **Adaptive correspondence gating based on state uncertainty**: Current Gate 2 is not uncertainty-aware. A Mahalanobis-distance gate using the predicted state covariance would reject outliers more intelligently.
- **Iterative reweighting within IEKF iterations**: Each IEKF iteration could reweight correspondences based on the current residuals, implementing iteratively-reweighted least squares (IRLS) within the filter.

## Additional Suggested Papers

- **RELEAD: Resilient Localization with Enhanced LiDAR Odometry in Adverse Environments** (Chen et al., 2024, ICRA)
  - Why: Implements constrained ESIKF updates with explicit outlier handling in the front end. Directly extends FAST-LIO2.
  - Gap filled: Robust state updates within ESIKF/IEKF with outlier rejection.

- **KISS-ICP: In Defense of Point-to-Point ICP** (Vizzo et al., 2023, RA-L)
  - Why: Adaptive correspondence threshold based on current motion estimate. Simple, effective, and directly integrable.
  - Gap filled: Motion-adaptive correspondence gating.

- **Kinematic-Aware Robust LIO-W** (Wu et al., 2026, IEEE Access)
  - Why: Tightly-coupled LIO framework with trajectory-based asynchronous compensation specifically designed for degraded environments. Includes robustness to bad correspondences.
  - Gap filled: Kinematic-aware robust outlier handling in LIO.

---

# Failure Mode 6: Motion Distortion / Deskew Errors

## Existing Papers

- **Eigen Is All You Need** (`Continuous-time/`)
  - Core idea: Continuous-time LIO using B-spline trajectory representation. Eliminates discrete deskewing by querying the continuous trajectory at each point's exact timestamp.
  - Strengths: Principled solution to motion distortion; handles arbitrary motion profiles.
  - Limitations: B-spline optimization is computationally heavier than IEKF forward propagation; full system redesign required (not a drop-in for FAST-LIO2).

- **HCTO** (`Hand-held/`)
  - Core idea: Hybrid continuous-time optimization for compact wearable mapping. Combines discrete IEKF with continuous-time refinement for critical frames.
  - Strengths: Hybrid approach avoids full system redesign; targeted at hand-held systems with aggressive motion.
  - Limitations: Optimality-aware selection of when to apply CT refinement adds complexity.

- **Fast and Robust LiDAR-Inertial Odometry by Tightly-Coupled IKS and Robocentric Voxels** (`Hand-held/`)
  - Core idea: Iterated Kalman Smoother (IKS) instead of filter. Smoothing within a scan window naturally improves deskewing by using future IMU data.
  - Strengths: IKS provides better deskewing than forward-only IMU propagation; robocentric voxels reduce map drift.
  - Limitations: Smoother adds latency (uses future data within window); computational overhead.

- **AS-LIO** (`Hand-held/`)
  - Core idea: Adaptive sliding window LIO for aggressive FOV variation. Accumulates multiple scans in a sliding window to handle aggressive motion.
  - Strengths: Directly addresses hand-held use case; adaptive window size.
  - Limitations: Sliding window approach doesn't improve per-point deskewing quality.

## Missing Gaps

- **Improved deskewing with IMU bias feedback**: Current deskewing uses forward-propagated IMU, but doesn't feed back the IEKF's bias estimate improvement from the current scan to re-deskew.
- **Iterative deskewing within IEKF iterations**: Each IEKF iteration updates the pose estimate, but the deskewed points are not re-deskewed with the updated trajectory. Re-deskewing would improve convergence during aggressive motion.
- **Quantification of deskew error and its propagation into measurement covariance**: Deskew uncertainty is not reflected in the measurement noise covariance $R$ in FAST-LIO2.

## Additional Suggested Papers

- **AC-LIO: Towards Asymptotic Compensation for Distortion in LiDAR-Inertial Odometry via Selective Intra-Frame Smoothing** (Zhang et al., 2024, arXiv)
  - Why: Proposes selective intra-frame smoothing for asymptotic deskew compensation. More efficient than full continuous-time approaches while improving deskewing beyond standard IMU propagation.
  - Gap filled: Improved deskewing without full CT redesign.

- **De-Skewing Point Clouds Based on Gaussian Process Regression in a Tightly Coupled LiDAR-IMU SLAM System** (Zheng et al., 2025)
  - Why: GP regression provides smooth, uncertainty-aware deskewing with naturally varying confidence.
  - Gap filled: Uncertainty-aware deskewing with propagation into measurement covariance.

- **A LiDAR SLAM System for Dense Forest Mapping with Iterated Motion Distortion Correction** (Nakao et al., 2026, Advanced Robotics)
  - Why: Iteratively corrects motion distortion across multiple registration iterations.
  - Gap filled: Iterative deskew refinement within the registration loop.

---

# Failure Mode 7: IMU Quality Limitations (Low-Cost MEMS)

## Existing Papers

- **AKF-LIO** (`Degeneracy-aware/`)
  - Core idea: Adaptive Kalman filter that online-tunes process and measurement covariances. Helps prevent the filter from over-trusting a low-cost IMU.
  - Strengths: Directly addresses the covariance tuning problem noted in failure_modes.md (inflated default parameters).
  - Limitations: Adaptation speed may lag behind rapid IMU quality changes.

- **LODESTAR** (`Degeneracy-aware/`)
  - Core idea: Data exploitation component carefully uses IMU data during LiDAR degradation.
  - Strengths: Addresses the critical interaction between IMU quality and LiDAR degradation.
  - Limitations: Primarily focused on degeneracy rather than general IMU quality improvement.

- **Selective Kalman Filter** (`Degeneracy-aware/`)
  - Core idea: Selectively fuses sensors per direction. When IMU is drifting, can increase trust in LiDAR for certain directions.
  - Strengths: Prevents unilateral IMU trust.
  - Limitations: Cannot improve IMU quality itself.

## Missing Gaps

- **Online IMU noise characterization**: FAST-LIO2 uses fixed noise parameters. No paper proposes online estimation of the actual IMU noise floor and bias instability within the IEKF.
- **Multi-rate IMU processing**: Higher-rate IMU sampling could reduce integration errors. Not explored for FAST-LIO2.
- **Dead-reckoning budget estimation**: No method estimates how long the current IMU can sustain acceptable dead-reckoning and triggers protective actions (slow down, stop mapping) before exceeding budget.
- **Allan variance-informed covariance scheduling**: Using pre-calibrated Allan variance curves to schedule process noise covariances as a function of time since last good LiDAR update.

## Additional Suggested Papers

- **A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling** (Malladi et al., 2026, RA-L)
  - Why: Eliminates sensor-specific tuning entirely; uses a robust approach that adapts to any IMU quality level without manual covariance tuning.
  - Gap filled: Sensor-agnostic LIO that handles varying IMU quality.

- **I2EKF-LO** (Yu et al., 2024, IROS)
  - Why: Dynamically adjusts process noise from the IMU based on current state estimation quality. Compensates for IMU limitations adaptively.
  - Gap filled: Adaptive IMU noise modelling in EKF framework.

- **LIO-EKF: High Frequency LiDAR-Inertial Odometry Using Extended Kalman Filters** (Wu et al., 2024, ICRA)
  - Why: Processes LiDAR at IMU rate, providing corrections at every IMU step rather than once per scan. Reduces dead-reckoning intervals to near-zero.
  - Gap filled: Eliminates long IMU dead-reckoning periods by high-frequency LiDAR updates.

---

# Failure Mode 8: Map-History Effects and Initialization Sensitivity

## Existing Papers

- **AS-LIO** (`Hand-held/`)
  - Core idea: Sliding window approach limits map age/staleness by controlling which scans contribute to the active map.
  - Strengths: Sliding window prevents ancient corrupted data from persisting.
  - Limitations: Window size tuning; doesn't address initialization specifically.

- **Faster-LIO** (`FAST-LIO, LIO-SAM, LOAM/`)
  - Core idea: Uses incremental voxels instead of ikd-Tree, which may provide different map update dynamics.
  - Strengths: Alternative map structure.
  - Limitations: Doesn't address map quality monitoring or initialization robustness.

## Missing Gaps

- **This is the most poorly covered failure mode.** No existing paper in the collection directly addresses:
  - **Initialization robustness**: Detecting and recovering from poor initialization conditions.
  - **Map quality monitoring**: Online detection of map corruption (inconsistent normals, ghost points, drift-induced distortion).
  - **Map rollback/reset**: Ability to discard recently-added corrupt map regions and re-register from a known-good state.
  - **Keyframe-based map management**: Selective insertion of well-conditioned scans only, preventing poorly-registered scans from corrupting the map.
  - **Initialization-specific failure recovery**: The Cameroon dataset shows that different initial conditions yield different failure locations — this sensitivity is entirely unaddressed.

## Additional Suggested Papers

- **Dynamic Initialization for LiDAR-Inertial SLAM** (Xu et al., 2025, IEEE/ASME Trans. Mechatronics)
  - Why: Directly addresses the initialization problem for LiDAR-inertial SLAM. Does not require stationary initialization or specific motion excitation patterns.
  - Gap filled: Robust initialization without restrictive assumptions.

- **Line-LIO** (Ding et al., 2026)
  - Why: Quality-driven keyframe optimization — only high-quality frames contribute to the map. Prevents poorly-registered scans from corrupting it.
  - Gap filled: Quality-gated map insertion.

- **SLAM2REF: Advancing Long-Term Mapping with Reference Map Integration** (Vega-Torres et al., 2024, Constr. Robotics)
  - Why: Reference map integration can detect and correct map drift/corruption by comparing against prior maps.
  - Gap filled: Map quality verification against external references.

---

# Failure Mode 9: Solid-State LiDAR Specific Challenges

## Existing Papers

- **LOAM-Livox** (`Solid-state LiDARs/`)
  - Core idea: Adapts LOAM for Livox solid-state LiDARs with small FoV. Custom feature extraction for non-repetitive scan patterns.
  - Strengths: Directly addresses Livox-specific feature extraction.
  - Limitations: LOAM-based (not IEKF); designed for smaller-FoV Livox sensors (Avia, Horizon), not Mid-360.

- **Towards High-Performance Solid-State-LiDAR-Inertial Odometry and Mapping** (`Solid-state LiDARs/`)
  - Core idea: Comprehensive system for solid-state LiDAR LIO, addressing the specific challenges of non-repetitive scanning.
  - Strengths: Full-system treatment of solid-state LiDAR challenges.
  - Limitations: System design may not integrate cleanly with FAST-LIO2's IEKF architecture.

- **Intensity Enhanced for Solid-State-LiDAR in SLAM** (`Intensity features/`)
  - Core idea: Intensity feature augmentation specifically for solid-state LiDAR SLAM.
  - Strengths: Directly addresses the limited geometric feature availability of solid-state LiDARs.
  - Limitations: Intensity quality varies by Livox model.

- **SE-LIO** (`Semantic-aware/`)
  - Core idea: Semantic LIO specifically for solid-state LiDARs.
  - Strengths: Designed for solid-state; leverages semantic information to compensate for geometric limitations.
  - Limitations: Semantic network overhead; environment-specific.

- **AS-LIO** (`Hand-held/`)
  - Core idea: Handles aggressive FOV variation, which is relevant for solid-state LiDARs transitioning between constrained and open spaces.
  - Strengths: FOV-aware sliding window.
  - Limitations: Not specifically designed for non-repetitive scan patterns.

## Missing Gaps

- **Non-repetitive scan accumulation strategies**: Methods to intelligently accumulate multiple non-repetitive scans to build denser, more uniform point coverage before registration.
- **Adaptive downsampling for non-uniform density**: Current voxel grid downsampling treats all regions equally, but Livox point density varies spatially. Adaptive downsampling could preserve features in sparse regions.
- **Scan-pattern-aware kNN**: The kNN search in the ikd-Tree doesn't account for the spatially varying point density of solid-state scans. Adaptive search radii could improve correspondence quality.

## Additional Suggested Papers

- **SR-LIO++: Efficient LiDAR-Inertial Odometry and Quantized Mapping with Sweep Reconstruction** (Yuan et al., 2025, arXiv)
  - Why: Sweep reconstruction addresses the partial-coverage problem of non-repetitive scans by reconstructing complete sweeps from multiple partial scans.
  - Gap filled: Sweep reconstruction for solid-state LiDAR LIO.

- **CTE-MLO: Continuous-Time and Efficient Multi-LiDAR Odometry with Localizability-Aware Point Cloud Sampling** (Shen et al., 2025, IEEE T-FR)
  - Why: Localizability-aware sampling that ensures geometric diversity regardless of scan pattern. Applicable to solid-state LiDARs.
  - Gap filled: Scan-pattern-agnostic point sampling strategy.

---

# Failure Mode 10: Scale Mismatch Between Translation and Rotation in Information Matrix

## Existing Papers

- **D²-LIO** (`Degeneracy-aware/`)
  - Core idea: Explicitly separates eigenvalue analysis into translation and rotation blocks (3×3 each), avoiding the scale mismatch problem. Applies directional attenuation independently per block.
  - Strengths: Directly solves this problem; proven on FAST-LIO2.
  - Limitations: Separation into two 3×3 blocks ignores cross-coupling between translation and rotation.

## Missing Gaps

- **Well-covered by D²-LIO** for the practical case. Remaining gap:
  - **Principled scaling/normalization of the 6×6 Hessian**: A systematic method to normalize translation and rotation Jacobian columns (e.g., using point cloud radius statistics or state covariance) before eigenvalue analysis, rather than separate block analysis.
  - **Cross-coupling analysis**: How translation degeneracy in one direction affects rotation estimation and vice versa through the off-diagonal blocks.

## Additional Suggested Papers

- No critical additional papers needed — D²-LIO's block-separated analysis is sufficient for practical purposes. The scale mismatch is a diagnostic/algorithmic concern rather than a standalone failure mode.

---

# Summary: Coverage Heat Map

| Failure Mode | Coverage Level | Key Existing Papers | Critical Gaps |
|---|---|---|---|
| 1. Geometric Degeneracy | **Strong** (16 papers) | D²-LIO, LODESTAR, Selective KF | Continuous attenuation; solid-state specific |
| 2. Low Features | **Moderate** (11 papers) | COIN-LIO, IGE-LIO, NV-LIO | Livox-compatible feature extraction |
| 3. Attitude Mis-Estimation | **Weak** (3 papers, indirect) | AKF-LIO (indirect) | Gravity constraint; attitude observability |
| 4. Dynamic Objects | **Weak** (2 papers) | RF-LIO | Lightweight detection; map aging |
| 5. Correspondence Errors | **Moderate** (3 papers) | D²-LIO | Robust kernels in IEKF; adaptive gating |
| 6. Motion Distortion | **Moderate** (4 papers) | Eigen, HCTO, IKS | Iterative re-deskewing; uncertainty propagation |
| 7. IMU Quality | **Weak** (3 papers, indirect) | AKF-LIO | Online noise characterization; high-freq updates |
| 8. Map-History / Init | **Very Weak** (2 papers, tangential) | AS-LIO (tangential) | Map quality monitoring; robust init; rollback |
| 9. Solid-State LiDAR | **Moderate** (5 papers) | LOAM-Livox, SS-LIO | Scan accumulation; adaptive downsampling |
| 10. Scale Mismatch | **Adequate** (1 paper, direct) | D²-LIO | Cross-coupling analysis |

**Priority for additional literature search** (by coverage gap severity):
1. FM8: Map-History Effects and Initialization (very weak)
2. FM3: Attitude Mis-Estimation (weak, high impact)
3. FM4: Dynamic Objects (weak, moderate impact)
4. FM7: IMU Quality (weak, but partially a consequence of other FMs)
5. FM5: Correspondence Errors (moderate, high interaction with other FMs)
