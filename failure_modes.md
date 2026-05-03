# Failure Modes of FAST-LIO2

This document catalogues and explains in detail the failure modes of FAST-LIO2 observed across the internal Tinamu datasets (Cameroon, Silo, Drift_GTC, Valis, Senegal) and from theoretical analysis of the IEKF pipeline with a Livox Mid-360 solid-state LiDAR and its integrated ICM40609 IMU.

---

## 1. Geometric Degeneracy (Insufficient LiDAR Constraints)

### Description

Geometric degeneracy occurs when the spatial distribution of LiDAR point-to-plane correspondences fails to constrain one or more degrees of freedom (DoF) of the pose. In the IEKF formulation, the measurement Jacobian $H$ maps state perturbations to predicted residuals. The pose information matrix

$$
A = H_{\text{pose}}^T R^{-1} H_{\text{pose}}
$$

has eigenvalues $\lambda_1 \leq \lambda_2 \leq \dots \leq \lambda_6$ corresponding to the six pose DoFs (3 translation, 3 rotation). When one or more $\lambda_i \to 0$, the corresponding eigenvector direction is unobservable from LiDAR alone, meaning the Kalman gain in that direction collapses and the state update is driven entirely by IMU propagation.

### Mechanism

FAST-LIO2 uses **point-to-plane residuals** exclusively:

$$
r_j = \mathbf{n}_j^T (T \cdot p_j - q_j)
$$

Each such residual constrains motion only along the surface normal $\mathbf{n}_j$. If all normals are (near-)parallel — e.g. a flat hallway with walls sharing the same normal directions — the system becomes rank-deficient in directions perpendicular to the normals. Specifically:

- **Long corridor / tunnel**: All surfaces share normals roughly along 2 axes (walls left-right, floor-ceiling up-down). The **forward/along-corridor translation** is unconstrained → translation degeneracy.
- **Cylindrical structures (silos, pipes)**: The symmetry of the curved surface means that rotation about the cylinder axis and translation along it are poorly constrained.
- **Large flat open areas**: Only the floor normal constrains the vertical axis; horizontal translation and yaw are weakly constrained.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Cameroon** | At ~200s (0–400s run), the LiDAR FoV narrows to a single wall section. The translation information matrix condition number spikes; $\lambda_{\min}$ drops to near zero. Translation is the primary degenerate subspace. |
| **Silo** | Cylindrical geometry of the silo provides few distinguishing planar features. $\lambda_{\min}$ is extremely small during 8–18s (LiDAR partially covered), and the rotational information matrix is also poorly conditioned due to the symmetric geometry. |
| **Senegal** | Classic hallway/tunnel degeneracy. Forward translation is unconstrained. Drift occurs laterally and vertically in addition to the expected along-corridor direction, suggesting cross-coupling through the IEKF when certain directions collapse. |
| **Valis** | Tunnel geometry causes persistent translational degeneracy from ~85s onwards. |

### Diagnostic indicators

- Eigenvalues of $H_{\text{pose}}^T H_{\text{pose}}$ (split into translation and rotation blocks separately due to scale differences)
- Condition number of the geometric (pre-update) and posterior (post-update) information matrices
- Eigenvector visualization showing the least-constrained direction

---

## 2. Insufficient / Low Number of Effective Features

### Description

Distinct from geometric degeneracy per se, this failure mode occurs when the **total number of valid point-to-plane correspondences** drops below the level needed to adequately constrain the pose, regardless of whether the spatial distribution is degenerate.

### Mechanism

FAST-LIO2 filters raw LiDAR points through:
1. Voxel grid downsampling
2. kNN search to find nearest map points and fit local planes
3. **Gate 1**: Fixed squared-distance cutoff on the kNN search ($d_{kNN}^2 \leq 5\text{ m}^2$)
4. **Gate 2**: Heuristic plane-residual score $s = 1 - 0.9 \frac{|p_{d2}|}{\sqrt{\|p_{\text{body}}\|}} > 0.9$

If many points fail these gates, or if the raw scan is intrinsically sparse, the effective feature count $N_{\text{eff}}$ can drop to $\sim$100 or fewer, compared to a typical $\sim$800–1000 for well-conditioned scans. With fewer measurement rows in $H$, the information matrix $A$ loses rank more easily.

### Causes

- **Narrow LiDAR field of view**: When the sensor enters a tight space (narrow corridor, doorway), much of the 360° × 59° FoV hits nothing or only very close surfaces, reducing usable points.
- **LiDAR occlusion/covering**: In the Silo dataset, the LiDAR was physically covered during 8–18s, yielding near-zero features.
- **Sparse environment**: Open outdoor areas, large empty rooms.
- **Aggressive motion**: Motion blur and deskew errors cause more points to fail the residual gates.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Cameroon** | Effective feature count drops from ~800 to ~100 at failure time (~200s). This drop precedes the drift. |
| **Silo (Case 1)** | Near-zero features during 8–18s (LiDAR covered). Initial drift during this period propagates into map errors that compound when the LiDAR is uncovered. |
| **Drift_GTC** | Extremely narrow corridor FoV causes feature counts to plummet at 0–25s, 42–50s, and 90–95s, directly correlated with drift events. Edge features also remain low during these periods, offering no additional relief. |
| **Valis** | Feature count drops drastically during low-FoV tunnel sections (145–190s). |

### Diagnostic indicators

- Effective feature count (number of inliers after gating)
- Inlier ratio (inliers / total downsampled points)
- Edge feature count from PCA-based extractor (to check if alternative features could help)

---

## 3. Attitude Mis-Estimation and Gravity Vector Corruption

### Description

FAST-LIO2's state includes the rotation matrix $R \in SO(3)$ (body-to-world) and accelerometer bias $b_a \in \mathbb{R}^3$. The compensated acceleration in the world frame is:

$$
a_{\text{world}} = R(a_m - b_a)
$$

where $a_m$ is the raw accelerometer reading. Under correct estimation, the component of $a_{\text{world}}$ aligned with gravity should match $g$, and the orthogonal component should reflect true platform acceleration. If $R$ or $b_a$ are poorly estimated, the filter perceives a **phantom lateral/vertical acceleration**, which it integrates into position, causing drift.

### Mechanism

The gravity vector in FAST-LIO2's state transition model satisfies ${}^G\dot{g} = 0$ (constant gravity assumption). The filter relies on the accelerometer to provide attitude observability: under zero or constant-velocity motion, $a_m \approx R^T g + b_a$, and the gravity direction constrains roll and pitch. However:

1. **Yaw is fundamentally unobservable from IMU alone** (gravity provides no yaw information).
2. During **aggressive rotation**, if the gyroscope readings are noisy or biased, the propagated $R$ can drift. This incorrect $R$ then misinterprets gravity as lateral acceleration.
3. With a **low-cost MEMS IMU** (ICM40609), the gyroscope bias stability and noise density are limited. Bias drift during periods without strong LiDAR corrections compounds the attitude error.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Valis** | During the extreme drift phase (145–160s), the very low FoV forces heavy IMU reliance. The accumulated attitude error from earlier mild degeneracy (~85s) causes the filter to perceive sideways acceleration, driving rapid lateral drift. The compensated acceleration vector visibly departs from the expected gravity-opposing direction. |
| **Drift_GTC** | Compensated acceleration is extremely jittery throughout, suggesting that the rotation matrix estimate is noisy. This is atypical compared to other datasets and may indicate compounding of rotation errors through the corridor sections. |
| **Cameroon** | Compensated acceleration remains consistent around failure time — acceleration/gravity diagnostics do not indicate attitude as the primary failure cause here. The drift is translation-dominated. |

### Diagnostic indicators

- Residual between gravity magnitude and the gravity-aligned component of compensated acceleration: $r = |g| - |a_\parallel|$
- Magnitude of the gravity-orthogonal acceleration component: $|a_\perp|$
- RViz visualization of the compensated acceleration vector vs. the gravity estimate
- Gyroscope bias estimates over time (rapid jumps indicate filter inconsistency)

---

## 4. Dynamic Objects and Ghosting

### Description

FAST-LIO2 assumes a **static world**: the ikd-Tree map accumulates all registered points, and scan-to-map correspondences assume map points are stationary. Moving objects (people, vehicles) that are scanned and inserted into the map create **ghost points** — map features at locations where no static structure exists.

### Mechanism

When a dynamic object is scanned at time $t_1$ and inserted into the map, it remains in the ikd-Tree. At a later time $t_2$, if the sensor revisits the same area but the object has moved, new scan points will attempt to match against the ghosted map points. This creates:

1. **Incorrect correspondences**: New points matched to ghost map points produce erroneous residuals that bias the state update.
2. **Map corruption**: The map no longer faithfully represents the environment, degrading all future registrations in that region.
3. **Reduced confidence in valid data**: The filter sees large residuals from correct points (which don't match the ghosted map), effectively treating valid geometry as outliers.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Cameroon** | A person recording the dataset appears multiple times in the final scan as "ghosts." These ghost features corrupt the local map and can cause the filter to distrust nearby valid structure. |

### Potential impact

While ghosting was not identified as the **primary** trigger for any of the observed catastrophic failures, it contributes to gradual map quality degradation that can push a marginally stable system into divergence. It is particularly problematic for:
- Hand-held systems where the operator's body is often in the FoV
- Industrial environments with moving equipment
- Long-duration mapping sessions where map corruption accumulates

---

## 5. Scan-to-Map Correspondence Errors (Outliers and Wrong Matches)

### Description

Even in a static, feature-rich environment, the nearest-neighbor search in the ikd-Tree can return incorrect correspondences due to:
- **Map drift**: If the estimated pose is already slightly wrong, the transformed scan points land near wrong map surfaces, and kNN returns incorrect neighbors.
- **Repetitive geometry**: Multiple surfaces at similar distances can cause ambiguous nearest-neighbor matches.
- **Noise and discretization**: Voxel downsampling and sensor noise can push correspondences across surface boundaries.

### Mechanism

FAST-LIO2's gating mechanism (Gate 1: kNN distance < 5m², Gate 2: heuristic residual score) is coarse:

- **Gate 2 is not motion-aware**: The threshold $|p_{d2}| < \sqrt{r}/9$ does not account for inter-scan translation or rotation. During aggressive motion, valid far-range correspondences can be rejected (the true correspondence moved further than the gate allows), while during near-static operation the gate is overly permissive.
- **No robust kernel**: FAST-LIO2's IEKF uses unweighted residuals in the measurement update. A single bad correspondence can disproportionately affect the state update.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **All datasets** | Residuals spike during failure modes, though this is partly a symptom rather than a cause. However, the D²-LIO adaptive outlier filter (motion-aware per-point thresholding) showed measurable improvement on the Cameroon dataset — the drift was noticeably slower, and the minimum eigenvalues stayed higher around the failure mode, suggesting that some of the original drift was exacerbated by bad correspondences passing the default gates. |

### Relationship to other failure modes

Correspondence errors are often a **secondary/amplifying** failure mode rather than a root cause. They interact with:
- **Geometric degeneracy**: When the problem is ill-conditioned, even small residual errors from bad correspondences get amplified along degenerate directions.
- **Low feature count**: With fewer correspondences, each one has more influence on the state update, making outliers more damaging.

---

## 6. Motion Distortion / Deskew Errors

### Description

LiDAR scans are not instantaneous — for the Livox Mid-360, a single scan accumulates points over ~100ms. During this time, the sensor moves, so raw points are captured at different poses. FAST-LIO2 compensates for this via **motion undistortion (deskewing)** using the IMU-propagated trajectory within each scan interval.

### Mechanism

Each raw point $p_j^{b(t_j)}$ captured at time $t_j$ within the scan is transformed to the scan-end pose using:

$$
p_j^{b(t_{\text{end}})} = R_{b(t_j)}^{b(t_{\text{end}})} \cdot p_j^{b(t_j)} + t_{b(t_j)}^{b(t_{\text{end}})}
$$

where the relative pose comes from IMU forward propagation. If the IMU data is:
- **Biased** (uncorrected or poorly estimated biases)
- **Noisy** (low-cost MEMS)
- **Time-misaligned** (hardware timestamp offsets between LiDAR and IMU)

then the deskewed points are incorrectly positioned, creating systematic errors in the point cloud that propagate into incorrect residuals.

### Impact

- **High angular velocity**: The deskew error grows with $\omega \cdot \Delta t$ — at 2 rad/s over 100ms, a point at 10m range moves ~2m due to rotation alone. Even 1% bias error in $\omega$ produces 2cm displacement per point.
- **High linear acceleration**: Jerky motions (observed in the Cameroon and Silo datasets as spikes in $a_z$) create larger deskew residuals.
- With the ICM40609's noise characteristics, deskew errors during aggressive hand-held motion can be on the order of centimeters, which is comparable to the point-to-plane distances used for gating.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Cameroon** | Failure correlates with a peak in linear acceleration in the z-direction without significant angular velocity change — a jerking motion that stresses the deskew model. |
| **Silo** | Similarly, failures are preceded by acceleration spikes. |
| **Drift_GTC** | High angular velocity peaks ($\omega_z$) precede the second failure mode, potentially causing deskew errors. |

---

## 7. IMU Quality Limitations (Low-Cost MEMS)

### Description

The Livox Mid-360's integrated IMU (InvenSense ICM40609) is a consumer-grade MEMS IMU with limited:
- **Gyroscope bias stability** (~several °/hr)
- **Accelerometer noise density**
- **Dead-reckoning endurance** (tested: ~3 seconds before significant drift)

### Mechanism

When LiDAR constraints weaken (any of the above failure modes), the IEKF falls back on IMU propagation. The state prediction follows:

$$
\hat{x}_{k+1} = f(\hat{x}_k, u_k)
$$

where $u_k$ contains gyroscope and accelerometer readings. Integration of noisy/biased IMU data without LiDAR corrections causes:
- **Position drift**: Double integration of accelerometer noise/bias → $O(t^2)$ position error
- **Attitude drift**: Integration of gyroscope bias → $O(t)$ orientation error, which then corrupts accelerometer interpretation (see Failure Mode 3)

### Critical interaction

This failure mode is not standalone — it is the **consequence** of any other failure mode that reduces LiDAR observability. Its severity depends on:
1. **Duration** of LiDAR degradation: 1–2 seconds is manageable; 10+ seconds leads to catastrophic drift
2. **Motion aggressiveness**: Higher dynamics amplify IMU integration errors
3. **Bias calibration quality**: The IEKF estimates biases online, but convergence requires informative LiDAR measurements

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Valis** | 3-second dead-reckoning test showed the ICM40609 can handle brief LiDAR outages, but the 45+ second low-FoV period (145–190s) far exceeds the IMU's drift budget. Probabilistic degeneracy mitigation (reducing LiDAR trust) performed worse than baseline here because the IMU wasn't good enough to compensate for the reduced LiDAR contribution over such a long duration. |
| **All datasets** | The default FAST-LIO2 IMU covariance parameters (`acc_cov: 0.1`, `gyr_cov: 0.1`) are intentionally tuned much larger than the physical sensor noise to absorb modeling errors. Reducing them to calibrated values caused immediate divergence, confirming that the filter requires inflated process noise to remain stable. |

---

## 8. Map-History Effects and Initialization Sensitivity

### Description

FAST-LIO2's incremental ikd-Tree means that the map is a function of all past state estimates. Errors made early in the trajectory are baked into the map and affect all future registrations.

### Mechanism

- **Early drift propagation**: If the state drifts during the first few seconds (e.g., due to covered LiDAR or degenerate initialization geometry), the map built during that period is distorted. When the LiDAR data quality improves, new scans are registered against a corrupted map, preventing recovery.
- **Initialization sensitivity**: The Cameroon dataset demonstrated this directly: starting from 0s causes failure at ~200s, but starting from 100s (skipping the problematic early section) allows the full remaining trajectory to succeed. Furthermore, adding the gyro initialization fix (skip frames with $\|\omega\| > 0.1$ rad/s) changed the failure time from ~200s to ~150s — a slightly different early map produces failure at a different location.
- **ikd-Tree point deletion**: When the map bounding box moves (as the sensor translates), old points are deleted. A sudden loss of map points can transiently degrade registration quality.

### Observed in datasets

| Dataset | Observation |
|---------|-------------|
| **Cameroon** | Running 0–400s fails at ~200s. Running 100–400s succeeds through the same physical location. The only difference is the map accumulated during 0–100s. |
| **Cameroon (with gyro init fix)** | Failure time shifts from ~200s to ~150s after a minor initialization change, demonstrating sensitivity to early trajectory/map quality. |
| **Silo** | Two different failure modes (forward drift vs. backward drift) arise from the same data depending on the early map accumulation during the covered-LiDAR period. |

---

## 9. Solid-State LiDAR Specific Challenges

### Description

The Livox Mid-360 uses a non-repetitive, rosette/flower scanning pattern rather than the regular ring-based pattern of mechanical spinning LiDARs. This creates specific challenges:

### Issues

1. **No scan lines**: Traditional LOAM-style edge/corner feature extraction relies on ordered scan lines to compute local curvature. The Livox `line` field (0–3) is not a scan line index. This prevents direct use of LOAM/LIO-SAM feature extraction pipelines.

2. **Irregular angular sampling**: The non-repetitive pattern means:
   - Point density varies spatially within a single scan
   - Spherical projection (used by COIN-LIO for intensity images) produces unstable pixel correspondences
   - k-nearest-neighbor statistics have spatially varying characteristics

3. **Non-repetitive scanning**: Unlike spinning LiDARs where each scan covers the same angular range, Livox scans fill in different parts of the FoV over time. This means:
   - A single scan has lower spatial coverage than a mechanical LiDAR scan
   - Consecutive scans see different parts of the scene
   - Feature tracking across scans is harder

4. **Intensity feature limitations**: Methods like COIN-LIO that project LiDAR scans to intensity images using spherical projection are incompatible with the Livox scan pattern without significant adaptation.

### Impact on failure modes

The solid-state scanning pattern **amplifies** geometric degeneracy: in a given scan, fewer points may cover geometrically informative surfaces compared to a 360° spinning LiDAR. It also limits the applicability of existing solutions from the literature that assume regular scan patterns.

---

## 10. Scale Mismatch Between Translation and Rotation in the Information Matrix

### Description

The measurement Jacobian $H_{\text{pose}}$ has 6 columns: 3 for translation and 3 for rotation. Due to the physics of point-to-plane residuals, the rotation columns have systematically larger magnitudes (Frobenius norm ratio of rotation to translation columns is 3–7×).

### Mechanism

This scale mismatch means:
- The **condition number** of the combined 6×6 information matrix is artificially inflated
- Eigenvalue analysis that mixes translation and rotation can give misleading degeneracy assessments
- A direction that appears "well-conditioned" in the combined matrix may actually be poorly conditioned when translation and rotation are analyzed separately

### Impact

This is not a failure mode per se, but a **diagnostic and algorithmic pitfall** that affects:
- Degeneracy detection thresholds (eigenvalue thresholds must be set separately for translation and rotation)
- Solution remapping / directional attenuation methods that operate on the combined Hessian
- The D²-LIO regularization strategy explicitly handles this by performing separate eigenspace analysis for rotation and translation

### Observed

Confirmed empirically: rotation column norms are consistently 3–7× larger than translation column norms. Scaling rotation columns by 1/10 reduces the condition number from ~200 to ~20–25 during well-conditioned operation.

---

## Summary: Failure Mode Interaction Map

The failure modes above do not act in isolation. The typical failure cascade observed across datasets is:

```
Geometric degeneracy / low features / narrow FoV
        ↓
  Information matrix becomes ill-conditioned
        ↓
  LiDAR update becomes unreliable (bad correspondences amplified)
        ↓
  Filter relies more on IMU propagation
        ↓
  Low-cost IMU drifts (position + attitude)
        ↓
  Attitude error corrupts accelerometer interpretation → phantom acceleration
        ↓
  Drift accelerates (positive feedback loop)
        ↓
  Map built on drifted poses → further degrades future registrations
        ↓
  Catastrophic divergence
```

The **primary triggers** are geometric degeneracy and low feature counts. The **amplifying factors** are IMU quality, correspondence errors, and map-history effects. The **solid-state LiDAR characteristics** of the Livox Mid-360 exacerbate the primary triggers relative to mechanical spinning LiDARs.
