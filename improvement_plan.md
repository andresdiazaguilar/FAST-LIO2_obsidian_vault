# Improvement Plan for FAST-LIO2 on Livox Mid-360

This document presents the optimal, realistic plan to improve FAST-LIO2's robustness against the failure modes documented in [[failure_modes]], drawing from the literature reviewed in [[papers_summary]] and the novel methods proposed in [[proposed_methods]]. The plan is designed to be **implementable incrementally** within the existing IEKF pipeline, with each addition composing cleanly with the others.

---

## Design Principles

1. **No full system redesign**: Every modification intervenes at a specific point in the existing FAST-LIO2 pipeline (prediction, deskewing, downsampling, measurement model, Kalman gain, covariance update, or map insertion). The state vector structure is unchanged throughout.
2. **Incremental testability**: Additions are ordered so that each can be implemented and evaluated independently before adding the next.
3. **Composition over competition**: Methods are selected to be non-overlapping in their pipeline intervention points and complementary in their failure mode coverage. No two methods modify the same equation in conflicting ways.
4. **Solid-state LiDAR native**: Every method is either already validated on Livox or requires only minor, well-defined adaptation (detailed below).

---

## The Plan: Six Coordinated Improvements

The plan consists of **six additions** to FAST-LIO2, ordered by implementation priority. The first three form the core contribution and are tightly coupled; the remaining three are independent enhancements.

```
Raw Livox Scan
     │
     ▼
[Addition 3] Normal-Diversity-Aware Downsampling ←── replaces voxel grid
     │
     ▼
FAST-LIO2 kNN Search + Plane Fitting (unchanged)
     │
     ▼
[Addition 2] Unified Heteroscedastic R ←── replaces constant R
     │
     ▼
[Addition 1] CEWS-KF + Cross-Covariance Protection + Welsch IRLS ←── modifies Kalman gain
     │        ├── Continuous degeneracy attenuation (replaces binary)
     │        ├── Cross-covariance decoupling (novel, breaks FM1→FM3 cascade)
     │        └── Directional robust residuals (replaces Gate 2)
     ▼
State Update → Covariance Update
     │
     ▼
[Addition 4] Quality-Gated Map Insertion ←── gates ikd-Tree insertion
     │
     ▼
[Addition 5] Online IMU Noise Estimation ←── modifies process noise Q
     │
     ▼
[Addition 6] Ground Plane Pseudo-Measurements ←── adds rows to H
```

---

## Addition 1: Continuous Eigenvalue-Weighted Schmidt-Kalman Filter with Cross-Covariance Protection and Directional Robust Estimation

**Failure modes addressed**: FM1 (geometric degeneracy), FM3 (attitude corruption via FM1→FM3 cascade), FM5 (correspondence errors), FM10 (scale mismatch)

**Source**: Novel — [[proposed_methods]] Methods 1, 2, and 3 combined. Draws on concepts from [[D2-LIO]] (block-separated eigenvalues), [[LODESTAR]] (Schmidt-Kalman consistency), and [[Kinematic-Aware Robust LIO-W]] (Welsch M-estimation).

**Impact**: ★★★★★ — This is the single highest-impact addition. Geometric degeneracy was the primary failure mode across Cameroon, Silo, Senegal, and Valis. The FM1→FM3 cascade (translation degeneracy leaking into attitude corruption) was directly responsible for the catastrophic divergence in Valis and Drift_GTC.

**Difficulty**: Low-Medium — All modifications occur after $H$ and $R$ are computed, before the state update. No new states, no architectural changes.

### What It Does

Three tightly coupled sub-modifications applied sequentially within each IEKF iteration:

#### 1a. Continuous Eigenvalue-Weighted Degeneracy Attenuation (CEWS-KF)

Replaces D²-LIO's binary zero-out of degenerate directions with a smooth, self-adapting weighting function.

**Pipeline intervention**: After computing $H$, before applying the Kalman gain.

**Equations**:

Compute block-separated information matrices (as in D²-LIO):

$$A_t = H_t^T R^{-1} H_t, \quad A_r = H_r^T R^{-1} H_r$$

Eigendecompose: $A_t = V_t \Lambda_t V_t^T$, $A_r = V_r \Lambda_r V_r^T$.

For each eigenvalue $\lambda_i$, compute the continuous weight:

$$w(\lambda_i) = \frac{\lambda_i^2}{\lambda_i^2 + \lambda_{\text{ref}}^2}, \quad \lambda_{\text{ref}} = \beta \cdot \text{median}(\lambda_1, \lambda_2, \lambda_3)$$

with $\beta \approx 0.1$ as the single tuning parameter. Construct weighting matrices:

$$W_t = V_t \, \text{diag}(w(\lambda_1^t), w(\lambda_2^t), w(\lambda_3^t)) \, V_t^T, \quad W_r = V_r \, \text{diag}(w(\lambda_1^r), w(\lambda_2^r), w(\lambda_3^r)) \, V_r^T$$

**Advantages over D²-LIO**:
- No hard threshold $\lambda_{\text{thresh}}$ to tune per environment — the median-based $\lambda_{\text{ref}}$ adapts automatically
- Smooth state updates (no trajectory discontinuities at threshold crossings)
- Preserves partial information during mild degeneracy instead of discarding it entirely
- Covariance-consistent (see below)

#### 1b. Cross-Covariance Attitude Protection

Prevents degenerate translation residuals from corrupting the rotation estimate through the IEKF's off-diagonal covariance blocks $P_{tr}$.

**Pipeline intervention**: Immediately before computing the Kalman gain, after the eigenvalue analysis from 1a.

**Equations**:

Attenuate the cross-covariance along degenerate translation directions:

$$\tilde{P}_{rt}^- = P_{rt}^- \left(I - \sum_{d} (1 - w(\lambda_d^t)) \, v_d v_d^T \right)$$

Use $\tilde{P}^-$ (with the attenuated off-diagonal blocks) when computing the Kalman gain:

$$K = \tilde{P}^- H^T (H \tilde{P}^- H^T + R)^{-1}$$

The same applies symmetrically: when rotation is degenerate (e.g., Silo's cylindrical geometry), attenuate $P_{tr}^-$ along degenerate rotation eigenvectors.

**Why this is critical**: No existing method addresses this. D²-LIO and LODESTAR handle the direct update along degenerate directions but leave the cross-coupling intact. The Valis failure analysis showed that translation degeneracy propagated into attitude corruption — this mechanism directly breaks that cascade.

#### 1c. Welsch IRLS with Directional Robust Scale

Replaces FAST-LIO2's unweighted residuals and coarse Gate 2 with Welsch M-estimation using per-eigenspace-direction scale estimation.

**Pipeline intervention**: Within each IEKF iteration, after computing residuals.

**Equations**:

Compute Welsch weights with MAD-based adaptive scale:

$$w_j = \exp\left(-r_j^2 / c^2\right), \quad c = 2.5 \cdot 1.4826 \cdot \text{median}(|r_j - \text{median}(r)|)$$

**Novel directional extension**: Instead of a single global MAD, project residuals onto each information matrix eigenvector $v_d$ and compute directional MAD $\hat{\sigma}_d$. This prevents degenerate-direction residuals (which are large even for correct correspondences) from inflating the global scale and masking outliers along well-conditioned directions.

Per-point directional weight:

$$w_j = \prod_{d=1}^{3} \exp\left(-(r_j^{(d)})^2 / c_d^2\right), \quad c_d = 2.5\hat{\sigma}_d$$

The weighted IEKF normal equations become:

$$(H^T W R^{-1} H + (P^-)^{-1}) \delta x = H^T W R^{-1} r + (P^-)^{-1} \delta x_{\text{pred}}$$

**Replaces**: FAST-LIO2's Gate 2 heuristic ($s = 1 - 0.9|p_{d2}|/\sqrt{\|p_{\text{body}}\|} > 0.9$) and unweighted least squares. Also subsumes D²-LIO's motion-aware outlier filter.

#### 1d. Covariance-Consistent Update

Apply the modified state update with the effective Kalman gain:

$$\delta x_{\text{pose}}^+ = \begin{bmatrix} W_t & 0 \\ 0 & W_r \end{bmatrix} \delta x_{\text{pose}}^{\text{IEKF}}$$

Posterior covariance using Joseph form with the attenuated gain:

$$P^+ = (I - K_{\text{eff}} H) \tilde{P}^- (I - K_{\text{eff}} H)^T + K_{\text{eff}} R K_{\text{eff}}^T$$

This ensures the covariance correctly reflects reduced confidence along attenuated directions, unlike D²-LIO which applies the standard covariance update even when the gain is attenuated (creating overconfident estimates in degenerate directions).

### Adaptation for IEKF and Livox Mid-360

- **IEKF compatibility**: Fully native. All modifications are within the iterative update loop — the eigenvalue analysis, weighting, and robust estimation are recomputed at each IEKF iteration with the re-linearized $H$.
- **Livox compatibility**: Sensor-agnostic. The block-separated eigenvalue analysis, continuous weighting, and directional robust estimation depend only on the point-to-plane Jacobian structure, not the scan pattern.
- **Threshold tuning**: The median-based $\lambda_{\text{ref}}$ automatically adapts to the Livox's lower per-scan point count (which produces systematically lower eigenvalues than spinning LiDARs). D²-LIO's fixed $\lambda_{\text{thresh}}$ would require manual re-tuning for the Mid-360.

### Computational Cost

- Two $3 \times 3$ eigendecompositions: $<0.01$ ms
- Continuous weight computation: negligible (6 scalar operations)
- Cross-covariance attenuation: one $3 \times 6$ matrix multiply
- Directional MAD: 6× median computation over ~500–1000 points: $<0.5$ ms (quickselect)
- Welsch weight computation: $N$ exponentials: $<0.1$ ms
- Total: $<1$ ms per IEKF iteration

---

## Addition 2: Unified Heteroscedastic Measurement Noise Model

**Failure modes addressed**: FM5 (correspondence errors, via better noise modeling), FM6 (deskew errors), FM2 (amplified by constant noise in low-feature scenarios)

**Source**: Novel — [[proposed_methods]] Method 5. Adapts the sensor noise model from [[LIO with Uncertainty IEKF and Ground Constraint]] and introduces an analytical deskew uncertainty model (novel, lighter-weight alternative to [[De-Skewing with GP Regression]]).

**Impact**: ★★★★ — Replaces FAST-LIO2's constant $R = \sigma^2 I$ with a per-point noise that accounts for three physically meaningful uncertainty sources. The deskew component is the most impactful: during the aggressive hand-held motions observed in Cameroon and Drift_GTC, deskew-induced point displacements are on the order of centimeters — comparable to the point-to-plane distances used for gating — yet FAST-LIO2 treats all points as equally noisy.

**Difficulty**: Low — Only the diagonal of $R$ changes. No structural modification to the IEKF.

### What It Does

**Pipeline intervention**: After kNN search and plane fitting, before the IEKF measurement update. Replaces the constant $R$ diagonal.

**Combined per-point noise**:

$$R_{jj} = \underbrace{\sigma_0^2 \left(1 + \frac{d_j^2}{d_{\text{ref}}^2} + \frac{1}{\cos^2(\alpha_j)}\right)}_{\text{sensor noise}} + \underbrace{n_j^T \left([p_j]_\times^T (\sigma_g^2 \Delta\tau_j) [p_j]_\times + \frac{\sigma_a^2 \Delta\tau_j^3}{3} I\right) n_j}_{\text{deskew noise (novel, analytical)}} + \underbrace{\sigma_{\text{map},j}^2}_{\text{map quality (Addition 4)}}$$

where:
- $d_j$ = range, $\alpha_j$ = incidence angle (from kNN plane normal vs. ray direction), $d_{\text{ref}} \approx 10$ m
- $\sigma_g$, $\sigma_a$ = gyroscope and accelerometer noise (from online estimation in Addition 5, or defaults)
- $\Delta\tau_j = \tau_j - t_0$ = time offset of point $j$ within the scan (from Livox per-point timestamps)
- $p_j$ = point position in body frame, $n_j$ = fitted plane normal
- $\sigma_{\text{map},j}^2$ = map point registration quality (from Addition 4; set to 0 until Addition 4 is implemented)

### Key Physical Insight (Deskew Noise)

The deskew noise term captures that:
- Points **later in the scan** ($\Delta\tau_j$ large) have more deskew uncertainty (longer IMU integration)
- Points at **longer range** ($\|p_j\|$ large) have more deskew uncertainty (rotation uncertainty amplified by lever arm via $[p_j]_\times$)
- Points during **aggressive motion** have more deskew uncertainty (through $\sigma_g$, $\sigma_a$)

This is a closed-form analytical model — no GP regression needed (unlike [[De-Skewing with GP Regression]]). The GP approach provides more accurate per-point uncertainty but at significantly higher computational cost and implementation complexity.

### Adaptation for IEKF and Livox Mid-360

- **IEKF compatibility**: Direct — $R$ is already a diagonal matrix in FAST-LIO2's IEKF. Changing its entries from constant to per-point requires no structural changes.
- **Livox compatibility**: The Livox driver provides per-point timestamps ($\tau_j$) with microsecond precision, making the deskew noise term directly computable. The non-uniform density of the Livox pattern makes heteroscedastic noise particularly valuable: edge-of-pattern points (higher range, lower density, worse incidence angles) automatically get higher noise.
- **Intensity term**: The intensity-dependent noise term from [[LIO with Uncertainty IEKF and Ground Constraint]] is omitted unless Mid-360 intensity is calibrated for range and angle dependence. This can be added later if intensity calibration is performed (required for Addition 3's IGE-LIO features if those are pursued).

### Computational Cost

- Incidence angle: one dot product per point (normal already computed, ray direction = $p_j / \|p_j\|$): $<0.05$ ms
- Deskew noise: one $3 \times 3$ matrix multiply + scalar operations per point: $<0.1$ ms
- Total: $<0.2$ ms per scan — negligible

### Interaction with Addition 1

Addition 1's Welsch IRLS operates on the weighted residuals $r_j / \sqrt{R_{jj}}$. With heteroscedastic $R$, the Welsch weights automatically adapt to the per-point noise level: a residual that is large relative to a well-observed point's noise gets down-weighted aggressively, while the same absolute residual from a poorly-observed point (high $R_{jj}$) is treated more leniently. The directional MAD (Addition 1c) is computed on the noise-normalized residuals, ensuring the robust scale reflects true outlier status rather than noise variation.

---

## Addition 3: Normal-Diversity-Aware Adaptive Downsampling

**Failure modes addressed**: FM2 (insufficient features), FM9 (solid-state LiDAR non-uniform density), FM1 (indirectly, by improving information matrix conditioning)

**Source**: Novel — [[proposed_methods]] Method 7. Draws on concepts from [[Geometrically Stable Sampling]] (normal diversity for conditioning) and [[Towards High-Performance SS-LIO]] (density-aware processing for solid-state LiDARs).

**Impact**: ★★★☆ — This is a preprocessing improvement that raises the floor of information matrix conditioning by ensuring the downsampled point cloud has balanced normal diversity. In corridor/tunnel scenarios (Senegal, Valis), the along-corridor direction has few points with informative normals; uniform voxel downsampling further discards these, deepening the degeneracy. Normal-binned downsampling preserves them. The density-adaptive voxel size prevents the Livox's non-uniform pattern from wasting features in sparse regions.

**Difficulty**: Medium — Replaces FAST-LIO2's voxel grid filter with a more complex two-stage pipeline. Requires modifying the downsampling module and adding a normal binning step.

### What It Does

**Pipeline intervention**: Replaces `voxel_filter.filter()` in the preprocessing pipeline.

**Stage 1: Normal-Direction Binning**

After kNN plane fitting (which FAST-LIO2 already performs), partition points by their fitted normal direction into $K = 6$ axis-aligned bins ($\pm x, \pm y, \pm z$):

$$b_j = \arg\max_{k \in \{1,...,6\}} |n_j \cdot c_k|$$

where $c_k$ are the 6 axis unit vectors.

Sample $N_{\text{target}} / K_{\text{occupied}}$ points from each non-empty bin, where $K_{\text{occupied}}$ is the number of bins with at least one point.

**Stage 2: Density-Adaptive Voxel Size (within each bin)**

$$v_{\text{local}} = v_0 \cdot \sqrt{\rho_{\text{ref}} / \rho_{\text{local}}(p_j)}$$

where $\rho_{\text{local}}$ is estimated from existing kNN distances: $\rho_{\text{local}} \approx K / (\frac{4}{3}\pi d_K^3)$. High-density regions get larger voxels (more aggressive downsampling); sparse regions get smaller voxels (preserving scarce features).

**Stage 3: Information-Aware Top-Up (conditional)**

If the downsampled set has $< N_{\min}$ points (e.g., 200), add points from the bin whose normal direction most improves $\lambda_{\min}$ of the current information matrix. This requires only one $3 \times 3$ eigendecomposition (from Addition 1, already available).

### Adaptation for Livox Mid-360

This method is **designed for** the Livox's non-uniform density. The density-adaptive voxel size directly addresses the non-repetitive rosette pattern where some angular regions have high density (freshly scanned) and others are sparse (not yet covered in the current scan). Uniform voxel grids over-downsample high-density regions and under-preserve sparse ones.

### Computational Cost

- Normal binning: $O(N)$ bin assignments — negligible
- Density estimation: reuses existing kNN distances
- Adaptive voxel filter: same complexity as the existing voxel filter, applied per-bin
- Total: comparable to the existing voxel filter (~1 ms)

---

## Addition 4: Quality-Gated Map Insertion with Temporal Aging

**Failure modes addressed**: FM8 (map-history effects — the most poorly covered failure mode), FM4 (dynamic objects/ghosting)

**Source**: Novel — [[proposed_methods]] Method 4. Draws on [[Line-LIO]] (quality-driven keyframes, but at per-point rather than per-frame granularity) and [[Online Dynamic Point Separation and Removal]] (dynamic object handling within FAST-LIO).

**Impact**: ★★★★ — FM8 was identified as the most poorly covered failure mode. The Cameroon dataset proved this definitively: the same physical trajectory succeeds or fails depending entirely on map quality from earlier sections. No existing method provides online map quality tracking or per-point insertion gating for FAST-LIO2's ikd-Tree.

**Difficulty**: Medium — Requires extending the ikd-Tree point structure with metadata and modifying the map insertion logic.

### What It Does

**Pipeline intervention**: After the IEKF converges, before `ikd_Tree.Add_Points()`.

#### Per-Point Map Metadata

Extend each ikd-Tree point with:

| Field | Size | Description |
|---|---|---|
| `cov_trace` | 4 B | $\text{tr}(P_{\text{pos}})$ at insertion time |
| `timestamp` | 4 B | Insertion wall-clock time |
| `match_count` | 2 B | Times used as successful correspondence |
| `quality_flag` | 1 B | 0 = suspect, 1 = normal, 2 = verified |

Memory overhead: 11 bytes per point. For a 500K-point map: ~5.5 MB — negligible.

#### Quality-Gated Insertion

For each point to be inserted, compute:

$$q_j = w(\lambda_{\min}^t) \cdot w(\lambda_{\min}^r) \cdot \frac{N_{\text{inlier}}}{N_{\text{total}}} \cdot \exp(-\bar{r}^2 / \tau_r^2)$$

where $w(\cdot)$ is the continuous weight from Addition 1. Points with $q_j < q_{\text{low}}$ are **not inserted**, preventing poorly-registered scans from corrupting the map.

This directly addresses the Cameroon failure mechanism: during the degeneracy at ~200 s, scans would normally be inserted into the map despite poor registration. With quality gating, these scans are blocked, preventing the map corruption that caused subsequent divergence.

#### Temporal Aging (Soft Ghost Removal)

During kNN correspondence matching, the measurement noise for each correspondence incorporates map point metadata:

$$R_{jj}^{\text{map}} = \alpha_{\text{reg}} \cdot \sigma_{\text{reg},j}^2 + \alpha_{\text{age}} \cdot (1 - e^{-\Delta t_j / \tau_{\text{age}}}) + \alpha_{\text{use}} / (1 + n_j)$$

Old, rarely-re-observed points (likely ghosts from dynamic objects or from a physically changed environment) are gradually down-weighted rather than hard-deleted. This is lighter-weight than [[RF-LIO]]'s explicit dynamic detection (which struggles with the Livox's non-repetitive pattern) and avoids the irreversibility of ikd-Tree point deletion.

#### Match Count Verification

When a map point produces a low residual (below the Welsch threshold from Addition 1c), its `match_count` is incremented. Points with high match counts and consistently low residuals are upgraded to `verified`. Points that consistently produce large residuals are downgraded to `suspect` and receive even higher measurement noise.

### Adaptation for Livox Mid-360

- The non-repetitive scan pattern means consecutive scans observe different parts of the scene. This makes point-level temporal consistency unreliable for dynamic detection (as noted for [[RF-LIO]]). The temporal aging approach avoids this: it doesn't require point-level re-observation, only that *some* point near the map point is re-observed. The kNN-based matching naturally handles this.
- Quality-gated insertion is particularly valuable for Livox: since single non-repetitive scans may have poor spatial coverage, some scans are inherently poorly conditioned and should not contribute to the map.

### Interaction with Addition 2

The map quality noise $\sigma_{\text{map},j}^2$ from this addition feeds directly into Addition 2's heteroscedastic $R$. Points matched against poorly-registered map regions automatically receive higher measurement noise, reducing their influence on the state update.

---

## Addition 5: Online IMU Noise Estimation

**Failure modes addressed**: FM7 (IMU quality limitations), FM6 (deskew errors, via better $\sigma_g$, $\sigma_a$ estimates)

**Source**: Literature — [[Robust LIO Without Sensor-Specific Modeling]], with a modification to handle degeneracy.

**Impact**: ★★★☆ — Directly solves the covariance tuning problem. FAST-LIO2's default `acc_cov: 0.1`, `gyr_cov: 0.1` are manually inflated ~10× above the ICM40609's physical noise to absorb modeling errors. This works but makes the filter unnecessarily conservative during well-conditioned operation and provides no adaptation when conditions change.

**Difficulty**: Low — Innovation-based covariance estimation on existing filter quantities. No structural changes.

### What It Does

**Pipeline intervention**: After each IEKF update, modifies the process noise $Q$ for the next prediction step.

**Equations** (from [[Robust LIO Without Sensor-Specific Modeling]]):

$$\hat{Q}_k = \frac{1}{W}\sum_{i=k-W+1}^{k}(K_i \nu_i)(K_i \nu_i)^T + P_k^+ - F_k P_{k-1}^+ F_k^T$$

Per-axis extraction: $\sigma_{g_x} = \sqrt{[\hat{Q}_k]_{4,4}} / \Delta t$, etc.

Exponential smoothing: $\hat{Q}_k^{\text{smooth}} = (1-\alpha)\hat{Q}_{k-1}^{\text{smooth}} + \alpha \hat{Q}_k^{\text{raw}}$

### Critical Modification: Degeneracy-Aware Gate

The original method has a **known interaction risk**: during LiDAR degeneracy, innovations reflect the degeneracy (not IMU noise), and the system may incorrectly reduce $Q$ (increase IMU trust) precisely when it should not.

**Fix**: Gate the $Q$ update on Addition 1's degeneracy indicators. When any continuous weight $w(\lambda_i) < w_{\text{gate}}$ (e.g., 0.5), freeze $\hat{Q}$ at its last well-conditioned value:

$$\hat{Q}_k^{\text{smooth}} = \begin{cases} (1-\alpha)\hat{Q}_{k-1}^{\text{smooth}} + \alpha \hat{Q}_k^{\text{raw}} & \text{if all } w(\lambda_i) > w_{\text{gate}} \\ \hat{Q}_{k-1}^{\text{smooth}} & \text{otherwise} \end{cases}$$

This prevents the dangerous positive feedback loop where degeneracy → inflated innovations → reduced $Q$ → increased IMU trust → worse estimation during degeneracy.

### Interaction with Addition 2

The online-estimated $\sigma_g$ and $\sigma_a$ feed directly into Addition 2's analytical deskew uncertainty model. As the IMU noise is estimated more accurately, the deskew noise component of $R$ automatically adjusts, creating a consistent noise pipeline from IMU → deskewing → measurement noise.

### Computational Cost

Windowed covariance estimation over existing innovations and Kalman gains: $<0.1$ ms per update.

---

## Addition 6: Ground Plane Pseudo-Measurements for Attitude Anchoring

**Failure modes addressed**: FM3 (attitude mis-estimation and gravity vector corruption)

**Source**: Literature — [[LIO with Uncertainty IEKF and Ground Constraint]], adapted for the Mid-360.

**Impact**: ★★★☆ — Provides a direct constraint on roll and pitch, anchoring the attitude estimate even when LiDAR point-to-plane constraints are weak. This specifically addresses the Valis failure cascade where attitude drift during low-FoV periods caused phantom acceleration. The impact is conditional: high when the robot operates on flat or near-flat ground (most indoor/industrial scenarios), zero when ground is not visible or terrain is highly uneven.

**Difficulty**: Low-Medium — Ground plane detection + 2–3 additional pseudo-measurement rows in $H$.

### What It Does

**Pipeline intervention**: Additional rows appended to $H$ and $z$ in the measurement model.

#### Ground Plane Detection

From the downsampled point cloud (after Addition 3), detect ground points:
1. Select points with $z_{\text{body}} < z_{\text{thresh}}$ (e.g., points below the sensor, accounting for the known LiDAR-to-body extrinsics)
2. Fit a plane via RANSAC or PCA on the candidate ground points
3. Verify: planarity score $>$ threshold, normal approximately aligned with the estimated gravity direction ($|n_g \cdot \hat{g}| > 0.95$)

#### Pseudo-Measurements

If a valid ground plane is detected:

**Height constraint** (restricts vertical translation):
$$h_{\text{height}}(x) = e_3^T (p^W + R_B^W p_{\text{LiDAR}}^B) = z_{\text{ground}}^W$$

**Attitude constraint** (restricts roll/pitch):
$$h_{\text{att}}(x) = R^{BW} n_g^W - e_3 = 0$$

These are appended to the existing measurement vector: $H_{\text{aug}} = [H_{\text{LiDAR}}; H_{\text{height}}; H_{\text{att}}]$ with corresponding noise entries in $R_{\text{aug}}$.

#### Adaptive Confidence

The ground constraint noise is set adaptively:

$$R_{\text{ground}}^{(k)} = R_{\text{ground},0} / \max(1, N_{\text{ground}} \cdot s_{\text{planarity}} \cdot s_{\text{consistency}})$$

where $N_{\text{ground}}$ is the number of ground inliers, $s_{\text{planarity}}$ is the PCA planarity score, and $s_{\text{consistency}}$ measures consistency with the previous frame's ground estimate. When ground detection is uncertain, $R_{\text{ground}}$ is large, and the constraint has minimal influence.

### Adaptation for Livox Mid-360

- The Mid-360's lower FoV extends to $-7°$ below horizontal, meaning it reliably observes the ground when the sensor is at typical hand-held height (1.0–1.5 m) in indoor environments.
- The non-repetitive scan pattern produces variable ground point density per scan, making the adaptive confidence essential — some scans will have many ground points, others few.
- **Limitation**: This constraint assumes approximately flat ground. For datasets with stairs, ramps, or highly uneven terrain, the ground constraint should be disabled (detectable from the planarity score dropping below threshold).

### Interaction with Addition 1

During geometric degeneracy, the ground pseudo-measurements provide attitude constraints that Addition 1 alone cannot: even if all LiDAR point-to-plane normals are degenerate (e.g., corridor with only wall normals), the ground constraint still anchors roll and pitch via gravity alignment. This is complementary: Addition 1 prevents the degenerate update from corrupting attitude, while Addition 6 provides positive attitude information.

### Computational Cost

- Ground point selection: $O(N)$ threshold check
- Plane fitting: RANSAC on ~50–200 ground candidates: $<0.5$ ms
- Pseudo-measurement Jacobian: 3 additional rows in $H$: negligible
- Total: $<1$ ms per scan

---

## Failure Mode Coverage Matrix

| Failure Mode | Primary Addition | Secondary Addition | Coverage |
|---|---|---|---|
| FM1: Geometric Degeneracy | Addition 1 (CEWS-KF) | Addition 3 (normal diversity) | ████████ Strong |
| FM2: Insufficient Features | Addition 3 (normal-diversity downsampling) | Addition 2 (heteroscedastic R) | ██████░░ Good |
| FM3: Attitude Corruption | Addition 1 (cross-covariance protection) | Addition 6 (ground constraint) | ████████ Strong |
| FM4: Dynamic Objects | Addition 4 (temporal aging) | Addition 1 (Welsch IRLS) | █████░░░ Moderate |
| FM5: Correspondence Errors | Addition 1 (Welsch IRLS) | Addition 2 (heteroscedastic R) | ████████ Strong |
| FM6: Deskew Errors | Addition 2 (deskew noise in R) | Addition 5 (online σ_g, σ_a) | ██████░░ Good |
| FM7: IMU Quality | Addition 5 (online noise estimation) | Addition 1 (degeneracy attenuation) | ██████░░ Good |
| FM8: Map Corruption | Addition 4 (quality-gated insertion) | — | ██████░░ Good |
| FM9: SS-LiDAR Challenges | Addition 3 (density-adaptive downsampling) | — | █████░░░ Moderate |
| FM10: Scale Mismatch | Addition 1 (block-separated analysis) | — | ████████ Strong |

### What Is NOT Included (and Why)

| Method | Why Excluded |
|---|---|
| [[LODESTAR]] Schmidt-Kalman | Addition 1's CEWS-KF provides the same benefit (covariance-consistent degeneracy handling) with continuous rather than binary weighting. Adding LODESTAR would be redundant and the binary partitioning is strictly less expressive than continuous weighting. |
| [[I2EKF-LO]] dual-iteration EKF | The continuous process noise adaptation is partially captured by Addition 5 (online IMU noise estimation with degeneracy gating). The full dual-iteration architecture would conflict with Addition 1's within-iteration Welsch IRLS. |
| [[RF-LIO]] / [[TRLO]] dynamic detection | Addition 4's temporal aging provides a lighter-weight, scan-pattern-agnostic alternative. Full dynamic detection is computationally expensive and struggles with the Livox's non-repetitive pattern (point-level temporal consistency is unreliable). |
| [[SR-LIO++]] sweep reconstruction | While valuable for the Livox, sweep reconstruction changes the scan-level interface to FAST-LIO2, complicating the integration of all other additions. The benefit (denser scans) is partially captured by Addition 3's density-adaptive downsampling. Consider as a future addition. |
| [[AC-LIO]] adaptive deskewing | Addition 2's analytical deskew noise model is a lighter-weight approach — instead of fixing the deskewing, it correctly quantifies the remaining uncertainty and propagates it into $R$. The two approaches are complementary but AC-LIO adds complexity for moderate gain given the analytical model. |
| [[IGE-LIO]] intensity gradients | High potential but requires Mid-360 intensity calibration (range- and angle-dependent correction) as a prerequisite. Without calibration, spurious intensity gradients degrade rather than improve performance. Recommend as a future addition after intensity calibration. |
| [[Dynamic Initialization for LiDAR-Inertial SLAM]] | Valuable for FM8, but addresses only initialization — a one-time event. The observed failures (Cameroon, Valis) occurred well after initialization. Priority is lower than the six additions above. |
| [[LIO-EKF]] per-point sequential updates | Architecturally radical — would require rewriting the entire IEKF pipeline. The benefit (reduced dead-reckoning intervals) is achievable less invasively through the combination of Additions 1 and 5. |
| [[Equivariant Filter for LIO]] | Requires fundamental reformulation of the filter structure (from IEKF to EqF). Theoretically superior for attitude consistency but incompatible with incremental additions to FAST-LIO2. |
| Predictive Degeneracy Detection ([[proposed_methods]] Method 6) | Architecturally the most complex proposed method. The proactive countermeasures it enables (sweep accumulation, preemptive IMU trust adjustment) provide diminishing returns once Additions 1–5 are handling degeneracy reactively. Consider as a future research direction. |

---

## Implementation Order and Expected Outcomes

### Phase 1: Core Robustification (Additions 1 + 2)

**Effort**: ~2–3 weeks. Both modify the IEKF measurement update with no structural changes.

**Expected outcome**: Resolution of catastrophic divergence in Cameroon, Valis, and Senegal. The combination of continuous degeneracy attenuation, cross-covariance protection, robust residuals, and heteroscedastic noise addresses the primary failure cascade (FM1 → FM5 → FM3 → divergence) documented across all datasets.

**Test**: Re-run the Cameroon 0–400 s, Valis full, and Senegal sequences. Success criterion: no divergence; drift bounded.

### Phase 2: Map Quality (Addition 4)

**Effort**: ~1–2 weeks. Requires ikd-Tree metadata extension.

**Expected outcome**: Resolution of the Cameroon initialization sensitivity (running 0–400 s should now match 100–400 s in performance) and reduced ghost artifacts.

**Test**: Run Cameroon 0–400 s with and without Addition 4. Compare trajectory quality. Also compare final map cleanness (ghost point count).

### Phase 3: Preprocessing (Additions 3 + 5)

**Effort**: ~2 weeks. Independent of each other, can be implemented in parallel.

**Expected outcome**: Improved baseline conditioning (Addition 3) and correctly calibrated IMU trust (Addition 5). These are "raise the floor" improvements — they don't fix specific failures but reduce the frequency and severity of near-degenerate situations.

**Test**: Monitor eigenvalue statistics across all datasets. Addition 3 should increase $\lambda_{\min}$ during previously degenerate periods. Addition 5 should produce per-axis noise estimates consistent with the ICM40609 datasheet during well-conditioned operation.

### Phase 4: Attitude Anchoring (Addition 6)

**Effort**: ~1 week. Straightforward pseudo-measurement addition.

**Expected outcome**: Improved roll/pitch stability during degeneracy in indoor/flat-ground scenarios. Most impactful for Drift_GTC (corridor with flat floor) and Valis (tunnel with floor visible).

**Test**: Compare compensated acceleration vector stability ($|a_\perp|$) with and without Addition 6 during degenerate periods.

---

## Ranking Summary

| Rank | Addition | Impact | Difficulty | Failure Modes |
|---|---|---|---|---|
| 1 | **Addition 1**: CEWS-KF + Cross-Cov + Welsch | ★★★★★ | Low-Medium | FM1, FM3, FM5, FM10 |
| 2 | **Addition 2**: Heteroscedastic R | ★★★★ | Low | FM5, FM6, FM2 |
| 3 | **Addition 4**: Quality-Gated Map | ★★★★ | Medium | FM8, FM4 |
| 4 | **Addition 5**: Online IMU Noise | ★★★☆ | Low | FM7, FM6 |
| 5 | **Addition 3**: Normal-Diversity Downsampling | ★★★☆ | Medium | FM2, FM9, FM1 |
| 6 | **Addition 6**: Ground Constraint | ★★★☆ | Low-Medium | FM3 |

---

## Publication Potential

The six additions form a coherent, publishable system. The strongest narrative centers on **Additions 1 + 2** as the core contribution:

**Title direction**: *"Degeneracy-Resilient LiDAR-Inertial Odometry via Continuous Eigenvalue-Weighted Filtering with Cross-Covariance Protection and Directional Robust Estimation"*

**Novel contributions for a publication**:
1. **Continuous eigenvalue weighting** with self-adapting threshold (vs. binary detection/attenuation in all prior work)
2. **Cross-covariance attitude protection** during translation degeneracy (previously unaddressed failure mechanism)
3. **Directional robust scale estimation** for anisotropic residual distributions during degeneracy (vs. global MAD)
4. **Analytical deskew uncertainty model** providing per-point, closed-form deskew noise without GP regression
5. **Quality-gated map insertion** with temporal aging for long-duration robustness

The experimental evaluation compares against:
- FAST-LIO2 baseline (current)
- FAST-LIO2 + D²-LIO (binary degeneracy handling, current state-of-the-art)
- FAST-LIO2 + RELEAD (constrained IEKF, recent)
- The proposed system (all six additions)

on the internal Tinamu datasets (Cameroon, Silo, Drift_GTC, Valis, Senegal) plus standard benchmarks (NTU-VIRAL for Livox, HiltiSLAM for degeneracy scenarios).

The combination of rigorous theoretical grounding (covariance consistency, cross-coupling analysis), practical impact (fixing documented real-world failures), and clean integration (no system redesign, composable additions) positions this well for a venue like ICRA, IROS, or RA-L.
