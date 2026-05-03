# Proposed Novel Methods for Improving FAST-LIO2 Robustness

This document proposes novel methods — or novel adaptations of existing ideas — to address the failure modes of FAST-LIO2 (as catalogued in [[failure_modes]]) on the Livox Mid-360 solid-state LiDAR with its ICM40609 IMU. Each method is designed to be implementable within or alongside FAST-LIO2's IEKF pipeline without full system redesign.

For existing literature context, see [[papers_summary]].

---

## Method 1: Continuous Eigenvalue-Weighted Schmidt-Kalman Filter (CEWS-KF)

**Addresses**: FM1 (Geometric Degeneracy), FM3 (Attitude Corruption, indirectly), FM10 (Scale Mismatch)

### Motivation and Novelty

Existing degeneracy mitigation strategies are **binary**: D²-LIO zeros out the Kalman gain along degenerate eigenvectors (complete information discard), while LODESTAR partitions states into "solve" and "consider" sets. Both use a hard eigenvalue threshold $\lambda_{\text{thresh}}$ that determines whether a direction is degenerate or not. This creates three problems:

1. **Threshold sensitivity**: The threshold must be manually tuned per environment and sensor. Too low → degeneracy missed; too high → useful information discarded.
2. **Discontinuous behavior**: The state update jumps when a direction crosses the threshold — creating trajectory discontinuities.
3. **Partial information waste**: During *mild* degeneracy ($\lambda_i$ is small but nonzero), the LiDAR still provides partial information along the "degenerate" direction. Binary methods discard this entirely.

**Novelty**: Replace the binary solve/consider partition with a **continuous weighting function** in the eigenspace of the block-separated information matrix, applied within a Schmidt-Kalman covariance framework for consistency.

### Technical Description

#### Step 1: Block-Separated Eigenvalue Analysis (from D²-LIO)

Compute the translation and rotation information matrices separately:

$$
A_t = H_t^T R^{-1} H_t \in \mathbb{R}^{3 \times 3}, \quad A_r = H_r^T R^{-1} H_r \in \mathbb{R}^{3 \times 3}
$$

Eigendecompose each:

$$
A_t = V_t \Lambda_t V_t^T, \quad A_r = V_r \Lambda_r V_r^T
$$

#### Step 2: Continuous Weighting Function

For each eigenvalue $\lambda_i$ (in either block), define the weight:

$$
w(\lambda_i) = \frac{\lambda_i^2}{\lambda_i^2 + \lambda_{\text{ref}}^2}
$$

where $\lambda_{\text{ref}}$ is an **adaptive reference eigenvalue** defined as:

$$
\lambda_{\text{ref}} = \beta \cdot \text{median}(\lambda_1, \lambda_2, \lambda_3)
$$

with $\beta \in (0, 1)$ as a single tuning parameter (e.g., $\beta = 0.1$).

Properties:
- $w \to 1$ when $\lambda_i \gg \lambda_{\text{ref}}$ (well-conditioned → full update)
- $w \to 0$ when $\lambda_i \ll \lambda_{\text{ref}}$ (degenerate → no update)
- Smooth transition with no discontinuity
- **Self-adapting**: $\lambda_{\text{ref}}$ tracks the current scene's information level via the median eigenvalue, so the method automatically adjusts to different environments and point densities without manual threshold tuning

Using the squared form $\lambda_i^2 / (\lambda_i^2 + \lambda_{\text{ref}}^2)$ rather than $\lambda_i / (\lambda_i + \lambda_{\text{ref}})$ provides a sharper roll-off, reducing the parameter sensitivity.

#### Step 3: Weighted Kalman Gain

Construct the per-block weighting matrices:

$$
W_t = V_t \, \text{diag}(w(\lambda_1^t), w(\lambda_2^t), w(\lambda_3^t)) \, V_t^T
$$

$$
W_r = V_r \, \text{diag}(w(\lambda_1^r), w(\lambda_2^r), w(\lambda_3^r)) \, V_r^T
$$

The modified state update for the pose components:

$$
\delta x_{\text{pose}}^+ = \begin{bmatrix} W_t & 0 \\ 0 & W_r \end{bmatrix} \delta x_{\text{pose}}^{\text{IEKF}}
$$

where $\delta x_{\text{pose}}^{\text{IEKF}}$ is the standard IEKF state update for the 6 pose states.

#### Step 4: Covariance-Consistent Update (Schmidt-Kalman Extension)

The key advantage over D²-LIO's regularization: the posterior covariance must reflect the *fractional* update. D²-LIO applies the standard Joseph-form covariance update as if the full Kalman gain were used, creating overconfident covariance along attenuated directions. CEWS-KF corrects this.

Define the effective Kalman gain:

$$
K_{\text{eff}} = \begin{bmatrix} W_t & 0 \\ 0 & W_r \\ 0 & 0 \\ \vdots & \vdots \end{bmatrix} K
$$

(The zero blocks correspond to the non-pose states — velocity, bias, gravity — which are updated normally.)

The covariance update uses the modified Joseph form:

$$
P^+ = (I - K_{\text{eff}} H) P^- (I - K_{\text{eff}} H)^T + K_{\text{eff}} R K_{\text{eff}}^T
$$

Since $K_{\text{eff}}$ is attenuated along degenerate directions, $P^+$ correctly preserves large uncertainty along those directions. Along well-conditioned directions ($w \approx 1$), the standard covariance reduction applies. This ensures **covariance consistency**: the filter never becomes overconfident in degenerate directions.

### Integration into FAST-LIO2

- **Intervention point**: After the IEKF computes $H$, $R$, and the standard Kalman gain $K$, but before applying the state update.
- **Computational overhead**: Two $3 \times 3$ eigendecompositions (negligible) + modified state update and covariance update (same matrix dimensions, only the gain is modified).
- **State vector**: Unchanged. No state augmentation.
- **Non-pose states**: Velocity, bias, and gravity states are updated normally (their Kalman gain rows are not attenuated).

### Why This Is Novel

- D²-LIO: Binary regularization, covariance-inconsistent.
- LODESTAR: Binary Schmidt-Kalman (solve/consider), covariance-consistent but still binary.
- Selective KF: Binary per-direction selection.
- AdaLIO: Continuous weighting but in optimization, not IEKF; no covariance consistency mechanism.
- **CEWS-KF**: Continuous weighting + covariance-consistent + block-separated + self-adapting threshold. This combination is novel.

---

## Method 2: Cross-Covariance-Aware Attitude Protection During Translation Degeneracy

**Addresses**: FM1 → FM3 cascade (Translation Degeneracy → Attitude Corruption), coverage gap "Cross-coupling in IEKF covariance"

### Motivation and Novelty

The failure cascade documented across Valis, Senegal, and Drift_GTC follows a consistent pattern:

1. Translation degeneracy detected ($\lambda_{\min}^t \to 0$)
2. Translation error accumulates (expected, mitigated by Method 1)
3. Translation error **leaks into attitude** through the IEKF's off-diagonal covariance blocks $P_{t,r}$
4. Corrupted attitude → phantom acceleration → catastrophic divergence

No existing method addresses step 3. D²-LIO and LODESTAR handle steps 1–2 by attenuating or ignoring the degenerate translation update. But the IEKF's coupled covariance means that even with a perfect degeneracy handler for translation, the *rotation update in the same IEKF step* is influenced by the translation-rotation cross-covariance — and this cross-term can be corrupted by residuals along the degenerate translation direction.

**Novelty**: Explicit cross-covariance protection during degeneracy events by decomposing the IEKF update to prevent degenerate-direction residuals from influencing non-degenerate states through the covariance structure.

### Technical Description

#### The Cross-Coupling Problem

In FAST-LIO2's IEKF, the Kalman gain is:

$$
K = P^- H^T (H P^- H^T + R)^{-1}
$$

The gain for the rotation states includes:

$$
K_r = P_{rr}^- H_r^T S^{-1} + P_{rt}^- H_t^T S^{-1}
$$

where $S = H P^- H^T + R$ is the innovation covariance, $P_{rr}^-$ is the rotation-rotation prior covariance, and $P_{rt}^-$ is the rotation-translation cross-covariance.

The second term, $P_{rt}^- H_t^T S^{-1}$, means that the rotation update is influenced by the translation Jacobian $H_t$ through the cross-covariance $P_{rt}^-$. Along a degenerate translation direction $v_d$, $H_t v_d$ produces large, noise-dominated residuals. These residuals, scaled by $P_{rt}^-$, corrupt the rotation update.

#### Proposed Solution: Eigenspace-Projected Cross-Covariance Attenuation

When translation degeneracy is detected (eigenvalue $\lambda_i^t$ is small), attenuate the cross-covariance *along the degenerate direction*:

1. Identify degenerate translation eigenvectors $\{v_d\}$ from $A_t$'s eigendecomposition
2. For each degenerate direction, attenuate the corresponding cross-covariance:

$$
\tilde{P}_{rt}^- = P_{rt}^- \left(I - \sum_{d} (1 - w(\lambda_d^t)) \, v_d v_d^T \right)
$$

where $w(\lambda_d^t)$ is the continuous weight from Method 1. This zeroes out the cross-covariance along directions where translation is degenerate, preventing degenerate translation residuals from influencing the rotation update.

3. Use $\tilde{P}_{rt}^-$ (and its transpose $\tilde{P}_{tr}^-$) when computing the Kalman gain:

$$
\tilde{P}^- = \begin{bmatrix} P_{tt}^- & \tilde{P}_{tr}^- \\ \tilde{P}_{rt}^- & P_{rr}^- \end{bmatrix}
$$

4. Compute the Kalman gain with $\tilde{P}^-$:

$$
K = \tilde{P}^- H^T (H \tilde{P}^- H^T + R)^{-1}
$$

#### Covariance Update

The posterior covariance is computed using the standard Joseph form but with $\tilde{P}^-$:

$$
P^+ = (I - K H) \tilde{P}^- (I - K H)^T + K R K^T
$$

The attenuation of $P_{rt}^-$ is conservative: it preserves uncertainty rather than creating it. The worst case (full attenuation) is equivalent to treating translation and rotation as uncorrelated, which is always a safe assumption.

#### Symmetry: Rotation Degeneracy → Translation Protection

The same mechanism works in reverse. When rotation is degenerate (e.g., cylindrical geometry in the Silo dataset), attenuate $P_{tr}^-$ along degenerate rotation eigenvectors to prevent rotation residuals from corrupting translation updates.

### Integration into FAST-LIO2

- **Intervention point**: Immediately before computing the Kalman gain, after Method 1's eigenvalue analysis.
- **Computational overhead**: One $6 \times 6$ matrix modification (attenuation of off-diagonal blocks) per IEKF iteration — negligible.
- **Synergy with Method 1**: Uses the same eigenvalue analysis and continuous weights. The two methods naturally compose: Method 1 attenuates direct updates, Method 2 attenuates cross-coupling.
- **State vector**: Unchanged.

### Why This Is Novel

No existing degeneracy mitigation method addresses the covariance cross-coupling between translation and rotation. D²-LIO treats them independently. LODESTAR's Schmidt-Kalman preserves cross-covariance but doesn't selectively attenuate it. This method directly targets the FM1 → FM3 failure cascade mechanism identified in the failure analysis.

---

## Method 3: Welsch IRLS-IEKF with Directional Robust Scale Estimation

**Addresses**: FM5 (Correspondence Errors), FM1 (amplification of degeneracy by outliers), coverage gap "Robust kernel in IEKF"

### Motivation and Novelty

FAST-LIO2 uses unweighted least squares in the IEKF measurement update. Bad correspondences (from map drift, dynamic objects, or kNN ambiguity) disproportionately affect the state update, especially when feature counts are low (FM2) or when the problem is ill-conditioned (FM1). The D²-LIO motion-aware outlier filter is a binary accept/reject gate; RELEAD uses Huber M-estimation; Kinematic-Aware Robust LIO-W uses Welsch but coupled with wheel odometry.

**Gap**: No method combines (a) Welsch M-estimation (stronger outlier rejection than Huber) with (b) MAD-based adaptive scale in (c) the IEKF framework for (d) handheld use, and critically, no method computes the robust scale **per eigenspace direction** to handle the interaction between outliers and degeneracy.

**Novelty**: Directional robust scale estimation — computing the residual scale separately along well-conditioned and degenerate directions, because residuals are distributed differently along these directions.

### Technical Description

#### Step 1: Standard Welsch IRLS within IEKF

At each IEKF iteration $i$:

1. Compute residuals: $r_j^{(i)} = n_j^T (T^{(i)} p_j - q_j)$ for each correspondence $j$
2. Compute the robust scale using MAD:
$$
\hat{\sigma} = 1.4826 \cdot \text{median}_j(|r_j - \text{median}(r_j)|)
$$
3. Compute Welsch weights:
$$
w_j = \exp\left(-\frac{r_j^2}{c^2}\right), \quad c = 2.5 \hat{\sigma}
$$
4. Solve the weighted IEKF normal equations:
$$
(H^T W R^{-1} H + (P^-)^{-1}) \delta x = H^T W R^{-1} r + (P^-)^{-1} \delta x_{\text{pred}}
$$
where $W = \text{diag}(w_1, \ldots, w_N)$.

5. Update the linearization point and repeat.

The Welsch function provides **redescending** influence — unlike Huber, which still gives linear weight to large residuals, Welsch drives the weight to zero exponentially. This is critical when outlier correspondences produce very large residuals (common during map drift).

#### Step 2: Directional Robust Scale (Novel Contribution)

The standard MAD computes a single global scale $\hat{\sigma}$. However, during degeneracy, the residual distribution is **anisotropic**:
- Along well-conditioned directions, residuals are small (good correspondences match well)
- Along degenerate directions, residuals are large *even for correct correspondences* (because the LiDAR provides no constraint)

A global MAD is dominated by the degenerate-direction residuals, setting $\hat{\sigma}$ too high and failing to reject outliers along well-conditioned directions.

**Solution**: Project each residual onto the eigenvectors of the information matrix:

$$
r_j^{(d)} = (v_d^T J_j^T) \cdot r_j / \|v_d^T J_j^T\|
$$

where $v_d$ are the eigenvectors of $A_t$ or $A_r$, and $J_j$ is the $j$-th row of the pose Jacobian. Compute directional MAD:

$$
\hat{\sigma}_d = 1.4826 \cdot \text{median}_j(|r_j^{(d)}|)
$$

The per-point weight becomes:

$$
w_j = \prod_{d=1}^{3} \exp\left(-\frac{(r_j^{(d)})^2}{c_d^2}\right), \quad c_d = 2.5 \hat{\sigma}_d
$$

This ensures that outliers are detected relative to the residual distribution along *each* geometric direction independently. Along degenerate directions, $\hat{\sigma}_d$ will be large (correctly tolerating the spread), while along well-conditioned directions, $\hat{\sigma}_d$ is tight (correctly rejecting outliers).

### Integration into FAST-LIO2

- **Intervention point**: Within each IEKF iteration, after computing residuals and before solving the normal equations.
- **Computational overhead**: One MAD computation per direction (3 for translation, 3 for rotation) per IEKF iteration. MAD is $O(N \log N)$ (quickselect) with $N \sim 500$–$1000$ — sub-millisecond.
- **State vector**: Unchanged.
- **Compatibility**: Composes naturally with Method 1 (CEWS-KF) — the robust weights clean the residuals, and the continuous eigenvalue weighting handles the remaining degeneracy.
- **Replaces Gate 2**: The Welsch weighting subsumes FAST-LIO2's heuristic Gate 2 ($s > 0.9$). Points with large residuals are automatically down-weighted rather than hard-rejected.

### Why This Is Novel

- RELEAD: Huber M-estimation (non-redescending, weaker outlier rejection)
- Kinematic-Aware LIO-W: Welsch with global MAD, tied to wheel odometry
- D²-LIO: Binary accept/reject outlier gate, motion-aware but not robust-kernel-based
- **This method**: Welsch (redescending) + MAD (adaptive) + directional scale (anisotropic) + IEKF-native. The directional scale estimation is the key innovation.

---

## Method 4: Covariance-Stamped Map with Quality-Gated Insertion

**Addresses**: FM8 (Map-History Effects), FM4 (Dynamic Objects/Ghosting), coverage gaps "Map corruption detection" and "Map point aging/expiry"

### Motivation and Novelty

FM8 is the **most poorly covered** failure mode in the literature. The Cameroon dataset demonstrated that the *same physical trajectory* succeeds or fails depending on the map accumulated during earlier sections. No existing method provides online map quality tracking or corruption prevention in the ikd-Tree.

Line-LIO's quality-driven keyframe selection is the closest approach, but it operates at the *frame level* (accept/reject entire scans). This is too coarse: a single scan may contain points registered against well-conditioned map regions (good) and points registered against corrupted map regions (bad).

**Novelty**: Per-point map quality tracking in the ikd-Tree, with quality-weighted correspondences during the IEKF update and adaptive map insertion gating.

### Technical Description

#### Step 1: Covariance-Stamped Map Points

Extend each point in the ikd-Tree with metadata:

```
struct MapPoint {
    float x, y, z;           // Position (existing)
    float cov_trace;          // tr(P_pos) at insertion time
    uint32_t timestamp;       // Insertion time
    uint16_t match_count;     // Times used as successful correspondence
    uint8_t quality_flag;     // 0=suspect, 1=normal, 2=verified
};
```

At insertion time, each map point stores $\sigma_{\text{reg}}^2 = \text{tr}(P_{\text{pos}}(t_{\text{insert}}))$, the trace of the position covariance at the time the point was registered. Points registered during well-conditioned periods have small $\sigma_{\text{reg}}^2$; points registered during degeneracy or low-feature-count periods have large $\sigma_{\text{reg}}^2$.

#### Step 2: Quality-Weighted Measurement Noise

During kNN matching, the measurement noise for each correspondence incorporates the map point's registration quality:

$$
R_{jj} = R_{\text{base}} + \alpha_{\text{reg}} \cdot \sigma_{\text{reg},j}^2 + \alpha_{\text{age}} \cdot f_{\text{age}}(\Delta t_j) + \alpha_{\text{use}} / (1 + n_j)
$$

where:
- $R_{\text{base}}$: Base measurement noise (sensor model, potentially heteroscedastic per Method 5)
- $\sigma_{\text{reg},j}^2$: Registration covariance of the map point — points registered during uncertain periods get higher noise
- $f_{\text{age}}(\Delta t_j) = 1 - \exp(-\Delta t_j / \tau_{\text{age}})$: Temporal aging factor — old points that haven't been re-observed get higher noise, handling ghost points (FM4)
- $n_j$: Match count — frequently matched points are more trustworthy

The aging term provides a **soft temporal decay** for map points: recently inserted, frequently re-observed points are trusted; old or rarely-matched points are down-weighted. This is lighter-weight than explicit dynamic object detection (RF-LIO, TRLO) and handles any source of map staleness.

#### Step 3: Quality-Gated Map Insertion

Before inserting a scan into the ikd-Tree, compute a per-point insertion quality score:

$$
q_j = \underbrace{w(\lambda_{\min}^t) \cdot w(\lambda_{\min}^r)}_{\text{information quality}} \cdot \underbrace{\frac{N_{\text{inlier}}}{N_{\text{total}}}}_{\text{inlier ratio}} \cdot \underbrace{\exp(-\bar{r}^2 / \tau_r^2)}_{\text{residual quality}}
$$

where $w(\cdot)$ is the continuous weight from Method 1.

- If $q_j > q_{\text{high}}$: Insert with `quality_flag = verified`
- If $q_{\text{low}} < q_j < q_{\text{high}}$: Insert with `quality_flag = normal`
- If $q_j < q_{\text{low}}$: **Do not insert** into the map

This prevents scans registered during degeneracy or with high residuals from corrupting the map — directly breaking the FM8 failure mechanism.

#### Step 4: Match Count Update and Re-Verification

When a map point is used as a correspondence and the residual is below the Welsch-weighted threshold (Method 3), increment its `match_count`. Points with high match counts and low residuals can be upgraded from `normal` to `verified`. Points that consistently produce large residuals across multiple scans are downgraded to `suspect`.

### Integration into FAST-LIO2

- **ikd-Tree modification**: Add 8 bytes per map point (cov_trace: 4B, timestamp: 4B, match_count: 2B, quality_flag: 1B, padding: 1B). For a typical 500K-point map, this adds ~4 MB — negligible.
- **Measurement noise modification**: After kNN search, compute $R_{jj}$ using the map point metadata. This replaces FAST-LIO2's constant $R$ diagonal.
- **Insertion gating**: After the IEKF converges, compute $q_j$ for each point and conditionally insert. This adds a quality check before the existing `ikd_Tree.Add_Points()` call.
- **State vector**: Unchanged.

### Why This Is Novel

- Line-LIO: Frame-level quality gating (coarse). No per-point map quality.
- RF-LIO/TRLO: Dynamic object detection (specific to moving objects). No general map quality tracking.
- AS-LIO: Sliding window for map age (structural, not quality-based).
- **This method**: Per-point covariance stamping + temporal aging + match-count verification + continuous quality-gated insertion. Addresses FM4 and FM8 simultaneously with a unified mechanism.

---

## Method 5: Unified Heteroscedastic Measurement Noise with Deskew Uncertainty Propagation

**Addresses**: FM5 (Correspondence Errors), FM6 (Deskew Errors), FM2 (amplification in low-feature scenarios)

### Motivation and Novelty

FAST-LIO2 uses a **constant** measurement noise $R_{jj} = \sigma^2$ for all points. In reality, per-point uncertainty varies dramatically due to:
1. **Sensor noise**: Range-dependent ($\propto d^2$), incidence-angle-dependent ($\propto 1/\cos^2\alpha$), intensity-dependent (from LIO with Uncertainty IEKF)
2. **Deskew noise**: Motion-dependent, varies within a scan (from GP deskewing concepts)
3. **Map quality noise**: Registration-dependent (from Method 4)

Individual papers address individual sources, but **no existing method combines all three into a unified heteroscedastic model** with the deskew component being the most overlooked.

**Novelty**: A closed-form analytical model for deskew-induced point position uncertainty that doesn't require GP regression (lighter-weight than the GP approach), propagated into the IEKF measurement noise alongside sensor and map noise.

### Technical Description

#### Source 1: Sensor Noise Model (adapted from LIO with Uncertainty IEKF)

$$
\sigma_{\text{sensor},j}^2 = \sigma_0^2 \left(1 + \frac{d_j^2}{d_{\text{ref}}^2} + \frac{1}{\cos^2(\alpha_j)}\right)
$$

where $d_j$ is the range, $\alpha_j$ is the incidence angle (computed from the kNN-fitted plane normal and the ray direction), and $d_{\text{ref}}$ is a reference range (e.g., 10 m). The intensity term from the original paper is omitted for the Mid-360 unless intensity is calibrated.

#### Source 2: Analytical Deskew Uncertainty (Novel)

Instead of GP regression, derive the deskew-induced point uncertainty analytically from the IMU noise model.

The deskewed point position is:

$$
p_j^{\text{deskewed}} = R(\tau_j) p_j^{\text{raw}} + t(\tau_j)
$$

where $R(\tau_j)$ and $t(\tau_j)$ come from IMU integration from the scan start $t_0$ to the point's timestamp $\tau_j$. The uncertainty in $R(\tau_j)$ and $t(\tau_j)$ propagates from the IMU noise:

**Rotation uncertainty** (from gyroscope noise integration):

$$
\Sigma_R(\tau_j) = \sigma_g^2 \cdot (\tau_j - t_0) \cdot I_3
$$

**Translation uncertainty** (from accelerometer noise double-integration):

$$
\Sigma_t(\tau_j) = \sigma_a^2 \cdot \frac{(\tau_j - t_0)^3}{3} \cdot I_3
$$

The propagated point position uncertainty:

$$
\Sigma_{\text{deskew},j} = [p_j^{\text{raw}}]_\times^T \Sigma_R(\tau_j) [p_j^{\text{raw}}]_\times + \Sigma_t(\tau_j)
$$

The point-to-plane residual noise contribution from deskewing:

$$
\sigma_{\text{deskew},j}^2 = n_j^T \Sigma_{\text{deskew},j} \, n_j
$$

**Key insight**: The deskew uncertainty is:
- **Time-dependent**: Points later in the scan ($\tau_j$ far from $t_0$) have more uncertainty
- **Range-dependent**: Far points have more deskew uncertainty (the $[p_j]_\times$ term scales with range)
- **Motion-dependent**: Through $\sigma_g$ and $\sigma_a$, which should use the online-estimated values from Method 6 of papers_summary (Robust LIO Without Sensor-Specific Modeling)

#### Source 3: Map Quality Noise (from Method 4)

$$
\sigma_{\text{map},j}^2 = \alpha_{\text{reg}} \cdot \sigma_{\text{reg},j}^2 + \alpha_{\text{age}} \cdot f_{\text{age}}(\Delta t_j)
$$

#### Combined Heteroscedastic Noise

$$
R_{jj} = \sigma_{\text{sensor},j}^2 + \sigma_{\text{deskew},j}^2 + \sigma_{\text{map},j}^2
$$

Each point gets a unique noise value in the $R$ diagonal. No structural change to the IEKF is needed — only the $R$ matrix changes from constant to diagonal-varying.

### Integration into FAST-LIO2

- **Sensor noise**: After kNN plane fitting, compute incidence angle from the fitted normal and ray direction $p_j / \|p_j\|$. Use range from $\|p_j\|$.
- **Deskew noise**: From the point timestamp $\tau_j$ (provided by the Livox driver) and the current $\sigma_g$, $\sigma_a$ estimates (from online IMU noise estimation).
- **Map noise**: From the map point metadata (Method 4).
- **R construction**: Replace the constant diagonal $R = \sigma^2 I$ with $R = \text{diag}(R_{11}, R_{22}, \ldots, R_{NN})$.
- **Computational overhead**: Per-point: one incidence angle computation (dot product), one timestamp-based deskew noise computation (arithmetic), one map metadata lookup (already done during kNN). Total: negligible.

### Why This Is Novel

- LIO with Uncertainty IEKF: Sensor noise only (no deskew, no map).
- GP Deskewing: Deskew uncertainty only, via expensive GP regression.
- Method 4 (this document): Map noise only.
- **This method**: All three sources combined, with a novel *analytical* (closed-form) deskew uncertainty model that avoids the computational cost of GP regression while providing per-point deskew-aware noise.

---

## Method 6: Predictive Degeneracy Detection via Map Structure Pre-Query

**Addresses**: FM1 (Geometric Degeneracy — proactive), FM7 (IMU drift budget), FM8 (Map corruption prevention), coverage gap "Dead-reckoning budget estimation"

### Motivation and Novelty

All existing degeneracy detection methods are **reactive**: they detect degeneracy *after* the scan has been processed and the information matrix has been computed. By the time degeneracy is detected, the IEKF has already attempted (and possibly failed) a measurement update. Furthermore, no method estimates how long the IMU can sustain acceptable accuracy — the "dead-reckoning budget."

**Novelty**: Use the predicted next pose (from IMU propagation) and the existing map structure (ikd-Tree) to **predict the information matrix of the upcoming scan** before it arrives. This enables proactive countermeasures.

### Technical Description

#### Step 1: Predicted Pose

The IMU forward-propagated pose $\hat{T}_{k+1}^-$ is already computed by FAST-LIO2 as part of the IEKF prediction. This is the expected sensor pose at the next scan time.

#### Step 2: Map Pre-Query

Query the ikd-Tree around the predicted position $\hat{T}_{k+1}^-$ to retrieve the local map structure. Specifically:

1. Define a virtual scan FOV using the Livox Mid-360's known FoV (360° × 59°) centered at $\hat{T}_{k+1}^-$
2. Query the ikd-Tree for all map points within a radius $r_{\max}$ of the predicted position
3. For each retrieved map point $q_j$ with normal $n_j^{\text{map}}$, compute the predicted Jacobian:

$$
\hat{J}_j = n_j^T \frac{\partial (\hat{T}_{k+1}^- q_j)}{\partial \xi}
$$

4. Compute the predicted information matrix:

$$
\hat{A}_{k+1} = \sum_j \hat{J}_j^T R_0^{-1} \hat{J}_j
$$

5. Eigendecompose $\hat{A}_{k+1}$ (block-separated for translation/rotation) to predict which directions will be degenerate.

#### Step 3: Proactive Countermeasures

Based on the predicted degeneracy level, trigger preemptive actions *before the next scan*:

| Predicted Condition | Action |
|---|---|
| $\hat{\lambda}_{\min}^t > \lambda_{\text{safe}}$ | Normal operation |
| $\hat{\lambda}_{\min}^t < \lambda_{\text{safe}}$ (mild) | Tighten IMU process noise $Q$ (temporarily increase IMU trust while it's still accurate); begin buffering scans for potential sweep accumulation |
| $\hat{\lambda}_{\min}^t < \lambda_{\text{critical}}$ (severe) | Activate sweep accumulation (accumulate 2–3 scans with motion compensation for higher point density); suspend map insertion (prevent corruption); further tighten $Q$ |
| Duration of predicted degeneracy exceeds IMU budget | Alert: sustained degeneracy expected. Maximize IMU trust, minimize map insertion, consider stopping and waiting for operator action |

#### Step 4: Dead-Reckoning Budget Estimation

Estimate how long the IMU can maintain acceptable accuracy using the current process noise estimates:

$$
T_{\text{budget}} = \sqrt{\frac{\epsilon_{\text{pos}}}{\sigma_a^2 / 3}}
$$

where $\epsilon_{\text{pos}}$ is the acceptable position error (e.g., 0.1 m) and $\sigma_a^2$ is the estimated accelerometer noise (from online IMU noise estimation). For the ICM40609, this gives $T_{\text{budget}} \approx 2$–$4$ seconds.

If the predicted degeneracy duration exceeds $T_{\text{budget}}$, the system knows that IMU dead-reckoning alone cannot prevent drift and can activate more aggressive countermeasures (sweep accumulation, intensity features from Method 7 below).

#### Computational Cost

The map pre-query uses the same ikd-Tree kNN search infrastructure already in FAST-LIO2. The key difference is querying with the *predicted* pose rather than waiting for the scan. The predicted information matrix uses only the map normals (no scan needed).

Cost: One ikd-Tree range query + $N_{\text{map}}$ Jacobian computations + two $3 \times 3$ eigendecompositions. With $N_{\text{map}} \sim 500$, this takes $< 1$ ms — easily real-time at 10 Hz.

### Integration into FAST-LIO2

- **Timing**: Execute between scan $k$'s IEKF completion and scan $k+1$'s arrival (10 ms budget at 100 Hz IMU / 10 Hz LiDAR).
- **IMU process noise adjustment**: Modify $Q$ in the prediction step for the next scan. This is a single matrix modification.
- **Sweep accumulation**: Buffer raw scans and merge with motion compensation (using the SR-LIO++ approach) when triggered.
- **Map insertion gating**: Already implemented in Method 4; this method provides an additional trigger.
- **State vector**: Unchanged.

### Why This Is Novel

- D²-LIO, LODESTAR, Selective KF: All reactive (detect after scan processing).
- RELEAD: Environmental degradation detection (not geometric degeneracy prediction).
- CTE-MLO: Localizability-aware sampling (adjusts *within* a scan, not *before*).
- **This method**: Predicts degeneracy *before the scan arrives* using the map structure. Enables proactive countermeasures. Also provides a dead-reckoning budget estimate — a completely unaddressed gap.

---

## Method 7: Normal-Diversity-Aware Adaptive Downsampling for Solid-State LiDARs

**Addresses**: FM2 (Insufficient Features), FM9 (Solid-State LiDAR Challenges), FM1 (indirectly, by improving information matrix conditioning), coverage gap "Adaptive downsampling for non-uniform density"

### Motivation and Novelty

FAST-LIO2 uses a uniform **voxel grid filter** for downsampling. This is geometry-unaware: in regions of high point density (typical for parts of the Livox non-repetitive pattern), many points are discarded that could contribute diverse normals. In sparse regions, few points survive, even if they carry valuable geometric information. The result is a downsampled point cloud that may have poor normal diversity, worsening the information matrix conditioning.

Geometrically Stable Sampling (GSS) proposes a greedy algorithm to maximize $\lambda_{\min}(A)$, but its per-point eigendecomposition is computationally expensive and doesn't account for the Livox Mid-360's specific spatial density characteristics.

**Novelty**: A two-stage downsampling pipeline that (a) uses normal-direction binning for geometric diversity and (b) compensates for the Livox's non-uniform density with an adaptive voxel size, all at a fraction of the cost of greedy GSS.

### Technical Description

#### Stage 1: Normal-Direction Binning

After kNN plane fitting (which FAST-LIO2 already does for every point), partition the points by their surface normal direction:

1. Define $K$ normal bins using the faces of a regular icosahedron (20 bins, covering the sphere uniformly) or a simpler axis-aligned scheme (6 bins: $\pm x, \pm y, \pm z$)
2. Assign each point to its nearest bin based on its fitted normal $n_j$:
$$
b_j = \arg\min_{k} \angle(n_j, c_k)
$$
where $c_k$ is the $k$-th bin center direction.

3. Sample $N_{\text{target}} / K$ points from each non-empty bin, using random or farthest-point sampling within each bin.

This ensures the downsampled set has **balanced normal diversity** regardless of the spatial point density. In a corridor (where most normals point left/right/up/down), the along-corridor direction gets equal representation if *any* along-corridor normals exist, maximizing the information matrix rank.

#### Stage 2: Density-Adaptive Voxel Size

Within each normal bin, apply voxel downsampling with an **adaptive voxel size** that compensates for the Livox's non-uniform density:

$$
v_{\text{local}} = v_0 \cdot \sqrt{\frac{\rho_{\text{ref}}}{\rho_{\text{local}}}}
$$

where $v_0$ is the base voxel size (e.g., 0.5 m), $\rho_{\text{ref}}$ is a reference density, and $\rho_{\text{local}}$ is the local point density estimated from the kNN distances:

$$
\rho_{\text{local}}(p_j) \approx \frac{K}{\frac{4}{3}\pi d_K^3}
$$

where $d_K$ is the distance to the $K$-th nearest neighbor. In high-density regions, $v_{\text{local}}$ increases (more aggressive downsampling); in sparse regions, $v_{\text{local}}$ decreases (preserving scarce points).

#### Stage 3: Information-Aware Top-Up (Optional)

After stages 1–2, if the resulting downsampled set has fewer than $N_{\min}$ points (e.g., 200), add points from the most informative bin — the bin whose normal direction most improves $\lambda_{\min}$ of the current information matrix. This is a single-direction version of GSS's greedy algorithm, requiring only one eigendecomposition rather than $O(N)$.

### Integration into FAST-LIO2

- **Replaces**: FAST-LIO2's `PointCloudXYZI::Ptr feats_down_body` voxel grid filter.
- **Input**: The full feature point cloud after kNN plane fitting (normals are already computed).
- **Output**: A downsampled point cloud with balanced normal diversity and density-adaptive resolution.
- **Computational overhead**:
  - Normal binning: $O(N)$ bin assignments — negligible.
  - Density estimation: Uses existing kNN distances (already computed for plane fitting).
  - Adaptive voxel filter: Same cost as the existing voxel filter, applied per-bin.
  - Total: Comparable to the existing voxel filter.
- **State vector**: Unchanged.

### Why This Is Novel

- FAST-LIO2: Uniform voxel grid (geometry-unaware, density-unaware).
- Geometrically Stable Sampling: Greedy per-point optimization (expensive, per-frame eigendecompositions).
- Towards High-Performance SS-LIO: Density-aware downsampling but without normal diversity.
- CTE-MLO: Localizability-aware sampling but in an optimization framework (not IEKF).
- **This method**: Normal-binned + density-adaptive + information-aware top-up. Specifically designed for the Livox non-repetitive pattern where density varies spatially. Achieves most of GSS's conditioning benefit at a fraction of the cost.

---

## Summary: Proposed Method Interaction Map

The seven methods are designed to compose into a coherent improvement pipeline:

```
Raw Livox Scan
     │
     ▼
[Method 7] Normal-Diversity-Aware Downsampling
     │  Balanced normals, density-adaptive
     ▼
[FAST-LIO2] kNN Search + Plane/Line Fitting
     │
     ▼
[Method 5] Unified Heteroscedastic R
     │  Per-point noise: sensor + deskew + map
     ▼
[Method 3] Welsch IRLS within IEKF Iteration
     │  Outlier-robust residuals with directional scale
     ▼
[Method 1] CEWS-KF: Continuous Eigenvalue Weighting
     │  Smooth degeneracy attenuation
     │
     ├──▶ [Method 2] Cross-Covariance Attitude Protection
     │         Prevents FM1 → FM3 cascade
     ▼
Kalman Gain K → State Update → Covariance Update
     │
     ▼
[Method 4] Covariance-Stamped Map Insertion
     │  Quality-gated, with temporal aging
     ▼
[Method 6] Predictive Degeneracy Detection
     │  Proactive countermeasures for next scan
     ▼
Map (ikd-Tree) with per-point quality metadata
```

### Failure Mode Coverage

| Failure Mode | Primary Method(s) | Secondary Method(s) |
|---|---|---|
| FM1: Geometric Degeneracy | Method 1 (CEWS-KF) | Methods 6, 7 |
| FM2: Insufficient Features | Method 7 (Normal-Diversity Downsampling) | Method 5 |
| FM3: Attitude Corruption | Method 2 (Cross-Covariance Protection) | Method 1 |
| FM4: Dynamic Objects | Method 4 (Temporal Aging) | Method 3 |
| FM5: Correspondence Errors | Method 3 (Welsch IRLS) | Method 5 |
| FM6: Deskew Errors | Method 5 (Deskew Uncertainty) | — |
| FM7: IMU Quality | Method 6 (DR Budget) | Method 1 |
| FM8: Map Corruption | Method 4 (Quality-Gated Insertion) | Method 6 |
| FM9: Solid-State Challenges | Method 7 (Density-Adaptive Downsampling) | — |
| FM10: Scale Mismatch | Method 1 (Block-Separated Analysis) | — |

### Implementation Priority

| Priority | Method | Impact | Effort | Rationale |
|---|---|---|---|---|
| 1 | Method 1 (CEWS-KF) | Very High | Low | Core degeneracy handler; foundation for Methods 2, 4, 6. Two 3×3 eigendecompositions + weight computation. |
| 2 | Method 3 (Welsch IRLS) | High | Low-Medium | Replaces unweighted residuals with robust ones. Requires IRLS loop within existing IEKF iterations. |
| 3 | Method 4 (Covariance-Stamped Map) | High | Medium | Addresses the most poorly covered FM (FM8). Requires ikd-Tree metadata extension. |
| 4 | Method 5 (Heteroscedastic R) | Medium-High | Low | Per-point noise is straightforward to implement. Largest gain from the deskew uncertainty component. |
| 5 | Method 2 (Cross-Covariance Protection) | High (cascade-breaking) | Low | Simple covariance block modification. Depends on Method 1's eigenvalue analysis. |
| 6 | Method 7 (Normal-Diversity Downsampling) | Medium | Medium | Replaces voxel grid filter. Improves conditioning *before* the IEKF. |
| 7 | Method 6 (Predictive Degeneracy) | Medium-High | Medium-High | Most architecturally novel. Requires map pre-query infrastructure. |

### Publication Potential

Methods 1+2+3 together form a coherent contribution: **"Continuous Eigenvalue-Weighted IEKF with Cross-Covariance Protection and Directional Robust Estimation for Degeneracy-Resilient LiDAR-Inertial Odometry."** The novelty is in the continuous weighting (vs. binary), the cross-coupling protection (previously unaddressed), and the directional robust scale (vs. global). All three compose cleanly within FAST-LIO2's IEKF.

Methods 4+5 form a second contribution: **"Covariance-Aware Map Quality Tracking with Unified Heteroscedastic Noise for Long-Duration LiDAR-Inertial Odometry."** The novelty is in per-point map quality tracking and the analytical deskew uncertainty model.

Method 6 is potentially a standalone contribution: **"Predictive Degeneracy Detection for Proactive LiDAR-Inertial Odometry."** The concept of predicting degeneracy before the scan arrives is entirely novel.
