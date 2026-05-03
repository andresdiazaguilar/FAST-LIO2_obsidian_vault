# Revised Improvement Plan for FAST-LIO2 on Livox Mid-360

This document revises the original [[improvement_plan]] to address a critical omission: **intensity features**. The original plan excluded intensity-based methods (notably [[COIN-LIO]], [[IGE-LIO]], [[Intensity-Enhanced LIO Gradient Flow Sampling]], [[Enhanced LiDAR-inertial SLAM Adaptive Intensity]], and [[Intensity Enhanced Solid-State-LiDAR SLAM]]) citing Livox Mid-360 calibration requirements as the blocker. After reconsideration, this revised plan integrates intensity features as a new addition, positioned to complement — not compete with — the existing six additions.

---

## Why Intensity Features Were Originally Excluded — and Why That Was Wrong

The original [[improvement_plan]] listed [[IGE-LIO]] under "What Is NOT Included" with the justification:

> *"High potential but requires Mid-360 intensity calibration (range- and angle-dependent correction) as a prerequisite. Without calibration, spurious intensity gradients degrade rather than improve performance."*

This reasoning has three weaknesses:

### 1. Calibration Is Not a Fundamental Barrier — It Is an Engineering Task

Intensity calibration for range ($f(r) = (r/r_0)^2$) and incidence angle ($g(\alpha) = 1/\cos(\alpha)$) is well-understood and requires only a one-time characterization of the Mid-360's response curves. [[Intensity Enhanced Solid-State-LiDAR SLAM]] has already validated this for other Livox sensors (Avia, Horizon). The calibration:
- Can be performed offline with known-reflectivity targets
- Adds $<0.01$ ms per point at runtime (two scalar multiplies)
- Is a **fixed cost** that enables all intensity methods simultaneously

Dismissing an entire information channel because of a prerequisite calibration step is disproportionate when the alternative (geometric-only methods) fundamentally cannot resolve certain degeneracies.

### 2. Intensity Provides Information Orthogonal to Geometry

The core insight from [[COIN-LIO]] is that intensity gradients constrain motion along directions **independent of surface normals**. In a long corridor where all normals point left-right or up-down, geometric methods (including the proposed CEWS-KF) can only attenuate the degenerate along-corridor direction — they cannot resolve it. But if the corridor walls have different materials, paint, posters, or texture, intensity gradients along the corridor axis provide the missing constraint. This is not a marginal improvement: COIN-LIO reports **30–50% APE reduction** in geometrically degenerate environments.

No amount of clever Kalman gain manipulation (Addition 1) or noise modeling (Addition 2) can create information that isn't in the measurements. Intensity features add genuinely new information to $H$.

### 3. The Original Plan Left FM2 Only Moderately Covered

The failure mode coverage matrix showed FM2 (Insufficient Features) at "Good" — the weakest coverage among the primary failure modes. Addition 3 (normal-diversity downsampling) improves the *distribution* of existing geometric features but cannot increase the *dimensionality of constraints*. Intensity features directly add measurement rows to $H$ with Jacobians in new directions, closing this gap.

---

## How Intensity Features Integrate Into the Proposed Pipeline

The key architectural decision is: **which intensity method to use?**

| Method | Livox Compatibility | IEKF Compatibility | Calibration Req. | Computational Cost |
|---|---|---|---|---|
| [[COIN-LIO]] (spherical projection) | ✗ Poor — non-repetitive scan breaks pixel correspondences | ✗ Optimization-based | High | Moderate |
| [[IGE-LIO]] (3D intensity gradients) | ✓ Good — kNN-based, scan-agnostic | ✓ Good — adds rows to H | Moderate | Low (reuses kNN) |
| [[Intensity-Enhanced LIO Gradient Flow Sampling]] | ○ Partial — simplified version OK | ✓ Good — preprocessing step | Moderate | High (flow tracing) |
| [[Enhanced LiDAR-inertial SLAM Adaptive Intensity]] | ✓ Good — quality-gated selection | ✓ Good — feature selection layer | Moderate | Low |
| [[Intensity Enhanced Solid-State-LiDAR SLAM]] | ✓ Designed for SS-LiDAR | ○ Partial — 4D kNN changes ikd-Tree | High | Moderate |

**Selected approach**: **IGE-LIO's 3D intensity gradient residuals**, combined with the adaptive quality gating from [[Enhanced LiDAR-inertial SLAM Adaptive Intensity]]. This combination is optimal because:

1. **No spherical projection** — avoids COIN-LIO's critical Livox incompatibility
2. **Reuses existing kNN** — FAST-LIO2 already performs kNN search for plane fitting; the same neighbors provide intensity gradient estimation at near-zero additional search cost
3. **Adds rows to $H$ directly** — fully IEKF-native, no architectural change
4. **Quality gating prevents degradation** — unreliable intensity features (weak gradients, sparse neighborhoods) are filtered out, ensuring intensity only helps, never hurts
5. **Adaptive inclusion threshold** — intensity features are admitted more aggressively when geometric information is weak (the exact scenario where they are most needed)

---

## The Revised Plan: Seven Coordinated Improvements

The revised plan adds **Addition 7: Quality-Gated 3D Intensity Gradient Features** and reorders the pipeline to show where intensity enters.

```
Raw Livox Scan
     │
     ▼
[Addition 0] Intensity Calibration ←── offline prerequisite (one-time)
     │         ├── Range correction: I_cal = I_raw · (r/r_0)²
     │         └── Incidence angle correction: I_cal = I_cal / cos(α)
     │
     ▼
[Addition 3] Normal-Diversity-Aware Downsampling ←── replaces voxel grid
     │
     ▼
FAST-LIO2 kNN Search + Plane Fitting (unchanged)
     │
     ├──────────────────────────────────────────┐
     ▼                                          ▼
[Addition 2] Unified Heteroscedastic R    [Addition 7] Quality-Gated 3D
     │                                    Intensity Gradient Features
     │                                          │
     │         ├── Intensity gradient from kNN   │
     │         ├── Quality gating (reliability)  │
     │         ├── Adaptive threshold (↓ when    │
     │         │   geometric info is weak)       │
     │         └── Adds rows to H and R          │
     │                                          │
     ├──────────────────────────────────────────┘
     ▼
[Addition 1] CEWS-KF + Cross-Covariance Protection + Welsch IRLS
     │        ├── Continuous degeneracy attenuation
     │        ├── Cross-covariance decoupling
     │        └── Directional robust residuals
     │              (now operates on geometric + intensity residuals)
     ▼
State Update → Covariance Update
     │
     ▼
[Addition 4] Quality-Gated Map Insertion ←── gates ikd-Tree insertion
     │         └── Now stores calibrated intensity per map point
     ▼
[Addition 5] Online IMU Noise Estimation ←── modifies process noise Q
     │
     ▼
[Addition 6] Ground Plane Pseudo-Measurements ←── adds rows to H
```

---

## Addition 7: Quality-Gated 3D Intensity Gradient Features

**Failure modes addressed**: FM2 (Insufficient Features — primary), FM1 (Geometric Degeneracy — provides information in degenerate directions), FM9 (SS-LiDAR challenges — adds constraints beyond sparse geometric features)

**Source**: [[IGE-LIO]] (3D intensity gradients) + [[Enhanced LiDAR-inertial SLAM Adaptive Intensity]] (quality-gated selection). Adapted for IEKF and Livox Mid-360.

**Impact**: ★★★★ — Provides genuinely new measurement dimensions that are orthogonal to surface normals. The impact is highest in exactly the scenarios where FAST-LIO2 fails: geometrically degenerate environments (corridors, tunnels) where different materials or surface textures provide intensity contrast that geometry alone cannot exploit. COIN-LIO demonstrated 30–50% APE reduction in such scenarios; we expect similar gains from the 3D gradient variant.

**Difficulty**: Medium — Requires intensity calibration as a prerequisite, extends the IEKF measurement model with new rows, and requires storing calibrated intensity in the ikd-Tree map.

### Prerequisites

#### Intensity Calibration (One-Time, Offline)

Before any intensity feature can be used, the Mid-360's raw intensity must be corrected for range and incidence angle effects:

$$I_{\text{cal}}(p_j) = I_{\text{raw}}(p_j) \cdot \underbrace{\left(\frac{r_j}{r_0}\right)^2}_{f(r_j)} \cdot \underbrace{\frac{1}{\cos(\alpha_j)}}_{g(\alpha_j)}$$

where:
- $r_j = \|p_j\|$ is the range
- $\alpha_j = \arccos(|n_j \cdot \hat{r}_j|)$ is the incidence angle (requires the plane normal, available after kNN fitting)
- $r_0 \approx 1$ m is a reference range

**Calibration procedure**: Place known-reflectivity targets (e.g., Spectralon panels) at varying ranges and angles, record intensity, and fit $f(r)$ and $g(\alpha)$. This is a one-time procedure per sensor unit.

**Runtime cost**: Two scalar multiplies per point — negligible.

**Note**: The incidence angle correction $g(\alpha)$ uses the same plane normal already computed for point-to-plane residuals. If the normal is unavailable (failed plane fit), the point is excluded from intensity features regardless.

### What It Does

**Pipeline intervention**: After kNN search and plane fitting (same as Addition 2), before the IEKF measurement update. Adds rows to $H$, $z$, and $R$.

#### Step 1: 3D Intensity Gradient Estimation

For each downsampled point $p_j$ with calibrated intensity $I_j$ and kNN neighbors $\mathcal{N}(j)$ (already computed for plane fitting), estimate the local 3D intensity gradient:

$$\nabla I(p_j) = (P_j^T W_j P_j)^{-1} P_j^T W_j \delta I_j$$

where:
- $P_j = [p_{k_1} - p_j, \dots, p_{k_K} - p_j]^T \in \mathbb{R}^{K \times 3}$: neighbor position differences
- $\delta I_j = [I_{k_1} - I_j, \dots, I_{k_K} - I_j]^T \in \mathbb{R}^K$: neighbor intensity differences
- $W_j = \text{diag}(\exp(-\|p_{k_i} - p_j\|^2 / 2h^2))$: distance-based weights

This is a weighted least-squares fit of the intensity field in the 3D neighborhood. The gradient $\nabla I \in \mathbb{R}^3$ points in the direction of steepest intensity increase on the local surface.

**Reuse of kNN**: The same $K$ neighbors from FAST-LIO2's plane fitting are used. No additional kNN search is required. The only new computation is retrieving the neighbors' intensity values (which must be stored in the ikd-Tree — see map augmentation below) and solving the $3 \times 3$ linear system.

#### Step 2: Quality-Gated Selection

Not all intensity gradients are useful. Compute a per-point reliability score (adapted from [[Enhanced LiDAR-inertial SLAM Adaptive Intensity]]):

$$s_j = \underbrace{\min\left(1, \frac{\|\nabla I(p_j)\|}{\tau_{\text{grad}}}\right)}_{s_{\text{grad}}} \cdot \underbrace{\frac{|\bar{\nabla I}_j|}{\overline{|\nabla I|}_j}}_{s_{\text{consist}}} \cdot \underbrace{\exp(-\beta_r \|p_j\|)}_{s_{\text{range}}} \cdot \underbrace{|n_j \cdot \hat{r}_j|}_{s_{\text{angle}}}$$

where:
- $s_{\text{grad}}$: Gradient magnitude — weak gradients ($\|\nabla I\| < \tau_{\text{grad}}$) indicate no intensity variation; unreliable
- $s_{\text{consist}}$: Gradient consistency — ratio of mean gradient vector magnitude to mean gradient magnitude across neighbors; near 1 = consistent direction, near 0 = noisy
- $s_{\text{range}}$: Range penalty — intensity features are less reliable at long range (higher noise, sparser neighbors)
- $s_{\text{angle}}$: Incidence angle — near-perpendicular incidence produces more reliable intensity; oblique views are noisy

**Adaptive threshold** (key design choice):

$$\tau_s = \tau_0 \cdot \max\left(1, \frac{\lambda_{\min}(\mathcal{I}_{\text{geo}})}{\lambda_{\text{target}}}\right)$$

When the geometric information matrix is well-conditioned ($\lambda_{\min}$ large), the threshold is high — only the highest-quality intensity features are admitted. When geometric information is weak ($\lambda_{\min}$ small), the threshold drops, admitting more intensity features to compensate. This prevents intensity features from degrading well-conditioned scans while maximizing their benefit during degeneracy.

Points with $s_j < \tau_s$ are excluded from intensity measurements.

#### Step 3: Intensity Residual and Jacobian

For each selected intensity point, the scalar intensity residual is:

$$r_{\text{I},j} = I_{\text{scan}}^{\text{cal}}(p_j) - I_{\text{map}}^{\text{cal}}(q_j)$$

where $q_j$ is the nearest map point to $T p_j$ (from the same kNN search used for geometric matching).

The Jacobian of this residual w.r.t. pose $\xi = [t^T, \theta^T]^T$:

$$J_{\text{I},j} = \nabla I_{\text{map}}(q_j)^T \frac{\partial (T p_j)}{\partial \xi} = \nabla I_{\text{map}}^T \begin{bmatrix} I_{3\times3} & -[R p_j]_\times \end{bmatrix}$$

**Critical observation**: The direction of this Jacobian is determined by $\nabla I_{\text{map}}$, which is generally **not aligned with the surface normal** $n_j$. This means the intensity residual constrains motion in directions that the point-to-plane residual cannot — precisely resolving degeneracy when geometric normals are all (near-)parallel.

#### Step 4: Augmented Measurement Model

The intensity residuals and Jacobians are appended to the geometric ones:

$$H_{\text{aug}} = \begin{bmatrix} H_{\text{geo}} \\ H_{\text{I}} \end{bmatrix}, \quad z_{\text{aug}} = \begin{bmatrix} z_{\text{geo}} \\ z_{\text{I}} \end{bmatrix}, \quad R_{\text{aug}} = \begin{bmatrix} R_{\text{geo}} & 0 \\ 0 & R_{\text{I}} \end{bmatrix}$$

where $R_{\text{I}} = \text{diag}(\sigma_{\text{I},j}^2)$ with per-point intensity noise:

$$\sigma_{\text{I},j}^2 = \frac{\sigma_{I,0}^2}{s_j^2}$$

Lower reliability → higher noise → less influence on the state update. This is more principled than hard gating alone: borderline features contribute proportionally to their quality.

### Interaction with Other Additions

#### With Addition 1 (CEWS-KF):

The eigenvalue analysis in CEWS-KF now operates on the **combined** information matrix $\mathcal{I}_{\text{total}} = \mathcal{I}_{\text{geo}} + \mathcal{I}_{\text{I}}$. In a corridor scenario:
- Without intensity: $\lambda_{\min}^t \approx 0$ (along-corridor direction degenerate) → CEWS-KF attenuates the update in that direction (correct but loses information)
- With intensity: intensity gradients from different wall materials add information along the corridor axis → $\lambda_{\min}^t$ increases → CEWS-KF applies a larger weight → the system actually *resolves* the degeneracy rather than merely surviving it

This is the fundamental difference: **Addition 1 is a defense mechanism (attenuate what you can't observe); Addition 7 is an offensive mechanism (observe what you previously couldn't)**. They compose naturally.

#### With Addition 2 (Heteroscedastic R):

The intensity measurement noise $\sigma_{\text{I},j}^2$ integrates into the same heteroscedastic $R$ framework. The intensity noise model accounts for:
- Calibration residual uncertainty (systematic)
- Intensity sensor noise (from Mid-360 characterization)
- Reliability-weighted quality ($1/s_j^2$)

#### With Addition 3 (Normal-Diversity Downsampling):

The downsampling now serves double duty: ensuring normal diversity for geometric constraints **and** preserving intensity-rich points. The information-aware top-up (Stage 3 of Addition 3) can use the combined information matrix — if a direction is weak in geometry but strong in intensity, the top-up correctly avoids wasting points on that direction.

A natural extension: during the normal-diversity binning (Stage 1), add an **intensity gradient diversity** criterion. Points with strong intensity gradients in directions perpendicular to the dominant surface normal are prioritized, as they provide the most complementary information.

#### With Addition 4 (Quality-Gated Map):

The ikd-Tree must store **calibrated intensity** per map point (one additional float per point, ~4 bytes → ~2 MB for 500K-point map). This is a minor extension to the metadata already added by Addition 4. The intensity map is updated with an exponential moving average when the same region is re-observed:

$$I_{\text{map}}^{\text{new}}(v) = \alpha_I \cdot I_{\text{scan}}^{\text{cal}} + (1 - \alpha_I) \cdot I_{\text{map}}^{\text{old}}(v)$$

This handles gradual intensity drift and averages out per-scan noise.

Map quality gating (Addition 4) now also considers intensity: points inserted with high intensity noise or from calibration-suspect ranges receive lower map intensity confidence.

### Adaptation for IEKF and Livox Mid-360

- **IEKF compatibility**: Fully native. Intensity residuals are standard IEKF measurements — additional rows in $H$ with scalar residuals and known noise. No change to the IEKF iteration structure. The intensity Jacobians are re-linearized at each IEKF iteration alongside the geometric ones.
- **Livox compatibility**: The 3D gradient approach is entirely scan-pattern agnostic. No spherical projection, no pixel-level correspondences, no assumptions about scan regularity. The kNN-based gradient estimation works on any point cloud topology.
- **Non-repetitive scan pattern advantage**: The Livox's non-repetitive pattern actually *helps* intensity mapping: successive scans cover different angular regions, progressively building a denser intensity map. After a few scans, the map has near-uniform intensity coverage even though individual scans are non-uniform.

### When Intensity Features Provide No Benefit

Intensity features are not a universal solution. They provide minimal benefit when:
- The environment has **uniform materials** (bare concrete tunnel, white-painted corridor) — no intensity variation to exploit
- The Mid-360's intensity is **saturated or clipped** — some highly reflective surfaces (retroreflective tape, wet metal) produce saturated values with no usable gradient
- The environment is **outdoor natural terrain** with uniform vegetation — similar intensity everywhere

The adaptive threshold mechanism handles all these cases gracefully: when no reliable intensity features pass the quality gate, the system reverts to geometry-only operation (equivalent to the original 6-addition plan). **Intensity features are purely additive — they can only help or have no effect, never degrade.**

### Computational Cost

| Component | Cost | Notes |
|---|---|---|
| Intensity calibration | $< 0.01$ ms | 2 scalar multiplies per point |
| 3D gradient estimation | $< 0.3$ ms | $3 \times 3$ solve per point, reuses kNN |
| Reliability scoring | $< 0.1$ ms | Scalar operations per point |
| Intensity Jacobian | $< 0.05$ ms | Simple matrix row per selected point |
| ikd-Tree intensity storage | — | 4 bytes per map point (memory only) |
| **Total** | $< 0.5$ ms | Per scan, well within real-time budget |

---

## Additions 1–6: Unchanged

The six additions from the original [[improvement_plan]] remain **exactly as described**, with two minor modifications to account for intensity:

### Modification to Addition 1 (CEWS-KF)

The block-separated information matrices now include intensity contributions:

$$A_t = H_t^T R^{-1} H_t = \underbrace{H_{t,\text{geo}}^T R_{\text{geo}}^{-1} H_{t,\text{geo}}}_{\text{geometric}} + \underbrace{H_{t,\text{I}}^T R_{\text{I}}^{-1} H_{t,\text{I}}}_{\text{intensity}}$$

This means the eigenvalue analysis in CEWS-KF automatically benefits from intensity information: directions that were degenerate in geometry-only may become well-conditioned when intensity contributions are included. The continuous weighting function $w(\lambda_i)$ naturally adapts — no changes to the CEWS-KF equations or logic.

### Modification to Addition 4 (Quality-Gated Map)

The ikd-Tree point structure adds one field:

| Field | Size | Description |
|---|---|---|
| `intensity_cal` | 4 B | Calibrated intensity value |

Total additional memory: ~2 MB for a 500K-point map.

---

## Revised Failure Mode Coverage Matrix

| Failure Mode | Primary Addition | Secondary Addition | Coverage | Change |
|---|---|---|---|---|
| FM1: Geometric Degeneracy | Addition 1 (CEWS-KF) | **Addition 7 (intensity)** + Addition 3 | █████████ Excellent | ↑ from Strong |
| FM2: Insufficient Features | **Addition 7 (intensity)** | Addition 3 (normal diversity) | ████████ Strong | ↑ from Good |
| FM3: Attitude Corruption | Addition 1 (cross-covariance) | Addition 6 (ground constraint) | ████████ Strong | — |
| FM4: Dynamic Objects | Addition 4 (temporal aging) | Addition 1 (Welsch IRLS) | █████░░░ Moderate | — |
| FM5: Correspondence Errors | Addition 1 (Welsch IRLS) | Addition 2 (heteroscedastic R) | ████████ Strong | — |
| FM6: Deskew Errors | Addition 2 (deskew noise in R) | Addition 5 (online σ_g, σ_a) | ██████░░ Good | — |
| FM7: IMU Quality | Addition 5 (online noise est.) | Addition 1 (degeneracy atten.) | ██████░░ Good | — |
| FM8: Map Corruption | Addition 4 (quality-gated) | — | ██████░░ Good | — |
| FM9: SS-LiDAR Challenges | Addition 3 (density-adaptive) | **Addition 7 (intensity)** | ██████░░ Good | ↑ from Moderate |
| FM10: Scale Mismatch | Addition 1 (block-separated) | — | ████████ Strong | — |

Key improvements:
- **FM1 coverage upgraded from Strong to Excellent**: Intensity features can *resolve* degeneracy (add information), not just *survive* it (attenuate the update). This is a qualitative improvement.
- **FM2 coverage upgraded from Good to Strong**: Intensity features add genuinely new measurement dimensions — the most direct possible response to "insufficient features."
- **FM9 coverage upgraded from Moderate to Good**: Intensity features help compensate for the geometric sparsity inherent in solid-state scan patterns.

---

## Revised Implementation Order

### Phase 1: Core Robustification (Additions 1 + 2) — unchanged

**Effort**: ~2–3 weeks.

**Expected outcome**: Resolution of catastrophic divergence in Cameroon, Valis, and Senegal.

### Phase 2: Map Quality (Addition 4) — unchanged

**Effort**: ~1–2 weeks. Now includes storing calibrated intensity per map point (trivial extension).

### Phase 3: Preprocessing + IMU (Additions 3 + 5) — unchanged

**Effort**: ~2 weeks (parallelizable).

### Phase 3.5: Intensity Calibration (Addition 7 prerequisite)

**Effort**: ~1 week (can be done in parallel with Phase 3).

**Tasks**:
1. Characterize Mid-360 intensity-range curve with known-reflectivity targets at 0.5–50 m
2. Characterize intensity-incidence angle curve at 0°–80° incidence
3. Fit correction functions $f(r)$ and $g(\alpha)$
4. Validate: corrected intensity should be approximately constant for the same surface at different ranges/angles

This is a one-time experimental task. Once done, it enables all intensity methods for any future work with the Mid-360.

### Phase 4: Intensity Features (Addition 7)

**Effort**: ~2 weeks.

**Tasks**:
1. Extend ikd-Tree to store calibrated intensity per point
2. Implement 3D intensity gradient estimation (reusing kNN neighbors)
3. Implement quality-gated selection with adaptive threshold
4. Add intensity rows to $H$, $z$, $R$ in the IEKF measurement update
5. Validate on corridor/tunnel datasets (Senegal, Valis) where intensity benefit is expected to be highest

**Expected outcome**: Measurable APE reduction in geometrically degenerate sequences beyond what Additions 1–6 alone achieve. The most dramatic improvement is expected in scenarios where Addition 1's CEWS-KF is attenuating the update (indicating degeneracy) but intensity gradients are available to fill the gap.

**Test**: Compare Addition 1–6 system vs. Addition 1–7 system on:
- Senegal (corridor — expected intensity diversity from different wall materials)
- Valis (tunnel — may have limited intensity diversity; test whether natural rock provides gradients)
- Cameroon (mixed — some sections with geometric degeneracy near textured surfaces)
- A controlled corridor experiment with placed intensity targets to isolate the intensity contribution

### Phase 5: Attitude Anchoring (Addition 6) — unchanged

**Effort**: ~1 week.

---

## Revised Ranking Summary

| Rank | Addition | Impact | Difficulty | Failure Modes |
|---|---|---|---|---|
| 1 | **Addition 1**: CEWS-KF + Cross-Cov + Welsch | ★★★★★ | Low-Medium | FM1, FM3, FM5, FM10 |
| 2 | **Addition 2**: Heteroscedastic R | ★★★★ | Low | FM5, FM6, FM2 |
| 3 | **Addition 7**: Quality-Gated Intensity Gradients | ★★★★ | Medium | FM2, FM1, FM9 |
| 4 | **Addition 4**: Quality-Gated Map | ★★★★ | Medium | FM8, FM4 |
| 5 | **Addition 5**: Online IMU Noise | ★★★☆ | Low | FM7, FM6 |
| 6 | **Addition 3**: Normal-Diversity Downsampling | ★★★☆ | Medium | FM2, FM9, FM1 |
| 7 | **Addition 6**: Ground Constraint | ★★★☆ | Low-Medium | FM3 |

Addition 7 ranks 3rd — above map quality gating — because it addresses the *qualitative* limitation of the original plan: the inability to add new information in degenerate directions. Additions 1–2 are defensive (handle degeneracy gracefully); Addition 7 is offensive (reduce or eliminate degeneracy). Together, they form the strongest possible response to FM1 and FM2.

---

## Updated "What Is NOT Included" Table

| Method | Why Excluded |
|---|---|
| [[COIN-LIO]] spherical projection | Replaced by IGE-LIO's 3D gradient approach (Addition 7). The spherical projection is fundamentally incompatible with the Livox Mid-360's non-repetitive scan pattern — the same angular region is covered by different points in successive scans, making pixel-level photometric correspondences unreliable. The 3D gradient approach achieves the same goal (intensity-based constraints in degenerate directions) without scan-pattern dependency. |
| [[Intensity-Enhanced LIO Gradient Flow Sampling]] full flow tracing | The full gradient flow ODE integration is computationally expensive and sensitive to intensity noise. The simplified concept (gradient-magnitude-weighted sampling) is partially captured by Addition 7's quality gating, which preferentially selects points with strong, reliable intensity gradients. |
| [[Intensity Enhanced Solid-State-LiDAR SLAM]] 4D kNN | Modifying the ikd-Tree distance metric from 3D to 4D (xyz + intensity) is a deeper architectural change than adding intensity rows to $H$. The 4D approach improves correspondence quality but conflicts with the "no structural redesign" principle. The 3D kNN + separate intensity residual approach achieves most of the benefit. |
| [[LODESTAR]] Schmidt-Kalman | Redundant with Addition 1's CEWS-KF (continuous > binary). |
| [[I2EKF-LO]] dual-iteration EKF | Conflicts with Addition 1's within-iteration Welsch IRLS. |
| [[RF-LIO]] / [[TRLO]] dynamic detection | Addition 4's temporal aging is lighter-weight and scan-pattern-agnostic. |
| [[SR-LIO++]] sweep reconstruction | Changes scan-level interface, complicating all other additions. |
| [[AC-LIO]] adaptive deskewing | Addition 2's analytical model is lighter-weight. |

---

## Revised Publication Potential

The seven additions now support a stronger publication narrative with two complementary thrusts:

**Thrust 1 (Defensive)**: Continuous eigenvalue-weighted filtering with cross-covariance protection — handling degeneracy when it occurs.

**Thrust 2 (Offensive)**: Quality-gated intensity gradient features — reducing or eliminating degeneracy by adding orthogonal measurement dimensions.

**Updated title direction**: *"Beyond Geometric Constraints: Degeneracy-Resilient LiDAR-Inertial Odometry via Continuous Eigenvalue-Weighted Filtering and Quality-Gated Intensity Gradient Augmentation"*

**Additional novel contribution**: The combination of adaptive intensity inclusion (threshold tied to $\lambda_{\min}(\mathcal{I}_{\text{geo}})$) with continuous eigenvalue weighting creates a system that **automatically transitions** between three regimes:
1. **Well-conditioned**: Few intensity features admitted (high threshold), full geometric update (all weights ≈ 1) — minimal overhead
2. **Mildly degenerate**: More intensity features admitted (lower threshold), partial geometric attenuation — intensity fills the gap
3. **Severely degenerate, no intensity contrast**: Many intensity features attempted but few pass quality gate, strong geometric attenuation — graceful degradation to IMU-reliant operation

This adaptive regime-switching is emergent from the interaction of Additions 1 and 7 — no explicit mode detection or switching logic is needed. This is a novel system-level contribution.

**Experimental evaluation** now includes an ablation that isolates the intensity contribution:
- System A: Additions 1–6 (geometry-only, defensive)
- System B: Additions 1–7 (geometry + intensity, defensive + offensive)
- COIN-LIO (intensity, but with spherical projection adapted as well as possible for Mid-360)
- FAST-LIO2 baseline
- FAST-LIO2 + D²-LIO

The A-vs-B comparison directly quantifies the value of intensity features within the proposed framework.
