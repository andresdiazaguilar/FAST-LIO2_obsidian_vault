# Enhanced LiDAR-Inertial SLAM with Adaptive Intensity Feature Extraction

**Category**: Intensity features  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Problem Statement

Intensity-based features can supplement geometric features, but not all intensity features are equally reliable. This paper proposes **adaptive intensity feature extraction** that selects intensity features based on their reliability and informational content, preventing unreliable intensity features from degrading registration quality.

---

## 2. Core Methodology

### 2.1 Intensity Feature Reliability Assessment

The paper introduces a reliability score for each potential intensity feature based on:

1. **Intensity gradient magnitude**: Strong gradients → reliable features
   $$
   s_{\text{grad},j} = \|\nabla I(p_j)\|
   $$

2. **Gradient consistency**: Consistent gradient direction across the neighborhood → stable feature
   $$
   s_{\text{consist},j} = \frac{|\bar{\nabla I}|}{\overline{|\nabla I|}}
   $$
   where $\bar{\nabla I}$ is the mean gradient vector and $\overline{|\nabla I|}$ is the mean gradient magnitude. Ratio near 1 → consistent; near 0 → noisy.

3. **Range stability**: Features at close range are more reliable than distant features
   $$
   s_{\text{range},j} = \exp(-\beta \|p_j\|)
   $$

4. **Incidence angle**: Near-perpendicular incidence → more reliable intensity
   $$
   s_{\text{angle},j} = |n_j \cdot \hat{r}_j|
   $$
   where $\hat{r}_j$ is the unit ray direction.

### 2.2 Adaptive Selection

The overall feature reliability is:
$$
s_j = s_{\text{grad},j} \cdot s_{\text{consist},j} \cdot s_{\text{range},j} \cdot s_{\text{angle},j}
$$

Features with $s_j > \tau_s$ are selected. The threshold $\tau_s$ is **adaptive**: if too few features pass, the threshold is lowered; if the geometric information matrix is already well-conditioned, the threshold is raised (only adding high-quality intensity features).

### 2.3 Integration with SLAM

Selected intensity features are added to the registration cost alongside geometric features, similar to COIN-LIO and IGE-LIO. The key difference is the **quality-gated selection** that prevents poor intensity features from being used.

---

## 3. Key Equations

### Combined reliability score
$$
s_j = \prod_{k} s_{k,j}^{\alpha_k}
$$

where $\alpha_k$ are importance weights for each reliability criterion.

### Adaptive threshold
$$
\tau_s = \tau_0 \cdot \max\left(1, \frac{\lambda_{\min}(\mathcal{I}_{\text{geo}})}{\lambda_{\text{target}}}\right)
$$

When geometric features are already sufficient ($\lambda_{\min} > \lambda_{\text{target}}$), the intensity threshold increases, admitting only the best intensity features. When geometric features are insufficient, the threshold drops to include more intensity features.

---

## 4. Assumptions

1. **Intensity reliability can be assessed locally**: The reliability metrics use only local point neighborhood information.
2. **Reliable intensity features exist**: If the environment has no intensity variation, the adaptive selection correctly selects nothing (but provides no benefit).
3. **Intensity features complement geometric features**: The selected intensity features should provide constraints in directions not already covered by geometric features. The adaptive threshold partially ensures this by correlating selection with geometric information adequacy.

---

## 5. Limitations

1. **Heuristic reliability metrics**: The reliability criteria and their combination are heuristic. The relative importance weights $\alpha_k$ require tuning.
2. **Per-LiDAR calibration**: The range and angle reliability functions need sensor-specific calibration.
3. **No formal information-theoretic selection**: The selection maximizes per-feature reliability, not the information contribution to the registration. A point with moderate reliability but high information content (contributing to a degenerate direction) is more valuable than a highly reliable point contributing to an already well-conditioned direction.
4. **Computational overhead**: Computing all reliability metrics for every point adds processing time.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable as a feature selection layer**:

- The adaptive intensity feature extraction can serve as a preprocessing step before adding intensity measurements to the IEKF.
- The reliability score can inform the measurement noise covariance: $R_{jj}^{\text{intensity}} \propto 1/s_j^2$ (more reliable features get lower noise).
- The adaptive threshold tied to $\lambda_{\min}(\mathcal{I}_{\text{geo}})$ is directly computable from FAST-LIO2's measurement Jacobian.
- Integration with IGE-LIO's 3D gradient approach would combine quality-gated selection with scan-pattern-agnostic gradient computation.

---

## 7. Applicability to Livox Mid-360

- **Range and angle correction**: Particularly important for Livox, whose intensity characteristics may differ from spinning LiDARs.
- **Density-aware gradient computation**: The gradient consistency check ($s_{\text{consist}}$) helps filter out unreliable gradients from sparse Livox regions.
- **Adaptive threshold**: Beneficial because the Livox's per-scan geometric information varies more than spinning LiDARs, making adaptive intensity inclusion valuable.
