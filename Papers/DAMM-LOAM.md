# DAMM-LOAM: Degeneracy-Aware Multi-Metric LiDAR Odometry and Mapping

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM2 (Insufficient Features)

---

## 1. Problem Statement

Single-metric LiDAR registration (e.g., point-to-plane alone) degenerates when the geometric distribution of correspondences cannot constrain all DoFs under that metric. DAMM-LOAM proposes using **multiple registration metrics simultaneously** — point-to-plane, point-to-point, and Generalized ICP (GICP) — and switching or blending between them based on the current degeneracy state. Different metrics have different degeneracy profiles, so combining them provides complementary constraints.

---

## 2. Core Methodology

### 2.1 Multi-Metric Registration

Three distance metrics are used simultaneously:

**Point-to-Plane**:
$$
r_{\text{p2pl},j} = n_j^T (T p_j - q_j)
$$

Constrains motion along surface normals. Degenerates when normals are co-planar (e.g., corridor).

**Point-to-Point**:
$$
r_{\text{p2pt},j} = \|T p_j - q_j\|
$$

Constrains motion in all directions equally per correspondence, but is sensitive to noise and local minima.

**Generalized ICP (GICP)**:
$$
r_{\text{GICP},j} = (T p_j - q_j)^T (C_j^q + T C_j^p T^T)^{-1} (T p_j - q_j)
$$

where $C_j^p, C_j^q$ are the local covariance matrices of the point neighborhoods. GICP adapts the metric based on local surface shape — it behaves like point-to-plane for well-defined surfaces and point-to-point for isotropic neighborhoods.

### 2.2 Degeneracy-Aware Metric Selection

DAMM-LOAM computes the information matrix for each metric independently:

$$
\mathcal{I}_m = J_m^T W_m J_m, \quad m \in \{\text{p2pl, p2pt, GICP}\}
$$

The metric with the best-conditioned information matrix (lowest condition number or highest minimum eigenvalue) is selected or given higher weight in the combined cost:

$$
E_{\text{total}} = \alpha_1 E_{\text{p2pl}} + \alpha_2 E_{\text{p2pt}} + \alpha_3 E_{\text{GICP}}
$$

where $\alpha_m$ are set inversely proportional to the condition number of $\mathcal{I}_m$.

### 2.3 LOAM Feature Extraction

The system uses LOAM-style scan-line feature extraction to identify:
- **Edge features**: High curvature points → used for point-to-point
- **Planar features**: Low curvature points → used for point-to-plane
- **General features**: All points → used for GICP

---

## 3. Key Equations

### Combined cost with adaptive weights
$$
E(\xi) = \sum_m \frac{\lambda_{\min}(\mathcal{I}_m)}{\sum_{m'} \lambda_{\min}(\mathcal{I}_{m'})} \sum_{j \in \mathcal{C}_m} r_{m,j}^2(\xi)
$$

### Degeneracy indicator per metric
$$
d_m = \frac{\lambda_{\max}(\mathcal{I}_m)}{\lambda_{\min}(\mathcal{I}_m) + \epsilon}
$$

Low $d_m$ → well-conditioned → trust this metric more.

---

## 4. Assumptions

1. **Scan-line structure**: LOAM feature extraction requires ordered scan lines with smooth curvature profiles. This is fundamental to the edge/plane separation.
2. **Sufficient points per feature type**: Each metric needs enough correspondences for meaningful information matrix computation.
3. **Correct correspondences per metric**: Nearest-neighbor correspondences are assumed correct for each metric independently.

---

## 5. Limitations

1. **LOAM-based pipeline**: Requires scan-line structure for feature extraction. The Livox Mid-360 does **not** provide ordered scan lines, making LOAM feature extraction directly incompatible.
2. **Not IEKF-based**: Uses Levenberg-Marquardt optimization, not a Kalman filter. Translation to IEKF requires significant adaptation.
3. **Point-to-point metric is noisy**: While it provides complementary constraints, point-to-point registration is significantly noisier than point-to-plane and can degrade overall accuracy when given too much weight.
4. **GICP computational overhead**: Computing local covariance matrices $C_j^p, C_j^q$ for all points is expensive.
5. **Metric weight oscillation**: The adaptive weights can oscillate rapidly in transitional environments, causing unstable behavior.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Partially applicable — the multi-metric concept is valuable, but implementation requires significant changes**:

- **Multi-metric residuals in IEKF**: FAST-LIO2 currently uses only point-to-plane residuals. Adding point-to-point or GICP residuals would require:
  1. Extending the measurement model $h(x)$ to include multiple residual types
  2. Stacking the Jacobians: $H = [H_{\text{p2pl}}^T, H_{\text{p2pt}}^T, H_{\text{GICP}}^T]^T$
  3. Setting appropriate per-metric noise covariances in $R$

- **Adaptive metric weighting in IEKF**: The weighting translates to modifying the measurement noise covariance: metrics with better conditioning get lower noise variance.

- **GICP adaptation**: GICP's per-point covariance model could provide per-correspondence measurement noise estimates, improving the $R$ matrix accuracy in the IEKF.

- **No scan-line requirement for point-to-point**: Point-to-point residuals don't require feature extraction and could supplement point-to-plane in FAST-LIO2 without LOAM features.

---

## 7. Applicability to Livox Mid-360

**Limited due to LOAM dependency**:

- The scan-line feature extraction is **incompatible** with the Livox non-repetitive scan pattern. The `line` field (0–3) in Livox messages is not a scan line.
- However, the **multi-metric concept** is independent of feature extraction. One could:
  - Keep FAST-LIO2's kNN-based plane fitting for point-to-plane
  - Add point-to-point residuals for kNN matches where plane fitting fails
  - Use GICP with local covariance estimation from the ikd-Tree
- The GICP metric is particularly attractive for Livox because it naturally adapts to the local point distribution without requiring scan lines.
