# AdaLIO: Robust Adaptive LiDAR-Inertial Odometry in Degenerate Indoor Environments

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM2 (Insufficient Features)

---

## 1. Problem Statement

Indoor environments (corridors, office rooms, staircases) frequently cause geometric degeneracy for LiDAR odometry due to repetitive structures and long featureless surfaces. AdaLIO proposes an **adaptive optimization framework** that adjusts per-direction optimization weights based on the information content of the current scan, preventing the optimizer from being driven by poorly-constrained directions.

---

## 2. Core Methodology

### 2.1 Per-Direction Information Analysis

AdaLIO analyzes the geometric information content of the LiDAR scan for each of the 6 DoFs independently. The information matrix:

$$
\mathcal{I} = J^T \Sigma^{-1} J
$$

is decomposed into its eigenstructure. Eigenvalues indicate information content per direction.

### 2.2 Adaptive Weighting

Instead of binary degeneracy detection, AdaLIO applies **continuous adaptive weights** to the optimization cost function:

$$
E(\xi) = \sum_{j=1}^{N} w_j \cdot r_j^2(\xi)
$$

where the weights $w_j$ are modulated based on the geometric distribution of the correspondences' normals relative to the degenerate/well-conditioned subspaces. Points whose normals contribute to well-conditioned directions receive higher weight; points contributing only to degenerate directions receive lower weight.

### 2.3 Indoor-Specific Adaptations

- **Corridor handling**: When forward translation degeneracy is detected (typical in corridors), AdaLIO reduces the influence of wall points (whose normals are perpendicular to the corridor axis) on the forward direction while maintaining their contribution to lateral and rotational constraints.
- **Room transitions**: At doorways, the system detects the rapid change in geometric structure and smoothly transitions weights.

---

## 3. Key Equations

### Information-weighted cost
$$
E_{\text{adaptive}}(\xi) = \sum_{j} w_j(\mathcal{I}) \cdot (n_j^T(T(\xi)p_j - q_j))^2
$$

### Weight computation
$$
w_j = f\left(\sum_{i=1}^{6} \frac{(n_j^T \frac{\partial Tp_j}{\partial \xi_i})^2}{\lambda_i + \epsilon}\right)
$$

where $f(\cdot)$ is a monotonically increasing function that assigns higher weight to correspondences contributing to well-conditioned directions.

---

## 4. Assumptions

1. **Indoor environments**: Designed and tested specifically for structured indoor spaces. Performance in unstructured outdoor environments is not validated.
2. **Factor-graph optimization**: Built on a factor-graph backend, not a Kalman filter.
3. **Sufficient point density**: Assumes enough points are available for meaningful information matrix computation.
4. **Known noise model**: Assumes the measurement noise covariance $\Sigma$ is accurately characterized.

---

## 5. Limitations

1. **Not IEKF-based**: AdaLIO uses optimization (factor graph), not an iterated extended Kalman filter. The adaptive weighting concept must be translated to modify the IEKF measurement noise covariance $R$ or equivalently scale the Jacobian rows.
2. **Indoor-specific tuning**: The weight functions and thresholds are tuned for indoor geometry patterns. Outdoor or mixed environments may require different parameterization.
3. **Computational overhead**: Per-point weight computation based on eigendecomposition adds overhead proportional to the number of correspondences.
4. **No explicit IMU adaptation**: The adaptive mechanism only affects the LiDAR cost; it doesn't jointly adapt IMU trust.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Conceptually applicable, requires translation to IEKF framework**:

- **Adaptive weighting → modified $R$ matrix**: In the IEKF, the equivalent of per-point weighting is modifying the diagonal of the measurement noise covariance $R$. Points with low information content get higher noise variance (lower weight):
  $$
  R_{jj}^{\text{adapted}} = \frac{R_{jj}}{w_j}
  $$

- **Integration point**: After constructing the Jacobian $H$ but before computing the Kalman gain $K = P^- H^T (H P^- H^T + R)^{-1}$, the adapted $R$ would change the effective Kalman gain.

- **Advantage over D²-LIO**: The continuous weighting avoids the binary degeneracy decision, providing smoother transitions and preserving partial information along mildly degenerate directions.

- **Challenge**: Computing per-point weights based on eigendecomposition at every IEKF iteration increases computational cost. However, since FAST-LIO2 already computes $H$, the eigendecomposition of $H^T H$ is a small $6 \times 6$ problem.

---

## 7. Applicability to Livox Mid-360

- The adaptive weighting is scan-pattern agnostic in principle.
- Indoor degeneracy patterns (corridors, rooms) are the same regardless of LiDAR type, so AdaLIO's indoor-specific adaptations remain relevant.
- The non-repetitive Livox pattern may actually benefit more from adaptive weighting because its per-scan coverage is more variable, creating more diverse degeneracy profiles.
- Point density variation in the Livox pattern means the information matrix eigenvalues will fluctuate more scan-to-scan, potentially causing the adaptive weights to be less stable.
