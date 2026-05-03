# GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM5 (Correspondence Errors)

---

## 1. Problem Statement

Traditional ICP and its variants (point-to-plane, GICP) treat all correspondences equally or use hand-crafted heuristics for weighting. In degenerate or cluttered environments, many correspondences are uninformative or harmful. GenZ-ICP learns to **adaptively weight** correspondences based on their geometric informativeness, achieving degeneracy robustness and outlier resilience simultaneously.

---

## 2. Core Methodology

### 2.1 Adaptive Per-Point Weighting

GenZ-ICP assigns each correspondence a weight $w_j \in [0, 1]$ based on its expected contribution to the registration quality:

$$
E(\xi) = \sum_{j=1}^{N} w_j \cdot d(T(\xi) p_j, q_j)^2
$$

where $d(\cdot, \cdot)$ is the distance metric (point-to-plane or point-to-point).

### 2.2 Weight Prediction Network

The weights are predicted by a lightweight neural network that takes local geometric features as input:

$$
w_j = f_\theta(\phi(p_j), \phi(q_j), n_j, d_j)
$$

where:
- $\phi(p_j)$: Local geometric descriptor of the source point (e.g., surface variation, planarity, linearity from PCA)
- $\phi(q_j)$: Same for the target point
- $n_j$: Surface normal at the target
- $d_j$: Initial correspondence distance

The network learns to:
- **Down-weight outlier correspondences** (incorrect nearest-neighbor matches)
- **Down-weight degenerate correspondences** (points contributing only to already well-conditioned directions)
- **Up-weight informative correspondences** (points constraining poorly-conditioned directions)

### 2.3 Training Strategy

The network is trained on diverse environments with known ground truth poses:
- **Loss function**: Combination of registration accuracy and conditioning:
  $$
  \mathcal{L} = \|T_{\text{pred}} - T_{\text{GT}}\|^2 + \beta \cdot \frac{\lambda_{\max}(\mathcal{I}_w)}{\lambda_{\min}(\mathcal{I}_w) + \epsilon}
  $$
  where $\mathcal{I}_w = \sum w_j J_j^T J_j$ is the weighted information matrix.
- The conditioning penalty encourages the network to distribute weights such that the weighted information matrix is well-conditioned.

### 2.4 Generalizability

The key claim is that the learned weights **generalize** across environments because they operate on local geometric features rather than global scene properties. A weight function trained on indoor data can work outdoors, and vice versa.

---

## 3. Key Equations

### Weighted information matrix
$$
\mathcal{I}_w = \sum_{j=1}^{N} w_j J_j^T R_j^{-1} J_j
$$

### Weight-modulated cost function
$$
E(\xi) = \sum_{j=1}^{N} w_j (n_j^T (T(\xi) p_j - q_j))^2
$$

### Condition number regularization
$$
\kappa(\mathcal{I}_w) = \frac{\lambda_{\max}(\mathcal{I}_w)}{\lambda_{\min}(\mathcal{I}_w)}
$$

The network is trained to minimize $\kappa(\mathcal{I}_w)$ jointly with registration error.

---

## 4. Assumptions

1. **Training data availability**: Requires diverse training environments with ground truth poses.
2. **Local geometric features are sufficient**: The weight function depends only on local geometry, assuming global context (e.g., scene type, degeneracy direction) can be inferred from local features.
3. **Correspondence quality**: The nearest-neighbor correspondences are the input; the weights can reject bad ones but cannot create better correspondences.
4. **Network generalization**: The trained network must generalize to unseen environments and sensor configurations.

---

## 5. Limitations

1. **Learning-based component**: Requires training data and neural network inference at runtime. Even a lightweight network adds latency.
2. **Generalization risk**: Despite claims of generalizability, the weight network may not perform optimally on environments very different from training data (e.g., underground mines, dense vegetation).
3. **Not filter-based**: GenZ-ICP is a standalone odometry system (optimization-based), not designed for IEKF integration.
4. **Solid-state LiDAR training**: If trained only on spinning LiDAR data, the geometric features may not capture the irregular density patterns of solid-state LiDARs, leading to suboptimal weights.
5. **Inference overhead**: Even a lightweight network adds ~1-2ms per scan, which may be significant in tight real-time budgets.
6. **No uncertainty quantification**: The weights are deterministic; there's no measure of confidence in the weight predictions.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Partially applicable — the concept is valuable but requires adaptation**:

- **Adaptive weighting → per-point $R$ modification**: In the IEKF, per-point weights translate to modifying the diagonal elements of $R$:
  $$
  R_{jj}^{\text{adapted}} = \frac{R_{jj}}{w_j}
  $$
  Low weight → high noise variance → reduced influence on the Kalman gain.

- **Alternative: learned gating function**: Instead of continuous weights, the network could provide a binary accept/reject decision, replacing FAST-LIO2's heuristic Gate 2 with a learned gating function.

- **Feature computation**: The local geometric descriptors (planarity, linearity, surface variation) can be computed from the ikd-Tree's kNN results, which are already available in FAST-LIO2.

- **Key challenge**: Integrating neural network inference into FAST-LIO2's tight computational loop. FAST-LIO2 processes scans in ~10ms; adding network inference must fit within this budget.

---

## 7. Applicability to Livox Mid-360

- **Training on Livox data needed**: The geometric features (point density, neighborhood shape) differ between spinning and solid-state LiDARs. Training or fine-tuning on Livox data is necessary.
- **Irregular density handling**: The weight network could potentially learn to handle the non-uniform point density of the Livox pattern, down-weighting sparse-region correspondences where plane fitting is unreliable.
- **Potential benefit**: The adaptive weighting could be particularly beneficial for Livox because the standard fixed-threshold gating (Gate 2) is poorly suited to the spatially varying density.
