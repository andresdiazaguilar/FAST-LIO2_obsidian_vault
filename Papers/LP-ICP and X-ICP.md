# LP-ICP / X-ICP: Localizability-Aware Point Cloud Registration for Robust Localization in Extreme Environments

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy)

---

## 1. Problem Statement

Point cloud registration in extreme environments (lava tubes, ice caves, subterranean passages — as encountered in JPL's EELS robot mission) faces severe geometric degeneracy. LP-ICP and X-ICP (extended version) propose **localizability-aware registration** that quantifies how well each direction is constrained and restricts the registration solution to well-conditioned directions only.

---

## 2. Core Methodology

### 2.1 Localizability Metric

The paper introduces a per-direction **localizability score** based on the Fisher Information Matrix (FIM):

$$
\mathcal{F} = J^T \Sigma^{-1} J \in \mathbb{R}^{6 \times 6}
$$

Eigendecomposition: $\mathcal{F} = U \Lambda U^T$ with eigenvalues $\lambda_1 \leq \dots \leq \lambda_6$.

The **localizability** along direction $u_i$ is simply $\lambda_i$. The overall localizability is characterized by the vector:

$$
\ell = [\lambda_1, \lambda_2, \dots, \lambda_6]
$$

### 2.2 Localizability-Constrained ICP

LP-ICP modifies the ICP cost function to only optimize along well-conditioned directions:

$$
\min_\xi \sum_{j} r_j^2(\xi) \quad \text{s.t.} \quad \xi \in \text{span}(U_{\text{good}})
$$

where $U_{\text{good}} = [u_{k+1}, \dots, u_6]$ contains the eigenvectors with eigenvalues above a threshold.

Implementation via subspace projection:

$$
\xi^* = U_{\text{good}} U_{\text{good}}^T \xi_{\text{unconstrained}}
$$

### 2.3 X-ICP: Extended Version

X-ICP extends LP-ICP with:

1. **Continuous localizability weighting**: Instead of binary selection, each direction gets a weight proportional to its localizability:
   $$
   w_i = \frac{\lambda_i}{\lambda_{\max}} \quad \text{(soft version)}
   $$

2. **Localizability-aware correspondence selection**: Points that contribute to poorly-conditioned directions are given lower weight in correspondence matching, not just in the final optimization.

3. **Multi-scale localizability**: Computes localizability at multiple spatial scales (local neighborhoods of varying sizes) to capture both local and global geometric structure.

### 2.4 Integration with Navigation Stack

For the EELS robot, the localizability metric is also used for:
- **Path planning**: Preferring paths with higher expected localizability
- **Uncertainty propagation**: The localizability metric feeds into the state estimator's covariance as a prior on registration quality

---

## 3. Key Equations

### Fisher Information Matrix
$$
\mathcal{F} = \sum_{j=1}^{N} \frac{1}{\sigma_j^2} J_j^T J_j
$$

where $J_j = n_j^T \frac{\partial(T(\xi) p_j)}{\partial \xi} \in \mathbb{R}^{1 \times 6}$.

### Subspace-constrained solution
$$
\xi^* = \underset{\xi \in \mathcal{S}}{\arg\min} \sum_j r_j^2(\xi), \quad \mathcal{S} = \text{span}\{u_i : \lambda_i > \lambda_{\text{thresh}}\}
$$

### Localizability-weighted cost
$$
E(\xi) = \xi^T U \text{diag}(w_1, \dots, w_6) U^T \xi + \sum_j r_j^2(\xi)
$$

---

## 4. Assumptions

1. **FIM accurately represents localizability**: The linearized FIM is a local measure; it may not capture nonlinear degeneracy.
2. **Sufficient point density for FIM estimation**: Sparse scans may produce unreliable FIM estimates.
3. **ICP convergence**: The subspace constraint must not prevent ICP from converging (initial alignment must be good enough).
4. **Static environment**: No dynamic objects.

---

## 5. Limitations

1. **ICP-based, not IEKF**: The constrained ICP framework operates on pose optimization, not within a Kalman filter update.
2. **Binary threshold (LP-ICP)**: The original LP-ICP uses a binary threshold for direction selection. X-ICP's soft weighting partially addresses this.
3. **Computational cost of multi-scale analysis (X-ICP)**: Computing localizability at multiple scales increases overhead.
4. **Environment-specific thresholds**: The localizability threshold needs environment-specific tuning — what's "degenerate" in a cave is different from a corridor.
5. **No IMU integration**: Purely LiDAR-based; doesn't consider how IMU prior affects the effective localizability.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Conceptually applicable, requires translation**:

- **FIM computation**: Directly available from FAST-LIO2's $H$ and $R$ matrices.
- **Subspace constraint → Kalman gain modification**: Same translation as other degeneracy-aware methods:
  $$
  K_{\text{constrained}} = U_{\text{good}} U_{\text{good}}^T K
  $$

- **X-ICP's continuous weighting**: More relevant for IEKF than LP-ICP's binary selection. The continuous weights can be incorporated via a modified measurement noise covariance or via a state-space regularization term.

- **Localizability as a covariance prior**: The localizability metric can inform the IEKF's measurement noise covariance — lower localizability directions get higher noise variance:
  $$
  R_{\text{eff}} = R + \alpha U_{\text{poor}} \Lambda_{\text{poor}}^{-1} U_{\text{poor}}^T
  $$

- **Path planning integration**: While not directly part of FAST-LIO2, the localizability metric could inform a higher-level autonomy stack about areas where odometry quality will degrade.

---

## 7. Applicability to Livox Mid-360

- **FIM-based localizability**: Fully applicable and scan-pattern agnostic.
- **Extreme environment relevance**: Designed for environments more extreme than typical FAST-LIO2 use cases, but the methods are relevant for any degenerate environment.
- **X-ICP's correspondence selection**: Could help with the Livox's spatially varying density by preferring correspondences in denser (more reliable) regions.
- **Multi-scale analysis**: The varying point density of the Livox pattern makes multi-scale localizability analysis more informative — some scales may be well-conditioned while others are not.
