# D²-LIO: Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM5 (Correspondence Errors), FM10 (Scale Mismatch)

---

## 1. Problem Statement

Standard IEKF-based LiDAR-inertial odometry (specifically FAST-LIO2) treats all directions equally during the Kalman update. When the point-to-plane residuals fail to constrain certain directions (geometric degeneracy), the filter still applies the full Kalman gain, causing unconstrained directions to be corrupted by noise-amplified updates.

D²-LIO identifies that degeneracy is **directional** — some DoFs may be well-constrained while others are not — and proposes directional regularization to attenuate the update only along degenerate directions while preserving well-conditioned updates.

---

## 2. Core Methodology

### 2.1 Directional Degeneracy Detection

The key innovation is performing eigenvalue analysis on the **pose information matrix** $A$, but **separately** for translation and rotation blocks:

$$
A = H_{\text{pose}}^T R^{-1} H_{\text{pose}} \in \mathbb{R}^{6 \times 6}
$$

where $H_{\text{pose}}$ is the measurement Jacobian w.r.t. pose states and $R$ is the measurement noise covariance.

**Block separation**: Rather than analyzing the full $6 \times 6$ matrix (which suffers from FM10: scale mismatch between translation and rotation Jacobian columns), D²-LIO splits:

$$
A_t = H_t^T R^{-1} H_t \in \mathbb{R}^{3 \times 3}, \quad A_r = H_r^T R^{-1} H_r \in \mathbb{R}^{3 \times 3}
$$

where $H_t$ and $H_r$ are the first 3 and last 3 columns of $H_{\text{pose}}$, respectively.

Eigendecomposition of each block:

$$
A_t = V_t \Lambda_t V_t^T, \quad A_r = V_r \Lambda_r V_r^T
$$

yields eigenvalues $\lambda_i^t$ and $\lambda_i^r$ with corresponding eigenvectors $v_i^t$ and $v_i^r$. A direction $v_i$ is considered **degenerate** if:

$$
\lambda_i < \lambda_{\text{thresh}}
$$

where $\lambda_{\text{thresh}}$ is a fixed threshold (set empirically; typical values ~10–100 for translation, different for rotation).

### 2.2 Directional Regularization

When degenerate eigenvectors are detected, D²-LIO modifies the Kalman gain to attenuate updates along those directions. The approach constructs a **regularization matrix**:

$$
W = I - \sum_{i \in \mathcal{D}} v_i v_i^T
$$

where $\mathcal{D}$ is the set of degenerate eigenvector indices. This projection matrix zeros out the state update component along degenerate directions:

$$
\delta x_{\text{regularized}} = W \cdot \delta x_{\text{IEKF}}
$$

In practice, the regularization is applied to the translation and rotation components independently using their respective degenerate eigenvectors.

### 2.3 Motion-Aware Adaptive Outlier Filter

D²-LIO also introduces a motion-aware point-to-plane residual gating mechanism that replaces FAST-LIO2's static Gate 2:

$$
\tau_j = \alpha \cdot (\|v\| \cdot \Delta t + \beta)
$$

where $v$ is the estimated velocity, $\Delta t$ is the inter-scan time, and $\alpha, \beta$ are tuning parameters. Points with residuals exceeding $\tau_j$ are rejected. This is **velocity-dependent**: during fast motion, the threshold loosens (accepting larger residuals from valid correspondences that moved further), and during slow motion, it tightens (rejecting outliers more aggressively).

---

## 3. Key Equations

### Information matrix block decomposition
$$
A_t = \sum_{j=1}^{N} \frac{1}{\sigma_j^2} (J_t^{(j)})^T J_t^{(j)}, \quad A_r = \sum_{j=1}^{N} \frac{1}{\sigma_j^2} (J_r^{(j)})^T J_r^{(j)}
$$

where $J_t^{(j)} = n_j^T \frac{\partial (R p_j + t)}{\partial t} = n_j^T$ and $J_r^{(j)} = n_j^T \frac{\partial (R p_j + t)}{\partial R}$.

### Regularized state update
$$
\delta x_{\text{pose}} = \begin{bmatrix} W_t & 0 \\ 0 & W_r \end{bmatrix} K (z - h(\hat{x}))
$$

where $K$ is the standard IEKF Kalman gain.

---

## 4. Assumptions

1. **Binary degeneracy detection**: A direction is either degenerate or not (no intermediate weighting). The threshold $\lambda_{\text{thresh}}$ is fixed.
2. **Independent translation/rotation degeneracy**: The block-separation assumption ignores cross-coupling between translation and rotation through the off-diagonal blocks of the full $6 \times 6$ information matrix.
3. **Eigenvector stability**: Assumes the degenerate eigenvectors are stable across IEKF iterations within a single scan. In practice, they can shift as the linearization point changes.
4. **Static threshold**: The threshold $\lambda_{\text{thresh}}$ does not adapt to environment complexity, scan density, or sensor characteristics.

---

## 5. Limitations

1. **Complete information discard along degenerate directions**: Rather than down-weighting, the regularization projects out the entire LiDAR contribution along degenerate eigenvectors. This is overly aggressive during **mild degeneracy** where the LiDAR still provides partial information.
2. **Threshold sensitivity**: Performance depends critically on $\lambda_{\text{thresh}}$. Too low → misses degeneracy; too high → over-attenuates, losing useful information.
3. **No smooth transition**: The switch between regularized and non-regularized updates is abrupt, potentially causing discontinuities in the state trajectory.
4. **Cross-coupling ignored**: Translation and rotation blocks are analyzed independently. In practice, a translation degeneracy in one direction can induce apparent rotation errors (and vice versa) through the coupled residual structure.
5. **The outlier filter is still heuristic**: While motion-aware, it lacks a principled statistical framework (e.g., robust kernels or M-estimation).

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable** — D²-LIO was explicitly developed and tested on the FAST-LIO2 codebase.

- The eigenvalue decomposition of the information matrix is computed from the same $H$ matrix already available in FAST-LIO2's IEKF.
- The regularization modifies the Kalman gain, which is the natural intervention point in the IEKF pipeline.
- Computational overhead is minimal: two $3 \times 3$ eigendecompositions per scan.
- Tested on real datasets with Livox-class LiDARs.

**Key integration point**: After the IEKF constructs $H$ and $R$, but before computing the Kalman gain. The regularization matrix $W$ modifies the effective Kalman gain.

---

## 7. Applicability to Livox Mid-360

- The block-separated eigenvalue analysis is **sensor-agnostic** — it operates on the information matrix regardless of scan pattern.
- The motion-aware outlier filter is also scan-pattern agnostic.
- However, the **eigenvalue threshold** may need re-tuning for the Mid-360's specific point density and spatial distribution, since the Livox non-repetitive scan pattern produces different eigenvalue profiles than spinning LiDARs.
- The non-repetitive scan pattern means eigenvalue magnitudes may be systematically lower (fewer points per scan covering each direction), requiring lower thresholds.

---

## 8. Results Highlights

- Tested on the Cameroon dataset: with D²-LIO's adaptive outlier filter, the drift was noticeably slower and minimum eigenvalues stayed higher around the failure region.
- The regularization prevented unconstrained updates during corridor/tunnel degeneracy, delaying or preventing catastrophic drift.
- Improvement was measurable but not always sufficient to prevent failure entirely in severe degeneracy cases.
