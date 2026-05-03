# RELEAD: Resilient Localization with Enhanced LiDAR Odometry in Adverse Environments

**Authors**: Chen et al., 2024, ICRA  
**Category**: Degeneracy-aware (suggested paper)  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM5 (Correspondence Errors)

---

## 1. Problem Statement

FAST-LIO2's ESIKF (error-state iterated Kalman filter) applies the full Kalman update regardless of the registration quality. In adverse environments (rain, fog, dust, geometric degeneracy), this leads to corrupted state estimates. RELEAD proposes **constrained ESIKF updates** that incorporate explicit outlier handling and degeneracy-aware constraints directly within the FAST-LIO2 architecture.

---

## 2. Core Methodology

### 2.1 Constrained State Update

RELEAD modifies the IEKF update to include **inequality constraints** that prevent large, potentially erroneous updates:

$$
\delta x^* = \arg\min_{\delta x} \|z - h(\hat{x}^- \boxplus \delta x)\|_R^2 + \|\delta x\|_{P^{-1}}^2 \quad \text{s.t.} \quad \|\delta x\| \leq \delta_{\max}
$$

The constraint $\|\delta x\| \leq \delta_{\max}$ limits the maximum state update magnitude, preventing the filter from making large jumps driven by outlier residuals.

### 2.2 Robust Front-End

RELEAD enhances the front-end (correspondence search and residual computation) with:

1. **Adaptive correspondence threshold**: Based on the current velocity and uncertainty:
   $$
   \tau_{\text{corr}} = \tau_0 + \beta \|v\| \Delta t + \gamma \sqrt{\text{tr}(P_{\text{pos}})}
   $$
   Faster motion or higher uncertainty → larger threshold (accepting more correspondences).

2. **Residual-based outlier rejection**: After the first IEKF iteration, points with residuals exceeding $3\sigma$ are removed:
   $$
   \text{outlier}(j) = |r_j| > 3 \sqrt{H_j P^- H_j^T + R_{jj}}
   $$

3. **Correspondence quality weighting**: Each correspondence gets a weight based on its reliability:
   $$
   w_j = \text{Huber}(r_j / \sigma_j)
   $$
   where the Huber function down-weights large residuals without completely rejecting them.

### 2.3 Degradation-Aware Processing

RELEAD detects environmental degradation (rain, fog) through:
- **Return intensity statistics**: Rain/fog reduce average intensity
- **Range distribution**: Fog creates a cluster of short-range returns
- **Point density monitoring**: Degradation reduces effective point count

When degradation is detected, the system:
1. Increases measurement noise $R$ (less trust in LiDAR)
2. Tightens the state update constraint $\delta_{\max}$
3. Adjusts the correspondence threshold

---

## 3. Key Equations

### Constrained IEKF update (QP formulation)
$$
\delta x^* = \arg\min_{\delta x} \frac{1}{2} \delta x^T (H^T R^{-1} H + (P^-)^{-1}) \delta x - (H^T R^{-1} r + (P^-)^{-1} \delta x_{\text{pred}})^T \delta x
$$
$$
\text{s.t.} \quad -\delta_{\max} \leq \delta x_i \leq \delta_{\max}, \quad i = 1, \dots, n
$$

### Huber-weighted residual
$$
\rho_H(r) = \begin{cases}
\frac{1}{2} r^2 & |r| \leq c \\
c(|r| - \frac{c}{2}) & |r| > c
\end{cases}
$$

The IRLS (iteratively reweighted least squares) weight:
$$
w_j = \begin{cases}
1 & |r_j/\sigma_j| \leq c \\
c / |r_j/\sigma_j| & |r_j/\sigma_j| > c
\end{cases}
$$

### State update bound
$$
\delta_{\max} = f(\text{degradation\_level}, \|v\|, \text{dt})
$$

---

## 4. Assumptions

1. **Degradation is detectable**: The front-end degradation detection assumes specific patterns (rain, fog) that affect intensity and range statistics. Novel degradation modes may not be detected.
2. **Residuals follow a known distribution**: The $3\sigma$ outlier rejection assumes Gaussian residuals. Heavy-tailed errors (common with incorrect correspondences) require robust alternatives.
3. **State update bound is appropriate**: The bound $\delta_{\max}$ must be set large enough to accept valid large updates (aggressive motion) but small enough to reject erroneous ones. This is context-dependent.

---

## 5. Limitations

1. **Constrained QP overhead**: Solving a constrained quadratic program is more expensive than the unconstrained Kalman update. For FAST-LIO2's 24-dimensional state, this is manageable but adds ~1-2ms per iteration.
2. **Bound setting**: The state update bound $\delta_{\max}$ is critical and difficult to set. Too tight → rejects valid updates during aggressive motion; too loose → doesn't prevent corruption.
3. **Environmental degradation focus**: Primarily designed for adverse weather (rain, fog, dust). Geometric degeneracy detection is secondary.
4. **No directional degeneracy handling**: The state update bound is isotropic — it limits the total update magnitude, not per-direction. A degenerate direction might receive a large (wrong) update that's masked by a small total update.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable** — RELEAD explicitly extends FAST-LIO2:

- **Same ESIKF/IEKF framework**: RELEAD builds on FAST-LIO2's ESIKF, making integration straightforward.
- **Robust front-end**: The Huber-weighted residuals and $3\sigma$ outlier rejection are directly integrable into FAST-LIO2's measurement update.
- **Constrained update**: Replacing the unconstrained Kalman update with a constrained QP adds robustness with moderate computational cost.
- **Applicable to any degradation**: While designed for weather, the robust front-end also handles geometric degeneracy (which produces large residuals along degenerate directions).

**Most valuable components for FAST-LIO2**:
1. Huber-weighted IEKF update (replaces unweighted least squares)
2. Residual-based outlier rejection after first IEKF iteration
3. Adaptive correspondence threshold

---

## 7. Applicability to Livox Mid-360

- **Adaptive correspondence threshold**: Valuable for the Mid-360's varying point density, where fixed thresholds may be too tight in sparse regions or too loose in dense regions.
- **Robust residuals**: The Huber function protects against the outlier correspondences that arise from the Livox's non-uniform spatial coverage.
- **Environmental degradation detection**: Useful if the system operates in adverse outdoor conditions.
- **Point density monitoring**: The Mid-360's non-repetitive pattern causes natural variation in effective point count per scan. RELEAD's density monitoring should be adapted to distinguish scan-pattern variation from actual degradation.
