# Principled ICP Covariance Modelling in Perceptually Degraded Environments for the EELS Mission Concept

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy)

---

## 1. Problem Statement

When ICP is used as a measurement source in a state estimator (e.g., EKF, factor graph), the registration covariance must be accurately estimated for the filter to correctly weight the ICP output against other sensors. Standard covariance approximations (Hessian inverse) are inaccurate in perceptually degraded environments because they don't account for the anisotropic uncertainty caused by geometric degeneracy. This paper proposes **principled covariance modelling** that correctly reflects directional uncertainty.

---

## 2. Core Methodology

### 2.1 Standard Covariance Approximation (and Its Failures)

The standard ICP covariance estimate uses the inverse Hessian:

$$
\Sigma_{\text{ICP}} = \sigma^2 (J^T J)^{-1}
$$

This is the Cramér-Rao lower bound under Gaussian noise. **Problem**: In degenerate environments, $J^T J$ is ill-conditioned, and $(J^T J)^{-1}$ has huge eigenvalues along degenerate directions. While this correctly indicates high uncertainty, the specific covariance values are unreliable because:
- The linearization assumption breaks down along high-uncertainty directions
- The noise model ($\sigma^2$ isotropic) doesn't capture the true error distribution
- Correspondences along degenerate directions may be systematically wrong, not just noisy

### 2.2 Censi's Covariance

The paper builds on Censi's closed-form ICP covariance:

$$
\Sigma_{\text{Censi}} = (J^T J)^{-1} J^T \Sigma_{\text{data}} J (J^T J)^{-1}
$$

where $\Sigma_{\text{data}}$ accounts for point-level noise, including the uncertainty of normal estimation. This is a better approximation but still relies on the Hessian inverse, which is numerically unstable in degenerate cases.

### 2.3 Principled Covariance Model

The proposed approach:

1. **Regularized Hessian**: Replace $(J^T J)^{-1}$ with a regularized inverse:
   $$
   (J^T J + \gamma I)^{-1}
   $$
   where $\gamma$ is a regularization parameter that prevents the covariance from exploding along degenerate directions.

2. **Degeneracy-aware scaling**: Scale the covariance along degenerate directions using a prior-informed model:
   $$
   \Sigma_{\text{principled}} = V \text{diag}\left(\min\left(\frac{\sigma^2}{\lambda_i}, \sigma_{\max}^2\right)\right) V^T
   $$
   where $\sigma_{\max}^2$ is a maximum allowed variance (from the prior or IMU prediction uncertainty).

3. **Monte Carlo validation**: The paper validates the covariance estimates using Monte Carlo simulations with known ground truth, showing that the principled model produces calibrated uncertainty estimates.

### 2.4 Integration with Navigation Filter

The corrected ICP covariance is fed into the EELS robot's navigation filter as the measurement noise covariance:

$$
R_{\text{ICP}} = \Sigma_{\text{principled}}
$$

This ensures the filter correctly up-weights IMU-based predictions along degenerate ICP directions.

---

## 3. Key Equations

### Censi's closed-form covariance
$$
\Sigma = A^{-1} B A^{-T}
$$

where:
- $A = J^T J$ (Hessian)
- $B = J^T \Sigma_{\text{noise}} J$ (noise-weighted information)

### Regularized covariance
$$
\Sigma_{\text{reg}} = (A + \gamma I)^{-1} B (A + \gamma I)^{-T}
$$

### Degeneracy-capped covariance
$$
\sigma_i^2 = \min\left(\frac{\text{tr}(B_i)}{(\lambda_i + \gamma)^2}, \sigma_{\text{prior},i}^2\right)
$$

---

## 4. Assumptions

1. **Gaussian noise model**: Point-level noise is Gaussian (potentially heteroscedastic via $\Sigma_{\text{data}}$).
2. **Correct correspondences on average**: While individual correspondences may be noisy, the overall matching is assumed correct in expectation.
3. **Regularization parameter selection**: $\gamma$ must be chosen appropriately — too small gives unstable covariance, too large gives overconfident covariance.
4. **Prior availability**: The capped covariance requires a prior estimate of maximum allowable uncertainty.

---

## 5. Limitations

1. **Covariance modelling, not registration improvement**: This paper doesn't improve the registration itself — it only provides better uncertainty estimates for downstream fusion.
2. **ICP-based pipeline**: Designed for ICP output fed into a navigation filter, not for the IEKF's internal point-to-plane update.
3. **Regularization sensitivity**: The quality of the covariance estimate depends on $\gamma$, which is environment-dependent.
4. **Computational overhead**: Monte Carlo validation is offline; the runtime covariance computation is fast but the principled model is more expensive than the simple Hessian inverse.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Indirectly applicable** — the concepts are relevant but the mechanism differs:

- **FAST-LIO2 doesn't use ICP as a measurement source**: FAST-LIO2 fuses raw point-to-plane residuals directly in the IEKF, not ICP-produced poses. So the "ICP covariance → filter measurement noise" pipeline doesn't apply directly.

- **However, the concept of accurate measurement covariance is critical**: FAST-LIO2 uses a fixed, isotropic $R$ (measurement noise) for all points. The principled covariance ideas suggest:
  1. **Per-point noise modelling**: Each point should have its own noise variance based on range, incidence angle, and local geometry quality.
  2. **Degeneracy-aware $R$**: When the overall registration is degenerate, inflating the measurement noise for correspondences contributing to degenerate directions would naturally limit the Kalman gain along those directions.

- **Regularized Hessian for IEKF**: The regularized Hessian $(H^T R^{-1} H + \gamma I)^{-1}$ could replace the standard IEKF Hessian inversion, providing numerical stability during degeneracy without needing explicit degeneracy detection.

---

## 7. Applicability to Livox Mid-360

- **Range-dependent noise**: The Livox Mid-360's measurement noise varies with range and incidence angle. Principled per-point noise modelling would improve FAST-LIO2's measurement model.
- **Non-uniform density effects**: The spatially varying density of the Livox pattern means some correspondences are based on dense neighborhoods (reliable normals) and others on sparse neighborhoods (unreliable normals). Per-point covariance modelling would capture this.
- **EELS heritage**: While designed for a different robot, the EELS mission's extreme environments (lava tubes, ice caves) share degeneracy characteristics with some of the failure-mode environments (tunnels, silos).
