# Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization

**Authors**: Hinduja et al.  
**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy)

---

## 1. Problem Statement

Point-to-plane ICP registration can fail silently when the geometric distribution of correspondences doesn't constrain all 6 DoFs. Standard ICP implementations produce a solution even when the problem is ill-conditioned, without indicating which directions are unreliable. This paper provides a **probabilistic framework** for detecting and characterizing directional degeneracy in point-to-plane registration.

---

## 2. Core Methodology

### 2.1 Degeneracy Characterization via the Hessian

For point-to-plane ICP, the cost function is:

$$
E(\xi) = \sum_{j=1}^{N} \left( n_j^T (T(\xi) p_j - q_j) \right)^2
$$

where $\xi \in \mathbb{R}^6$ is the pose perturbation (translation + rotation), $n_j$ is the normal at correspondence $q_j$, and $T(\xi)$ is the transformation parameterized by $\xi$.

The Hessian (approximately the information matrix) is:

$$
H = J^T J = \sum_{j=1}^{N} J_j^T J_j
$$

where $J_j = n_j^T \frac{\partial (T p_j)}{\partial \xi}$ is the $1 \times 6$ Jacobian of the $j$-th residual.

### 2.2 Condition Number Analysis

The paper proposes using the **condition number** $\kappa(H)$ as the primary degeneracy indicator:

$$
\kappa(H) = \frac{\lambda_{\max}}{\lambda_{\min}}
$$

A high condition number indicates directional degeneracy. The paper establishes that:
- $\kappa \sim 1$: Well-conditioned (all directions equally constrained)
- $\kappa \gg 1$: Directionally degenerate (some directions much less constrained)
- $\kappa \to \infty$: Fully degenerate (rank-deficient)

### 2.3 Probabilistic Framework

Rather than a binary degeneracy decision, the paper proposes a **probabilistic model** for the reliability of each eigenvector direction. The key insight is that the eigenvalue magnitudes can be interpreted as inverse variances of the solution along each eigenvector direction:

$$
\text{Var}(\xi_i) \propto \frac{\sigma^2}{\lambda_i}
$$

where $\sigma^2$ is the measurement noise variance and $\lambda_i$ is the $i$-th eigenvalue of $H$. Small eigenvalues → large variance → unreliable direction.

### 2.4 Solution Remapping

When degeneracy is detected, the paper proposes **solution remapping**: projecting the ICP solution onto the well-conditioned subspace. Given eigendecomposition $H = V \Lambda V^T$:

1. Identify degenerate eigenvectors: those with $\lambda_i < \lambda_{\text{thresh}}$
2. Construct the well-conditioned subspace: $V_{\text{good}} = [v_{k+1}, \dots, v_6]$ (the $6-k$ well-conditioned eigenvectors)
3. Project the solution: $\xi_{\text{remapped}} = V_{\text{good}} V_{\text{good}}^T \xi_{\text{ICP}}$

This removes the component of the solution along degenerate directions, replacing it with zero (i.e., no update along degenerate directions).

---

## 3. Key Equations

### Eigenvalue ratio test
$$
\frac{\lambda_i}{\lambda_{i+1}} < \gamma \quad \Rightarrow \quad \text{directions } 1, \dots, i \text{ are degenerate}
$$

where $\gamma$ is a threshold on the eigenvalue ratio gap.

### Solution covariance in eigenvector basis
$$
\Sigma_\xi = \sigma^2 (J^T J)^{-1} = \sigma^2 V \Lambda^{-1} V^T
$$

The covariance along eigenvector $v_i$ is $\sigma^2 / \lambda_i$, making degenerate directions immediately visible.

---

## 4. Assumptions

1. **Gaussian noise model**: Measurement noise is assumed i.i.d. Gaussian with known variance $\sigma^2$.
2. **Correct correspondences**: The analysis assumes the nearest-neighbor correspondences are correct — degeneracy analysis is meaningless if correspondences are wrong.
3. **Local linearity**: The Hessian analysis is valid only near the true solution (local analysis).
4. **Static scene**: No dynamic objects.

---

## 5. Limitations

1. **Designed for optimization-based ICP, not filters**: The solution remapping operates on the ICP solution $\xi$, not on a Kalman gain. In an IEKF, the state update is $\delta x = K(z - h(\hat{x}))$, and remapping the state update is not equivalent to remapping the Kalman gain.
2. **Binary threshold**: Despite the probabilistic framing, the actual degeneracy decision still relies on a threshold ($\gamma$ or $\lambda_{\text{thresh}}$).
3. **No integration with IMU**: The method is purely LiDAR-based; it doesn't consider how the filter's prior (from IMU propagation) affects the effective observability.
4. **Zero replacement**: Degenerate directions get zero update, which is equivalent to full IMU trust. This may not be optimal if IMU is also drifting.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Indirectly applicable** — the theoretical framework is highly relevant, but the solution remapping mechanism requires adaptation:

- **Degeneracy detection**: The eigenvalue analysis of $H^T R^{-1} H$ is directly computable from FAST-LIO2's measurement Jacobian. This is the same analysis D²-LIO performs.
- **Solution remapping → Kalman gain modification**: In an IEKF, the equivalent of "solution remapping" is modifying the Kalman gain to zero out updates along degenerate eigenvectors. This is exactly what D²-LIO implements, making this paper the **theoretical foundation** for D²-LIO's approach.
- **Probabilistic interpretation**: The covariance interpretation ($\text{Var} \propto 1/\lambda$) can inform how the IEKF's posterior covariance should be handled during degeneracy — the posterior covariance should remain large along degenerate directions rather than artificially shrinking.

**Key insight for IEKF**: In FAST-LIO2, the posterior covariance is:

$$
P^+ = (I - KH)P^-
$$

If $K$ is modified to zero out degenerate directions, $P^+$ naturally preserves the prior covariance along those directions, which is the correct behavior.

---

## 7. Applicability to Livox Mid-360

- The eigenvalue/condition number analysis is scan-pattern agnostic.
- The Livox Mid-360's non-repetitive pattern may produce different eigenvalue distributions than assumed in the paper's experiments (which used spinning LiDARs).
- The paper's threshold recommendations may need recalibration for Livox point densities.
