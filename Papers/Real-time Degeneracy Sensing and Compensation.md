# A Real-time Degeneracy Sensing and Compensation Method for Enhanced LiDAR SLAM

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy)

---

## 1. Problem Statement

LiDAR SLAM systems can silently drift in directions where geometric constraints are insufficient. This paper proposes a **real-time** degeneracy detection method based on eigenvalue analysis combined with a **compensation strategy** using constrained optimization that prevents state updates along degenerate directions.

---

## 2. Core Methodology

### 2.1 Eigenvalue-Based Degeneracy Sensing

The method computes the Hessian matrix of the point-to-plane registration cost:

$$
H = J^T J = \sum_{j=1}^{N} J_j^T J_j \in \mathbb{R}^{6 \times 6}
$$

Eigenvalue analysis yields $\lambda_1 \leq \lambda_2 \leq \dots \leq \lambda_6$ with corresponding eigenvectors $v_1, \dots, v_6$.

**Degeneracy criterion**: A direction $v_i$ is degenerate if:
$$
\lambda_i < \eta \cdot \lambda_{\text{median}}
$$

where $\eta$ is a relative threshold. Using a relative (rather than absolute) threshold makes the detection more robust to changes in point density and scan size.

### 2.2 Constrained Optimization Compensation

When degenerate directions are identified, the registration optimization is constrained to prevent movement along those directions:

$$
\min_\xi E(\xi) \quad \text{subject to} \quad V_d^T \xi = 0
$$

where $V_d = [v_1, \dots, v_k]$ are the $k$ degenerate eigenvectors. This constraint forces the solution to lie in the well-conditioned subspace.

The constrained optimization is implemented using **Lagrange multipliers** or equivalently by projecting the gradient:

$$
\xi^{(t+1)} = \xi^{(t)} - \alpha \Pi_{\perp} \nabla E(\xi^{(t)})
$$

where $\Pi_{\perp} = I - V_d V_d^T$ is the projection onto the complement of the degenerate subspace.

### 2.3 Real-Time Implementation

- The eigendecomposition of a $6 \times 6$ matrix is $O(1)$ (constant-time for fixed dimensions).
- The projection is a simple matrix-vector multiplication.
- The method adds negligible computational overhead to the existing registration pipeline.

---

## 3. Key Equations

### Relative degeneracy threshold
$$
\text{degenerate}_i = \begin{cases} 1 & \text{if } \lambda_i < \eta \cdot \text{median}(\lambda_1, \dots, \lambda_6) \\ 0 & \text{otherwise} \end{cases}
$$

### Constrained gradient descent
$$
\nabla_{\text{proj}} = (I - V_d V_d^T) \nabla E
$$

### Equivalent unconstrained problem (penalty method)
$$
E_{\text{comp}}(\xi) = E(\xi) + \mu \sum_{i=1}^{k} (v_i^T \xi)^2
$$

where $\mu \to \infty$ penalizes movement along degenerate directions.

---

## 4. Assumptions

1. **Correct eigenvector estimation**: The Hessian accurately reflects the geometry, and its eigenvectors correctly identify degenerate directions.
2. **Static threshold**: While relative, $\eta$ is still fixed and may not adapt to all environments.
3. **Optimization-based registration**: The constrained optimization framework assumes a gradient-based optimizer.
4. **Linear degeneracy**: Degeneracy is along eigenvector directions of the linearized Hessian, which may not capture nonlinear degeneracy patterns.

---

## 5. Limitations

1. **Optimization-based, not filter-based**: The constrained optimization approach doesn't directly translate to the Kalman filter framework. In IEKF, the state update is computed analytically, not iteratively via gradient descent.
2. **Binary degeneracy detection**: Despite using a relative threshold, the decision is still binary per direction.
3. **No prior information integration**: In a Kalman filter, the prior (from IMU propagation) provides information about all directions. The constrained optimization doesn't account for this — it prevents movement along degenerate directions even if the prior strongly suggests movement.
4. **Validated on spinning LiDARs**: No experiments with solid-state LiDARs.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Conceptually applicable, but requires translation to Kalman filter framework**:

- **Degeneracy detection**: The eigenvalue analysis of $H^T R^{-1} H$ is directly applicable. The relative threshold (ratio to median) is more robust than D²-LIO's absolute threshold.
- **Constrained optimization → modified Kalman gain**: The equivalent of constraining optimization to the well-conditioned subspace is projecting the Kalman gain:
  $$
  K_{\text{proj}} = (I - V_d V_d^T) K
  $$
  This is essentially what D²-LIO does.
- **Relative threshold contribution**: The idea of using a **relative** threshold (ratio to median eigenvalue) is directly useful for FAST-LIO2, as it naturally adapts to different point densities and scan sizes that the Livox Mid-360 produces.
- **Penalty method translation**: The penalty method interpretation suggests an alternative IEKF implementation: add virtual measurements $v_i^T \delta x = 0$ for degenerate directions with very low noise, effectively constraining the state update.

---

## 7. Applicability to Livox Mid-360

- The relative threshold is particularly beneficial for solid-state LiDARs because the absolute eigenvalue scale varies significantly with the Livox's non-repetitive scan pattern.
- The computational overhead is negligible, which is important for real-time systems.
- The method is feature-type agnostic (works with any point-to-plane registration).
