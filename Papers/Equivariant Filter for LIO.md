# Equivariant Filter for Tightly Coupled LiDAR-Inertial Odometry

**Authors**: Tao et al., 2025, ICRA  
**Category**: State estimation theory (suggested paper)  
**Failure Modes Addressed**: FM3 (Attitude Mis-Estimation)

---

## 1. Problem Statement

FAST-LIO2's IEKF linearizes the state dynamics around the current estimate and iterates the measurement update. However, the IEKF's linearization error depends on the estimation error — when the error is large (e.g., during fast rotation), the linearization is poor, leading to inconsistent covariance estimates and potential divergence. **Equivariant filters** exploit the symmetry structure of the state space ($SE(3)$ for pose) to obtain linearization errors that are **independent of the state**, leading to better consistency and convergence.

---

## 2. Core Methodology

### 2.1 Symmetry Groups for LIO

The LIO state lives on the Lie group $SE_2(3) \times \mathbb{R}^6$:
- $SE_2(3)$: Extended special Euclidean group (rotation, velocity, position)
- $\mathbb{R}^6$: Bias states (gyro bias, accel bias)

The key insight is that the IMU dynamics have a **group-affine** structure:
$$
\dot{X} = f_u(X) = AX + XB + C
$$

where $X \in SE_2(3)$, and $A, B, C$ depend on IMU inputs. This structure means the error dynamics (on the group) are **autonomous** (don't depend on the state itself).

### 2.2 Equivariant Error Definition

Instead of the standard error $\delta x = x - \hat{x}$ (additive) or $\delta x = \hat{x}^{-1} x$ (right-invariant), the equivariant filter uses:

$$
\eta = \hat{X}^{-1} X \cdot \phi(\hat{X})
$$

where $\phi$ is a carefully chosen symmetry action that makes the error dynamics state-independent. The resulting error propagation:

$$
\dot{\eta} = A_0 \eta + O(\|\eta\|^2)
$$

The matrix $A_0$ is **constant** (depends on input, not on the state estimate $\hat{X}$). This means the linearization is always valid, regardless of the estimation error.

### 2.3 Comparison with IEKF

**IEKF**: Error dynamics $\delta \dot{x} = A(\hat{x}) \delta x + O(\|\delta x\|^2)$
- $A(\hat{x})$ depends on $\hat{x}$ → linearization quality depends on estimate quality
- Large errors → poor $A(\hat{x})$ → inconsistent covariance → filter degradation

**Equivariant Filter (EqF)**: Error dynamics $\dot{\eta} = A_0 \eta + O(\|\eta\|^2)$  
- $A_0$ is state-independent → linearization is always the same quality
- Large errors → same $A_0$ → consistent covariance → robust filter

### 2.4 LiDAR Measurement in EqF

The LiDAR point-to-plane measurement must be expressed in the equivariant framework:

$$
h(X, p_j) = n_j^T (R p_j + t - q_j)
$$

The measurement Jacobian in the equivariant error coordinates differs from the standard formulation:

$$
C_j = \frac{\partial h}{\partial \eta}\bigg|_{\eta=0} = n_j^T \frac{\partial (R(\eta) p_j + t(\eta))}{\partial \eta}\bigg|_{\eta=0}
$$

The key difference is that this Jacobian is computed with respect to the equivariant error $\eta$, not the standard error $\delta x$.

---

## 3. Key Equations

### Extended pose matrix
$$
X = \begin{bmatrix} R & v & p \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} \in SE_2(3)
$$

### Group-affine dynamics
$$
\dot{X} = \begin{bmatrix} [\omega_m - b_g]_\times & a_m - b_a & v \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{bmatrix} X + X \begin{bmatrix} 0 & g & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}
$$

### State-independent error propagation
$$
P^- = \Phi_0 P^+ \Phi_0^T + Q
$$

where $\Phi_0 = \exp(A_0 \Delta t)$ is computed from the **constant** $A_0$ matrix.

### EqF update
$$
K = P^- C^T (C P^- C^T + R)^{-1}
$$
$$
\hat{\eta} = K (z - h(\hat{X}))
$$
$$
\hat{X}^+ = \hat{X}^- \cdot \text{Exp}(\hat{\eta})
$$

---

## 4. Assumptions

1. **Group-affine dynamics**: The dynamics must have the group-affine structure. The IMU dynamics on $SE_2(3)$ do satisfy this, but adding additional states (e.g., map-related states) may break the structure.
2. **Known symmetry group**: The equivariant filter requires identifying the appropriate symmetry group. For LIO, this is $SE_2(3) \times \mathbb{R}^6$.
3. **Bias dynamics are approximately zero**: The bias states don't have group structure. They're treated as Euclidean additions, which partially breaks the equivariance.

---

## 5. Limitations

1. **Implementation complexity**: The equivariant filter requires implementing Lie group operations ($SE_2(3)$ exponential, logarithm, adjoint) that are more complex than the standard IEKF's quaternion/rotation matrix operations.
2. **Bias treatment**: The biases break perfect equivariance. The bias dynamics are treated with standard additive errors, reducing the advantage.
3. **LiDAR measurement complexity**: The point-to-plane measurement doesn't naturally fit the equivariant structure. The measurement Jacobian must be carefully derived in equivariant coordinates.
4. **Marginal improvement in practice**: For well-initialized systems with good IMU, the advantage over IEKF is small. The benefit is most apparent during large errors or poor initialization.
5. **Less mature tooling**: Fewer libraries and references for implementing equivariant filters compared to IEKF.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable but requires significant refactoring**:

- **Replaces the IEKF**: The equivariant filter is a fundamentally different filter, not a modification of the IEKF. The state representation, error definition, propagation, and update all change.
- **Better consistency**: The state-independent linearization improves the filter's consistency, particularly during aggressive maneuvers or poor initialization.
- **Attitude robustness**: The equivariant filter handles large rotation errors better than the IEKF, directly addressing FM3 (attitude mis-estimation).
- **Same computational structure**: Despite the different mathematical formulation, the computational structure is similar (predict → iterate update → correct). The matrices are different shapes but similar sizes.

**Recommended approach**: This is a long-term improvement, not a quick modification. Consider implementing it as an alternative filter backend within the FAST-LIO2 architecture, allowing A/B comparison.

---

## 7. Applicability to Livox Mid-360

- **Aggressive motion handling**: The equivariant filter's robustness to large errors is valuable for hand-held operation with the Mid-360, where sudden movements can cause large prediction errors.
- **ICM40609 IMU**: The IMU dynamics on $SE_2(3)$ are independent of IMU quality. The equivariant filter's advantage applies equally regardless of IMU performance.
- **Practical priority**: Given the implementation complexity, this is lower priority than simpler improvements (degeneracy detection, robust correspondences) but represents a principled long-term solution.
