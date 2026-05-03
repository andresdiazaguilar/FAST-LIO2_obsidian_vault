# Eigen Is All You Need: Efficient Lidar-Inertial Continuous-Time Odometry with Internal Association

**Category**: Continuous-time  
**Failure Modes Addressed**: FM6 (Motion Distortion / Deskew Errors)

---

## 1. Problem Statement

Discrete-time LIO (FAST-LIO2) deskews each scan using IMU-propagated poses, then treats the deskewed scan as if captured instantaneously. This approximation introduces errors during aggressive motion. **Continuous-time** approaches represent the trajectory as a continuous function, allowing each point to be associated with the exact pose at its timestamp, eliminating the discrete deskewing step entirely.

---

## 2. Core Methodology

### 2.1 B-Spline Trajectory Representation

The trajectory is represented as a **cumulative B-spline** in $SE(3)$:

$$
T(t) = T_0 \prod_{i=1}^{k} \exp(\tilde{B}_i(t) \cdot \Omega_i)
$$

where:
- $T_0 \in SE(3)$ is the initial pose
- $\tilde{B}_i(t)$ are the B-spline basis functions evaluated at time $t$
- $\Omega_i \in \mathfrak{se}(3)$ are the control point increments (Lie algebra elements)
- $k$ is the spline order (typically cubic, $k=4$)

The B-spline provides:
- **Continuity**: $C^{k-2}$ continuous trajectory (for cubic splines, $C^2$)
- **Compact representation**: Only control points need to be stored
- **Analytical derivatives**: Velocity, acceleration, angular velocity are analytically available

### 2.2 Point-to-Trajectory Association

Each LiDAR point $p_j$ with timestamp $t_j$ is associated with the pose $T(t_j)$ from the continuous trajectory:

$$
p_j^{\text{world}} = T(t_j) \cdot p_j^{\text{body}}
$$

No separate deskewing step is needed — the point's world-frame position naturally accounts for the sensor's continuous motion.

### 2.3 IMU Integration as Trajectory Constraints

IMU measurements constrain the trajectory derivatives:

**Angular velocity**:
$$
\omega(t) = \dot{R}(t) R(t)^T \quad \Rightarrow \quad r_{\omega} = \omega_{\text{IMU}}(t) - \omega_{\text{spline}}(t)
$$

**Linear acceleration**:
$$
a(t) = R(t)^T (\ddot{p}(t) - g) + b_a \quad \Rightarrow \quad r_a = a_{\text{IMU}}(t) - a_{\text{spline}}(t)
$$

These residuals constrain the spline control points such that the trajectory is dynamically consistent with the IMU data.

### 2.4 LiDAR Residuals

Point-to-plane residuals use the spline-evaluated pose:
$$
r_{\text{LiDAR},j} = n_j^T (T(t_j) p_j - q_j)
$$

The Jacobian of this residual w.r.t. the spline control points involves the B-spline basis function derivatives.

### 2.5 Joint Optimization

All residuals (LiDAR + IMU) are optimized jointly:

$$
\min_{\{\Omega_i\}} \sum_j w_L r_{\text{LiDAR},j}^2 + \sum_m w_\omega r_{\omega,m}^2 + \sum_m w_a r_{a,m}^2
$$

### 2.6 Internal Association

The "Eigen Is All You Need" title refers to the efficient computation of B-spline evaluations using eigendecomposition of the basis function matrix, enabling real-time performance.

---

## 3. Key Equations

### Cubic B-spline evaluation (cumulative form)
$$
T(u) = T_i \prod_{j=1}^{3} \exp(\tilde{B}_j(u) \cdot \Omega_{i+j})
$$

where $u = (t - t_i) / \Delta t \in [0, 1]$ is the normalized time within the $i$-th spline segment.

### Basis function matrix (cubic)
$$
\mathbf{B} = \frac{1}{6} \begin{bmatrix} 5 & 3 & -3 & 1 \\ 1 & 3 & 3 & -2 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### Spline velocity
$$
\dot{T}(t) = T(t) \hat{\xi}(t)
$$

where $\hat{\xi}(t) \in \mathfrak{se}(3)$ is the body-frame velocity.

### IMU angular velocity residual
$$
r_\omega = \omega_{\text{IMU}} - J_l^{-1}(\Omega_i) \dot{\tilde{B}}_i(t) + [\omega_{\text{IMU}}]_\times b_g
$$

---

## 4. Assumptions

1. **Smooth trajectory**: B-splines enforce smooth trajectories ($C^2$ for cubic). This may over-smooth during impacts or sudden motion changes.
2. **Temporal accuracy**: Point timestamps and IMU timestamps must be accurate and synchronized. Clock drift or jitter degrades the continuous-time advantage.
3. **Sufficient spline knots**: The control point density must match the trajectory complexity. Too few → over-smoothing; too many → overfitting and computational cost.
4. **Known spline order**: Cubic B-splines are standard but may not be optimal for all motion profiles.

---

## 5. Limitations

1. **Full system redesign required**: Replacing FAST-LIO2's IEKF with a continuous-time B-spline optimization requires a complete architectural change. This is **not a drop-in modification**.
2. **Computational cost**: B-spline optimization involves a larger state (many control points) and denser Jacobian structure than the IEKF's fixed-dimension state. Real-time performance requires careful optimization.
3. **Latency**: The optimization window covers the entire scan interval. Unlike the IEKF (which provides instant updates), B-spline optimization has inherent latency.
4. **Spline knot spacing**: The temporal resolution of the trajectory is limited by the knot spacing. For FAST-LIO2's 10Hz scan rate and 200Hz IMU, knot spacing of ~5ms covers the motion well but increases optimization variables.
5. **No filter-based uncertainty**: The optimization produces point estimates without the principled uncertainty propagation of the IEKF. Covariance estimation requires additional computation (e.g., marginalizing the Hessian).

---

## 6. Applicability to FAST-LIO2 / IEKF

**Not directly applicable as a modification — requires system redesign**:

- The IEKF and B-spline optimization are fundamentally different estimation paradigms. You cannot "add" continuous-time estimation to the IEKF.
- **However**, the concept inspires improvements:
  1. **Iterative re-deskewing**: Within each IEKF iteration, re-deskew the scan using the updated pose estimate. This approximates the continuous-time benefit without changing the architecture.
  2. **Smooth trajectory within scan**: Use IMU data at higher rate (200Hz) to create a smooth intra-scan trajectory for deskewing, rather than linear interpolation.
  3. **Spline-based IMU preintegration**: Use B-spline representation for the IMU preintegration within each scan interval, improving deskew accuracy.

**HCTO** (see separate notes) proposes a **hybrid** approach that may be more practical for FAST-LIO2.

---

## 7. Applicability to Livox Mid-360

- **Scan timing**: The Mid-360 scan accumulation period (~100ms) is long enough that continuous-time correction is beneficial during aggressive motion.
- **Point timestamps**: The Livox Mid-360 provides per-point timestamps, which is essential for continuous-time methods.
- **Non-repetitive pattern**: The non-repetitive scan pattern means points within a single scan cover different spatial regions. Continuous-time registration naturally handles this diversity.
- **But**: The system redesign requirement makes this impractical as a FAST-LIO2 modification. The hybrid approach (HCTO) or iterative re-deskewing within the IEKF is more feasible.
