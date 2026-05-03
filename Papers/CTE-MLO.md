# CTE-MLO: Continuous-Time Estimation for Multi-LiDAR Odometry with Localizability-Aware Sampling

**Authors**: Shen et al., 2025, IEEE Transactions on Field Robotics  
**Category**: Solid-state / Continuous-time (suggested paper)  
**Failure Modes Addressed**: FM9 (Solid-State LiDAR Challenges)

---

## 1. Problem Statement

Multi-LiDAR systems and solid-state LiDARs produce points at different times, making the "discrete scan" assumption of standard SLAM systems problematic. CTE-MLO proposes:
1. A **continuous-time trajectory representation** that naturally handles asynchronous points from multiple LiDARs
2. A **localizability-aware sampling** strategy that selects points providing the most geometric information for registration

---

## 2. Core Methodology

### 2.1 Continuous-Time Trajectory

The sensor trajectory is represented as a **B-spline on SE(3)**:

$$
T(\tau) = \prod_{i=0}^{k} \exp(\tilde{B}_i(\tau) \cdot \Omega_i)
$$

where:
- $\tilde{B}_i(\tau)$ are the cumulative B-spline basis functions
- $\Omega_i \in \mathfrak{se}(3)$ are the control knots

The B-spline provides:
- **Continuous evaluation**: $T(\tau)$ can be evaluated at any time $\tau$ (not just discrete scan times)
- **Smooth derivatives**: Angular velocity and linear acceleration are continuous
- **Local support**: Each point's residual depends on only $k+1$ nearby knots

### 2.2 Localizability-Aware Point Sampling

Not all points contribute equally to localizability. CTE-MLO selects points based on their **localizability contribution**:

For a candidate point $p_j$ with surface normal $n_j$, the localizability contribution is:
$$
\ell(p_j) = \lambda_{\min}\left(\mathcal{I}_{\text{current}} + n_j n_j^T\right) - \lambda_{\min}(\mathcal{I}_{\text{current}})
$$

This is the improvement in the minimum eigenvalue of the information matrix when this point is added. Points that improve the weakest direction the most are prioritized.

### 2.3 Greedy Selection

The sampling is a greedy selection:
1. Start with $\mathcal{I} = \mathcal{I}_{\text{prior}}$ (from IMU prediction)
2. For each iteration:
   a. Compute $\ell(p_j)$ for all remaining candidate points
   b. Select $p^* = \arg\max_j \ell(p_j)$
   c. Update $\mathcal{I} \leftarrow \mathcal{I} + n_{j^*} n_{j^*}^T$
3. Repeat until $N_{\text{target}}$ points are selected or $\lambda_{\min}(\mathcal{I}) > \tau$

### 2.4 Multi-LiDAR Fusion

With multiple LiDARs, points from all sensors are treated uniformly in the continuous-time framework:

$$
r_j = n_j^T (T(\tau_j) \cdot T_{\text{extrinsic},s_j} \cdot p_j - q_j)
$$

where $s_j$ identifies which LiDAR sensor the point came from, and $T_{\text{extrinsic},s_j}$ is the extrinsic calibration of that sensor.

---

## 3. Key Equations

### Cubic B-spline on SE(3)
$$
T(\tau) = T_i \prod_{j=1}^{3} \exp(B_j(u) \cdot \Omega_{i+j})
$$

where $u = (\tau - t_i) / \Delta t$ and $B_j$ are the uniform B-spline basis functions.

### Knot update (Gauss-Newton on manifold)
$$
\delta \Omega^* = -(J^T R^{-1} J + \Lambda)^{-1} J^T R^{-1} r
$$

where $J$ is the Jacobian of all residuals w.r.t. the control knots, $\Lambda$ is a prior/regularization term.

### Localizability contribution (approximation)
$$
\ell(p_j) \approx v_{\min}^T (n_j n_j^T) v_{\min} = (v_{\min}^T n_j)^2
$$

where $v_{\min}$ is the eigenvector corresponding to $\lambda_{\min}(\mathcal{I})$. This approximation avoids recomputing the eigendecomposition for each candidate point.

### Greedy sampling condition
$$
\text{stop when: } \lambda_{\min}(\mathcal{I}) > \tau \text{ or } |\mathcal{P}_{\text{selected}}| = N_{\text{target}}
$$

---

## 4. Assumptions

1. **Smooth trajectory**: B-splines enforce $C^{k-1}$ continuity. Discontinuous motion (impacts) violates this.
2. **Known extrinsic calibration**: Multi-LiDAR fusion requires accurate extrinsic calibration between sensors.
3. **Sufficient computational budget**: The greedy localizability-aware sampling is more expensive than random or uniform sampling.
4. **Knot spacing is appropriate**: The B-spline knot spacing determines the temporal resolution. Too coarse → can't capture fast motion; too fine → overfitting and computational overhead.

---

## 5. Limitations

1. **Computational cost**: B-spline optimization over $N_{\text{knots}}$ knots is more expensive than discrete IEKF updates. For real-time operation, the number of knots must be limited.
2. **Not filter-based**: The continuous-time formulation is optimization-based (batch), not filter-based. It doesn't naturally provide covariance propagation between scans.
3. **Greedy sampling is suboptimal**: The greedy selection is approximately optimal but not guaranteed to find the globally best point set.
4. **Multi-LiDAR requirement**: The full benefit requires multiple LiDARs. For single LiDAR, the localizability-aware sampling is still valuable, but the continuous-time framework is less critical.
5. **Implementation complexity**: B-splines on SE(3) require careful implementation of Lie group operations, derivative computation, and efficient Jacobian evaluation.

---

## 6. Applicability to FAST-LIO2 / IEKF

**The localizability-aware sampling is directly applicable; the CT framework is not**:

- **Localizability-aware sampling**: Use the greedy sampling to select points for the IEKF update. This replaces uniform voxel downsampling with information-theoretic sampling. Directly improves the information matrix conditioning.
  
  Implementation in FAST-LIO2:
  1. After voxel downsampling, compute normals for candidate points
  2. Run greedy selection based on $\ell(p_j) = (v_{\min}^T n_j)^2$
  3. Use only selected points in the IEKF update

- **CT trajectory**: Replacing FAST-LIO2's discrete IEKF with B-spline optimization is a fundamental architectural change. Not recommended as a modification.

**Connection to other papers**: The localizability-aware sampling is related to GSS (Geometrically Stable Sampling) and D²-LIO's FIM conditioning. CTE-MLO provides the most principled formulation.

---

## 7. Applicability to Livox Mid-360

- **Localizability-aware sampling**: Directly addresses the non-uniform density of the Livox pattern. Points in over-represented directions are down-weighted; points in under-represented directions are prioritized.
- **Single LiDAR**: The multi-LiDAR fusion aspects are less relevant for a single Mid-360 setup.
- **Non-repetitive pattern**: The localizability-aware sampling automatically adapts to each scan's unique pattern, selecting the most informative points regardless of which part of the FoV they come from.
- **Computational consideration**: The greedy selection adds overhead. For the Mid-360's ~20K points per scan after downsampling, the selection over ~5K candidates to pick ~2K is feasible in real-time with the approximation $\ell(p_j) \approx (v_{\min}^T n_j)^2$.
