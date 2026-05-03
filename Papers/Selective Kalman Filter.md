# Selective Kalman Filter: When and How to Fuse Multi-Sensor Information to Overcome Degeneracy in SLAM

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM3 (Attitude Mis-Estimation), FM7 (IMU Quality)

---

## 1. Problem Statement

Multi-sensor fusion systems must decide **when** and **how** to trust each sensor. During degeneracy, the LiDAR provides unreliable information in certain directions, but blindly switching to IMU-only propagation risks accumulating drift. The Selective Kalman Filter (SKF) proposes a principled framework for **selectively fusing** information from multiple sensors based on per-direction observability.

---

## 2. Core Methodology

### 2.1 Per-Direction Observability Analysis

For each sensor modality $m$ (LiDAR, IMU, etc.), the paper computes the information contribution along each state direction:

$$
\mathcal{I}_m = H_m^T R_m^{-1} H_m
$$

The eigendecomposition reveals which directions each sensor informs:

$$
\mathcal{I}_m = V_m \Lambda_m V_m^T
$$

### 2.2 Selective Fusion

Rather than fusing all sensor information uniformly, the Selective KF constructs a **per-direction fusion strategy**:

For each eigenvector direction $v_i$ of the combined information matrix:
1. Compute the information contribution from each sensor: $\lambda_i^{(m)} = v_i^T \mathcal{I}_m v_i$
2. If the LiDAR information $\lambda_i^{(\text{LiDAR})}$ exceeds a threshold → use LiDAR for this direction
3. If only IMU information is sufficient → use IMU for this direction
4. If neither is sufficient → flag the direction as unobservable and preserve uncertainty

### 2.3 Modified Kalman Gain

The fusion strategy is implemented by constructing a modified Kalman gain:

$$
K_{\text{selective}} = P^- H_{\text{sel}}^T (H_{\text{sel}} P^- H_{\text{sel}}^T + R_{\text{sel}})^{-1}
$$

where $H_{\text{sel}}$ and $R_{\text{sel}}$ are the Jacobian and noise covariance of the selected measurement subset. Measurements that don't contribute useful information in any direction are excluded.

Alternatively, the paper proposes a projection-based approach:

$$
K_{\text{selective}} = \Pi_{\text{good}} \cdot K_{\text{full}}
$$

where $\Pi_{\text{good}}$ projects out degenerate directions.

---

## 3. Key Equations

### Per-direction sensor information
$$
\lambda_i^{(m)} = v_i^T H_m^T R_m^{-1} H_m v_i
$$

### Selective update rule
$$
\delta x_i = \begin{cases}
(K z)_i & \text{if } \lambda_i^{(\text{LiDAR})} > \lambda_{\text{thresh}} \\
0 & \text{if } \lambda_i^{(\text{LiDAR})} \leq \lambda_{\text{thresh}}
\end{cases}
$$

### Covariance consistency
$$
P^+_{ii} = \begin{cases}
P^-_{ii} - K_i H P^-_{ii} & \text{if direction } i \text{ is updated} \\
P^-_{ii} & \text{if direction } i \text{ is not updated}
\end{cases}
$$

---

## 4. Assumptions

1. **Direction independence**: Assumes degenerate and well-conditioned directions can be treated independently. In practice, state coupling through the dynamics model creates interdependencies.
2. **Known sensor noise models**: The information analysis requires accurate noise covariances $R_m$ for each sensor.
3. **Eigenvector stability**: The per-direction analysis assumes the eigenvectors of the information matrix are stable during the current scan/update.
4. **Discrete sensor selection**: The fusion is binary per direction (use sensor or don't), not a soft weighting.

---

## 5. Limitations

1. **Binary per-direction decision**: Like D²-LIO, the decision to trust or distrust a direction is threshold-based. No smooth transition between full trust and full distrust.
2. **Doesn't improve IMU quality**: Selective fusion can prevent bad LiDAR updates from corrupting the state, but it cannot improve the IMU's own drift characteristics. If both LiDAR and IMU are unreliable in a direction, the state is truly unobservable.
3. **Cross-direction coupling ignored**: Selecting different sensors for different directions doesn't account for how errors in one direction propagate to others through the state dynamics.
4. **Computational cost**: Per-direction analysis and selective construction of the Kalman gain add overhead, especially when multiple sensors are involved.
5. **Two-sensor focus**: Primarily designed for LiDAR + IMU fusion. Extension to more sensors (camera, wheel odometry) requires additional analysis.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable** — the Selective KF is native to Kalman filter architectures:

- **FAST-LIO2's IEKF already has $H$ and $R$**: The per-direction information analysis uses the same matrices.
- **Kalman gain modification**: The selective fusion modifies the Kalman gain, which is the same intervention point as D²-LIO and LODESTAR.
- **IMU as fallback**: When LiDAR directions are degenerate, the IEKF prediction (from IMU) naturally provides the state estimate for those directions. The Selective KF formalizes this by preventing LiDAR corrections along degenerate directions.

**Relationship to LODESTAR**: The Selective KF is conceptually similar to LODESTAR's Schmidt-Kalman approach, but simpler in implementation. The key difference is that LODESTAR's SKF maintains explicit cross-covariance between solve and consider states, while the Selective KF's projection approach may not fully preserve these cross-terms.

**Integration steps**:
1. Compute $\mathcal{I} = H_{\text{pose}}^T R^{-1} H_{\text{pose}}$ (already available in FAST-LIO2)
2. Eigendecompose $\mathcal{I}$
3. Identify degenerate eigenvectors
4. Modify $K$ to zero out updates along degenerate eigenvectors
5. Adjust covariance update to preserve prior covariance along unupdated directions

---

## 7. Applicability to Livox Mid-360

- **Fully applicable**: The selective fusion framework is sensor-agnostic.
- **IMU quality concern**: The ICM40609's limited quality means that when LiDAR directions are deselected, the IMU fallback may not hold for extended periods. This is a fundamental limitation: the Selective KF can prevent corruption but cannot create information that doesn't exist.
- **Solid-state degeneracy**: The Livox Mid-360's per-scan point distribution may create more frequent mild degeneracy episodes compared to spinning LiDARs, potentially causing the Selective KF to frequently deselect directions. This could be problematic if the threshold is too aggressive.
