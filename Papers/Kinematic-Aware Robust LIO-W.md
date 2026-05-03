# Kinematic-Aware Robust LIO-W: LiDAR-Inertial-Wheel Odometry with Weighted Optimization

**Authors**: Wu et al., 2026, IEEE Access  
**Category**: Multi-sensor / Robust (suggested paper)  
**Failure Modes Addressed**: FM5 (Correspondence Errors)

---

## 1. Problem Statement

In environments with many outlier correspondences (vegetation, clutter, dynamic objects), standard point-to-plane registration fails because outlier residuals dominate the optimization. LIO-W (LiDAR-Inertial-Wheel) proposes:
1. **Kinematic model integration** using wheel odometry for additional motion constraints
2. **Weighted optimization** that adaptively down-weights outlier correspondences based on residual statistics

---

## 2. Core Methodology

### 2.1 Wheel Odometry Integration

For ground robots, wheel encoders provide an independent velocity estimate:
$$
v_{\text{wheel}} = r_w \cdot \omega_w
$$

where $r_w$ is the wheel radius and $\omega_w$ is the wheel angular velocity.

This is integrated as an additional measurement in the IEKF:
$$
h_{\text{wheel}}(x) = R^{BW} v^W = v^B_{\text{predicted}}
$$

The wheel velocity constraint:
$$
r_{\text{wheel}} = v^B_{\text{predicted}} - v_{\text{wheel}} \cdot \hat{e}_{\text{forward}}
$$

where $\hat{e}_{\text{forward}}$ is the forward direction of the robot.

### 2.2 Non-Holonomic Constraints

For differential-drive or Ackermann-steering robots, lateral and vertical velocities are approximately zero:
$$
r_{\text{lateral}} = v^B_y \approx 0
$$
$$
r_{\text{vertical}} = v^B_z \approx 0
$$

These constraints are added as pseudo-measurements with appropriate noise:
$$
R_{\text{nh}} = \text{diag}(\sigma_{\text{lateral}}^2, \sigma_{\text{vertical}}^2)
$$

### 2.3 Weighted Optimization with M-Estimation

LIO-W uses **M-estimation** (specifically, the Welsch/Leclerc function) to down-weight outlier correspondences:

$$
\rho_W(r) = \frac{c^2}{2} \left(1 - \exp\left(-\frac{r^2}{c^2}\right)\right)
$$

The corresponding weight in IRLS:
$$
w_j = \exp\left(-\frac{r_j^2}{c^2}\right)
$$

This gives exponentially decreasing weight for large residuals, providing stronger outlier rejection than the Huber function.

### 2.4 Adaptive Scale Parameter

The scale parameter $c$ adapts to the current residual distribution:
$$
c_k = \gamma \cdot \text{MAD}(\{r_j\})
$$

where $\text{MAD}$ is the median absolute deviation:
$$
\text{MAD} = \text{median}(|r_j - \text{median}(r_j)|)
$$

This makes the outlier rejection threshold data-adaptive.

---

## 3. Key Equations

### Combined IEKF measurement model
$$
H = \begin{bmatrix} H_{\text{LiDAR}} \\ H_{\text{wheel}} \\ H_{\text{nh}} \end{bmatrix}, \quad
z = \begin{bmatrix} r_{\text{LiDAR}} \\ r_{\text{wheel}} \\ r_{\text{nh}} \end{bmatrix}
$$

### Welsch weight function
$$
w_j = \exp(-r_j^2 / c^2)
$$

### Weighted normal equations in IEKF
$$
(H^T W R^{-1} H + (P^-)^{-1}) \delta x = H^T W R^{-1} z + (P^-)^{-1} \delta x_{\text{pred}}
$$

where $W = \text{diag}(w_1, \ldots, w_N)$.

### MAD-based scale
$$
\hat{\sigma} = 1.4826 \cdot \text{MAD}(\{r_j\}), \quad c = 2.5 \hat{\sigma}
$$

The factor 1.4826 makes MAD a consistent estimator of the standard deviation for Gaussian data.

### Non-holonomic velocity constraint Jacobian
$$
H_{\text{nh}} = \begin{bmatrix} 0 & 0 & 0 & e_y^T R^{BW} & 0_{1\times 15} \\ 0 & 0 & 0 & e_z^T R^{BW} & 0_{1\times 15} \end{bmatrix}
$$

---

## 4. Assumptions

1. **Ground robot with wheels**: The wheel odometry and non-holonomic constraints assume a wheeled ground robot. Not applicable to drones, hand-held, or legged robots.
2. **No wheel slip**: Wheel odometry assumes no slip. On slippery surfaces (ice, mud, gravel), wheel odometry is unreliable.
3. **Flat ground for non-holonomic constraints**: The vertical velocity constraint ($v_z \approx 0$) assumes flat terrain.
4. **Residuals follow a contaminated Gaussian**: The M-estimation assumes most residuals are Gaussian with a fraction of outliers. If the outlier fraction exceeds ~50%, the robust estimation breaks down.

---

## 5. Limitations

1. **Platform-specific**: Wheel odometry integration is limited to wheeled robots. Not applicable to drones, hand-held devices, or legged robots.
2. **Wheel radius calibration**: Accurate wheel odometry requires calibrated wheel radius, which changes with tire pressure, load, and wear.
3. **Slip detection**: No explicit wheel slip detection. Slip events corrupt the wheel velocity estimate.
4. **M-estimation convergence**: The IRLS procedure with Welsch weights may converge slowly or to local minima when the outlier fraction is high.
5. **Scale parameter sensitivity**: The MAD-based scale works well for moderate outlier fractions (<30%) but may be unreliable for extreme cases.

---

## 6. Applicability to FAST-LIO2 / IEKF

**The weighted optimization is broadly applicable; wheel odometry is platform-specific**:

### Weighted IEKF (applicable to any platform)
- **Welsch-weighted residuals**: Replace the unweighted residuals in FAST-LIO2's IEKF with Welsch-weighted ones. This provides strong outlier rejection.
- **MAD-based adaptive scale**: Use the MAD of residuals to set the rejection threshold. More robust than fixed thresholds.
- **Implementation**: Modify the IEKF update to include a weight matrix $W$ in the normal equations. Run IRLS: compute residuals → compute weights → solve weighted least squares → repeat.

### Wheel odometry integration (ground robots only)
- **Additional measurement rows**: Add wheel velocity and non-holonomic constraint rows to $H$.
- **State unchanged**: No new states needed if wheel odometry is treated as a velocity measurement.

### Key benefit
The Welsch M-estimator provides the strongest outlier rejection among standard robust estimators (compared to Huber in RELEAD). For environments with many incorrect correspondences (dynamic objects, vegetation), this is critical.

---

## 7. Applicability to Livox Mid-360

- **Weighted optimization**: Directly applicable regardless of LiDAR type. The Welsch-weighted IEKF is particularly valuable for the Livox when operating in cluttered environments where incorrect correspondences arise from the non-uniform scan density.
- **MAD-based scale**: The Mid-360's residual distribution varies across scans. The data-adaptive scale ensures the outlier threshold is appropriate for each scan.
- **Wheel odometry**: If the Mid-360 is mounted on a wheeled robot, the wheel constraints significantly improve robustness.
- **Non-holonomic constraints**: For ground robots with the Mid-360, these constraints prevent vertical drift — a common problem.
- **Hand-held use**: For hand-held use (no wheels), only the weighted optimization is applicable. The M-estimation is still highly valuable.
