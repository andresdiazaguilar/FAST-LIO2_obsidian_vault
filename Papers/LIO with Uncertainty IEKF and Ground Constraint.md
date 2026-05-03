# LiDAR-Inertial Odometry with Uncertainty-Aware IEKF and Ground Constraint

**Authors**: Luo & Cao, 2024  
**Category**: State estimation (suggested paper)  
**Failure Modes Addressed**: FM3 (Attitude Mis-Estimation)

---

## 1. Problem Statement

FAST-LIO2's IEKF treats all LiDAR measurements equally and doesn't explicitly incorporate ground constraints. This leads to:
1. Poor vertical accuracy (height drift) when ground features dominate
2. Roll/pitch corruption when the gravity direction is uncertain
3. No mechanism to leverage the known flatness of many environments

This paper proposes an **uncertainty-aware IEKF** that dynamically adjusts measurement trust based on per-point uncertainty, combined with explicit **ground plane constraints** for improved attitude and height estimation.

---

## 2. Core Methodology

### 2.1 Per-Point Uncertainty Model

Each point's measurement noise is modeled as a function of:
- **Range**: $\sigma_r^2 \propto d_j^2$ (range noise increases with distance)
- **Incidence angle**: $\sigma_\theta^2 \propto 1/\cos^2(\alpha_j)$ (grazing incidence is noisier)
- **Return intensity**: $\sigma_I^2 \propto 1/I_j$ (low intensity indicates unreliable returns)

The combined per-point noise:
$$
R_{jj} = \sigma_0^2 + \beta_r d_j^2 + \beta_\theta / \cos^2(\alpha_j) + \beta_I / I_j
$$

This replaces FAST-LIO2's constant measurement noise with a **heteroscedastic** model.

### 2.2 Ground Plane Detection and Constraint

**Detection**: Points below a height threshold (from the gravity-aligned frame) with surface normal nearly aligned with gravity:
$$
\text{ground}(p_j) = (p_j^z < h_{\text{thresh}}) \wedge (|n_j^T g / \|g\|| > \cos(\alpha_{\text{thresh}}))
$$

**Constraint**: The detected ground plane $\pi = (n_g, d_g)$ provides:

1. **Height constraint**: $z_{\text{sensor}} = d_g / n_g^z$ (assuming sensor is above flat ground)
2. **Roll/pitch constraint**: The ground normal, rotated to body frame, should align with the body's $z$-axis:
   $$
   r_{\text{attitude}} = R^T n_g - e_3
   $$

### 2.3 Integration in IEKF

The ground constraints are added as **pseudo-measurements** in the IEKF:

$$
H_{\text{aug}} = \begin{bmatrix} H_{\text{LiDAR}} \\ H_{\text{ground\_height}} \\ H_{\text{ground\_attitude}} \end{bmatrix}, \quad
R_{\text{aug}} = \begin{bmatrix} R_{\text{LiDAR}} & & \\ & R_{\text{height}} & \\ & & R_{\text{attitude}} \end{bmatrix}
$$

The ground constraint noise $R_{\text{height}}$ and $R_{\text{attitude}}$ are set based on the confidence of the ground detection (number of ground points, planarity, consistency with IMU).

### 2.4 Adaptive Ground Confidence

The ground constraint weight adapts based on consistency:

$$
R_{\text{ground}}^{(k)} = R_0 / \max(1, N_{\text{ground}} \cdot \text{planarity} \cdot \text{consistency})
$$

where consistency measures agreement between the detected ground plane and the IMU's gravity estimate:
$$
\text{consistency} = \exp(-\alpha \|R^T_{\text{IMU}} g - n_g\|^2)
$$

---

## 3. Key Equations

### Per-point measurement noise
$$
R_{jj} = \sigma_0^2 \left(1 + \frac{d_j^2}{d_{\text{ref}}^2} + \frac{1}{\cos^2(\alpha_j)} + \frac{I_{\text{ref}}}{I_j}\right)
$$

### Ground height pseudo-measurement
$$
h_{\text{height}}(x) = e_3^T (p^W + R^W_B p^B_{\text{LiDAR}})
$$

### Ground attitude pseudo-measurement
$$
h_{\text{att}}(x) = R^{BW} n_g^W - \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

### Height constraint Jacobian
$$
H_{\text{height}} = e_3^T \begin{bmatrix} I_3 & -R^W_B [p^B_{\text{LiDAR}}]_\times & 0_{1 \times 18} \end{bmatrix}
$$

---

## 4. Assumptions

1. **Ground is flat**: The ground constraint assumes local planarity. On rough terrain, stairs, or slopes, the constraint must be relaxed.
2. **Ground is visible**: The sensor must see the ground. If the LiDAR points downward and the ground is within range, this is usually satisfied.
3. **Intensity is informative**: The per-point noise model uses intensity, which may not correlate with measurement quality for all surfaces.
4. **Incidence angle is computable**: Requires surface normal estimation (from kNN) for incidence angle computation.

---

## 5. Limitations

1. **Ground flatness assumption**: Fails on uneven terrain, stairs, ramps. Need terrain classification to identify valid ground regions.
2. **Ground detection errors**: Detecting the wrong surface as ground (e.g., a table, a car hood) corrupts the constraint.
3. **Per-point noise calibration**: The noise model parameters ($\beta_r, \beta_\theta, \beta_I$) require calibration for each LiDAR model.
4. **Added computational cost**: Ground detection, incidence angle computation, and adaptive noise computation add overhead.
5. **Indoor/multi-floor limitation**: The ground height constraint doesn't work when transitioning between floors.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable — enhances the existing IEKF**:

- **Per-point noise**: Replace the constant $R$ diagonal with the heteroscedastic model. Each point gets its own noise value based on range, incidence angle, and intensity. This requires no structural change to the IEKF — only modifying the $R$ diagonal.
- **Ground constraint**: Add as pseudo-measurement rows in the IEKF. This is straightforward: compute ground plane parameters from ground-classified points, add the constraint rows to $H$ with appropriate $R$.
- **Attitude stabilization**: The ground attitude constraint directly prevents roll/pitch drift (FM3). When the ground is visible, it provides a stable reference.

**Implementation priority**:
1. Per-point noise model (easy, high impact)
2. Ground height constraint (moderate, high impact for height drift)
3. Ground attitude constraint (moderate, addresses FM3)

---

## 7. Applicability to Livox Mid-360

- **Ground visibility**: The Mid-360's lower FoV reaches -7°, so it sees the ground in most configurations. Good for ground detection.
- **Per-point noise**: The Livox's varying intensity and non-uniform density make the heteroscedastic model particularly valuable. Points at the edge of the scan pattern (lower density, possibly lower quality) get higher noise.
- **Incidence angle**: Computable from kNN normals, which FAST-LIO2 already estimates.
- **ICM40609 IMU gravity**: The ground constraint provides an independent gravity reference, which helps when the IMU's gravity estimate is corrupted by vibration or bias.
