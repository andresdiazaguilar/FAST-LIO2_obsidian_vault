# A Robust Approach for LiDAR-Inertial Odometry Without Sensor-Specific Modeling

**Authors**: Malladi et al., 2026, RA-L  
**Category**: IMU handling (suggested paper)  
**Failure Modes Addressed**: FM7 (IMU Quality Limitations)

---

## 1. Problem Statement

LIO systems like FAST-LIO2 require accurate IMU noise parameters (gyroscope noise density, accelerometer noise density, random walk, bias instability) that are specific to each IMU model. These parameters are often:
1. **Unavailable** for low-cost IMUs
2. **Inaccurate** (datasheet values vs. real-world values differ significantly)
3. **Variable** across individual units and operating conditions

This paper proposes a LIO system that operates **without requiring sensor-specific IMU noise modeling**, using **online estimation** of all noise parameters from the data itself.

---

## 2. Core Methodology

### 2.1 Online IMU Noise Estimation

Instead of fixed noise parameters from the datasheet, the system estimates them online using **innovation-based adaptive estimation (IAE)**:

The innovation $\nu_k = z_k - h(\hat{x}_k^-)$ should be white noise with covariance $S_k = H_k P_k^- H_k^T + R_k$ if the filter parameters are correct. Deviations indicate mismodeled noise.

**Process noise estimation**:
$$
\hat{Q}_k = K_k \hat{S}_k K_k^T + P_k^+ - F_k P_{k-1}^+ F_k^T
$$

where $\hat{S}_k$ is the empirical innovation covariance over a window:
$$
\hat{S}_k = \frac{1}{W} \sum_{i=k-W+1}^{k} \nu_i \nu_i^T
$$

### 2.2 Per-Axis IMU Noise Estimation

The system estimates separate noise parameters for each IMU axis:
- **3-axis gyroscope noise**: $\sigma_{g_x}, \sigma_{g_y}, \sigma_{g_z}$
- **3-axis accelerometer noise**: $\sigma_{a_x}, \sigma_{a_y}, \sigma_{a_z}$
- **6-axis bias random walk**: $\sigma_{b_{gx}}, \ldots, \sigma_{b_{az}}$

This captures **axis-dependent noise**, which is common in MEMS IMUs where one axis may be noisier due to fabrication or mounting (e.g., vertical axis experiencing more vibration).

### 2.3 Robustness to Initialization

The system initializes with deliberately **conservative** (large) noise parameters:
$$
Q_0 = \alpha \cdot Q_{\text{conservative}}
$$

where $\alpha \gg 1$. This ensures the filter initially trusts LiDAR over IMU, allowing the online estimation to converge to correct values without initial divergence.

### 2.4 Convergence Monitoring

The system monitors the convergence of the noise estimates:
$$
\text{converged} = \frac{\|\hat{Q}_k - \hat{Q}_{k-1}\|}{\|\hat{Q}_{k-1}\|} < \epsilon_Q
$$

Before convergence, the system uses conservative parameters. After convergence, it uses the estimated parameters.

---

## 3. Key Equations

### Innovation-based $Q$ estimation
$$
\hat{Q}_k = \frac{1}{W} \sum_{i=k-W+1}^{k} (K_i \nu_i)(K_i \nu_i)^T + P_k^+ - F_k P_{k-1}^+ F_k^T
$$

### Per-axis noise extraction
$$
\sigma_{g_x} = \sqrt{[\hat{Q}_k]_{4,4}} / \Delta t
$$
$$
\sigma_{a_x} = \sqrt{[\hat{Q}_k]_{7,7}} / \Delta t
$$

(indices correspond to the gyro and accel noise components of the process noise matrix)

### Exponential moving average for stability
$$
\hat{Q}_k^{\text{smooth}} = (1 - \alpha) \hat{Q}_{k-1}^{\text{smooth}} + \alpha \hat{Q}_k^{\text{raw}}
$$

### Conservative initialization
$$
\sigma_{g,0} = 10 \times \sigma_{g,\text{datasheet}}, \quad \sigma_{a,0} = 10 \times \sigma_{a,\text{datasheet}}
$$

---

## 4. Assumptions

1. **Filter is observable**: The innovation-based estimation requires sufficient observability. During degeneracy, the innovations may not reflect true process noise.
2. **Noise is approximately stationary within the window**: The windowed estimation assumes noise doesn't change rapidly.
3. **LiDAR provides reliable measurements**: The innovation-based method uses LiDAR innovations as ground truth. If LiDAR is also degraded, the noise estimates are corrupted.
4. **Sufficient data for estimation**: The window $W$ must contain enough measurements for reliable covariance estimation. Small $W$ → noisy estimates; large $W$ → slow adaptation.

---

## 5. Limitations

1. **Convergence period**: During the initial convergence phase, the system uses conservative parameters, which may be suboptimal. Convergence typically requires 5-10 seconds.
2. **Coupled estimation**: IMU noise and LiDAR noise are coupled in the innovation. Separating them requires assumptions about which is better known.
3. **Degeneracy interaction**: During LiDAR degeneracy, the innovations reflect the degeneracy, not the IMU noise. The system may incorrectly increase IMU trust (reducing $Q$) when LiDAR is degenerate.
4. **No hardware modeling**: Without a physical model of the IMU, the estimated noise is purely statistical. Temperature effects, vibration modes, and other physical phenomena are captured only through their statistical signature.
5. **Window size tradeoff**: Short windows → noisy $\hat{Q}$, oscillating filter; long windows → slow adaptation to changing conditions.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable — replaces FAST-LIO2's manual IMU parameter tuning**:

- **Remove hardcoded IMU parameters**: FAST-LIO2 requires manually setting `acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov`. These can be replaced with online-estimated values.
- **Per-axis estimation**: The per-axis approach is more accurate than FAST-LIO2's isotropic noise assumption (same noise for all 3 gyro axes, same for all 3 accel axes).
- **Conservative initialization**: Start with large noise → trust LiDAR → converge to correct IMU noise. This is safer than starting with incorrect datasheet values.
- **Complementary to I2EKF-LO**: I2EKF-LO iterates the prediction; this paper provides the correct noise parameters for the prediction. Combining both gives the best result.

**Implementation**: Replace the constant $Q$ diagonal entries in FAST-LIO2's IEKF with the smoothed online estimates. Monitor convergence and fall back to conservative values during degeneracy.

---

## 7. Applicability to Livox Mid-360

- **ICM40609 IMU**: The ICM40609's noise characteristics may differ from the datasheet, especially under vibration or temperature variation. Online estimation captures the actual operating characteristics.
- **No calibration needed**: The system doesn't require Allan variance analysis or other offline IMU characterization.
- **Per-axis benefit**: If one axis of the ICM40609 is mounted differently (e.g., the vertical axis experiencing more vibration from a drone), per-axis estimation captures this.
- **Rapid deployment**: For field robotics, the system can be deployed with any IMU without sensor-specific tuning, significantly reducing setup time.
