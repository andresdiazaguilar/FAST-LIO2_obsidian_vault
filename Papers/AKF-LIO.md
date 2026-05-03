# AKF-LIO: LiDAR-Inertial Odometry with Gaussian Map by Adaptive Kalman Filter

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM3 (Attitude Mis-Estimation), FM7 (IMU Quality)

---

## 1. Problem Statement

FAST-LIO2's IEKF uses **fixed process and measurement noise covariances** ($Q$ and $R$), which are manually tuned and remain constant regardless of the current estimation quality, sensor conditions, or environmental complexity. This leads to:
- **Over-trusting LiDAR** when correspondences are poor (fixed $R$ too low)
- **Over-trusting IMU** when biases are poorly estimated (fixed $Q$ too low)
- **Suboptimal fusion** that doesn't adapt to changing conditions

AKF-LIO proposes an **adaptive Kalman filter** that online-tunes both process and measurement noise covariances.

---

## 2. Core Methodology

### 2.1 Adaptive Process Noise ($Q$)

The process noise covariance $Q$ governs how much the filter trusts the IMU prediction. AKF-LIO estimates $Q$ online using the **innovation sequence**:

The innovation is:
$$
\nu_k = z_k - h(\hat{x}_k^-)
$$

The theoretical innovation covariance is:
$$
S_k = H_k P_k^- H_k^T + R_k
$$

The **actual innovation covariance** can be estimated from a sliding window of innovations:

$$
\hat{S}_k = \frac{1}{W} \sum_{i=k-W+1}^{k} \nu_i \nu_i^T
$$

If $\hat{S}_k > S_k$ (innovations are larger than expected), the process noise is too small → increase $Q$.
If $\hat{S}_k < S_k$ (innovations are smaller than expected), the process noise is too large → decrease $Q$.

### 2.2 Adaptive Measurement Noise ($R$)

Similarly, $R$ is adapted based on the residual statistics:

$$
\hat{R}_k = \frac{1}{N} \sum_{j=1}^{N} r_j^2 - H_k P_k^- H_k^T
$$

where $r_j$ are the point-to-plane residuals. If the residuals are systematically larger than the predicted measurement uncertainty, $R$ is increased.

### 2.3 Covariance Matching (Myers-Tapley)

The specific adaptation method is based on **covariance matching**:

$$
Q_k = K_k \hat{C}_\nu K_k^T
$$

where $\hat{C}_\nu$ is the estimated innovation covariance and $K_k$ is the Kalman gain. This ensures the filter's internal model matches the observed statistics.

### 2.4 Gaussian Map Representation

AKF-LIO also uses a Gaussian map (representing each map element as a Gaussian distribution) rather than raw points, enabling:
- Natural uncertainty representation for map points
- Better correspondence matching using Mahalanobis distance
- Implicit outlier handling through the Gaussian likelihood

---

## 3. Key Equations

### Innovation-based $Q$ adaptation
$$
Q_k = K_k \left(\frac{1}{W} \sum_{i=k-W+1}^{k} \nu_i \nu_i^T\right) K_k^T + P_k^+ - F_k P_{k-1}^+ F_k^T
$$

### Residual-based $R$ adaptation
$$
R_k = \frac{1}{N} \sum_{j=1}^{N} (z_j - h(\hat{x}_k^-)) (z_j - h(\hat{x}_k^-))^T - H_k P_k^- H_k^T
$$

### Positive-definiteness enforcement
$$
Q_k \leftarrow \frac{1}{2}(Q_k + Q_k^T) + \epsilon I
$$

---

## 4. Assumptions

1. **Ergodicity of innovation sequence**: The sliding window average converges to the true innovation covariance. Requires stationary conditions within the window.
2. **Linear approximation validity**: The covariance matching assumes the extended Kalman filter's linear approximation is sufficiently accurate.
3. **Correct model structure**: The adaptation tunes parameters of the correct model. If the model structure is wrong (e.g., missing states), adaptation cannot compensate.
4. **Gaussian noise**: Both process and measurement noise are assumed Gaussian.

---

## 5. Limitations

1. **Adaptation lag**: The sliding window introduces delay — the covariance estimates reflect conditions from $W$ steps ago. During rapid transitions (entering a degenerate corridor), the adaptation may be too slow.
2. **Window size tradeoff**: Large $W$ → stable estimates but slow adaptation. Small $W$ → fast adaptation but noisy estimates. Typical $W$ = 20-50 scans.
3. **Doesn't prevent bad updates**: Adaptive $R$ can reduce the weight of bad LiDAR updates, but it cannot entirely prevent a corrupted update from entering the state. It's reactive, not preventive.
4. **No directional awareness**: The adaptation modifies scalar (or diagonal) noise parameters, not per-direction noise. It doesn't detect directional degeneracy.
5. **Positive-definiteness**: Ensuring $Q_k$ and $R_k$ remain positive definite requires clamping, which can distort the adaptation.
6. **Interaction with IEKF iterations**: FAST-LIO2's IEKF iterates multiple times per scan. The adaptation should use post-convergence residuals, not intermediate ones.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable** — AKF-LIO is designed for KF-based LIO systems:

- **Drop-in for covariance tuning**: Replaces FAST-LIO2's fixed `acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov` with online-adapted values.
- **Addresses the covariance inflation problem**: FAST-LIO2's default parameters are intentionally inflated beyond physical values (acc_cov = 0.1 vs. actual ~0.01). Adaptive tuning can start from physical values and let the filter learn the effective noise level.
- **Innovation monitoring**: The innovation-based $Q$ estimation provides a diagnostic signal for filter health — diverging innovations indicate system problems.

**Integration complexity**: Low. The main changes are:
1. Add innovation accumulation (sliding window buffer)
2. Compute $\hat{Q}_k$ and $\hat{R}_k$ at each scan
3. Use adapted values in the Kalman gain computation

**Key concern**: The IEKF performs multiple iterations per scan. The adaptation should use the final-iteration residuals, and the adapted $R$ should be applied starting from the next scan (not the current one).

---

## 7. Applicability to Livox Mid-360

- **ICM40609 IMU adaptation**: The adaptive $Q$ is particularly valuable for the low-cost ICM40609 IMU, whose noise characteristics may vary with temperature, vibration, and bias drift rate.
- **Variable scan quality**: The Livox's non-repetitive pattern produces scans of varying geometric quality. Adaptive $R$ would naturally increase measurement noise during poorly-conditioned scans.
- **Complementary to degeneracy detection**: AKF-LIO's adaptive noise is a scalar/diagonal modification, not directional. Combining it with D²-LIO-style directional degeneracy detection would provide both global adaptation and directional mitigation.
