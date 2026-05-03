# LIO-EKF: High-Frequency LiDAR-Inertial Odometry Using EKF with Sequential Processing

**Authors**: Wu et al., 2024, ICRA  
**Category**: Filter-based LIO (suggested paper)  
**Failure Modes Addressed**: FM7 (IMU Quality Limitations)

---

## 1. Problem Statement

FAST-LIO2 processes LiDAR scans at the scan rate (typically 10 Hz), accumulating all IMU measurements between scans into a single prediction step. This means:
1. The IEKF update occurs only 10× per second
2. Between updates, the state drifts with only IMU-based prediction
3. IMU bias estimation is limited by the update rate

LIO-EKF proposes **sequential per-point processing**, where each LiDAR point is processed individually, enabling effectively continuous LiDAR updates at the point rate.

---

## 2. Core Methodology

### 2.1 Sequential Point Processing

Instead of accumulating a full scan and performing a batch IEKF update, LIO-EKF processes each point as it arrives:

For each point $p_j$ at time $\tau_j$:
1. **Propagate** the state from $\tau_{j-1}$ to $\tau_j$ using IMU measurements
2. **Update** the state with the single point's measurement
3. Move to the next point

This gives an update rate equal to the point rate (~200 kHz for the Mid-360), rather than the scan rate (10 Hz).

### 2.2 Point-Level EKF Update

Each point provides a scalar measurement (point-to-plane distance):
$$
z_j = n_j^T (R(\tau_j) p_j^B + t(\tau_j) - q_j)
$$

The EKF update for a single scalar measurement is extremely efficient:
$$
K_j = P^- h_j^T / (h_j P^- h_j^T + r_j) \quad \text{(scalar division)}
$$
$$
\hat{x}^+ = \hat{x}^- + K_j (z_j - h_j \hat{x}^-)
$$
$$
P^+ = (I - K_j h_j) P^-
$$

where $h_j$ is the measurement Jacobian row vector and $r_j$ is the scalar measurement noise.

### 2.3 Implicit Deskewing

A major advantage of sequential processing is that **deskewing is implicit**. Each point is processed at its own timestamp, so no explicit deskewing transform is needed. The state is already at the correct time $\tau_j$ when point $p_j$ is processed.

### 2.4 High-Frequency Bias Estimation

With per-point updates, the IMU bias estimates are corrected ~200K times per second instead of 10 times. This enables:
- **Faster bias convergence**: New bias values are learned orders of magnitude faster
- **Better bias tracking**: Rapid bias changes (thermal drift, vibration-induced bias) are tracked in near-real-time
- **Reduced IMU integration error**: Between any two points, the IMU integration interval is microseconds, so integration errors are negligible

---

## 3. Key Equations

### Per-point propagation (between points $j-1$ and $j$)
$$
\hat{x}(\tau_j) = f(\hat{x}(\tau_{j-1}), u[\tau_{j-1}, \tau_j])
$$

For the short interval $\Delta \tau = \tau_j - \tau_{j-1} \approx 5\mu s$:
$$
R(\tau_j) \approx R(\tau_{j-1}) \exp((\omega_m - b_g) \Delta\tau)
$$
$$
v(\tau_j) \approx v(\tau_{j-1}) + (R(\tau_{j-1})(a_m - b_a) + g) \Delta\tau
$$
$$
p(\tau_j) \approx p(\tau_{j-1}) + v(\tau_{j-1}) \Delta\tau
$$

### Scalar Kalman gain
$$
k_j = \frac{P^- h_j^T}{h_j P^- h_j^T + r_j} \in \mathbb{R}^{24 \times 1}
$$

### Sequential covariance update (Joseph form for stability)
$$
P^+ = (I - K_j h_j) P^- (I - K_j h_j)^T + K_j r_j K_j^T
$$

### Effective update rate
$$
f_{\text{update}} = N_{\text{points}} \cdot f_{\text{scan}} \approx 20000 \times 10 = 200\text{kHz}
$$

---

## 4. Assumptions

1. **Point ordering by timestamp**: Points must be processed in temporal order. The Livox provides per-point timestamps, enabling this.
2. **IMU at sufficient rate**: The IMU rate must be at least as high as the point rate, or interpolation is needed. At 200 Hz IMU vs 200 kHz points, each IMU measurement covers ~1000 points.
3. **Independent measurements**: Each point is treated as independent. Spatial correlations between nearby points are ignored.
4. **Static environment during scan**: Like batch processing, the environment must be static during the scan period.

---

## 5. Limitations

1. **Computational ordering**: Points must be processed sequentially (no parallelism). For $N$ points, the update requires $N$ sequential matrix operations. With $P \in \mathbb{R}^{24 \times 24}$, each update is $O(24^2)$, giving $O(N \cdot 576)$ operations per scan.
2. **Information vs. iteration**: The sequential EKF applies each measurement once. The IEKF iterates, re-linearizing at better estimates. For nonlinear measurements, the IEKF may converge to a better solution than the sequential EKF.
3. **No re-linearization**: Unlike the IEKF, the sequential EKF doesn't re-linearize. For large prediction errors, the first few points update with poor Jacobians.
4. **Point quality**: Low-quality points (noise, dynamic objects) corrupt the estimate immediately, with no opportunity for batch outlier rejection.
5. **IMU interpolation**: With IMU at 200 Hz and points at 200 kHz, the same IMU measurement is used for ~1000 consecutive points. This is equivalent to assuming constant angular velocity and acceleration over 5ms intervals.

---

## 6. Applicability to FAST-LIO2 / IEKF

**An alternative approach to FAST-LIO2's batch IEKF**:

- **Eliminates deskewing**: Sequential processing inherently handles motion distortion. This removes FM6 entirely.
- **Faster bias estimation**: Addresses FM7 by updating bias estimates at the point rate.
- **Different tradeoffs**: Sequential EKF trades iteration (IEKF's strength) for frequency (per-point updates). In well-behaved environments, the sequential approach may be sufficient. In challenging environments with large initial errors, the IEKF's iteration may be necessary.

**Hybrid approach**: Process points sequentially for the first pass (getting a rough estimate), then use the result as initialization for one IEKF iteration over the full scan. This combines both approaches.

**Implementation consideration**: FAST-LIO2's ikd-Tree kNN search is the bottleneck. For sequential processing, each point requires a separate kNN query, which is the same total kNN work as batch processing but without batch parallelism.

---

## 7. Applicability to Livox Mid-360

- **Per-point timestamps**: The Mid-360 provides per-point timestamps with ~1μs accuracy, perfectly suited for sequential processing.
- **~200K points/sec**: The point rate determines the update rate. With ~20K points per 100ms scan, updates are every ~5μs.
- **ICM40609 at 200 Hz**: With 200 Hz IMU and ~200 kHz point rate, IMU interpolation is needed. The constant assumption over 5ms intervals is reasonable for the ICM40609's noise level.
- **High-frequency bias correction**: The sequential approach's rapid bias estimation directly compensates for the ICM40609's bias drift.
- **Best combined with outlier rejection**: Add per-point outlier tests (Mahalanobis gating) to prevent individual bad points from corrupting the estimate.
