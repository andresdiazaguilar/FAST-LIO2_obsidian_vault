# I2EKF-LO: A Dual-Iteration Extended Kalman Filter Based LiDAR Odometry

**Authors**: Yu et al., 2024, IROS  
**Category**: Degeneracy-aware (suggested paper)  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM3 (Attitude Mis-Estimation), FM7 (IMU Quality)

---

## 1. Problem Statement

Standard EKF-based LiDAR odometry (including FAST-LIO2's IEKF) uses fixed process noise covariance $Q$ and iterates the measurement update but not the prediction step. When the system dynamics are uncertain (poor IMU, varying motion profiles), the fixed $Q$ leads to suboptimal prediction, which degrades the IEKF's convergence. I2EKF-LO proposes a **dual-iteration** EKF that iterates both the prediction and the measurement update, with **adaptive process noise** estimation.

---

## 2. Core Methodology

### 2.1 Dual-Iteration Structure

The standard IEKF iterates only the measurement update:
1. **Prediction** (once): $\hat{x}^- = f(\hat{x}^+_{k-1}, u_k)$, $P^- = F P^+ F^T + Q$
2. **Update** (iterated): $\hat{x}^+_{(i+1)} = \hat{x}^- + K_{(i)}(z - h(\hat{x}^+_{(i)}))$

I2EKF-LO adds an outer iteration that also refines the prediction:

**Outer iteration** (prediction refinement):
1. Predict with current best estimate of process noise: $\hat{x}^- = f(\hat{x}^+_{k-1}, u_k, Q^{(j)})$
2. Run inner iterations (standard IEKF measurement update)
3. Update process noise estimate $Q^{(j+1)}$ based on the innovation statistics
4. Repeat outer iteration until convergence

### 2.2 Adaptive Process Noise

The process noise $Q$ is estimated from the innovation sequence and posterior residuals:

$$
Q^{(j+1)} = \frac{1}{W} \sum_{i=k-W+1}^{k} (\hat{x}_{i|i} - F \hat{x}_{i-1|i-1})(\hat{x}_{i|i} - F \hat{x}_{i-1|i-1})^T - F P_{i-1|i-1} F^T
$$

This estimates the actual process noise by comparing the filter's predictions with its posterior estimates over a window. If the IMU is noisier than assumed, $Q$ increases; if the IMU is better than assumed, $Q$ decreases.

### 2.3 Per-State Adaptation

I2EKF-LO adapts $Q$ per state component:
- **Rotation states**: $Q_R$ adapts based on gyroscope residual statistics
- **Velocity/position states**: $Q_{v,p}$ adapts based on accelerometer residual statistics
- **Bias states**: $Q_b$ adapts based on bias estimation stability

This allows different state components to have different levels of process noise adaptation.

---

## 3. Key Equations

### Dual iteration loop
Outer loop ($j = 1, \dots, J$):
$$
P^{-,(j)} = F P^+ F^T + Q^{(j)}
$$

Inner loop ($i = 1, \dots, I$) — standard IEKF:
$$
K^{(i)} = P^{-,(j)} H^{(i)T} (H^{(i)} P^{-,(j)} H^{(i)T} + R)^{-1}
$$
$$
\hat{x}^{+,(i+1)} = \hat{x}^- + K^{(i)}(z - h(\hat{x}^{+,(i)}) - H^{(i)}(\hat{x}^- - \hat{x}^{+,(i)}))
$$

### Process noise update
$$
Q^{(j+1)} = \alpha Q^{(j)} + (1-\alpha) \hat{Q}_{\text{innovation}}
$$

where $\alpha$ is a smoothing factor to prevent abrupt changes.

### Innovation-based estimation
$$
\hat{Q}_{\text{innovation}} = K \hat{S} K^T + P^+ - F P^+_{k-1} F^T
$$

where $\hat{S}$ is the estimated innovation covariance.

---

## 4. Assumptions

1. **Innovation sequence is stationary within the window**: The sliding window estimation assumes the process noise doesn't change rapidly compared to the window size.
2. **Linear dynamics approximation**: The process noise estimation uses the linearized dynamics $F$, which may be inaccurate during aggressive maneuvers.
3. **Sufficient measurements for innovation estimation**: The innovation window must contain enough measurement updates for reliable covariance estimation.

---

## 5. Limitations

1. **Computational overhead**: The outer iteration adds computational cost. With $J=2-3$ outer iterations and $I=3-4$ inner iterations, the total iterations increase 2-3×.
2. **Convergence**: The dual-iteration scheme is not guaranteed to converge. In practice, 2-3 outer iterations are sufficient.
3. **Innovation window lag**: Like AKF-LIO, the adaptation lags behind rapid changes.
4. **Positive-definiteness**: Ensuring $Q^{(j)}$ remains positive definite requires clamping.
5. **No directional degeneracy handling**: The adaptive $Q$ is a global (or per-state-component) modification, not directional. It doesn't address which LiDAR directions are degenerate.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable** — I2EKF-LO extends the IEKF that FAST-LIO2 already uses:

- **Same state vector**: No state augmentation needed.
- **Prediction iteration**: Add an outer loop around the existing IEKF iteration. After the IEKF converges, update $Q$ and re-run the prediction + IEKF.
- **Adaptive IMU covariance**: Directly addresses the covariance inflation problem in FAST-LIO2 (acc_cov: 0.1, gyr_cov: 0.1 are far from physical values).
- **During degeneracy**: When LiDAR is degenerate, the adaptive $Q$ will correctly increase (reflecting the reduced observability), preventing over-reliance on the uncertain prediction.

**Integration complexity**: Moderate. Requires modifying the IEKF loop structure and adding innovation monitoring.

---

## 7. Applicability to Livox Mid-360

- **ICM40609 IMU**: The adaptive process noise is particularly valuable for the low-cost IMU, whose noise characteristics vary with conditions.
- **Varying scan quality**: The adaptive $Q$ indirectly responds to scan quality changes (poor LiDAR → larger innovations → larger $Q$ → less trust in prediction).
- **Complements degeneracy detection**: While I2EKF-LO doesn't detect directional degeneracy, its adaptive $Q$ provides a complementary mechanism that adjusts the overall prediction-vs-measurement balance.
