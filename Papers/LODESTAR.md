# LODESTAR: Degeneracy-Aware LiDAR-Inertial Odometry with Adaptive Schmidt-Kalman Filter and Data Exploitation

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM7 (IMU Quality)

---

## 1. Problem Statement

During geometric degeneracy, standard IEKF-based LIO systems face a dilemma: they can either (a) blindly apply the Kalman update (corrupting degenerate directions), (b) zero out the update along degenerate directions (fully trusting IMU), or (c) shut down the LiDAR update entirely. LODESTAR proposes option (d): use a **Schmidt-Kalman filter (SKF)** to "consider" (acknowledge without updating) degenerate state components while still updating well-conditioned components, combined with improved **IMU data exploitation** during degraded periods.

---

## 2. Core Methodology

### 2.1 Schmidt-Kalman Filter for Degeneracy

The Schmidt-Kalman filter (also called the consider filter) partitions the state into **solve** and **consider** components:

$$
x = \begin{bmatrix} x_s \\ x_c \end{bmatrix}
$$

- **Solve states** ($x_s$): States that are well-conditioned and should be updated by the measurement.
- **Consider states** ($x_c$): States that are poorly conditioned (degenerate). Their uncertainty is **considered** (propagated) in the covariance but they are **not updated** by the measurement.

The Kalman gain is modified:

$$
K_{\text{SKF}} = \begin{bmatrix} K_s \\ 0 \end{bmatrix}
$$

where only $K_s$ is non-zero. The state update is:

$$
\hat{x}^+ = \hat{x}^- + K_{\text{SKF}} (z - h(\hat{x}^-))
$$

Critically, the **covariance update** accounts for the consider states:

$$
P^+ = \begin{bmatrix} P_{ss}^+ & P_{sc}^- \\ P_{cs}^- & P_{cc}^- \end{bmatrix}
$$

where:
$$
P_{ss}^+ = (I - K_s H_s) P_{ss}^- (I - K_s H_s)^T + K_s R K_s^T
$$

The cross-covariance $P_{sc}$ is preserved, ensuring that the uncertainty coupling between solve and consider states is maintained. This is the key advantage over simply zeroing out the Kalman gain — the covariance remains consistent.

### 2.2 Degeneracy Detection for State Partitioning

LODESTAR determines which states are degenerate by analyzing the eigenvalues of the information matrix, similar to D²-LIO. States corresponding to degenerate eigenvectors are moved to the consider partition.

### 2.3 IMU Data Exploitation

During LiDAR degradation, LODESTAR more carefully exploits IMU data:

1. **Gravity constraint exploitation**: When the platform is approximately static or in constant-velocity motion, the accelerometer reading is dominated by gravity. LODESTAR uses this to constrain roll and pitch even when LiDAR is degraded.

2. **Pseudo-measurements from IMU**: Generates pseudo-measurements of velocity (from integrated accelerometer with gravity subtraction) and uses them to prevent velocity drift during LiDAR degradation.

3. **Adaptive IMU trust**: Adjusts the process noise covariance based on the detected level of LiDAR degradation — when LiDAR is more degraded, the IMU process noise is tightened (assuming the IMU data is relatively trustworthy over short periods).

---

## 3. Key Equations

### Schmidt-Kalman gain
$$
K_{\text{SKF}} = P^- H^T_s (H_s P^-_{ss} H_s^T + R)^{-1} \begin{bmatrix} I \\ 0 \end{bmatrix}
$$

### Covariance propagation with consider states
$$
P^+_{ss} = P^-_{ss} - K_s (H_s P^-_{ss} H_s^T + R) K_s^T
$$
$$
P^+_{cc} = P^-_{cc} \quad \text{(unchanged)}
$$
$$
P^+_{sc} = (I - K_s H_s) P^-_{sc}
$$

### Gravity pseudo-measurement
When near-static ($\|a_m - R^T g - b_a\| < \epsilon$):
$$
z_g = a_m - b_a \approx R^T g
$$

This provides a measurement of $R$ (specifically roll and pitch).

---

## 4. Assumptions

1. **Degeneracy is identifiable**: The eigenvalue analysis can reliably distinguish degenerate from well-conditioned states.
2. **IMU quality is sufficient for short-term dead reckoning**: The data exploitation component assumes the IMU can provide useful information during LiDAR degradation.
3. **State partitioning is correct**: Misclassifying a well-conditioned state as "consider" wastes information; misclassifying a degenerate state as "solve" corrupts the estimate.
4. **Near-static periods exist**: The gravity exploitation requires periods of low acceleration, which may not always occur during hand-held operation.

---

## 5. Limitations

1. **Computational overhead**: The SKF requires maintaining and propagating cross-covariance blocks, increasing the computational cost compared to a standard IEKF. For FAST-LIO2's 24-dimensional state, the covariance matrix is already $24 \times 24$; the SKF doesn't increase its dimension but does require more careful bookkeeping.
2. **Threshold sensitivity**: Like D²-LIO, the degeneracy detection relies on eigenvalue thresholds. The transition between "solve" and "consider" for each state is binary.
3. **Limited IMU exploitation during aggressive motion**: The gravity constraint only works during low-dynamics periods. During aggressive hand-held motion, the accelerometer reading is dominated by platform acceleration, not gravity.
4. **No IMU bias improvement**: While the SKF preserves uncertainty, it doesn't actively improve IMU bias estimates during degradation — it just prevents corruption.
5. **Validated primarily on spinning LiDARs**: Solid-state LiDAR validation is limited.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Highly applicable** — the Schmidt-Kalman filter is a natural extension of the IEKF:

- **Same state vector**: The SKF operates on the same state vector as FAST-LIO2's IEKF. No state augmentation is needed.
- **State partitioning in IEKF**: At each IEKF iteration, the eigenvalue analysis determines which of the 6 pose states (3 translation, 3 rotation) are well-conditioned. Well-conditioned states are "solve"; degenerate states are "consider."
- **Covariance consistency**: Unlike D²-LIO's regularization (which can make the covariance inconsistent by zeroing out Kalman gain components without adjusting the covariance), the SKF maintains covariance consistency automatically.
- **IMU data exploitation**: The pseudo-measurements can be added as additional measurement rows in the IEKF, with appropriate noise covariances.

**Integration complexity**: Moderate. The main challenge is implementing the state partitioning logic and modifying the Kalman gain computation to zero out consider states while preserving cross-covariances.

---

## 7. Applicability to Livox Mid-360

- **Degeneracy detection**: Fully applicable — eigenvalue analysis of the information matrix is scan-pattern agnostic.
- **Schmidt-Kalman mechanism**: Fully applicable — independent of LiDAR type.
- **IMU data exploitation**: The Livox Mid-360's integrated ICM40609 IMU has limited quality, so the effectiveness of gravity constraints and pseudo-measurements depends on the IMU noise floor. During truly static periods, even the ICM40609 should provide useful gravity direction.
- **Solid-state challenges**: The SKF doesn't directly address the Livox-specific issues (non-repetitive scan, irregular density), but it provides a more graceful degradation when those issues cause degeneracy.

---

## 8. Key Insight

The fundamental advantage of the SKF over D²-LIO's regularization is **covariance consistency**. When D²-LIO zeroes out the Kalman gain along degenerate directions, the standard Joseph-form covariance update still shrinks the posterior covariance as if a full update was applied, creating an inconsistently overconfident estimate. The SKF avoids this by explicitly not updating the consider states' covariance.
