# AC-LIO: LiDAR-Inertial Odometry with Selective Intra-Frame Smoothing for Deskew

**Authors**: Zhang et al., 2024  
**Category**: Motion distortion (suggested paper)  
**Failure Modes Addressed**: FM6 (Motion Distortion / Deskew Errors)

---

## 1. Problem Statement

FAST-LIO2's deskewing uses IMU-propagated poses to transform each point to a common reference time. This assumes:
1. The IMU motion model is accurate
2. The IMU data rate is high enough to capture all dynamics
3. The bias estimates are correct

When these assumptions fail (poor IMU, vibration, bias drift), the deskewing introduces systematic errors. AC-LIO proposes **asymptotic compensation (AC)** — a selective intra-frame smoothing approach that adaptively blends IMU-based deskewing with a simpler constant-velocity model based on the estimated quality of each.

---

## 2. Core Methodology

### 2.1 Dual Motion Model

AC-LIO maintains two deskewing models simultaneously:

**Model A — IMU-based** (detailed but potentially biased):
$$
T_A(\tau) = \int_{t_0}^{\tau} \begin{bmatrix} [\omega_m - b_g]_\times & a_m - b_a \\ 0 & 0 \end{bmatrix} d\tau'
$$

**Model B — Constant-velocity** (simple but robust):
$$
T_B(\tau) = T_{k-1}^{-1} T_k \cdot \frac{\tau - t_0}{t_1 - t_0}
$$

where $T_{k-1}^{-1} T_k$ is the relative pose from the previous frame, interpolated linearly.

### 2.2 Model Quality Assessment

AC-LIO assesses which model is more reliable based on:

**IMU consistency metric**:
$$
c_{\text{IMU}} = \exp\left(-\frac{\|a_m - R^T g - b_a\|^2}{2\sigma_a^2}\right) \cdot \exp\left(-\frac{\|\omega_m - b_g\|^2}{2\sigma_g^2}\right)
$$

When the accelerometer residual (after removing gravity) is large, $c_{\text{IMU}}$ is small, indicating poor IMU quality.

**Velocity consistency metric**:
$$
c_v = \exp\left(-\frac{\|v_k - v_{k-1}\|^2}{2\sigma_v^2}\right)
$$

When velocity changes rapidly (high acceleration), the constant-velocity model is poor.

### 2.3 Adaptive Blending

The deskewing transform is a blend:
$$
T_{\text{AC}}(\tau) = w_A \cdot T_A(\tau) + w_B \cdot T_B(\tau)
$$

where the blending is done on the Lie algebra:
$$
\xi_{\text{AC}}(\tau) = w_A \cdot \log(T_A(\tau)) + w_B \cdot \log(T_B(\tau))
$$
$$
T_{\text{AC}}(\tau) = \exp(\xi_{\text{AC}}(\tau))
$$

Weights:
$$
w_A = \frac{c_{\text{IMU}}}{c_{\text{IMU}} + c_v}, \quad w_B = \frac{c_v}{c_{\text{IMU}} + c_v}
$$

### 2.4 Selective Smoothing

Additionally, AC-LIO applies **temporal smoothing** within the frame for the selected model:

$$
\xi_{\text{smooth}}(\tau) = \sum_{i} K(\tau, \tau_i) \xi(\tau_i)
$$

where $K$ is a smoothing kernel. This reduces high-frequency noise in the deskewing trajectory without removing the genuine motion signal.

---

## 3. Key Equations

### IMU consistency (accelerometer)
$$
c_a = \exp\left(-\frac{\sum_{i=1}^{N_{\text{IMU}}} (\|a_i\| - \|g\|)^2}{N_{\text{IMU}} \sigma_a^2}\right)
$$

When $\|a_i\| \approx \|g\|$ consistently, the IMU is measuring only gravity (stationary or smooth motion), and the accelerometer is reliable.

### Constant-velocity deskewing
$$
T_B(\tau_j) = \exp\left(\frac{\tau_j - t_{\text{start}}}{t_{\text{end}} - t_{\text{start}}} \cdot \log(T_{k-1}^{-1} T_k)\right)
$$

### Adaptive weight
$$
w_A = \text{softmax}(\beta \cdot c_{\text{IMU}}), \quad w_B = \text{softmax}(\beta \cdot c_v)
$$

where $\beta$ is a temperature parameter controlling the sharpness of the blending.

---

## 4. Assumptions

1. **One of the two models is approximately correct**: If both the IMU and constant-velocity models are wrong (e.g., sudden impact), the blending doesn't help.
2. **Smooth motion within a frame**: The constant-velocity model assumes smooth motion over one frame (~100ms). For very aggressive motion, even this is violated.
3. **IMU metrics are reliable indicators**: The consistency metrics must correctly identify when the IMU is unreliable. Correlated noise or systematic errors may not be detected.

---

## 5. Limitations

1. **Lie algebra interpolation**: Blending on the Lie algebra is an approximation. For large rotations, the interpolation may not produce valid poses.
2. **Constant-velocity baseline**: The fallback model is simple. A constant-acceleration model would be a better alternative but requires second-order velocity estimation.
3. **Smoothing kernel design**: The temporal smoothing kernel must balance noise reduction with signal preservation. Too much smoothing removes genuine high-frequency motion.
4. **Overhead**: Computing two deskewing models and blending adds computation compared to single-model deskewing.
5. **Previous frame quality**: The constant-velocity model depends on the previous frame's pose estimate. If the previous frame was poorly estimated, the fallback is also corrupted.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable as a deskewing enhancement**:

- **Replace simple IMU deskewing**: FAST-LIO2's deskewing is purely IMU-based. Replace with the adaptive blending of IMU + constant-velocity.
- **IMU consistency monitoring**: The consistency metric $c_{\text{IMU}}$ can also trigger other protective actions (increase $Q$, reduce map insertion rate).
- **No IEKF modification needed**: The deskewing is a pre-processing step before the IEKF. The improved deskewed point cloud feeds into the standard IEKF pipeline.

**Highest-value scenario**: When the ICM40609 IMU has vibration-induced noise or bias drift, the constant-velocity fallback provides a more stable deskewing reference.

---

## 7. Applicability to Livox Mid-360

- **Per-point timestamps**: The Livox provides microsecond-accurate per-point timestamps, enabling precise deskewing regardless of the motion model used.
- **100ms scan period**: At 10 Hz, each scan spans 100ms. The constant-velocity assumption is reasonable at typical motion speeds but breaks during sharp turns.
- **ICM40609 noise**: The low-cost IMU is the primary motivation for adaptive deskewing. When the IMU noise increases (vibration, electromagnetic interference), the system automatically reduces IMU trust for deskewing.
- **Complementary to other improvements**: AC-LIO's deskewing improvement is orthogonal to degeneracy detection, robust correspondences, etc. It can be combined with all other modifications.
