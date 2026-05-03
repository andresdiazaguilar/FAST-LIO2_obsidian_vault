# KISS-ICP: In Defense of Point-to-Point ICP — Simple, Efficient, and Robust Registration

**Authors**: Vizzo et al., 2023, RA-L  
**Category**: Correspondence errors (suggested paper)  
**Failure Modes Addressed**: FM5 (Correspondence Errors)

---

## 1. Problem Statement

Modern LiDAR odometry systems use complex feature extraction and matching pipelines. KISS-ICP argues that a well-engineered point-to-point ICP with **motion-adaptive correspondence thresholds** and **robust rejection** can match or exceed the performance of more complex systems, while being simpler and more generalizable.

---

## 2. Core Methodology

### 2.1 Adaptive Correspondence Threshold

The key innovation is a motion-adaptive threshold for correspondence rejection:

$$
\tau_k = 3 \sigma_k
$$

where $\sigma_k$ is estimated from the current motion uncertainty:

$$
\sigma_k^2 = \sigma_{\text{odom}}^2 + \sigma_{\text{map}}^2
$$

- $\sigma_{\text{odom}}^2 \propto \|v_k\| \Delta t$: Uncertainty from odometry prediction (larger for faster motion)
- $\sigma_{\text{map}}^2$: Uncertainty from map resolution (voxel size)

This means:
- **Fast motion**: Larger threshold → accepts more correspondences (valid points may be further from predicted positions)
- **Slow motion**: Smaller threshold → rejects outliers more aggressively

### 2.2 Point-to-Point ICP

KISS-ICP uses simple point-to-point registration:

$$
T^* = \arg\min_T \sum_{j=1}^{N} \|T p_j - q_j\|^2
$$

This has a **closed-form solution** via SVD (no iterative optimization needed per step):

$$
T^* = \arg\min \|P - T Q\|_F
$$

The closed-form solution is faster and more numerically stable than iterative point-to-plane optimization.

### 2.3 Robust Kernel

KISS-ICP applies a robust kernel to handle outlier correspondences:

$$
E(T) = \sum_j \rho(\|T p_j - q_j\|, \tau_k)
$$

with:
$$
\rho(d, \tau) = \begin{cases}
d^2 & d \leq \tau \\
\tau^2 & d > \tau \\
\end{cases}
$$

This is a **truncated least squares** — correspondences beyond the threshold contribute a constant cost (effectively ignored).

### 2.4 Voxel-Based Downsampling with Local Map

- Voxel grid downsampling of each scan
- Local map maintained as a hash-based voxel grid (similar to Faster-LIO's iVox)
- Points older than $N$ scans are removed (temporal sliding window)

---

## 3. Key Equations

### Adaptive threshold
$$
\tau_k = \max\left(\sigma_{\min}, 3\sqrt{(\|v_{k-1}\| \cdot \Delta t)^2 + s_{\text{voxel}}^2}\right)
$$

where $s_{\text{voxel}}$ is the voxel size and $\sigma_{\min}$ is a minimum threshold.

### Point-to-point registration (SVD solution)
Given correspondences $(p_j, q_j)$:
$$
H = \sum_j (p_j - \bar{p})(q_j - \bar{q})^T
$$
$$
[U, \Sigma, V] = \text{SVD}(H)
$$
$$
R^* = V U^T, \quad t^* = \bar{q} - R^* \bar{p}
$$

### Velocity estimation
$$
v_k = \frac{t_k - t_{k-1}}{\Delta t}
$$

---

## 4. Assumptions

1. **Point-to-point is sufficient**: Assumes the environment has enough 3D structure for point-to-point registration. In planar environments, point-to-point may perform worse than point-to-plane.
2. **Velocity is approximately constant**: The adaptive threshold uses the previous velocity estimate. During rapid acceleration/deceleration, this may be inaccurate.
3. **No IMU integration**: KISS-ICP is LiDAR-only. The adaptive threshold uses odometry velocity, not IMU data.

---

## 5. Limitations

1. **LiDAR-only**: No IMU integration. The adaptive threshold concept is useful, but the full system lacks tightly-coupled inertial aiding.
2. **Point-to-point registration is noisier**: Point-to-point is less accurate than point-to-plane for planar environments. FAST-LIO2's point-to-plane provides better accuracy in structured environments.
3. **No filter-based estimation**: KISS-ICP uses frame-to-frame registration without a filter. No state estimation, no covariance propagation.
4. **Closed-form SVD doesn't iterate**: While efficient, the single-step SVD solution doesn't iterate towards better correspondences like ICP typically does (KISS-ICP relies on good initialization from the previous frame).

---

## 6. Applicability to FAST-LIO2 / IEKF

**The adaptive threshold concept is directly applicable**:

- **Replace Gate 2 with motion-adaptive threshold**: FAST-LIO2's heuristic Gate 2 ($s = 1 - 0.9|p_{d2}|/\sqrt{\|p_{\text{body}}\|} > 0.9$) can be replaced with KISS-ICP's velocity-dependent threshold:
  $$
  \tau_j = 3\sqrt{(\|v\| \Delta t)^2 + s_{\text{voxel}}^2}
  $$
  This is simpler, more principled, and adapts to motion speed.

- **Truncated least squares in IEKF**: The truncated loss function can be implemented in the IEKF by zeroing out residuals (and corresponding Jacobian rows) that exceed the threshold. This is equivalent to removing outlier correspondences after each IEKF iteration.

- **Not replacing the full system**: FAST-LIO2's IEKF with point-to-plane is more accurate than KISS-ICP's point-to-point. The value is in adopting the adaptive threshold, not the registration method.

**Most valuable component**: The motion-adaptive correspondence threshold. This directly addresses D²-LIO's observation that FAST-LIO2's fixed gating is motion-unaware.

---

## 7. Applicability to Livox Mid-360

- **Velocity-adaptive threshold**: Particularly valuable for hand-held operation where motion speed varies dramatically.
- **Voxel-based map**: Similar to Faster-LIO's iVox, providing fast correspondence search.
- **Point-to-point as fallback**: When point-to-plane fails (insufficient planar features), point-to-point correspondences with KISS-ICP's adaptive threshold could serve as a fallback.
- **Simple and robust**: The simplicity of the approach makes it easy to integrate as a component within FAST-LIO2.
