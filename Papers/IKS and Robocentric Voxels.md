# Fast and Robust LiDAR-Inertial Odometry by Tightly-Coupled Iterated Kalman Smoother and Robocentric Voxels

**Category**: Hand-held  
**Failure Modes Addressed**: FM6 (Motion Distortion / Deskew Errors)

---

## 1. Problem Statement

FAST-LIO2's IEKF is a **filter** — it estimates the current state using only past and current data. An **Iterated Kalman Smoother (IKS)** can use a short window of future data to refine the estimate, improving deskewing quality by using both past and (near-)future IMU data within the scan window.

---

## 2. Core Methodology

### 2.1 Iterated Kalman Smoother (IKS)

The IKS extends the IEKF by performing **smoothing** within a fixed-lag window:

**Forward pass (filter)**:
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k(z_k - h(\hat{x}_{k|k-1}))
$$

**Backward pass (smoother)**:
$$
\hat{x}_{k|N} = \hat{x}_{k|k} + G_k(\hat{x}_{k+1|N} - \hat{x}_{k+1|k})
$$

where:
- $G_k = P_{k|k} F_{k+1}^T P_{k+1|k}^{-1}$ is the smoother gain
- $\hat{x}_{k|N}$ is the smoothed estimate using all $N$ measurements in the window

For deskewing, the IKS provides smoothed intra-scan poses by using IMU data from both before and after each point's timestamp within the scan window.

### 2.2 Smoothing for Deskewing

In the IEKF (FAST-LIO2), deskewing uses **forward-only** IMU propagation:
$$
T(t_j) = T(t_{\text{start}}) \cdot \int_{t_{\text{start}}}^{t_j} \omega(s) ds, \quad p(t_j) = p(t_{\text{start}}) + \int_{t_{\text{start}}}^{t_j} v(s) ds
$$

This only uses IMU data up to time $t_j$. The IKS uses all IMU data within the scan window $[t_{\text{start}}, t_{\text{end}}]$ to estimate $T(t_j)$ for any $t_j$ in that interval:

$$
T_{\text{smoothed}}(t_j) = T_{\text{forward}}(t_j) + G(t_j) \cdot (T_{\text{end}} - T_{\text{forward}}(t_{\text{end}}))
$$

The backward pass corrects the forward propagation using the end-of-scan information.

### 2.3 Robocentric Voxels

Instead of maintaining the map in a fixed world frame, the paper proposes **robocentric voxels**:
- The voxel map is defined relative to the current robot pose
- As the robot moves, the voxel grid shifts with it
- This reduces numerical precision issues for long trajectories
- Map points are represented in the robot-centered frame

Benefits:
- Better numerical conditioning (all coordinates are near-zero)
- Natural sliding window behavior (old voxels outside the range are dropped)
- Reduced drift accumulation in the map representation

---

## 3. Key Equations

### Rauch-Tung-Striebel (RTS) smoother gain
$$
G_k = P_{k|k} F_k^T P_{k+1|k}^{-1}
$$

### Smoothed state estimate
$$
\hat{x}_{k|N} = \hat{x}_{k|k} + G_k(\hat{x}_{k+1|N} - F_k \hat{x}_{k|k})
$$

### Smoothed covariance
$$
P_{k|N} = P_{k|k} + G_k(P_{k+1|N} - P_{k+1|k})G_k^T
$$

### Robocentric coordinate transform
$$
p_j^{\text{robo}} = R_k^T (p_j^{\text{world}} - t_k)
$$

All map points are stored relative to the current pose, updated at each step.

---

## 4. Assumptions

1. **Fixed-lag smoothing window**: The smoothing window is fixed (typically one scan interval). Longer windows provide better smoothing but increase latency and computation.
2. **Linear dynamics within window**: The RTS smoother assumes linear state dynamics within the window, which is valid for short windows with the linearized IMU model.
3. **Batch processing within scan**: All points in a scan are processed together after the scan is complete, adding one scan-interval of latency.

---

## 5. Limitations

1. **Added latency**: The smoother requires future data (the full scan + some look-ahead IMU). This adds at least one scan interval (~100ms) of latency compared to the IEKF's instantaneous estimate.
2. **Computational overhead**: The backward pass doubles the computation of the forward pass (approximately). For FAST-LIO2's tight budget, this may be marginal.
3. **Limited benefit for slow motion**: During slow, smooth motion, the difference between filtered and smoothed estimates is negligible. The benefit is concentrated during aggressive motion.
4. **Robocentric map maintenance**: Shifting the voxel grid with the robot requires re-computing relative coordinates for all map points at each step. With a large ikd-Tree, this is expensive. In practice, the paper uses separate voxel structures rather than the ikd-Tree.
5. **Incompatible with ikd-Tree**: The ikd-Tree in FAST-LIO2 stores points in the world frame. Switching to robocentric voxels would require replacing the ikd-Tree entirely.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Partially applicable — smoothing concept is useful, but implementation requires changes**:

- **IKS within scan interval**: The IEKF could be extended to perform a backward smoothing pass within each scan interval, improving deskewing. This is a localized modification.
- **Implementation**: After the standard IEKF update at scan end, run a backward pass through the intra-scan IMU data to produce smoothed intra-scan poses. Re-deskew the scan using these smoothed poses and re-run the IEKF update with the improved deskewing.
- **Latency tradeoff**: The smoothed deskewing is one scan behind. For real-time navigation, the filtered (non-smoothed) estimate can be used for control, while the smoothed estimate updates the map.

**Robocentric voxels**: Would require replacing the ikd-Tree with a robocentric voxel structure. This is a significant change but could provide numerical benefits for long trajectories.

---

## 7. Applicability to Livox Mid-360

- **Deskewing improvement**: The Mid-360's 100ms scan window benefits from smoothed deskewing, especially during hand-held operation with aggressive motion.
- **Point timestamps**: The per-point timestamps from the Mid-360 enable accurate intra-scan smoothing.
- **Non-repetitive pattern**: Points in different spatial regions within a scan may have very different deskewing requirements (depending on their direction relative to the motion). Smoothing provides better estimates for all points.
- **Latency concern**: For mapping applications, 100ms latency is typically acceptable. For real-time control, the filtered estimate can be used in parallel.
