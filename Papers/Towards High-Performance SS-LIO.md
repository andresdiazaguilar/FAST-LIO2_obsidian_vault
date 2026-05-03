# Towards High-Performance Solid-State-LiDAR-Inertial Odometry and Mapping

**Category**: Solid-state LiDARs  
**Failure Modes Addressed**: FM9 (Solid-State LiDAR Challenges)

---

## 1. Problem Statement

This paper presents a comprehensive LiDAR-inertial odometry system specifically designed for solid-state LiDARs, addressing the unique challenges of non-repetitive scanning, small FoV, and irregular point distribution.

---

## 2. Core Methodology

### 2.1 Scan Processing for Non-Repetitive Patterns

**Adaptive scan accumulation**: Rather than processing individual scans, the system accumulates scans until a minimum point density criterion is met:

$$
\text{accumulate until: } \min_{v \in \text{active\_voxels}} \text{density}(v) > \rho_{\min}
$$

This ensures that the accumulated scan has sufficient points in all spatial regions for reliable registration.

**Density-aware downsampling**: Instead of uniform voxel grid downsampling:
$$
\text{keep}(p_j) = \begin{cases} \text{always} & \text{if } \rho(\text{voxel}(p_j)) < \rho_{\text{low}} \\ \text{with prob } \rho_{\text{target}}/\rho(\text{voxel}(p_j)) & \text{otherwise} \end{cases}
$$

Points in sparse regions are always kept; points in dense regions are probabilistically subsampled. This produces a more uniform point distribution.

### 2.2 Feature-Rich Matching

The system uses multiple matching strategies simultaneously:

1. **Point-to-plane** for well-defined planar surfaces
2. **Point-to-edge** for linear structures (detected via PCA)
3. **Point-to-point** for irregular surfaces where neither plane nor edge fits well

The matching strategy is selected per-point based on the local PCA eigenvalue ratios.

### 2.3 Robust IMU Integration

Tightly-coupled IMU integration with:
- **Adaptive IMU trust**: Based on IMU data quality assessment (noise level, bias drift rate)
- **Gravity-aligned initialization**: Uses accelerometer data during initialization to establish the gravity direction, constraining roll and pitch from the start
- **Bias-aware deskewing**: Uses the most recent bias estimates (not just raw IMU) for deskewing

### 2.4 Map Management

**Adaptive voxel resolution**: Regions with more geometric detail get finer voxel resolution:
$$
\text{voxel\_size}(v) = \text{base\_size} / \text{clamp}(\text{planarity}(v), 0.1, 1.0)
$$

Highly planar regions (low detail) get coarser voxels; edge/corner regions (high detail) get finer voxels.

---

## 3. Key Equations

### Density-aware sampling probability
$$
P(\text{keep} \mid p_j) = \min\left(1, \frac{\rho_{\text{target}}}{\rho(\text{voxel}(p_j))}\right)
$$

### Per-point matching strategy selection
$$
\text{strategy}(p_j) = \begin{cases}
\text{point-to-plane} & \text{if } a_{2D}(p_j) > 0.8 \\
\text{point-to-edge} & \text{if } a_{1D}(p_j) > 0.8 \\
\text{point-to-point} & \text{otherwise}
\end{cases}
$$

### Adaptive voxel resolution
$$
r_v = r_0 \cdot \sigma_v^{-1/3}
$$

where $\sigma_v$ is the surface variation in voxel $v$.

---

## 4. Assumptions

1. **Sufficient geometric structure**: Even with multi-strategy matching, the environment must contain some geometric features.
2. **IMU quality is characterized**: The adaptive IMU trust requires knowledge of the IMU's noise characteristics.
3. **Computational budget allows multi-strategy matching**: Running three matching strategies is more expensive than a single one.

---

## 5. Limitations

1. **System complexity**: The multi-strategy matching, adaptive voxel resolution, and density-aware sampling create a complex system with many parameters.
2. **Not IEKF-based**: The optimization approach differs from FAST-LIO2's IEKF.
3. **Designed for smaller FoV**: Like LOAM-Livox, many optimizations target small-FoV solid-state LiDARs and are less necessary for the Mid-360's 360° coverage.
4. **Adaptive voxel resolution**: Maintaining multiple voxel resolutions complicates the map data structure significantly.
5. **Computational overhead**: Multi-strategy matching and adaptive sampling are expensive.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Several components are directly applicable**:

- **Density-aware downsampling**: Replace FAST-LIO2's uniform voxel grid filter with density-aware sampling. This preserves features in sparse Livox regions while reducing points in over-sampled regions. Simple to implement.

- **Multi-strategy matching**: Add point-to-edge residuals alongside point-to-plane in the IEKF. The PCA classification is already computed during plane fitting.

- **Bias-aware deskewing**: FAST-LIO2 already uses bias-corrected IMU for deskewing, so this is already implemented.

- **Adaptive voxel resolution**: Would require significant changes to the ikd-Tree. Not recommended as a first modification.

**Highest-value components for FAST-LIO2**:
1. Density-aware downsampling (easy, high impact for Livox)
2. Edge feature addition (moderate difficulty, adds geometric diversity)

---

## 7. Applicability to Livox Mid-360

**Directly relevant — designed for solid-state LiDARs**:

- **Density-aware sampling**: Critical for the Mid-360's non-uniform density. Standard voxel grid filter over-samples the dense center of the scan pattern and under-samples the sparse periphery.
- **Multi-strategy matching**: Helps when the environment provides edges but few planes (or vice versa).
- **360° advantage**: Many of the small-FoV workarounds are unnecessary for the Mid-360, but the solid-state-specific optimizations (density-aware sampling, non-scan-line feature extraction) remain valuable.
- **Scan accumulation**: For the Mid-360's non-repetitive pattern, the density criterion for accumulation ensures sufficient coverage before registration.
