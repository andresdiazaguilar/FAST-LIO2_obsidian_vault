# SR-LIO++: Sweep Reconstruction for Solid-State LiDAR-Inertial Odometry

**Authors**: Yuan et al., 2025  
**Category**: Solid-state LiDARs (suggested paper)  
**Failure Modes Addressed**: FM9 (Solid-State LiDAR Challenges)

---

## 1. Problem Statement

Solid-state LiDARs like the Livox Mid-360 have **non-repetitive scan patterns**. Each scan covers a different part of the FoV, and it takes multiple scans to achieve full coverage. This creates two problems:
1. **Incomplete spatial coverage per scan**: Any single scan has large gaps
2. **Temporal inconsistency**: Points from different parts of the scene are captured at different times within a scan

SR-LIO++ proposes **sweep reconstruction** — combining multiple consecutive non-repetitive scans into a single, spatially-complete "sweep" while accounting for the motion between scans.

---

## 2. Core Methodology

### 2.1 Sweep Reconstruction

A sweep $\mathcal{S}$ is constructed by accumulating $N$ consecutive scans:
$$
\mathcal{S}_k = \bigcup_{i=k-N+1}^{k} T_{k|i} S_i
$$

where $T_{k|i}$ transforms scan $i$ into the coordinate frame of scan $k$.

The number of scans $N$ is chosen to achieve approximately full FoV coverage:
- Livox Avia: $N \approx 10$ (70.4° × 77.2° FoV, flower pattern)
- Livox Mid-360: $N \approx 3-5$ (360° × 59° FoV, faster coverage)

### 2.2 Motion-Compensated Accumulation

Simply stacking scans introduces motion blur. SR-LIO++ compensates by using the estimated trajectory to align all accumulated scans:

$$
p_j^{\text{sweep}} = T_k^{-1} T(\tau_j) p_j^{\text{raw}}
$$

where $T(\tau_j)$ is the sensor pose at the exact time each point was captured, obtained from IEKF state propagation.

### 2.3 Adaptive Accumulation Count

The number of accumulated scans adapts based on motion:
- **Fast motion**: Fewer scans (motion compensation errors accumulate with time)
- **Slow motion**: More scans (longer accumulation window is tolerable)

$$
N_k = \text{clamp}\left(\lfloor N_0 / (1 + \alpha \|v_k\|) \rfloor, N_{\min}, N_{\max}\right)
$$

### 2.4 Density-Uniform Downsampling

After accumulation, the sweep has non-uniform density (center of scan pattern is denser). SR-LIO++ applies **density-uniform downsampling**:

$$
\text{keep}(p_j) = \begin{cases}
\text{always} & \rho(v_j) < \rho_{\min} \\
\text{prob } \rho_{\text{target}} / \rho(v_j) & \text{otherwise}
\end{cases}
$$

where $\rho(v_j)$ is the point density in the voxel containing $p_j$.

### 2.5 Registration

The reconstructed sweep is registered against the map using standard point-to-plane matching within the IEKF framework. The key difference from standard FAST-LIO2 is that the "scan" being registered is the multi-scan sweep, providing:
- More points
- Better spatial coverage
- More diverse normal directions

---

## 3. Key Equations

### Sweep construction
$$
\mathcal{S}_k = \{T_k^{-1} T(\tau_j) p_j^{\text{raw}} : p_j \in S_i, i \in [k-N+1, k]\}
$$

### Adaptive accumulation count
$$
N_k = \max\left(N_{\min}, \left\lfloor \frac{N_0}{1 + \alpha (\|v_k\| + \beta \|\omega_k\|)} \right\rfloor\right)
$$

### Coverage metric
$$
\text{coverage}(\mathcal{S}_k) = \frac{|\{v : \rho(v) > 0, v \in \text{FoV}\}|}{|\{v : v \in \text{FoV}\}|}
$$

### Point density equalization
$$
w_j = \min\left(1, \frac{\bar{\rho}}{\rho(v_j)}\right)
$$

where $\bar{\rho}$ is the mean density across all occupied voxels.

---

## 4. Assumptions

1. **Accurate inter-scan poses**: Motion compensation requires accurate relative poses between accumulated scans. Errors in these poses cause blurring in the sweep.
2. **Static environment during accumulation**: Points accumulated over $N$ scans (0.5-1.0 s at 10 Hz) assume the environment is static. Dynamic objects create inconsistencies.
3. **Livox scan pattern is known**: The density-uniform downsampling requires knowledge of the scan pattern's density distribution.

---

## 5. Limitations

1. **Latency**: The sweep is complete only after $N$ scans are received. This adds $N \times 100$ms latency to the output. For $N=5$, that's 500ms.
2. **Computational overhead**: Accumulating and motion-compensating $N$ scans increases the per-sweep point count by $N\times$. Even after downsampling, more points are processed.
3. **Motion compensation errors**: If the trajectory estimate is poor (e.g., during IEKF divergence), the sweep is corrupted rather than just one scan.
4. **Dynamic objects amplified**: Dynamic objects appear in multiple accumulated scans, creating larger ghost trails in the sweep.
5. **Memory**: Storing $N$ scans for accumulation requires additional memory.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable as a pre-processing enhancement**:

- **Replace single-scan input**: Instead of feeding individual scans to the IEKF, feed reconstructed sweeps. The IEKF itself doesn't change — it sees a denser, more spatially-complete "scan."
- **Better conditioning**: The sweep's broader spatial coverage improves the information matrix conditioning, directly addressing FM1 (geometric degeneracy) and FM10 (scale mismatch).
- **Latency tradeoff**: The output rate drops from 10 Hz to $10/N$ Hz. For applications requiring high-rate output, continue producing per-scan estimates with the sweep used only for map updates.

**Hybrid approach**:
1. Per-scan IEKF at 10 Hz (using individual scans) for high-rate output
2. Per-sweep correction at $10/N$ Hz (using accumulated sweeps) for accuracy

---

## 7. Applicability to Livox Mid-360

**Directly designed for this class of sensor**:

- **Non-repetitive pattern**: The Mid-360's non-repetitive scan pattern is exactly the motivation for sweep reconstruction. Each scan covers ~70% of the FoV; accumulating 3-5 scans achieves near-100%.
- **360° FoV**: The Mid-360's full horizontal coverage means sweeps achieve complete azimuthal coverage. The primary benefit is filling the vertical gaps.
- **Moderate $N$ needed**: With 360° coverage, fewer scans are needed for full coverage compared to small-FoV sensors. $N=3-5$ is typically sufficient.
- **Point density equalization**: The Mid-360's density varies significantly across the FoV. Equalization produces a more uniform input for registration.
- **Practical recommendation**: For the Mid-360, $N=3$ provides a good balance between coverage improvement and latency (300ms delay, 3.3 Hz sweep rate).
