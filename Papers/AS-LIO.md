# AS-LIO: Spatial Overlap Guided Adaptive Sliding Window LiDAR-Inertial Odometry for Aggressive FOV Variation

**Category**: Hand-held  
**Failure Modes Addressed**: FM2 (Insufficient Features), FM9 (Solid-State LiDAR)

---

## 1. Problem Statement

When a LiDAR undergoes aggressive motion (rapid rotation, sharp turns), the field of view (FoV) changes dramatically between consecutive scans. If consecutive scans have low **spatial overlap**, scan-to-map registration may fail because the current scan sees a different part of the environment than the map was built from. AS-LIO addresses this by using an **adaptive sliding window** that accumulates multiple scans to ensure sufficient overlap with the map.

---

## 2. Core Methodology

### 2.1 Spatial Overlap Quantification

AS-LIO quantifies the **spatial overlap** between the current scan and the active map:

$$
\text{overlap}(S_k, M) = \frac{|\{p_j \in S_k : d(T p_j, M) < \tau\}|}{|S_k|}
$$

where $d(T p_j, M)$ is the distance from the transformed scan point to the nearest map point. High overlap → good registration conditions; low overlap → need more scan accumulation.

### 2.2 Adaptive Window Size

The sliding window size $W_k$ adapts based on the overlap:

$$
W_k = \begin{cases}
W_{\min} & \text{if } \text{overlap}_k > \tau_{\text{high}} \\
W_{k-1} + 1 & \text{if } \text{overlap}_k < \tau_{\text{low}} \\
W_{k-1} & \text{otherwise}
\end{cases}
$$

When overlap is low (aggressive FoV change), the window grows, accumulating more scans to build a denser current-frame point cloud. When overlap recovers, the window shrinks back to the minimum.

### 2.3 Multi-Scan Accumulation

Points from the sliding window $[k-W_k+1, k]$ are accumulated and deskewed to the current frame:

$$
P_{\text{accum}} = \bigcup_{i=k-W_k+1}^{k} T_{k|i} \cdot S_i
$$

where $T_{k|i}$ transforms scan $i$ to the frame of scan $k$ using the IMU-propagated relative pose.

The accumulated point cloud is denser and has broader spatial coverage than any single scan, improving registration robustness.

### 2.4 Keyframe Selection

AS-LIO uses an overlap-based keyframe selection:
- A new keyframe is inserted when the overlap between the current accumulated scan and the last keyframe drops below a threshold
- Keyframes are used for long-term map management, while the sliding window handles short-term registration

---

## 3. Key Equations

### Overlap computation (efficient approximation)
$$
\text{overlap}_k \approx \frac{|\{v \in \text{VoxelGrid}(S_k) : v \cap M \neq \emptyset\}|}{|\text{VoxelGrid}(S_k)|}
$$

Using voxel-level overlap rather than point-level for efficiency.

### Window size adaptation
$$
W_{k+1} = \text{clamp}\left(W_k + \Delta W \cdot \text{sign}(\tau_{\text{target}} - \text{overlap}_k), W_{\min}, W_{\max}\right)
$$

### Accumulated scan transformation
$$
p_j^{(k)} = R_k^T R_i (p_j^{(i)} - R_i^T t_i) + R_k^T t_k
$$

for transforming point $p_j$ from scan $i$'s frame to scan $k$'s frame.

---

## 4. Assumptions

1. **IMU-based relative pose is accurate enough for accumulation**: The multi-scan accumulation relies on IMU-propagated relative poses. If the IMU drift is significant over the window, the accumulated cloud is distorted.
2. **Spatial overlap correlates with registration quality**: Low overlap → bad registration. This is generally true but not always — a scene with many distinctive features may register well even with low overlap.
3. **Static scene within window**: Points from different scans in the window are assumed to see the same static structure.

---

## 5. Limitations

1. **Doesn't improve per-point deskewing**: The sliding window accumulates more points but doesn't improve the deskewing quality of individual scans. Deskew errors in individual scans propagate into the accumulated cloud.
2. **IMU drift in large windows**: With $W_{\max}$ = 5-10 scans (0.5-1.0s), IMU drift can distort the accumulated cloud. The ICM40609's drift over 1 second is non-negligible during aggressive motion.
3. **Memory and computation**: Larger windows mean more points to process per registration. This scales linearly with $W$.
4. **Reactive, not predictive**: The window size adapts after low overlap is detected, not before. During rapid FoV changes, there's a 1-2 scan delay before the window grows.
5. **Map staleness**: With a large window, the "current scan" includes old data that may not represent the current environment if the scene has changed.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable**:

- **Scan accumulation before IEKF**: Instead of processing single scans, accumulate $W_k$ scans and process the accumulated cloud through the IEKF.
- **Overlap monitoring**: Compute overlap between the current (accumulated) scan and the ikd-Tree map. This uses the same kNN infrastructure already in FAST-LIO2.
- **Adaptive scan processing rate**: With larger windows, the IEKF update rate decreases (fewer updates per second), but each update uses more measurements. This tradeoff is acceptable for mapping applications.

**Integration steps**:
1. Buffer incoming scans in a ring buffer of size $W_{\max}$
2. At each step, compute overlap with the map
3. Adjust window size based on overlap
4. Accumulate and deskew $W_k$ scans using IMU relative poses
5. Run standard IEKF update with the accumulated cloud

**Concern**: The IEKF's prediction step must account for the variable time interval between updates when the window size changes.

---

## 7. Applicability to Livox Mid-360

**Highly applicable**:

- **Non-repetitive scan pattern**: The Livox's non-repetitive pattern means single scans have incomplete coverage. Accumulating multiple scans naturally fills in the coverage gaps, creating a denser, more uniform point cloud.
- **360° coverage mitigates FoV variation**: Unlike limited-FoV solid-state LiDARs, the Mid-360's 360° horizontal coverage provides inherently better overlap between consecutive scans. The main overlap concern is in the **vertical** direction (59° vertical FoV) during aggressive pitch/roll motion.
- **Scan accumulation benefit**: Even 2-3 accumulated non-repetitive scans provide significantly better coverage than a single scan, improving both feature count and geometric diversity.
- **The primary use case**: For hand-held operation with the Mid-360, scan accumulation is one of the most practical improvements — it directly addresses the limited per-scan coverage without requiring algorithm changes.
