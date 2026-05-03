# RF-LIO: Removal-First Tightly-Coupled LiDAR Inertial Odometry in High Dynamic Environments

**Category**: Dynamic objects  
**Failure Modes Addressed**: FM4 (Dynamic Objects and Ghosting)

---

## 1. Problem Statement

FAST-LIO2 assumes a static world — all points are registered into the map and persist indefinitely. In dynamic environments (people walking, vehicles moving), this creates ghost points in the map and incorrect correspondences. RF-LIO proposes a **removal-first** strategy: detect and remove dynamic points before they enter the LIO pipeline, preventing map corruption.

---

## 2. Core Methodology

### 2.1 Dynamic Object Detection

RF-LIO uses a **multi-scan consistency check** to identify dynamic points:

1. **Scan-to-map comparison**: Transform the current scan to the estimated world frame and compare with the existing map. Points that don't match any map points are candidates for either:
   - New static structure (not yet mapped)
   - Dynamic objects (moved since last mapped)

2. **Temporal consistency**: Track candidate points across multiple consecutive scans. A point that appears in one location in scan $t$ but disappears or moves in scan $t+1$ is likely dynamic.

3. **Cluster-level detection**: Dynamic points tend to cluster (a person is a cluster of points, not isolated). RF-LIO clusters candidate points and classifies entire clusters as dynamic or static:
   $$
   \text{dynamic}(\mathcal{C}) = \begin{cases} 1 & \text{if } \frac{|\mathcal{C} \cap \mathcal{C}_{\text{inconsistent}}|}{|\mathcal{C}|} > \tau_d \\ 0 & \text{otherwise} \end{cases}
   $$

### 2.2 Removal Strategy

Dynamic points are removed at two levels:

**Scan level**: Before registration, dynamic points in the current scan are excluded from the residual computation:
$$
H_{\text{static}} = H[\mathcal{I}_{\text{static}}, :]
$$

where $\mathcal{I}_{\text{static}}$ are the indices of static points.

**Map level**: Dynamic points that were previously inserted into the map are removed from the ikd-Tree:
$$
\text{ikd-Tree.delete}(p_j) \quad \forall p_j \in \mathcal{P}_{\text{dynamic}}
$$

### 2.3 Tight Coupling with IMU

RF-LIO maintains tight coupling between LiDAR and IMU throughout the dynamic removal process:
- IMU propagation is unaffected by dynamic objects
- The Kalman update uses only static points
- The IMU prior helps distinguish between sensor motion and object motion in the scan

---

## 3. Key Equations

### Map consistency check
For point $p_j$ in the current scan, transformed to world frame as $p_j^w = T p_j$:
$$
d_{\text{map}}(p_j^w) = \min_{q \in \text{ikd-Tree}} \|p_j^w - q\|
$$

If $d_{\text{map}} > \tau_{\text{new}}$ and no map expansion expected → candidate dynamic.

### Temporal consistency score
$$
s_{\text{temporal}}(\mathcal{C}) = \frac{1}{W} \sum_{t=k-W+1}^{k} \mathbb{1}[\mathcal{C} \text{ inconsistent at time } t]
$$

High $s_{\text{temporal}}$ → likely dynamic.

### Filtered measurement update
$$
K = P^- H_{\text{static}}^T (H_{\text{static}} P^- H_{\text{static}}^T + R_{\text{static}})^{-1}
$$

Only static point correspondences contribute to the Kalman gain.

---

## 4. Assumptions

1. **Static background dominates**: Most of the scene is static; dynamic objects are a minority. If the scene is predominantly dynamic, the remaining static points may be insufficient for registration.
2. **Dynamic objects are detectable**: Objects must move enough between scans to be detected by the consistency check. Slowly moving objects (moving at similar speed to the sensor) may not be detected.
3. **Cluster-level dynamics**: Assumes dynamic objects form spatially coherent clusters. Distributed dynamic effects (flowing water, wind-blown vegetation) may not cluster.
4. **Map quality**: The initial map must be sufficiently accurate for the consistency check to work. Early map errors can cause false dynamic detections.

---

## 5. Limitations

1. **Computational overhead**: Multi-scan consistency checking, clustering, and temporal tracking add significant computation. For FAST-LIO2's tight real-time budget, this may be too heavy.
2. **Detection latency**: Temporal consistency requires observing points across $W$ scans before classifying them. During this window, some dynamic points may enter the map.
3. **Slow-moving objects**: Objects moving slowly relative to the scan rate are difficult to detect via consistency checks.
4. **False positives**: In environments with rapid geometric change (e.g., a door opening, a curtain moving), static objects may be falsely classified as dynamic, removing useful constraints.
5. **Map deletion risk**: Removing points from the ikd-Tree is irreversible in the current implementation. Falsely deleted static points reduce map quality.
6. **Not specifically designed for solid-state LiDARs**: The non-repetitive Livox pattern means the same object may look different in consecutive scans, complicating consistency checks.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable with computational adaptations**:

- **Preprocessing step**: Dynamic point removal is a preprocessing step before the IEKF update. Architecturally clean.
- **ikd-Tree integration**: RF-LIO's map-level removal uses ikd-Tree deletion, which is already supported by FAST-LIO2's ikd-Tree.
- **Static-only $H$**: Constructing $H$ from only static points is straightforward — just skip dynamic points during the Jacobian construction.

**Computational concern**: The multi-scan consistency check is the bottleneck. Simplified alternatives:
1. **Point-level map consistency**: For each current scan point, check if the nearest map point is within a threshold. Inconsistent → candidate dynamic. This is $O(N \log M)$ using the ikd-Tree (already done for correspondence search).
2. **Residual-based detection**: After the first IEKF iteration, points with very large residuals are candidates for dynamic objects. This reuses existing computation.

**For the operator body problem** (hand-held operation): A simpler approach is a **static blind-zone mask** — exclude all points within a fixed angular/distance region that corresponds to the operator's body position.

---

## 7. Applicability to Livox Mid-360

- **Non-repetitive scan challenge**: Consecutive Livox scans see different parts of the scene due to the non-repetitive pattern. Point-level temporal consistency is difficult because the same point is rarely observed in consecutive scans.
- **Cluster-level consistency**: Comparing clusters (spatial regions) rather than individual points is more robust to scan pattern variation.
- **Accumulated scan consistency**: Compare multi-scan accumulations rather than single scans. Accumulating 3-5 non-repetitive scans provides denser coverage for consistency checking.
- **Residual-based approach preferred**: For Livox, using IEKF residuals to detect dynamic points is simpler and scan-pattern agnostic.
