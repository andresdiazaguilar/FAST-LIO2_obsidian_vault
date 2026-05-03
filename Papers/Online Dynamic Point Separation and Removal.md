# An Online Dynamic Point Separation and Removal SLAM Framework

**Authors**: Zhu et al., 2025, Australian Journal of Structural Engineering  
**Category**: Dynamic objects (suggested paper)  
**Failure Modes Addressed**: FM4 (Dynamic Objects / Ghosting)

---

## 1. Problem Statement

LiDAR SLAM systems accumulate dynamic object points into the map, creating **ghost artifacts** that corrupt future scan-to-map registration. This paper proposes an online framework for separating dynamic points from static points and removing them from both the current scan (for registration) and the map (for long-term consistency), without requiring semantic labels or GPU-based neural networks.

---

## 2. Core Methodology

### 2.1 Visibility-Based Dynamic Detection

The core idea is that a static point, once observed, should remain visible from the same viewpoint. If a map point is **occluded by a closer point** in a new scan, one of them must be dynamic:

$$
\text{conflict}(p_{\text{map}}, p_{\text{scan}}) = (d_{\text{scan}} < d_{\text{map}} - \epsilon) \wedge (\angle(p_{\text{scan}}, p_{\text{map}}) < \theta_{\text{max}})
$$

where $d$ is the range from the sensor and $\angle$ is the angular distance between the two points as seen from the sensor.

**Interpretation**:
- If the scan point is **closer** than the map point (and roughly same direction), the scan point may be a new object (dynamic) OR the map point was created by a previously dynamic object.
- If the map point is **closer** than the scan point, the map point may have moved away (was dynamic).

### 2.2 Free-Space Violation

A stronger test: if a laser beam passes **through** a map point (the scan point is farther and in the same ray direction), the map point must have been created by a dynamic object:

$$
\text{free\_space\_violation}(p_{\text{map}}) = \exists p_{\text{scan}} : (d_{\text{scan}} > d_{\text{map}} + \epsilon) \wedge (\angle(p_{\text{scan}}, p_{\text{map}}) < \theta_{\text{res}})
$$

Free-space violation is a strong indicator: the laser beam passed through the space where the map point claims to exist, so the map point is a ghost.

### 2.3 Multi-Frame Confirmation

To avoid false positives, dynamic classification requires confirmation over multiple frames:

$$
\text{confirmed\_dynamic}(p_{\text{map}}) = \frac{1}{K} \sum_{k=t-K+1}^{t} \mathbb{1}[\text{violation}(p_{\text{map}}, S_k)] > \tau_{\text{conf}}
$$

A map point is only classified as dynamic if it's violated in $>\tau_{\text{conf}} \cdot K$ of the last $K$ frames.

### 2.4 Online Map Cleaning

Confirmed dynamic points are removed from the map data structure:
1. Mark the point as dynamic
2. Remove it from the spatial index (kd-tree/voxel map)
3. Optionally, propagate the removal to spatially adjacent points (dynamic objects occupy contiguous regions)

---

## 3. Key Equations

### Ray-based conflict detection
For sensor at $o$, map point $p_m$, scan point $p_s$:
$$
\text{conflict} = \left(\frac{\|p_s - o\|}{\|p_m - o\|} < 1 - \epsilon_r\right) \wedge \left(\left\|\frac{p_m}{\|p_m\|} - \frac{p_s}{\|p_s\|}\right\| < \delta_\theta\right)
$$

### Free-space violation probability
$$
P(\text{dynamic} \mid p_m) = 1 - \prod_{k=1}^{K} (1 - P_k(\text{violation} \mid p_m))
$$

### Spatial propagation
$$
\text{remove}(p_i) \text{ if } \exists p_j \in \text{kNN}(p_i) : \text{dynamic}(p_j) \wedge \|p_i - p_j\| < r_{\text{prop}}
$$

---

## 4. Assumptions

1. **Sensor viewpoint changes**: The visibility test requires the sensor to move relative to dynamic objects. If the sensor is stationary, no conflicts arise.
2. **Range accuracy**: The conflict detection depends on accurate range measurements. Noisy ranges can create false conflicts.
3. **Map is mostly static**: The initial map must be dominated by static points. In a highly dynamic environment, the map itself may be too corrupted for comparison.
4. **Angular resolution**: The angular comparison requires sufficient angular resolution to distinguish nearby objects.

---

## 5. Limitations

1. **False positives from sensor noise**: Range noise can cause false free-space violations, especially at long range where $\epsilon$ needs to be larger.
2. **Objects leaving the FoV**: Dynamic objects that leave the sensor's FoV can't be detected as dynamic from visibility alone. Their map points persist until the sensor returns to that viewpoint.
3. **Newly appearing static objects**: A new static object (e.g., a parked car) will initially create conflicts with existing map points behind it. The multi-frame confirmation helps, but the first few frames may misclassify.
4. **Computational cost of ray casting**: Checking visibility for all map points against all scan points is $O(N_{\text{map}} \cdot N_{\text{scan}})$ in the worst case. Spatial indexing reduces this, but it's still significant.
5. **No object-level reasoning**: The system operates on individual points, not objects. An object partially occluded by a static structure may not be fully detected.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable as a map maintenance module**:

- **ikd-Tree dynamic point removal**: After each IEKF convergence, check map points in the current FoV for free-space violations. Remove confirmed dynamic points from the ikd-Tree.
- **No IEKF modification needed**: The dynamic removal operates on the map, not the filter.
- **Pre-registration scan cleaning**: Before IEKF update, remove scan points that conflict with static map points (the scan point is likely dynamic).

**Integration with FAST-LIO2's ikd-Tree**:
- The ikd-Tree supports deletion operations. Marking and removing dynamic points is feasible.
- The computational overhead of visibility checks should be limited by only checking map points within the current scan's FoV.

---

## 7. Applicability to Livox Mid-360

- **360° FoV**: The Mid-360's full horizontal coverage means more map points are checked for free-space violations per scan, improving dynamic detection coverage.
- **Non-repetitive pattern**: Different scan patterns observe slightly different parts of the scene, which can generate false conflicts (a map point not seen in one scan pattern but seen in another). The multi-frame confirmation handles this.
- **Point density**: The Mid-360's density is sufficient for visibility-based reasoning in typical environments (indoor, urban outdoor).
- **Practical value**: For environments with moderate dynamic objects (offices, warehouses), this approach provides good dynamic point removal without the complexity of full object tracking (TRLO) or semantic segmentation (RF-LIO).
