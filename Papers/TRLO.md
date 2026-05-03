# TRLO: Efficient LiDAR Odometry with Dynamic Object Tracking and Removal

**Authors**: Jia et al., 2025  
**Category**: Dynamic objects (suggested paper)  
**Failure Modes Addressed**: FM4 (Dynamic Objects / Ghosting)

---

## 1. Problem Statement

Dynamic objects (vehicles, pedestrians, robots) cause **ghosting artifacts** in the map and corrupt scan-to-map registration. Existing approaches either remove all dynamic points (losing potentially useful information) or ignore dynamics entirely. TRLO takes a different approach: it **tracks** dynamic objects and uses the tracking to both (1) remove dynamic points from registration and (2) estimate their motion, enabling robust odometry in highly dynamic environments.

---

## 2. Core Methodology

### 2.1 Object Detection and Tracking

TRLO uses a multi-stage pipeline:

1. **Cluster extraction**: After ground removal, remaining points are clustered using connected-component labeling on a range image or Euclidean clustering.

2. **Static/dynamic classification**: Each cluster is classified by comparing it to the map:
   $$
   \text{dynamic}(C_k) = \frac{|\{p \in C_k : d_{\text{map}}(p) > \tau_d\}|}{|C_k|} > \tau_{\text{ratio}}
   $$
   A cluster is dynamic if most of its points are far from any map point.

3. **Multi-object tracking**: Dynamic clusters are tracked using a **multi-hypothesis tracker** with:
   - State: $[x, y, z, \theta, v_x, v_y]$ per object
   - Motion model: Constant-velocity with noise
   - Association: Hungarian algorithm on Mahalanobis distance

### 2.2 Tracking-Informed Removal

Rather than simply removing all dynamic points, TRLO uses tracking predictions to remove only the points belonging to tracked dynamic objects:

$$
\text{remove}(p_j) = \exists k : p_j \in \text{predicted\_volume}(C_k^{t+\Delta t})
$$

This is more precise than simple visibility-based removal because it accounts for where dynamic objects are expected to be in the current scan.

### 2.3 Registration with Dynamic-Free Scan

After removing dynamic object points, the remaining points are used for standard scan-to-map registration:

$$
T^* = \arg\min_T \sum_{j \notin \mathcal{D}} \rho(n_j^T (T p_j - q_j))
$$

where $\mathcal{D}$ is the set of points classified as dynamic.

### 2.4 Map Maintenance

TRLO performs **retroactive map cleaning**: when an object is confirmed as dynamic (tracked for $N$ frames), all map points within its historical trajectory are removed:

$$
\text{clean\_map}: \forall t, \text{remove points in } \text{volume}(C_k^t) \text{ from map}
$$

This eliminates ghost trails from objects that moved through the map before being detected as dynamic.

---

## 3. Key Equations

### Dynamic classification
$$
s_{\text{dyn}}(C_k) = \frac{1}{|C_k|} \sum_{p \in C_k} \mathbb{1}[d_{\text{map}}(p) > \tau_d]
$$

### Object state prediction
$$
\hat{s}_k^{t+1} = A s_k^t + B u_k + w_k
$$

with constant-velocity model:
$$
A = \begin{bmatrix} I_3 & \Delta t \cdot I_3 \\ 0 & I_3 \end{bmatrix}
$$

### Hungarian association cost
$$
c_{ij} = (z_i - \hat{s}_j)^T S_j^{-1} (z_i - \hat{s}_j)
$$

where $S_j = H P_j H^T + R$ is the innovation covariance.

---

## 4. Assumptions

1. **Dynamic objects are clusterable**: Objects must be spatially separable from the background. Objects touching walls or merged with other objects are harder to detect.
2. **Map is initially static**: The map must be mostly correct for dynamic classification (comparing current scan to map).
3. **Objects move consistently**: The constant-velocity tracking model assumes smooth motion. Rapidly accelerating/decelerating objects may be lost.
4. **Ground removal works**: The pipeline assumes reliable ground segmentation, which may fail on slopes or irregular terrain.

---

## 5. Limitations

1. **Clustering overhead**: Euclidean clustering or range-image connected components are computationally expensive for dense scans.
2. **Small/distant objects**: Small objects at distance have few points and may not form distinct clusters.
3. **Slow-moving objects**: Objects moving slowly relative to the sensor may not be classified as dynamic (small difference from map).
4. **Map dependency**: Dynamic classification requires a pre-existing map, creating a chicken-and-egg problem during initialization.
5. **Not learning-based**: No semantic understanding — a parked car (static) and a moving car (dynamic) look similar. The system relies entirely on motion cues.
6. **Multi-hypothesis tracking complexity**: The tracker's computational cost grows with the number of dynamic objects.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable as a pre-processing module**:

- **Dynamic point removal before IEKF**: Remove detected dynamic points before computing residuals and Jacobians. No change to the IEKF itself.
- **ikd-Tree map cleaning**: Use the retroactive cleaning to remove ghost points from the ikd-Tree. This requires implementing a volume-based deletion in the ikd-Tree.
- **Computational budget**: The tracking pipeline adds significant overhead. For real-time operation with FAST-LIO2, the clustering and tracking must be efficient.

**Integration approach**:
1. After deskewing, run ground removal and clustering
2. Classify clusters as static/dynamic using the ikd-Tree as map
3. Remove dynamic cluster points from the scan
4. Pass the cleaned scan to the IEKF
5. After IEKF convergence, clean ghost points from the ikd-Tree

**Simpler alternative**: For many applications, the residual-based outlier rejection (as in RELEAD) is sufficient to handle moderate dynamic object presence without the full tracking pipeline.

---

## 7. Applicability to Livox Mid-360

- **360° coverage**: The Mid-360's full horizontal coverage means dynamic objects are visible from all directions, improving tracking quality.
- **Non-repetitive pattern**: Dynamic object detection benefits from the non-repetitive pattern — consecutive scans cover different areas, so a dynamic object that moves between scans is more easily detected.
- **Point density**: The Mid-360's moderate point density (~200K pts/s) may limit clustering quality for distant objects. Objects at >30m may have too few points.
- **Computational constraint**: The tracking pipeline's overhead must be balanced against FAST-LIO2's real-time requirement. Consider running tracking at a lower rate or only when dynamic objects are suspected.
