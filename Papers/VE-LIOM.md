# VE-LIOM: A Versatile and Efficient LiDAR-Inertial Odometry and Mapping

**Category**: Other features  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Problem Statement

Single-feature-type LiDAR odometry (plane-only or edge-only) is vulnerable when that feature type is scarce. VE-LIOM proposes a **multi-feature-type** approach that extracts and uses planes, edges, and general surface features simultaneously, providing robustness through feature diversity.

---

## 2. Core Methodology

### 2.1 Multi-Feature Extraction

VE-LIOM extracts three feature types:

**Planar features**: Low-curvature points on flat surfaces.
$$
c_j = \frac{1}{|\mathcal{N}(j)|} \sum_{k \in \mathcal{N}(j)} \frac{\|p_k - p_j\| \cdot (1 - |n_j \cdot \hat{d}_{jk}|)}{r_{\max}}
$$

where $\hat{d}_{jk}$ is the unit direction from $p_j$ to $p_k$. Low $c_j$ → planar.

**Edge features**: High-curvature points on edges/corners. Detected using LOAM-style curvature:
$$
c_j = \frac{1}{|\mathcal{S}_j|} \left\| \sum_{k \in \mathcal{S}_j} (p_k - p_j) \right\|
$$

where $\mathcal{S}_j$ is the set of neighboring points on the same scan line. High $c_j$ → edge.

**General features**: All remaining points that are neither clearly planar nor clearly edge-like. Used with point-to-point or GICP metrics.

### 2.2 Feature-Specific Residuals

Each feature type uses a different residual model:

**Plane residual** (point-to-plane):
$$
r_{\text{plane},j} = n_j^T (T p_j - q_j)
$$

**Edge residual** (point-to-line):
$$
r_{\text{edge},j} = \frac{\|(T p_j - q_{j,1}) \times (T p_j - q_{j,2})\|}{\|q_{j,1} - q_{j,2}\|}
$$

where $q_{j,1}, q_{j,2}$ are two points on the nearest map edge line.

**General residual** (point-to-point or GICP):
$$
r_{\text{general},j} = \|T p_j - q_j\|
$$

### 2.3 Adaptive Feature Weighting

VE-LIOM weights features based on their current utility:

$$
E(\xi) = \alpha_p \sum_j r_{\text{plane},j}^2 + \alpha_e \sum_j r_{\text{edge},j}^2 + \alpha_g \sum_j r_{\text{general},j}^2
$$

where the weights $\alpha_p, \alpha_e, \alpha_g$ are set based on the number and quality of each feature type.

### 2.4 Efficient Implementation

VE-LIOM uses:
- Separate kd-trees for each feature type (edges, planes, general)
- Parallel correspondence search
- Feature-type-specific gating thresholds

---

## 3. Key Equations

### Point-to-line Jacobian (edge residual)
$$
J_{\text{edge},j} = \frac{(T p_j - q_{j,1}) \times l}{\|(T p_j - q_{j,1}) \times l\|} \cdot [I_{3 \times 3}, -[Tp_j]_\times]
$$

where $l = (q_{j,2} - q_{j,1})/\|q_{j,2} - q_{j,1}\|$ is the edge direction.

### Edge constraint geometry
The edge residual constrains motion perpendicular to the edge direction. For a vertical edge, it constrains horizontal translation and roll/pitch. For a horizontal edge, it constrains vertical translation and yaw.

### Feature count adaptation
$$
\alpha_m = \frac{\max(N_m, N_{\min})}{N_m} \cdot \frac{1}{\bar{r}_m^2}
$$

where $N_m$ is the number of features of type $m$ and $\bar{r}_m$ is the mean residual.

---

## 4. Assumptions

1. **Scan-line structure for edge detection**: LOAM-style edge detection requires ordered scan lines, which is the biggest limitation for Livox.
2. **Three distinct feature types**: The environment must have surfaces, edges, and general structure.
3. **Separate map structures**: Each feature type needs its own map structure for correspondence search.

---

## 5. Limitations

1. **Scan-line dependency**: Edge feature extraction relies on LOAM's scan-line curvature computation. The Livox Mid-360 does **not** provide ordered scan lines, making this extraction **incompatible**.
2. **Point-to-point noise**: General features using point-to-point matching are noisier than point-to-plane, potentially degrading accuracy.
3. **Memory overhead**: Three separate map structures (kd-trees or equivalent) increase memory usage.
4. **Optimization-based**: Not designed for IEKF integration.
5. **Feature extraction overhead**: Computing curvature and classifying all points into three categories adds preprocessing time.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Partially applicable — multi-feature concept is valuable, but edge extraction incompatible**:

- **Planar features**: Already used by FAST-LIO2. No change needed.
- **Edge features**: Cannot use LOAM-style extraction with Livox. Alternative edge detection methods for unstructured point clouds (PCA-based: $\lambda_2/\lambda_1 > \tau$ indicates edge) could replace LOAM's curvature computation.
- **Point-to-line residuals in IEKF**: Adding edge residuals requires computing the point-to-line Jacobian and stacking it with point-to-plane Jacobians in $H$.
- **General features**: Point-to-point residuals can supplement when planar and edge features are insufficient, but with higher noise.

**Key adaptation for FAST-LIO2**:
- Replace LOAM edge detection with PCA-based edge detection from kNN neighborhoods (already computed for plane fitting)
- Edge criterion: $\lambda_2 / \lambda_1 > \tau_{\text{edge}}$ and $\lambda_3 / \lambda_2 > \tau_{\text{edge}}$ (1D structure)
- Add point-to-line residuals for detected edges

---

## 7. Applicability to Livox Mid-360

**Limited in current form due to scan-line dependency**:

- LOAM curvature computation is fundamentally incompatible with Livox.
- **PCA-based edge detection alternative**: Using the eigenvalue ratios of the local covariance matrix, edges can be detected without scan lines:
  - Plane: $\lambda_1 \ll \lambda_2 \approx \lambda_3$ (one small eigenvalue)
  - Edge: $\lambda_1 \approx \lambda_2 \ll \lambda_3$ (two small eigenvalues)
  - Corner: $\lambda_1 \approx \lambda_2 \approx \lambda_3$ (no dominant direction)
- This PCA-based classification is compatible with any scan pattern and reuses kNN results from the ikd-Tree.
- The multi-feature concept (planes + edges + general) remains valuable for feature diversity with the adapted extraction method.
