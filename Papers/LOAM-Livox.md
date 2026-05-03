# LOAM-Livox: A Fast, Robust, High-Precision LiDAR Odometry and Mapping Package for LiDARs of Small FoV

**Category**: Solid-state LiDARs  
**Failure Modes Addressed**: FM9 (Solid-State LiDAR Challenges), FM2 (Insufficient Features)

---

## 1. Problem Statement

Standard LOAM assumes a spinning LiDAR with 360° coverage and ordered scan lines. Livox solid-state LiDARs have small FoV (e.g., Avia: 70.4° × 77.2°), non-repetitive scanning, and no scan-line structure. LOAM-Livox adapts the LOAM framework for these characteristics.

---

## 2. Core Methodology

### 2.1 Feature Extraction Without Scan Lines

Since Livox doesn't provide ordered scan lines, LOAM-Livox uses **kNN-based feature extraction**:

**Edge feature detection**:
For each point $p_j$, compute the local covariance from kNN:
$$
C_j = \frac{1}{K} \sum_{k \in \mathcal{N}(j)} (p_k - \bar{p})(p_k - \bar{p})^T
$$

Edge criterion: $\lambda_3 / (\lambda_1 + \lambda_2 + \lambda_3) > \tau_{\text{edge}}$ (dominant 1D structure).

**Plane feature detection**:
Plane criterion: $\lambda_1 / (\lambda_1 + \lambda_2 + \lambda_3) < \tau_{\text{plane}}$ (dominant 2D structure).

This replaces LOAM's curvature-based extraction with PCA-based classification, which works on unstructured point clouds.

### 2.2 Small FoV Handling

With a small FoV:
- Individual scans cover only a fraction of the environment
- Rapid rotation causes the entire visible scene to change
- Point density per scan is lower than spinning LiDARs

LOAM-Livox addresses this by:
1. **Multi-scan feature accumulation**: Aggregating features from $N$ consecutive scans to build a denser feature set
2. **Sliding window map**: Maintaining a recent-history map rather than a global map, reducing the impact of old, potentially inaccurate data
3. **FoV-aware feature selection**: Prioritizing features near the FoV center (more accurate) over edge features (higher noise)

### 2.3 Registration

LOAM-Livox uses separate optimization for edge and plane features:

**Edge-to-line matching**:
$$
r_{\text{edge},j} = \frac{\|(T p_j - q_1) \times (T p_j - q_2)\|}{\|q_1 - q_2\|}
$$

**Plane matching** (point-to-plane):
$$
r_{\text{plane},j} = n_j^T (T p_j - q_j)
$$

Combined:
$$
E(\xi) = \alpha_e \sum_j r_{\text{edge},j}^2 + \alpha_p \sum_j r_{\text{plane},j}^2
$$

### 2.4 Livox-Specific Considerations

- **Non-repetitive coverage**: Exploited by accumulating scans — each scan adds new spatial coverage, so multi-scan accumulation quickly builds complete coverage.
- **Point ordering**: The `line` field in Livox messages (0–3) represents the laser source, not a scan line. Points must be treated as unordered.
- **Timestamp accuracy**: Livox provides per-point timestamps, enabling accurate deskewing.

---

## 3. Key Equations

### PCA-based edge detection
$$
a_{1D}(p_j) = \frac{\lambda_3 - \lambda_2}{\lambda_3}
$$

Point is an edge if $a_{1D} > \tau_e$.

### PCA-based plane detection
$$
a_{2D}(p_j) = \frac{\lambda_2 - \lambda_1}{\lambda_3}
$$

Point is planar if $a_{2D} > \tau_p$.

### Multi-scan accumulation
$$
P_{\text{accum}} = \bigcup_{i=k-N+1}^{k} T_{k|i} S_i
$$

---

## 4. Assumptions

1. **Sufficient features in FoV**: Even with a small FoV, there must be geometric features visible. In a featureless corridor looking at one wall, even multi-scan accumulation doesn't help.
2. **Relative pose accuracy for accumulation**: IMU or previous registration provides accurate relative poses for accumulation.
3. **Static environment**: Multi-scan accumulation assumes the scene doesn't change during the accumulation window.

---

## 5. Limitations

1. **Designed for smaller FoV Livox (Avia, Horizon), not Mid-360**: The Mid-360's 360° × 59° FoV is much larger than the sensors LOAM-Livox targets. Many of the small-FoV workarounds are less necessary.
2. **LOAM-based, not IEKF**: Uses LOAM's optimization pipeline, not FAST-LIO2's IEKF.
3. **Feature extraction overhead**: PCA-based extraction for every point is more expensive than LOAM's scan-line curvature.
4. **No IMU tight coupling**: LOAM-Livox's IMU integration is looser than FAST-LIO2's tightly-coupled IEKF.
5. **Edge detection reliability**: In the Livox's non-repetitive pattern, edges may not be consistently detected across scans due to varying point density.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Selectively applicable — the feature extraction methods are valuable**:

- **PCA-based feature classification**: Can be adopted directly in FAST-LIO2 to identify edge features from the kNN neighborhoods already computed for plane fitting.
- **Edge features in IEKF**: Detected edge features can use point-to-line residuals, adding measurement rows to $H$ with a different Jacobian structure than point-to-plane.
- **Multi-scan accumulation**: Already discussed in AS-LIO; applicable to FAST-LIO2.
- **kNN reuse**: FAST-LIO2 already performs kNN from the ikd-Tree. The same results can be used for PCA-based feature classification.

**What not to adopt**: The LOAM optimization pipeline — FAST-LIO2's IEKF is more principled for tightly-coupled LIO.

---

## 7. Applicability to Livox Mid-360

**Partially relevant**:

- **PCA-based feature extraction**: Directly applicable to the Mid-360's unstructured point clouds.
- **Small FoV mitigations less needed**: The Mid-360's 360° horizontal FoV means most small-FoV concerns (overlap, coverage) are less severe. The vertical FoV (59°) is the limiting factor.
- **Non-repetitive scan handling**: LOAM-Livox's multi-scan accumulation is beneficial for the Mid-360's non-repetitive pattern.
- **Edge feature extraction**: The PCA-based edge detection from LOAM-Livox provides the scan-pattern-agnostic alternative to LOAM's curvature-based method.
