# Extracting General-Purpose Features from LiDAR Data

**Category**: Other features  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Overview

This paper presents methods for extracting **general-purpose geometric features** from LiDAR point clouds that are useful across multiple tasks (registration, classification, segmentation). The features are designed to capture local geometric properties without task-specific assumptions.

---

## 2. Core Feature Types

### 2.1 Fast Point Feature Histograms (FPFH)

FPFH is a local geometric descriptor that captures the surface variation around each point:

1. For each pair of neighboring points $(p_j, p_k)$, compute the **Darboux frame** and extract angular features $(\alpha, \phi, \theta)$:
   $$
   \alpha = v \cdot n_k, \quad \phi = u \cdot (p_k - p_j) / \|p_k - p_j\|, \quad \theta = \arctan(w \cdot n_k, u \cdot n_k)
   $$
   where $(u, v, w)$ is the Darboux frame at $p_j$ based on the normal $n_j$.

2. Histogramize these features over the neighborhood to create a descriptor vector.

3. FPFH uses a simplified computation that avoids the full $O(K^2)$ pair computation of PFH.

### 2.2 Spin Images

Project the neighborhood onto a cylindrical coordinate system aligned with the surface normal, creating a 2D histogram (spin image) that is view-independent and robust to partial occlusion.

### 2.3 Surface Variation Features

A set of scalar features computed from PCA eigenvalues:
- **Surface variation**: $\sigma_j = \lambda_1 / (\lambda_1 + \lambda_2 + \lambda_3)$
- **Planarity**: $p_j = (\lambda_2 - \lambda_1) / \lambda_3$
- **Linearity**: $l_j = (\lambda_3 - \lambda_2) / \lambda_3$
- **Sphericity**: $s_j = \lambda_1 / \lambda_3$
- **Omnivariance**: $o_j = (\lambda_1 \lambda_2 \lambda_3)^{1/3}$
- **Eigenentropy**: $e_j = -\sum_{i} \bar{\lambda}_i \ln(\bar{\lambda}_i)$ where $\bar{\lambda}_i = \lambda_i / \sum \lambda_i$

### 2.4 Normal-Based Features

- **Normal orientation**: The surface normal direction
- **Normal variation**: How much the normal changes across the neighborhood
- **Normal histogram**: Distribution of neighbor normals

---

## 3. Registration Applications

### Feature-Based Registration

Rather than raw point matching, feature descriptors enable:
1. **Feature matching**: Match points with similar descriptors across scans
2. **Correspondence filtering**: Use descriptor similarity as an additional filter for kNN correspondences
3. **Coarse alignment**: FPFH + RANSAC for initial alignment before fine ICP

### Correspondence Quality Assessment

Feature descriptors can assess correspondence quality:
$$
q(p_j, q_j) = \exp\left(-\frac{\|f(p_j) - f(q_j)\|^2}{2\sigma_f^2}\right)
$$

where $f(\cdot)$ is the feature descriptor. Low quality → likely outlier.

---

## 4. Applicability to FAST-LIO2 / IEKF

**Selectively applicable**:

- **Full FPFH/descriptor computation is too expensive** for real-time odometry. These features are more suited for loop closure or offline processing.
- **Surface variation features** (computed from PCA eigenvalues) are essentially free since FAST-LIO2 already performs PCA for plane fitting.
- **Correspondence quality via PCA features**: Using the simple PCA features (planarity, linearity) to assess correspondence quality and set per-point measurement noise in $R$ is practical and valuable.
- **Feature-based outlier rejection**: Comparing PCA descriptors between scan point and map point can filter outlier correspondences before the IEKF update.

**Practical integration**:
1. For each kNN match, compute planarity/linearity of both the scan neighborhood and map neighborhood
2. If the local geometric structure doesn't match (e.g., edge point matched to plane region), flag as outlier
3. Use the match quality to set per-correspondence noise in $R$

---

## 5. Applicability to Livox Mid-360

- **PCA-based features work on unstructured point clouds**: Fully compatible with Livox.
- **FPFH**: Compatible in principle but computationally expensive for real-time use.
- **Surface variation features**: Computed from the same kNN used for plane fitting; no additional kNN search needed.
- **Varying point density**: Feature quality depends on neighborhood size. Adaptive $K$ (larger neighborhoods in sparse regions) would improve feature consistency across the non-uniform Livox pattern.
