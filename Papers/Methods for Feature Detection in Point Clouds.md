# Methods for Feature Detection in Point Clouds

**Category**: Other features  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Overview

This is a survey/methods paper covering feature detection techniques for unstructured point clouds, without assuming scan-line structure. It provides the algorithmic foundation for extracting edges, corners, planes, and other geometric primitives from arbitrary point clouds — directly relevant to Livox Mid-360 data.

---

## 2. Feature Detection Methods

### 2.1 PCA-Based Feature Classification

The dominant approach for unstructured point clouds uses **local PCA** of the k-nearest neighbors:

For point $p_j$ with neighbors $\{p_k\}_{k \in \mathcal{N}(j)}$, compute the covariance:

$$
C_j = \frac{1}{K} \sum_{k=1}^{K} (p_k - \bar{p})(p_k - \bar{p})^T
$$

with eigenvalues $\lambda_1 \leq \lambda_2 \leq \lambda_3$.

**Feature classification based on eigenvalue ratios**:

| Feature Type | Condition | Geometric Interpretation |
|---|---|---|
| Plane | $\lambda_1 \ll \lambda_2 \approx \lambda_3$ | Points spread on a 2D surface |
| Edge/Line | $\lambda_1 \approx \lambda_2 \ll \lambda_3$ | Points spread along a 1D line |
| Corner/Sphere | $\lambda_1 \approx \lambda_2 \approx \lambda_3$ | Points spread in 3D |
| Scatter | All $\lambda_i$ small | Insufficient neighborhood |

**Dimensionality descriptors**:
$$
a_{1D} = \frac{\lambda_3 - \lambda_2}{\lambda_3}, \quad a_{2D} = \frac{\lambda_2 - \lambda_1}{\lambda_3}, \quad a_{3D} = \frac{\lambda_1}{\lambda_3}
$$

- $a_{1D} \to 1$: Linear structure (edge)
- $a_{2D} \to 1$: Planar structure
- $a_{3D} \to 1$: Volumetric/isotropic structure

### 2.2 Curvature-Based Features

**Gaussian curvature** and **mean curvature** can be estimated from the point cloud to detect:
- Edges (high mean curvature)
- Corners (high Gaussian curvature)
- Flat surfaces (zero curvature)

Estimated from the local covariance eigenvalues:
$$
\kappa_1 \approx \frac{\lambda_1}{\lambda_1 + \lambda_2 + \lambda_3}
$$

### 2.3 Normal-Based Edge Detection

Edges can be detected by finding locations where surface normals change abruptly:

$$
\Delta n(p_j) = \max_{k \in \mathcal{N}(j)} \|n_j - n_k\|
$$

High $\Delta n$ → edge between two planar surfaces with different orientations.

### 2.4 Harris 3D Keypoint Detection

Extension of the 2D Harris corner detector to 3D:

$$
H_j = \sum_{k \in \mathcal{N}(j)} w_k \cdot n_k n_k^T
$$

The response function:
$$
R_j = \det(H_j) - k \cdot \text{tr}(H_j)^2
$$

High $R_j$ → keypoint (corner-like structure).

### 2.5 ISS (Intrinsic Shape Signatures) Keypoints

Based on the eigenvalue ratios of the scatter matrix:
$$
\text{ISS}(p_j) = \begin{cases} 1 & \text{if } \frac{\lambda_2}{\lambda_1} < \gamma_1 \text{ and } \frac{\lambda_3}{\lambda_2} < \gamma_2 \\ 0 & \text{otherwise} \end{cases}
$$

Keypoints have distinct local shape (not purely planar, linear, or isotropic).

---

## 3. Relevance to Feature-Poor Environments

In environments where planes are insufficient:
- **Edge features** provide translational constraints along the edge direction (where planes don't)
- **Corner/keypoint features** provide full 3D constraints but are rare
- **Multi-scale analysis** (varying $K$ in kNN) can reveal features at different spatial scales

---

## 4. Applicability to FAST-LIO2 / IEKF

**Directly applicable as a feature extraction upgrade**:

- **PCA-based classification**: FAST-LIO2 already computes local PCA for plane fitting. The eigenvalue ratios are available as a byproduct. Adding classification into plane/edge/corner/scatter requires minimal additional computation.
- **Edge features in IEKF**: Detected edges can be used with point-to-line residuals (see VE-LIOM), adding measurement rows to $H$ with different Jacobian structure.
- **Corner features**: If corner features are detected, point-to-point residuals provide full 3D constraints but with higher noise.
- **Feature diversity monitoring**: Tracking the distribution of feature types per scan provides a diagnostic for FM2 — low feature counts in any category signal potential weakness.

---

## 5. Applicability to Livox Mid-360

**Highly applicable — these methods are designed for unstructured point clouds**:

- No scan-line assumption required
- PCA-based methods work directly with kNN neighborhoods from the ikd-Tree
- The non-uniform density of the Livox pattern affects the kNN neighborhood size and shape, which must be accounted for (adaptive $K$ or radius-based neighbors)
- **Multi-scale feature detection** is particularly relevant for Livox: at small scales, the non-repetitive pattern may produce different features than at large scales
- These methods provide the **scan-pattern-agnostic edge detection** that replaces LOAM's scan-line curvature for use with Livox
