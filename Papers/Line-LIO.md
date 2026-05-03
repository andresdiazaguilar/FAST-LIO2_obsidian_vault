# Line-LIO: High-Precision Line Feature Integration and Quality-Driven Keyframe Optimization for LiDAR-Inertial Odometry

**Authors**: Ding et al., 2026, Meas. Sci. Technol.  
**Category**: Features (suggested paper)  
**Failure Modes Addressed**: FM2 (Insufficient Features), FM8 (Map-History Effects)

---

## 1. Problem Statement

FAST-LIO2 uses only point-to-plane residuals. When planar features are sparse (e.g., outdoor environments with poles, wires, tree branches), the registration becomes under-constrained. Line-LIO integrates **line features** as additional constraints alongside planes, and introduces **quality-driven keyframe selection** to prevent poorly-registered frames from corrupting the map.

---

## 2. Core Methodology

### 2.1 Line Feature Extraction

Lines are detected in the point cloud using PCA-based classification:

For each point $p_j$ with neighbors $\mathcal{N}(j)$, compute the covariance matrix and its eigenvalues $\lambda_1 \leq \lambda_2 \leq \lambda_3$:

$$
\text{is\_line}(p_j) = \frac{\lambda_3 - \lambda_2}{\lambda_3} > \tau_{\text{line}}
$$

Points classified as lines are grouped into line segments using DBSCAN clustering.

### 2.2 Line Feature Representation

Each line segment is represented by:
- **Direction vector**: $\hat{l} = v_3$ (eigenvector of largest eigenvalue)
- **Centroid**: $\bar{p}$
- **Length**: Extent along $\hat{l}$

The line feature in the map is stored as a parametric line:
$$
\ell = (\bar{p}, \hat{l})
$$

### 2.3 Point-to-Line Residual

For a current scan point $p_j$ matched to a map line $\ell_j = (\bar{q}_j, \hat{l}_j)$:

$$
r_{\text{line},j} = \|(T p_j - \bar{q}_j) - ((T p_j - \bar{q}_j)^T \hat{l}_j) \hat{l}_j\| = \|(T p_j - \bar{q}_j) \times \hat{l}_j\|
$$

This is the perpendicular distance from the transformed point to the line.

### 2.4 Combined IEKF Update

The measurement model combines plane and line residuals:

$$
H = \begin{bmatrix} H_{\text{plane}} \\ H_{\text{line}} \end{bmatrix}, \quad z = \begin{bmatrix} r_{\text{plane}} \\ r_{\text{line}} \end{bmatrix}
$$

### 2.5 Quality-Driven Keyframe Selection

Line-LIO introduces a **registration quality metric** to decide which frames should be inserted as keyframes into the map:

$$
q_k = \frac{\lambda_{\min}(\mathcal{I}_k)}{\lambda_{\max}(\mathcal{I}_k)} \cdot \frac{N_{\text{inlier},k}}{N_{\text{total},k}} \cdot \exp\left(-\frac{\bar{r}_k}{\tau_r}\right)
$$

where:
- $\mathcal{I}_k$ is the information matrix of frame $k$
- $N_{\text{inlier}}$ / $N_{\text{total}}$ is the inlier ratio
- $\bar{r}_k$ is the mean residual

Only frames with $q_k > q_{\text{min}}$ are inserted as keyframes. This prevents poorly-registered scans from corrupting the map.

---

## 3. Key Equations

### Point-to-line Jacobian
$$
J_{\text{line},j} = \frac{\partial r_{\text{line},j}}{\partial \xi} = \frac{[\hat{l}_j]_\times^2 (T p_j - \bar{q}_j)}{\|(T p_j - \bar{q}_j) \times \hat{l}_j\|} \cdot \frac{\partial (T p_j)}{\partial \xi}
$$

### Quality metric for keyframe selection
$$
q_k = \underbrace{\kappa^{-1}(\mathcal{I}_k)}_{\text{conditioning}} \cdot \underbrace{\frac{N_{\text{inlier}}}{N_{\text{total}}}}_{\text{inlier ratio}} \cdot \underbrace{\exp(-\bar{r}_k / \tau)}_{\text{residual quality}}
$$

---

## 4. Assumptions

1. **Line features exist**: The environment must have linear structures (poles, edges, cables, rails). In open outdoor or featureless indoor environments, line features may be absent.
2. **PCA-based detection is reliable**: Line detection depends on neighborhood quality. Sparse point clouds produce unreliable PCA results.
3. **Line persistence**: Map lines are assumed stable over time (static environment).

---

## 5. Limitations

1. **Line detection overhead**: PCA-based line detection adds computation, though it reuses kNN results.
2. **Line matching ambiguity**: In environments with many parallel lines (urban canyons, forests), matching the correct line is difficult.
3. **Point-to-line singularity**: When a point lies on the line, the residual is zero and the Jacobian is singular. A minimum distance threshold is needed.
4. **Quality metric tuning**: The keyframe quality metric has multiple components and thresholds that need tuning.
5. **Not validated on Livox**: Primarily tested on spinning LiDARs; line detection quality on Livox patterns is uncertain.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable**:

- **Line detection from existing kNN**: FAST-LIO2 already computes kNN for plane fitting. Checking the eigenvalue ratio to detect lines requires no additional kNN search.
- **Point-to-line residuals in IEKF**: Additional rows in $H$ with different Jacobian structure. Stacks naturally with point-to-plane rows.
- **Quality-driven keyframe selection**: Add a quality check before inserting points into the ikd-Tree. This directly addresses FM8 (map-history effects) by preventing poorly-registered scans from corrupting the map.

**Key integration points**:
1. During kNN plane fitting, classify as plane/line/neither
2. Compute appropriate residual and Jacobian for each feature type
3. After IEKF convergence, compute quality metric $q_k$
4. Only insert scan points into ikd-Tree if $q_k > q_{\text{min}}$

---

## 7. Applicability to Livox Mid-360

- **PCA-based line detection**: Compatible with unstructured point clouds. No scan-line dependency.
- **Density variation**: Line detection quality varies with Livox's non-uniform density. In dense regions, lines are well-detected; in sparse regions, PCA may be unreliable. Apply a minimum eigenvalue ratio threshold.
- **Quality-driven keyframes**: Particularly valuable for the Livox, where some non-repetitive scans may be poorly-conditioned. Preventing these from entering the map improves long-term robustness.
- **Feature diversity**: Lines provide complementary constraints to planes, improving the information matrix conditioning.
