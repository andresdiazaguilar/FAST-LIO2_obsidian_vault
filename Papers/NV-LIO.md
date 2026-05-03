# NV-LIO: LiDAR-Inertial Odometry using Normal Vectors Towards Robust SLAM in Multifloor Environments

**Category**: Other features  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Problem Statement

Point-to-plane residuals constrain translation along surface normals but provide **weak rotational constraints**, especially when surfaces are large and flat. NV-LIO adds **normal vector matching** as an additional measurement type, where the consistency of estimated surface normals between the current scan and the map provides explicit rotational constraints.

---

## 2. Core Methodology

### 2.1 Normal Vector Residual

For each point $p_j$ in the current scan with estimated normal $n_j^{\text{scan}}$ and its matched map point $q_j$ with map normal $n_j^{\text{map}}$, the normal vector residual is:

$$
r_{\text{nv},j} = R n_j^{\text{scan}} - n_j^{\text{map}}
$$

This is a $3 \times 1$ residual that measures the angular discrepancy between the scan normal (rotated to world frame) and the map normal.

### 2.2 Jacobian w.r.t. Rotation

The key value of normal vector matching is its Jacobian. For small rotation perturbation $\delta \theta$:

$$
\frac{\partial r_{\text{nv},j}}{\partial \delta \theta} = -[R n_j^{\text{scan}}]_\times
$$

This Jacobian is **independent of point position** and depends only on the normal direction and current rotation estimate. It directly constrains the rotational states.

### 2.3 Point-to-Plane + Normal Vector Combined Residual

NV-LIO uses both residual types:

$$
r_j = \begin{bmatrix} r_{\text{p2pl},j} \\ r_{\text{nv},j} \end{bmatrix} = \begin{bmatrix} n_j^{\text{map} T} (R p_j + t - q_j) \\ R n_j^{\text{scan}} - n_j^{\text{map}} \end{bmatrix}
$$

The combined Jacobian:

$$
J_j = \begin{bmatrix} J_{\text{p2pl},j} \\ J_{\text{nv},j} \end{bmatrix} = \begin{bmatrix} n_j^T & n_j^T [R p_j]_\times \\ 0_{3 \times 3} & -[R n_j^{\text{scan}}]_\times \end{bmatrix}
$$

**Critical observation**: The normal vector Jacobian has **zero translation columns**. This means:
- Normal vectors constrain **only rotation**, not translation
- They are complementary to point-to-plane residuals (which constrain translation along normals and rotation indirectly through point positions)
- In large-room environments where rotational constraints are weak, normal vectors provide significant additional information

### 2.4 Normal Estimation Quality

NV-LIO uses PCA-based normal estimation:

$$
C_j = \frac{1}{K} \sum_{k \in \mathcal{N}(j)} (p_k - \bar{p})(p_k - \bar{p})^T
$$

The normal is the eigenvector corresponding to the smallest eigenvalue of $C_j$. The ratio $\lambda_{\min}/\lambda_{\text{med}}$ indicates planarity — low ratio means well-defined plane and reliable normal.

### 2.5 Multi-Floor Application

NV-LIO was specifically tested in **multi-floor environments** (buildings with multiple stories connected by stairs/elevators), where:
- Large flat surfaces (floors, walls) provide good translational constraints but weak rotational constraints
- Normal vector matching helps maintain rotational consistency across floor transitions
- The vertical normal (floor/ceiling) constrains roll and pitch, while wall normals constrain yaw

---

## 3. Key Equations

### Normal vector residual Jacobian
$$
J_{\text{nv},j} = \begin{bmatrix} 0_{3 \times 3} & -[R n_j^{\text{scan}}]_\times \end{bmatrix} \in \mathbb{R}^{3 \times 6}
$$

### Rotational information from normals
$$
\mathcal{I}_{\text{rot,nv}} = \sum_{j=1}^{N} [R n_j^{\text{scan}}]_\times^T [R n_j^{\text{scan}}]_\times
$$

### Normal quality metric
$$
q_j = 1 - \frac{\lambda_1(C_j)}{\lambda_2(C_j)}
$$

where $\lambda_1 \leq \lambda_2 \leq \lambda_3$ are eigenvalues of the local covariance. $q_j \to 1$ means high planarity → reliable normal.

---

## 4. Assumptions

1. **Reliable normal estimation**: PCA-based normals require sufficient nearby points with good spatial distribution. In sparse regions, normals are unreliable.
2. **Normal consistency**: The same surface has the same normal from different viewpoints. This holds for planar surfaces but fails for curved surfaces or surfaces with fine texture.
3. **Planar surfaces dominate**: Normal matching is most useful in environments with many planar surfaces (indoor, urban). In natural/unstructured environments, normals are more variable.

---

## 5. Limitations

1. **No translational constraint**: Normal vector matching provides zero translational information. It must be combined with point-to-plane or other translational constraints.
2. **Normal estimation sensitivity**: Normal quality depends on local point density, which varies with range and scan pattern. For Livox, spatially varying density means normals are reliable in dense regions but poor in sparse regions.
3. **Curved surfaces**: On curved surfaces, normals change with position. Normal matching between scan and map requires accurate position for the normal comparison to be meaningful — creating a chicken-and-egg problem.
4. **Computational overhead**: Normal estimation via PCA adds kNN search and eigendecomposition per point. However, FAST-LIO2 already fits planes via PCA, so normals are a byproduct.
5. **Redundancy with point-to-plane**: For well-distributed planes, point-to-plane residuals already provide implicit rotational constraints through the lever arm effect ($[p_j]_\times$ in the Jacobian). Normal matching adds explicit rotational constraints but with some redundancy.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable with minor modifications**:

- **Normal estimation already available**: FAST-LIO2's plane fitting computes normals for every matched point. These normals can be used directly for normal vector matching.
- **Additional measurement rows**: For each point with a reliable normal, add 3 rows to $H$:
  $$
  H_{\text{augmented}} = \begin{bmatrix} H_{\text{p2pl}} \\ H_{\text{nv}} \end{bmatrix}
  $$

- **Measurement noise**: The normal vector residual noise depends on the quality of normal estimation. Set per-point noise $R_{\text{nv},jj}$ based on the PCA planarity metric.

- **Minimal code change**: The normal Jacobian is simple ($-[Rn]_\times$) and the normal residual is straightforward ($Rn_{\text{scan}} - n_{\text{map}}$).

**Key benefit for FAST-LIO2**: In environments where geometric degeneracy is primarily **rotational** (large open rooms, symmetrical structures), normal matching provides the missing rotational constraints without requiring additional feature types.

---

## 7. Applicability to Livox Mid-360

- **Normal quality varies spatially**: The Livox's non-uniform density means normals are reliable in dense regions but poor in sparse regions. A planarity threshold ($q_j > 0.9$) should gate which normals are used.
- **Complementary to intensity features**: Normal matching provides rotational constraints; intensity features can provide translational constraints along surfaces. Together, they address different gaps in the measurement model.
- **No scan-pattern dependency**: Normal estimation and matching work on unstructured point clouds.
- **Mid-360's 360° coverage**: Provides normals from all around the sensor, improving rotational observability compared to limited-FoV Livox models.
