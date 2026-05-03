# IGE-LIO: Intensity Gradient Enhanced Tightly Coupled LiDAR-Inertial Odometry

**Category**: Intensity features  
**Failure Modes Addressed**: FM2 (Insufficient Features), FM1 (Geometric Degeneracy, indirectly)

---

## 1. Problem Statement

Like COIN-LIO, IGE-LIO aims to augment geometric LiDAR features with intensity information. However, rather than using spherical image projection (which is scan-pattern dependent), IGE-LIO uses **3D intensity gradients** computed directly on the point cloud, making it potentially more compatible with non-standard scan patterns.

---

## 2. Core Methodology

### 2.1 3D Intensity Gradient Estimation

For each point $p_j$ with intensity $I_j$, the local intensity gradient is estimated from the point's k-nearest neighbors:

$$
\nabla I(p_j) = \left(\sum_{k \in \mathcal{N}(j)} (p_k - p_j)(p_k - p_j)^T\right)^{-1} \sum_{k \in \mathcal{N}(j)} (p_k - p_j)(I_k - I_j)
$$

This gives a 3D intensity gradient vector $\nabla I \in \mathbb{R}^3$ at each point, representing the direction and magnitude of intensity change on the surface.

### 2.2 Intensity Gradient Residual

The intensity gradient provides a constraint different from the surface normal. The residual measures the consistency of the intensity gradient between the current scan and the map:

$$
r_{\text{IG},j} = \nabla I_{\text{scan}}(p_j) - R^T \nabla I_{\text{map}}(q_j)
$$

where $q_j$ is the nearest map point to $T p_j$, and the map gradient is rotated to the body frame for comparison.

Alternatively, a scalar residual based on intensity value matching:

$$
r_{\text{I},j} = I_{\text{scan}}(p_j) - I_{\text{map}}(q_j)
$$

with the Jacobian derived from the intensity gradient:

$$
\frac{\partial r_{\text{I},j}}{\partial \xi} = \nabla I_{\text{map}}^T \frac{\partial (T p_j)}{\partial \xi}
$$

### 2.3 Tight Coupling with Geometric Residuals

The intensity gradient residuals are **tightly coupled** with geometric point-to-plane residuals in a unified optimization:

$$
E(\xi) = \sum_j w_g r_{\text{geo},j}^2 + \sum_j w_I r_{\text{IG},j}^2
$$

### 2.4 Gradient Quality Assessment

Not all intensity gradients are useful. IGE-LIO filters out:
- **Weak gradients**: $\|\nabla I\| < \tau_I$ (no intensity variation → no constraint)
- **Noisy gradients**: Points where the kNN-based gradient estimate has high residual variance
- **Range-dependent gradients**: Intensity varies with range; gradients due to range variation (not material change) are spurious

---

## 3. Key Equations

### 3D intensity gradient (weighted least squares)
$$
\nabla I = (P^T W P)^{-1} P^T W \delta I
$$

where:
- $P = [p_1 - p_j, \dots, p_K - p_j]^T \in \mathbb{R}^{K \times 3}$: Neighbor position differences
- $\delta I = [I_1 - I_j, \dots, I_K - I_j]^T \in \mathbb{R}^K$: Neighbor intensity differences
- $W$: Distance-based weights

### Intensity-augmented Jacobian
$$
J_{\text{total},j} = \begin{bmatrix} J_{\text{geo},j} \\ J_{\text{IG},j} \end{bmatrix} = \begin{bmatrix} n_j^T \frac{\partial(Tp_j)}{\partial \xi} \\ \nabla I_j^T \frac{\partial(Tp_j)}{\partial \xi} \end{bmatrix}
$$

### Combined information matrix
$$
\mathcal{I}_{\text{total}} = \sum_j J_{\text{geo},j}^T J_{\text{geo},j} + \sum_j J_{\text{IG},j}^T J_{\text{IG},j}
$$

---

## 4. Assumptions

1. **Intensity gradients are spatially consistent**: The same surface region has the same intensity gradient from different viewpoints. This fails for specular surfaces and when range-dependent intensity effects are not corrected.
2. **Sufficient kNN density for gradient estimation**: The intensity gradient requires enough nearby points. In sparse regions of a Livox scan, gradient estimates may be unreliable.
3. **Intensity variation exists**: Uniform-material environments provide no intensity gradients.
4. **Gradients are stable over time**: The map's intensity gradients remain valid as the environment doesn't change.

---

## 5. Limitations

1. **Gradient estimation quality**: The kNN-based gradient depends on local point density and distribution. With the Livox's non-uniform density, gradient quality varies spatially within each scan.
2. **Range-intensity coupling**: LiDAR intensity varies with range (inverse-square law, partially corrected). Gradients along the range direction contain a systematic component that's not related to surface material, creating spurious constraints.
3. **Incidence angle effects**: Intensity decreases with incidence angle. Points on oblique surfaces show intensity gradients that are artifacts of viewing geometry, not material properties.
4. **Computational overhead**: kNN search for gradient estimation adds computation on top of FAST-LIO2's existing kNN for plane fitting. However, the same kNN results could potentially be reused.
5. **Optimization-based**: Designed for optimization, not explicitly for IEKF.

---

## 6. Applicability to FAST-LIO2 / IEKF

**More directly applicable than COIN-LIO**:

- **No spherical projection needed**: The 3D intensity gradient approach works directly with the kNN results already computed by FAST-LIO2.
- **Additional measurement rows**: Each intensity gradient point adds up to 3 measurement rows to $H$ (one per gradient component, or 1 row for the scalar intensity residual).
- **Reuse existing kNN**: FAST-LIO2 already performs kNN search in the ikd-Tree for plane fitting. The same neighbors can be used for intensity gradient estimation.
- **Measurement noise**: The intensity residual noise needs characterization for the Mid-360. It should be added to $R$ as a separate block.

**Integration steps**:
1. During ikd-Tree kNN search, also retrieve intensity values of neighbors
2. Estimate 3D intensity gradient at each point
3. Filter out weak/unreliable gradients
4. Add intensity residuals and their Jacobians to $H$ and $z$
5. Set appropriate intensity noise variance in $R$

---

## 7. Applicability to Livox Mid-360

**Moderately applicable**:

- **No scan-pattern dependency**: The 3D gradient approach doesn't assume any particular scan pattern, making it compatible with the Livox in principle.
- **Density variation concern**: The Livox's spatially varying density means kNN neighborhoods have varying sizes and shapes. In sparse regions, gradient estimates will be poor. A quality threshold ($\|\nabla I\| > \tau$ and gradient residual variance) must be used to filter out unreliable gradients.
- **Intensity characterization needed**: The Mid-360's intensity-range and intensity-angle relationships must be characterized and corrected before gradient computation. Without correction, systematic gradients appear along the range direction.
- **Potential benefit**: In corridor environments where all surface normals are similar (causing geometric degeneracy), different wall materials, floor patterns, or surface textures provide intensity gradients that can resolve the degeneracy.
