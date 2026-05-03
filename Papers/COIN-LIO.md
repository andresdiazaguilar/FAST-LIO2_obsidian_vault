# COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry

**Category**: Intensity features  
**Failure Modes Addressed**: FM2 (Insufficient Features), FM1 (Geometric Degeneracy, indirectly)

---

## 1. Problem Statement

Point-to-plane registration constrains motion only along surface normals. In environments with limited geometric diversity (corridors, tunnels), some DoFs become unobservable. COIN-LIO proposes **augmenting geometric residuals with intensity-based photometric residuals**, effectively adding new measurement dimensions that are independent of surface geometry.

---

## 2. Core Methodology

### 2.1 Spherical Intensity Image Generation

COIN-LIO projects LiDAR points onto a **spherical image** where each pixel stores the intensity value:

$$
I(\theta, \phi) = \text{intensity}(p_j) \quad \text{where} \quad \begin{cases} \theta = \arctan(y/x) \\ \phi = \arcsin(z/r) \end{cases}
$$

This creates a 2D intensity image analogous to a camera image, enabling photometric methods.

### 2.2 Photometric Residual

For each pixel $u = (\theta, \phi)$ in the current intensity image $I_k$ with a corresponding pixel in the map intensity image $I_{\text{map}}$, the photometric residual is:

$$
r_{\text{photo},j} = I_k(u_j) - I_{\text{map}}(\pi(T \cdot p_j))
$$

where $\pi(\cdot)$ projects a 3D point to spherical coordinates.

The Jacobian of this residual w.r.t. pose provides constraints in directions determined by the **intensity gradient** rather than the surface normal:

$$
J_{\text{photo},j} = \nabla I_{\text{map}} \cdot \frac{\partial \pi(T p_j)}{\partial \xi}
$$

### 2.3 Combined Optimization

COIN-LIO combines geometric and photometric residuals:

$$
E(\xi) = \sum_{j=1}^{N_g} w_g \cdot r_{\text{geo},j}^2 + \sum_{j=1}^{N_p} w_p \cdot r_{\text{photo},j}^2
$$

where $w_g$ and $w_p$ balance the geometric and photometric terms. The weights are set based on the relative information content:

$$
\frac{w_p}{w_g} \propto \frac{\lambda_{\min}(\mathcal{I}_{\text{geo}})}{\lambda_{\min}(\mathcal{I}_{\text{photo}})}
$$

### 2.4 Intensity Map Maintenance

A map of intensity values is maintained alongside the geometric map. For each map voxel, the intensity is stored and updated with an exponential moving average to handle varying viewing angles:

$$
I_{\text{map}}(v) = \alpha \cdot I_{\text{new}} + (1 - \alpha) \cdot I_{\text{old}}
$$

---

## 3. Key Equations

### Photometric Jacobian
$$
J_{\text{photo}} = \frac{\partial r_{\text{photo}}}{\partial \xi} = \nabla I \cdot \frac{\partial \pi}{\partial p} \cdot \frac{\partial (Tp)}{\partial \xi}
$$

where:
- $\nabla I \in \mathbb{R}^{1 \times 2}$: Image gradient
- $\frac{\partial \pi}{\partial p} \in \mathbb{R}^{2 \times 3}$: Projection Jacobian
- $\frac{\partial (Tp)}{\partial \xi} \in \mathbb{R}^{3 \times 6}$: Transformation Jacobian

### Combined information matrix
$$
\mathcal{I}_{\text{total}} = \mathcal{I}_{\text{geo}} + \mathcal{I}_{\text{photo}}
$$

The photometric term adds information in directions perpendicular to the intensity gradient, which is generally different from the surface normal direction.

### Intensity normalization
$$
\hat{I}(p) = \frac{I(p) - \mu_I}{\sigma_I}
$$

to handle varying ambient conditions and sensor-specific intensity scales.

---

## 4. Assumptions

1. **Lambertian reflectance**: Intensity is assumed view-independent (same surface looks the same from different angles). Real surfaces exhibit specular reflections.
2. **Intensity consistency**: The same surface returns the same intensity value across scans. In practice, intensity varies with range, incidence angle, and material properties.
3. **Spherical projection stability**: The mapping from 3D to spherical coordinates is assumed to produce stable pixel correspondences across scans.
4. **Sufficient intensity texture**: The environment must have intensity variations (different materials, painted surfaces, etc.). In uniform-material environments (concrete tunnels), intensity adds no information.

---

## 5. Limitations

1. **Spherical projection incompatibility with Livox Mid-360**: The non-repetitive scan pattern of the Livox creates **unstable spherical projections** — the same angular region is covered by different points in successive scans, making pixel-level photometric correspondences unreliable. This is a **critical limitation** for the Mid-360.

2. **Intensity calibration required**: LiDAR intensity depends on:
   - Range (inverse-square falloff, partially corrected in firmware)
   - Incidence angle (cosine falloff)
   - Material reflectivity
   Without calibration, photometric residuals contain systematic errors.

3. **Limited information in uniform environments**: Concrete tunnels, white-painted corridors, and natural rock faces often have minimal intensity variation.

4. **Computational overhead**: Maintaining and querying an intensity map adds memory and computation.

5. **Not IEKF-native**: COIN-LIO uses optimization-based registration. Integration into the IEKF requires formulating photometric residuals as IEKF measurements.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Conceptually valuable, but with significant implementation challenges**:

- **Photometric residuals as IEKF measurements**: Each photometric residual adds a row to the measurement model:
  $$
  h_{\text{photo}}(x) = I_{\text{map}}(\pi(R p + t))
  $$
  with Jacobian $J_{\text{photo}}$ contributing to $H$ alongside point-to-plane Jacobians.

- **Rank improvement**: Adding photometric rows to $H$ directly increases the effective rank of the information matrix, potentially resolving degeneracy that exists in the geometric-only information matrix.

- **Measurement noise for intensity**: Need to set appropriate $R$ for photometric residuals. This requires characterizing the intensity noise of the Livox Mid-360.

- **Map augmentation**: The ikd-Tree would need to store intensity values alongside point coordinates. This is feasible with minimal modification.

**Key challenge**: The spherical projection is the bottleneck for Livox compatibility. An alternative approach:
- Use **direct 3D intensity matching** (compare intensities of kNN-matched points without image projection)
- Use **intensity gradient in 3D** (compute 3D intensity gradients on the map surface)

---

## 7. Applicability to Livox Mid-360

**Limited in current form, but the concept is valuable**:

- **Spherical projection fails**: The non-repetitive scan produces inconsistent pixel locations across scans. A spherical intensity image built from a Livox scan has many holes and aliasing artifacts.

- **Alternative: 3D intensity residuals**: Instead of projecting to a 2D image, compare intensity values directly between matched 3D points:
  $$
  r_{\text{intensity},j} = I(p_j) - I_{\text{map}}(\text{NN}(T p_j))
  $$
  This avoids the spherical projection entirely and is compatible with any scan pattern.

- **Livox intensity quality**: The Mid-360's intensity values need characterization — noise level, range dependence, and angular dependence should be measured to set appropriate measurement noise covariances.

- **Gradient-based alternatives**: IGE-LIO and Gradient Flow Sampling methods may be more compatible with Livox (see separate notes).
