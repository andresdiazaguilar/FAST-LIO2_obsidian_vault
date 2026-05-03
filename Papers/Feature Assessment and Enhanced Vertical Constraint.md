# Feature Assessment and Enhanced Vertical Constraint LiDAR Odometry

**Authors**: Li et al., 2025, IEEE Transactions on Instrumentation and Measurement  
**Category**: Features / Degeneracy (suggested paper)  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Problem Statement

In environments where horizontal features dominate (flat ground, ceilings) but vertical features are scarce (open fields, large rooms), the $z$-direction observability is poor. Standard point-to-plane registration may correctly estimate $x$, $y$, and yaw but poorly estimate $z$, pitch, and roll. This paper proposes a feature assessment mechanism and an enhanced vertical constraint to improve height estimation.

---

## 2. Core Methodology

### 2.1 Feature Distribution Assessment

The system evaluates the spatial distribution of features to detect **directional feature imbalance**:

$$
\mathcal{D} = \frac{\lambda_1^{(\mathcal{F})}}{\lambda_1^{(\mathcal{F})} + \lambda_2^{(\mathcal{F})} + \lambda_3^{(\mathcal{F})}}
$$

where $\lambda_i^{(\mathcal{F})}$ are eigenvalues of the covariance of all normal vectors in the scan. When $\mathcal{D}$ approaches 1, the normals are concentrated along one direction (e.g., all vertical normals from horizontal surfaces), indicating directional degeneracy.

### 2.2 Enhanced Vertical Constraint

When vertical observability is poor ($\mathcal{D} > \tau_D$), the system adds explicit vertical constraints:

**Ground plane constraint**: If a ground plane is detected, the height constraint is:
$$
r_{\text{ground}} = n_g^T (p_{\text{IMU}} + R p_{\text{footprint}}) - d_g
$$

where $n_g$ is the ground normal, $d_g$ is the ground plane offset, and $p_{\text{footprint}}$ is the estimated foot position.

**Gravity-aligned vertical constraint**: Using the IMU's gravity estimate, constrain the vertical component:
$$
r_{\text{vert}} = e_3^T (p_k - p_{k-1}) - v_z \Delta t - \frac{1}{2} g_z \Delta t^2
$$

This enforces that the vertical displacement is consistent with the IMU's vertical velocity and gravity estimates.

### 2.3 Feature Quality Weighting

Features are weighted based on their spatial quality:

$$
w_j = w_{\text{normal}}(j) \cdot w_{\text{dist}}(j) \cdot w_{\text{direction}}(j)
$$

where:
- $w_{\text{normal}}$: Normal estimation confidence (from eigenvalue ratio)
- $w_{\text{dist}}$: Distance-based weight (closer points are more accurate)
- $w_{\text{direction}}$: Direction-based weight (features providing rare directional information are up-weighted)

The direction-based weight is key: if all normals point vertically (horizontal surfaces), any feature with a horizontal normal is given high weight because it provides the missing directional information.

### 2.4 Adaptive Feature Selection

Rather than uniform downsampling, the system performs **directionally-balanced sampling**:

$$
P(\text{keep}(p_j)) \propto \frac{1}{\text{density}(\hat{n}_j)}
$$

where $\text{density}(\hat{n}_j)$ is the density of normals in the direction $\hat{n}_j$. This keeps more features from under-represented directions.

---

## 3. Key Equations

### Feature distribution degeneracy metric
$$
\mathcal{D}_{\text{vert}} = \frac{|n_g^T \bar{n}|}{\|\bar{n}\|}
$$

where $\bar{n} = \frac{1}{N}\sum_j \hat{n}_j$ is the mean normal direction. When $\mathcal{D}_{\text{vert}} \to 1$, most normals align with gravity.

### Ground plane constraint in IEKF
$$
h_{\text{ground}}(x) = n_g^T (R^W_B t^W + R^W_B p^B_{\text{LiDAR}}) - d_g
$$

Jacobian with respect to state:
$$
H_{\text{ground}} = n_g^T [I_3 \quad -R^W_B [p^B_{\text{LiDAR}}]_\times \quad 0_{1 \times 18}]
$$

### Directionally-balanced sampling weight
$$
w_{\text{dir}}(p_j) = \frac{N}{\sum_k \mathbb{1}[\angle(\hat{n}_j, \hat{n}_k) < \theta_{\text{bin}}]}
$$

---

## 4. Assumptions

1. **Ground plane is detectable**: The vertical constraint assumes a flat ground plane is visible and detectable. In multi-level environments, the ground reference changes.
2. **Gravity direction is known**: Relies on the IMU's gravity estimate, which is corrupted by accelerometer bias.
3. **Feature distribution is assessable**: Requires enough features to compute the normal distribution statistics.

---

## 5. Limitations

1. **Ground detection failure**: If no ground plane is visible (elevated platforms, indoor shelves), the vertical constraint is unavailable.
2. **Bias coupling**: The gravity-aligned vertical constraint couples the height estimate to accelerometer bias. If the bias is poorly estimated, the constraint introduces error.
3. **Computational overhead**: Directionally-balanced sampling requires computing the normal distribution histogram, which is moderately expensive.
4. **Not robust to slope**: On sloped terrain, the "ground" normal changes, and the ground constraint must adapt.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable**:

- **Normal distribution analysis**: After kNN plane fitting, compute the distribution of normals. This requires only collecting the normals already computed.
- **Ground plane constraint**: Add as a pseudo-measurement in the IEKF. Minimal state modification needed.
- **Directionally-balanced sampling**: Replace FAST-LIO2's uniform voxel downsampling with normal-direction-aware sampling. Straightforward implementation.
- **Feature quality weighting**: Weight the residual rows in $H$ by the feature quality. This modifies the effective measurement noise per point.

**Key benefit**: Directly addresses the common failure mode where FAST-LIO2 loses height accuracy in environments with mostly horizontal surfaces (large rooms, outdoor fields).

---

## 7. Applicability to Livox Mid-360

- **Vertical FoV advantage**: The Mid-360's 59° vertical FoV captures ground returns, enabling ground plane detection in most configurations.
- **Normal distribution**: The Livox's non-uniform density affects normal estimation quality. Dense regions produce reliable normals; sparse regions produce noisy normals. The feature quality weighting addresses this.
- **Height stability**: The vertical constraint is particularly valuable for hand-held or legged-robot configurations with the Mid-360, where height estimation often drifts.
