# Intensity Enhanced for Solid-State-LiDAR in SLAM

**Category**: Intensity features / Solid-state LiDARs  
**Failure Modes Addressed**: FM2 (Insufficient Features), FM9 (Solid-State LiDAR Challenges)

---

## 1. Problem Statement

Solid-state LiDARs have limited geometric features due to their small FoV (for early models) and non-repetitive scanning patterns. This paper specifically addresses intensity feature usage for **solid-state LiDAR SLAM**, designing intensity augmentation methods that are compatible with the unique characteristics of these sensors.

---

## 2. Core Methodology

### 2.1 Solid-State Intensity Characteristics

The paper first characterizes how solid-state LiDARs (Livox Avia, Horizon, and similar) produce intensity data:

- **Range-intensity relationship**: Non-linear, with different curves for different target reflectivities
- **Incidence angle effect**: More pronounced than spinning LiDARs due to the scanning geometry
- **Temporal stability**: Intensity values are relatively stable for static surfaces with the same viewing angle
- **Noise characteristics**: Higher intensity noise at long ranges and oblique angles

### 2.2 Intensity Calibration Model

A calibration model corrects for range and angle effects:

$$
I_{\text{calibrated}} = I_{\text{raw}} \cdot f(r) \cdot g(\alpha)
$$

where:
- $f(r) = (r/r_0)^2$ corrects for inverse-square range falloff
- $g(\alpha) = 1/\cos(\alpha)$ corrects for incidence angle $\alpha$
- $r_0$ is a reference range

### 2.3 Intensity Feature Types

The paper extracts several types of intensity features:

1. **Intensity edges**: Points where intensity changes sharply (material boundaries)
   $$
   \text{edge}(p_j) = \max_{k \in \mathcal{N}(j)} |I_j - I_k| / \|p_j - p_k\|
   $$

2. **Intensity texture regions**: Areas with high intensity variation (textured surfaces)
   $$
   \text{texture}(p_j) = \text{Var}(\{I_k : k \in \mathcal{N}(j)\})
   $$

3. **Intensity surfaces**: Using calibrated intensity as a 4th dimension for enhanced surface fitting
   $$
   p_j^{\text{4D}} = [x_j, y_j, z_j, \beta I_j]^T
   $$
   where $\beta$ scales intensity to be commensurate with spatial coordinates.

### 2.4 Integration into SLAM

Intensity features are integrated into the registration by:
1. Adding intensity edges as additional correspondences with geometric+intensity distance
2. Using 4D point matching (xyz + intensity) for kNN search, improving correspondence quality
3. Adding intensity consistency checks to reject outlier correspondences

---

## 3. Key Equations

### 4D distance metric
$$
d_{\text{4D}}(p_i, p_j) = \sqrt{\|p_i^{xyz} - p_j^{xyz}\|^2 + \beta^2 (I_i - I_j)^2}
$$

### Intensity-augmented plane residual
$$
r_j = [n_j^T, \beta \nabla I_j^T] \cdot [T p_j - q_j; I(p_j) - I(q_j)]
$$

### Intensity edge detection
$$
\text{is\_edge}(p_j) = \begin{cases} 1 & \text{if } \max_{k \in \mathcal{N}(j)} \frac{|I_j - I_k|}{\|p_j - p_k\|} > \tau_{\text{edge}} \\ 0 & \text{otherwise} \end{cases}
$$

---

## 4. Assumptions

1. **Intensity calibration accuracy**: The calibration model must accurately remove range and angle effects. Residual systematic errors degrade feature quality.
2. **Material-based intensity variation**: The environment has surfaces with different reflective properties. Uniform-material environments provide no benefit.
3. **Temporal consistency**: Intensity values for the same surface are consistent across time (no lighting changes for LiDAR).

---

## 5. Limitations

1. **Sensor-specific calibration**: The intensity calibration model is sensor-specific. Livox Mid-360 may have different intensity characteristics than the sensors tested (Avia, Horizon).
2. **4D kNN overhead**: Adding intensity to the kNN search changes the distance metric and may require different data structures (ikd-Tree currently uses 3D spatial coordinates).
3. **Intensity edge sparsity**: Intensity edges are typically sparse (material boundaries are infrequent), providing few additional constraints per scan.
4. **Scale parameter $\beta$**: The intensity-to-spatial scaling factor $\beta$ is critical and environment-dependent. Too large → intensity dominates spatial matching; too small → intensity is ignored.
5. **Limited to specific Livox models**: Tested on early Livox models (Avia, Horizon); Mid-360 compatibility needs validation.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Moderately applicable with modifications**:

- **Intensity calibration**: Can be applied as a preprocessing step before FAST-LIO2's pipeline.
- **4D kNN in ikd-Tree**: Would require modifying the ikd-Tree to use a 4D distance metric. This is a non-trivial change but would improve correspondence quality.
- **Intensity residuals in IEKF**: Adding intensity-based residual rows to $H$ follows the same pattern as IGE-LIO integration.
- **Intensity edge features**: Can supplement point-to-plane features, adding constraints in directions parallel to surfaces where geometric features are weak.

---

## 7. Applicability to Livox Mid-360

**Directly relevant but requires validation**:

- **Intensity calibration for Mid-360**: The range-intensity and angle-intensity curves need to be measured specifically for the Mid-360 sensor.
- **Non-repetitive scan pattern compatibility**: The intensity feature extraction works on unstructured point clouds (kNN-based), so it's compatible with the Livox pattern.
- **Density variation**: In sparse regions of the Livox scan, intensity gradient and edge detection will be unreliable. Quality filtering (as in the Adaptive Intensity Feature Extraction paper) is essential.
- **360° coverage advantage**: Unlike the Avia/Horizon (limited FoV), the Mid-360's 360° coverage provides more diverse intensity features per scan.
