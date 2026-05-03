# SE-LIO: Semantic-Enhanced Solid-State-LiDAR-Inertial Odometry for Tree-Rich Environments

**Category**: Semantic-aware  
**Failure Modes Addressed**: FM2 (Insufficient Features), FM4 (Dynamic Objects), FM9 (Solid-State LiDAR)

---

## 1. Problem Statement

In natural environments (forests, parks), geometric features are unreliable: tree trunks and foliage create irregular, non-planar surfaces that are difficult to match. SE-LIO proposes using **semantic information** (specifically tree trunk detection) to create semantically-enhanced features that persist even when geometric features are weak.

---

## 2. Core Methodology

### 2.1 Semantic Segmentation

SE-LIO runs a lightweight **3D semantic segmentation network** on each LiDAR scan to classify points into semantic categories:
- **Ground**: Used for height/roll/pitch constraints
- **Tree trunks**: Used as cylindrical landmark features
- **Foliage**: Filtered out (unreliable for registration)
- **Buildings/walls**: Standard geometric features
- **Dynamic objects**: Filtered out (people, vehicles)

### 2.2 Semantic Feature Types

**Ground plane constraint**:
$$
r_{\text{ground},j} = n_{\text{ground}}^T (T p_j - q_j^{\text{ground}})
$$

Ground points provide strong vertical (z) and roll/pitch constraints.

**Tree trunk features**:
Tree trunks are modeled as vertical cylinders. Each trunk detection provides:
- Trunk center position $(x_c, y_c)$ — translational constraint in the horizontal plane
- Trunk radius $r_c$ — used for correspondence validation

$$
r_{\text{trunk},j} = \sqrt{(T p_j - c_j)_x^2 + (T p_j - c_j)_y^2} - r_{c,j}
$$

**Dynamic object filtering**:
Points classified as dynamic objects (people, vehicles) are removed before registration, preventing map corruption.

### 2.3 Semantic Weighting

Different semantic classes receive different weights in the optimization:
$$
E(\xi) = \sum_{\text{class}} w_{\text{class}} \sum_{j \in \text{class}} r_j^2(\xi)
$$

- Ground: high weight (reliable, constrains attitude)
- Buildings: high weight (stable geometry)
- Tree trunks: medium weight (cylindrical model may not fit perfectly)
- Foliage: zero weight (removed)
- Dynamic: zero weight (removed)

### 2.4 Solid-State LiDAR Adaptation

SE-LIO is specifically designed for solid-state LiDARs:
- No scan-line features needed
- Works with the non-repetitive scan pattern
- Accumulates multiple scans for better semantic segmentation
- Adaptive feature selection based on FoV coverage

---

## 3. Key Equations

### Cylinder model residual
$$
r_{\text{cyl},j} = \left\| \begin{bmatrix} (Tp_j)_x - c_x \\ (Tp_j)_y - c_y \end{bmatrix} \right\| - r_c
$$

### Semantic class weight assignment
$$
w_j = w_{\text{class}(j)} \cdot w_{\text{quality}(j)}
$$

where $w_{\text{quality}}$ is based on segmentation confidence.

### Ground plane extraction
$$
n_{\text{ground}} = \text{PCA\_min\_eigvec}(\{p_j : \text{class}(j) = \text{ground}\})
$$

---

## 4. Assumptions

1. **Semantic segmentation availability**: Requires a trained segmentation network. Performance depends on the network's accuracy and computational speed.
2. **Tree-rich environments**: The tree trunk feature is specific to forested/tree-lined environments. In indoor or urban canyons, this feature type is irrelevant.
3. **Static semantic categories**: The semantic classes (ground, building, trunk) are assumed static. Seasonal changes (leaf fall) can change the semantic landscape.
4. **Vertical tree trunks**: The cylinder model assumes vertical trunks. Leaning or bent trees don't fit well.

---

## 5. Limitations

1. **Environment-specific**: Designed specifically for tree-rich environments. Not directly useful for indoor corridors, tunnels, or urban canyons.
2. **Semantic network overhead**: Running a segmentation network per scan adds significant computational cost (~10-20ms per scan for lightweight networks).
3. **Training data requirements**: The network needs training data for the specific environment type and LiDAR sensor.
4. **Limited dynamic categories**: Only detects dynamic objects in trained categories. Novel dynamic objects (e.g., animals, drones) are missed.
5. **Segmentation errors propagate**: Misclassified points can corrupt registration (e.g., a building point classified as foliage → removed → lost constraint).

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable with computational concerns**:

- **Semantic gating before IEKF update**: Remove foliage and dynamic points before constructing $H$. This is a preprocessing step.
- **Per-class measurement noise**: Set $R_{jj}$ based on semantic class — ground points get low noise (reliable), trunk points get medium noise, etc.
- **Ground plane pseudo-measurement**: Add a ground plane constraint as a virtual measurement in the IEKF:
  $$
  h_{\text{ground}}(x) = n_{\text{up}}^T p_{\text{ground}} + z_{\text{height}}
  $$
  constraining vertical position and roll/pitch.
- **Computational budget**: The segmentation network must fit within FAST-LIO2's ~10ms per-scan budget, which is challenging for accurate models. Quantized/pruned networks or running segmentation asynchronously (every $N$-th scan) could help.

---

## 7. Applicability to Livox Mid-360

**Directly applicable — designed for solid-state LiDARs**:

- No scan-line dependency; works with unstructured point clouds.
- The Mid-360's 360° × 59° FoV provides good coverage for semantic segmentation.
- The non-repetitive scan pattern may challenge frame-level segmentation (different points each frame). Accumulating 2-3 frames before segmentation improves coverage.
- **Tree trunk detection**: Relevant for outdoor datasets where trees are present.
- **Ground plane**: Always relevant for ground-based robots.
- **Dynamic filtering**: Valuable for hand-held operation where the operator's body is often in the FoV. A "human body" class would address the operator ghosting issue.
