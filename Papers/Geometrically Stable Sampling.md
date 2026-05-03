# Geometrically Stable Sampling for the ICP Algorithm

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy), FM5 (Correspondence Errors)

---

## 1. Problem Statement

ICP performance depends heavily on which points are selected for registration. Random or uniform subsampling can result in ill-conditioned problems if the selected points don't adequately represent all geometric directions. This paper proposes **geometrically stable sampling** — selecting point subsets that maximize the conditioning of the registration problem, preventing ill-conditioning before it occurs.

---

## 2. Core Methodology

### 2.1 Stability Criterion

A set of correspondences is "geometrically stable" if the resulting normal matrix $A = J^T J$ is well-conditioned. The paper defines stability as:

$$
\text{stability}(\mathcal{S}) = \lambda_{\min}(A(\mathcal{S}))
$$

where $\mathcal{S}$ is the selected point subset and $A(\mathcal{S})$ is the normal matrix computed from those points.

### 2.2 Greedy Point Selection

Finding the optimal subset that maximizes $\lambda_{\min}$ is NP-hard. The paper proposes a **greedy algorithm**:

1. Initialize with a random point
2. At each step, add the point that maximally increases $\lambda_{\min}(A)$
3. Equivalently, add the point whose Jacobian row $J_j$ has the largest projection onto the current minimum eigenvector:
   $$
   j^* = \underset{j \notin \mathcal{S}}{\arg\max} \; |v_{\min}^T J_j^T|^2
   $$
   where $v_{\min}$ is the current minimum eigenvector of $A(\mathcal{S})$

4. Repeat until $|\mathcal{S}|$ reaches the desired size or $\lambda_{\min}$ exceeds a threshold

### 2.3 Normal-Based Approximation

For computational efficiency, the paper also proposes a simplified version:

- Cluster surface normals into directional bins (e.g., using the 6 axis-aligned directions or icosahedron faces)
- Sample equally from each bin
- This ensures normal diversity, which is the primary requirement for non-degeneracy (per the Zhang et al. theory paper)

### 2.4 Spatial Diversity

Beyond normal diversity, the method also considers **spatial diversity** of selected points to ensure the rotation components are well-conditioned. Points are selected to maximize the spatial spread:

$$
\text{diversity}(\mathcal{S}) = \text{det}\left(\sum_{j \in \mathcal{S}} (p_j - \bar{p})(p_j - \bar{p})^T\right)
$$

---

## 3. Key Equations

### Greedy selection criterion
$$
j^* = \underset{j}{\arg\max} \; v_{\min}^T J_j^T J_j v_{\min}
$$

### Combined stability score
$$
\text{score}(j) = \alpha \cdot v_{\min}^T J_j^T J_j v_{\min} + (1-\alpha) \cdot \|p_j - \bar{p}\|^2
$$

### Normal diversity via binning
Partition $S^2$ into $K$ bins. For each bin $B_k$:
$$
n_k = |\{j : n_j \in B_k\}|
$$
Sample $N/K$ points from each non-empty bin.

---

## 4. Assumptions

1. **Pre-computed correspondences**: The selection operates after nearest-neighbor matching, not before. The quality of correspondences is assumed.
2. **Sufficient total points**: There must be enough points in each geometric category to sample from. In very sparse scans, geometric stability may be impossible to achieve.
3. **Static geometry**: Surface normals don't change during the scan.
4. **Linearized analysis**: The greedy criterion uses the linearized Jacobian, which is accurate only near the true solution.

---

## 5. Limitations

1. **Computational overhead**: The greedy selection requires updating the eigendecomposition at each step. With $N$ candidate points and selecting $M$, the cost is $O(M \cdot N)$ eigendecompositions of a $6 \times 6$ matrix. Still real-time feasible but adds latency.
2. **No dynamic adaptation**: The selection is per-scan; it doesn't consider temporal evolution of degeneracy.
3. **Doesn't create geometric information**: If the environment truly lacks normal diversity (e.g., a tunnel), no selection strategy can avoid degeneracy — it can only detect it.
4. **Normal-binning approximation is coarse**: The simplified binning approach doesn't consider the full $6 \times 6$ conditioning, only the $3 \times 3$ normal diversity.
5. **Incompatible with scan-line features**: The selection is point-level, not feature-level, which is actually a benefit for Livox.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable as a preprocessing step**:

- **Before the IEKF update**: After computing kNN correspondences from the ikd-Tree, apply geometrically stable sampling to select the subset of correspondences used in the measurement update.
- **Replaces random downsampling**: Instead of FAST-LIO2's voxel grid downsampling (which is geometry-unaware), use stability-aware sampling.
- **Improved $H$ conditioning**: By selecting geometrically diverse correspondences, the measurement Jacobian $H$ will be better-conditioned, reducing the need for degeneracy mitigation in the first place.
- **Computational tradeoff**: The greedy algorithm adds overhead, but the normal-binning approximation is very fast and could replace the voxel grid filter.

**Integration point**: After the kNN search and plane fitting, but before constructing $H$ and performing the IEKF update.

---

## 7. Applicability to Livox Mid-360

**Particularly beneficial**:

- **Non-uniform density**: The Livox's spatially varying density means voxel grid downsampling can over-represent some regions and under-represent others. Stability-aware sampling would naturally balance across geometric directions.
- **Normal diversity preservation**: In the Livox's non-repetitive scan, different regions of the FoV may contribute different normals. Stability-aware sampling ensures all normal directions are represented even when point density is uneven.
- **No scan-line dependency**: The method operates on unstructured point clouds, making it fully compatible with Livox.
- **Small scan size**: Livox scans are typically smaller (fewer points per scan) than spinning LiDAR scans, making the greedy algorithm more computationally feasible.
