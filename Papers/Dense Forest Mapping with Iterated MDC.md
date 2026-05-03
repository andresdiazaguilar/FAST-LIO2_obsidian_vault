# A LiDAR SLAM System for Dense Forest Mapping with Iterated Motion Distortion Correction

**Authors**: Nakao et al., 2026, Advanced Robotics  
**Category**: Motion distortion / challenging environments (suggested paper)  
**Failure Modes Addressed**: FM6 (Motion Distortion / Deskew Errors)

---

## 1. Problem Statement

Dense forests present extreme challenges for LiDAR SLAM:
1. Many thin, closely-spaced obstacles (tree trunks, branches) create complex geometry
2. Hand-held or UAV-mounted LiDARs experience aggressive motion (vibration, wind gusts)
3. Standard single-pass deskewing may be insufficient when the initial pose estimate is poor

This paper proposes **iterated motion distortion correction (MDC)** — re-deskewing the point cloud using the improved pose estimate from each IEKF iteration, rather than deskewing only once with the initial prediction.

---

## 2. Core Methodology

### 2.1 Iterated Motion Distortion Correction

Standard approach (FAST-LIO2):
1. Predict pose using IMU → deskew scan once → run IEKF iterations → final pose

Iterated MDC approach:
1. Predict pose using IMU → deskew scan → IEKF iteration 1 → **re-deskew scan with updated pose** → IEKF iteration 2 → **re-deskew again** → ... → convergence

The key insight: if the IMU-predicted pose has significant error (common during aggressive motion), the initial deskewing is wrong. The first IEKF iteration corrects the pose partially, and re-deskewing with this improved pose gives a better point cloud for the next iteration.

### 2.2 Deskewing Within IEKF Iteration

At IEKF iteration $i$ with current estimate $\hat{x}^{(i)}$:

$$
p_j^{(i)} = T_{\text{ref}}^{-1} T^{(i)}(\tau_j) p_j^{\text{raw}}
$$

where $T^{(i)}(\tau_j)$ is the interpolated pose at point time $\tau_j$ computed from $\hat{x}^{(i)}$.

The Jacobian must account for the dependency of the deskewed point on the state:
$$
\frac{\partial r_j}{\partial x} = \frac{\partial r_j}{\partial p_j^{(i)}} \frac{\partial p_j^{(i)}}{\partial x} + \frac{\partial r_j}{\partial T} \frac{\partial T}{\partial x}
$$

The first term captures how the deskewed point position changes with the state (through the deskewing transform).

### 2.3 Forest-Specific Adaptations

- **Multi-scale feature matching**: Trees have features at multiple scales (trunks: large, branches: medium, leaves: small). Use different search radii for different feature types.
- **Cylindrical feature model**: Tree trunks are cylindrical, not planar. Point-to-cylinder residuals:
  $$
  r_{\text{cyl},j} = d(T p_j, \text{cylinder}(\hat{c}_k, \hat{r}_k, \hat{a}_k))
  $$
  where $\hat{c}$ is the center, $\hat{r}$ is the radius, and $\hat{a}$ is the axis direction.

- **Branch filtering**: Points on small, moving branches (wind) are treated as semi-dynamic and down-weighted.

---

## 3. Key Equations

### Iterated deskewing within IEKF
At iteration $i$:
$$
T^{(i)}(\tau_j) = T^{(i)}_{\text{start}} \cdot \exp\left(\frac{\tau_j - t_0}{t_1 - t_0} \log((T^{(i)}_{\text{start}})^{-1} T^{(i)}_{\text{end}})\right)
$$

### Deskewed point Jacobian (additional term)
$$
\frac{\partial p_j^{(i)}}{\partial \delta \xi} = -[p_j^{(i)}]_\times \cdot \frac{\tau_j - t_0}{t_1 - t_0}
$$

This term is proportional to the fractional time offset — points near the scan start are barely affected by the state update; points near the scan end are most affected.

### Point-to-cylinder residual
For a cylinder with axis $\hat{a}$, center $c$, radius $r$:
$$
r_{\text{cyl}} = \|(p - c) - ((p-c)^T \hat{a}) \hat{a}\| - r
$$

---

## 4. Assumptions

1. **Smooth motion between IEKF iterations**: The re-deskewing uses linear interpolation between start and end poses. Assumes smooth motion within the scan.
2. **Convergence**: The iterated MDC assumes the pose estimate converges. In pathological cases, re-deskewing might oscillate.
3. **Cylindrical trees**: The cylindrical model assumes straight, circular tree trunks. Bent, hollow, or non-circular trees violate this.

---

## 5. Limitations

1. **Computational overhead**: Re-deskewing at each iteration adds kNN re-queries (the point positions change, so the nearest neighbors change). This approximately doubles the per-iteration cost.
2. **kNN re-query**: After re-deskewing, the nearest map points may change. Properly, kNN should be re-run after each re-deskewing, which is expensive.
3. **Forest-specific**: The cylindrical features and branch filtering are forest-specific. The iterated MDC concept is general, but the feature models are not.
4. **Marginal benefit for slow motion**: If the initial deskewing is already good (slow motion, good IMU), iterated MDC adds cost without benefit.
5. **Not yet widely validated**: Limited validation beyond forest environments.

---

## 6. Applicability to FAST-LIO2 / IEKF

**The iterated MDC concept is directly applicable**:

- **Within FAST-LIO2's existing IEKF loop**: After each IEKF iteration, re-deskew the scan using the updated pose, then continue the next iteration with the improved point cloud. This is conceptually simple but requires re-computing correspondences.
- **Selective re-deskewing**: Only re-deskew when the pose update is large (the deskewing change is negligible for small updates). Use a threshold: if $\|\delta x^{(i)}\| > \tau_{\text{redeskew}}$, re-deskew; otherwise, skip.
- **Jacobian modification**: The additional Jacobian term for the deskewing dependency can be included or ignored (ignoring it means the IEKF doesn't account for how deskewing changes with the state, which is a minor approximation).

**Forest features**: Not applicable to general FAST-LIO2. The cylindrical model is forest-specific.

**Highest-value component**: Iterated MDC with selective re-deskewing. Adds robustness during aggressive motion at moderate computational cost.

---

## 7. Applicability to Livox Mid-360

- **100ms scan period**: The Mid-360's scan period means significant motion distortion during aggressive hand-held use. Iterated MDC directly helps.
- **Non-repetitive pattern**: The Livox's pattern means different points within a scan observe different spatial regions. Deskewing errors affect each point differently, making iterated correction valuable.
- **Computational budget**: Re-deskewing and kNN re-query add ~50-100% overhead per IEKF iteration. With FAST-LIO2's typical 3-4 iterations, this is significant. The selective approach (only re-deskew when needed) mitigates this.
- **Forest applications**: If the system is used in forested environments (forestry, environmental monitoring), the cylindrical features provide additional constraints.
