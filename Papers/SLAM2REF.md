# SLAM2REF: Advancing Long-Term Mapping with Reference Map Integration

**Authors**: Vega-Torres et al., 2024  
**Category**: Map management (suggested paper)  
**Failure Modes Addressed**: FM8 (Map-History / Initialization Sensitivity)

---

## 1. Problem Statement

LiDAR SLAM systems build maps incrementally, accumulating drift over time. When a robot returns to a previously visited area, the current map may have drifted significantly from the true geometry. SLAM2REF proposes integrating a **pre-existing reference map** (e.g., from surveying, BIM models, or a previous high-quality mapping run) to:
1. Correct accumulated drift
2. Provide absolute localization
3. Enable long-term map consistency

---

## 2. Core Methodology

### 2.1 Reference Map Integration

The system maintains two maps:
- **Online map $\mathcal{M}_O$**: Built incrementally by the LIO system (e.g., FAST-LIO2's ikd-Tree)
- **Reference map $\mathcal{M}_R$**: Pre-existing, high-quality map (loaded at startup)

### 2.2 Dual Registration

Each scan is registered against **both maps**:

1. **Online registration**: $T_O = \text{IEKF}(S_k, \mathcal{M}_O)$ — standard FAST-LIO2 registration
2. **Reference registration**: $T_R = \text{ICP}(S_k, \mathcal{M}_R)$ — registration against the reference map

The final pose is a weighted combination:
$$
T_k = \text{Exp}\left(w_O \log(T_O) + w_R \log(T_R)\right)
$$

where weights depend on the quality of each registration:
$$
w_O = \frac{1/\sigma_O^2}{1/\sigma_O^2 + 1/\sigma_R^2}, \quad w_R = \frac{1/\sigma_R^2}{1/\sigma_O^2 + 1/\sigma_R^2}
$$

### 2.3 Reference Map Matching Quality

The reference registration quality is assessed by:

1. **Overlap score**: Fraction of scan points with valid correspondences in the reference map:
   $$
   \text{overlap} = \frac{|\{p_j : d(T_R p_j, \mathcal{M}_R) < \tau\}|}{|S_k|}
   $$

2. **Registration fitness**: Mean residual of the reference registration:
   $$
   \text{fitness} = \frac{1}{N_{\text{inlier}}} \sum_{j \in \text{inlier}} |r_j|
   $$

When overlap is low (exploring unknown areas not in the reference map), $w_R \to 0$ and the system falls back to online-only SLAM.

### 2.4 Map Correction

When the reference registration is confident (high overlap, low fitness), the system corrects the online map:

$$
\Delta T = T_R \cdot T_O^{-1}
$$

This drift correction is applied to:
1. The current pose
2. Recent trajectory poses (within a window)
3. The online map points (within the corrected region)

---

## 3. Key Equations

### Weighted pose fusion on SE(3)
$$
\xi_k = \frac{\sigma_R^{-2} \log(T_R) + \sigma_O^{-2} \log(T_O)}{\sigma_R^{-2} + \sigma_O^{-2}}
$$
$$
T_k = \exp(\xi_k)
$$

### Overlap-based reference weight
$$
w_R = \text{overlap} \cdot \exp(-\text{fitness} / \tau_f)
$$

### Drift correction propagation
For trajectory pose $T_i$ at time $t_i$ within the correction window $[t_c - \Delta t, t_c]$:
$$
T_i^{\text{corrected}} = \exp\left(\frac{t_i - (t_c - \Delta t)}{\Delta t} \cdot \log(\Delta T)\right) \cdot T_i
$$

---

## 4. Assumptions

1. **Reference map is accurate**: The reference map is assumed to be geometrically correct. Errors in the reference map propagate to the corrected trajectory.
2. **Reference map is in the same coordinate frame**: Requires initial alignment between the SLAM frame and the reference map frame.
3. **Environment hasn't changed significantly**: The reference map represents the current environment. Construction, seasonal changes, or rearranged furniture reduce the reference's usefulness.
4. **Sufficient overlap**: The robot must visit areas covered by the reference map for the correction to activate.

---

## 5. Limitations

1. **Reference map requirement**: Requires a pre-existing map, which may not be available for novel environments.
2. **Initial alignment**: The reference and online maps must be aligned to a common frame. This is a non-trivial global localization problem.
3. **Dynamic changes**: If the environment has changed since the reference map was created, the reference registration gives incorrect corrections.
4. **Computational overhead**: Dual registration (against online and reference maps) roughly doubles the registration cost.
5. **Map deformation**: Correcting drift by deforming the recent trajectory/map can introduce artifacts.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable as a correction layer on top of FAST-LIO2**:

- **Post-IEKF correction**: After FAST-LIO2's IEKF convergence, register the scan against the reference map and apply the correction. No IEKF modification needed.
- **Alternative: reference constraint in IEKF**: Add the reference registration as a pseudo-measurement in the IEKF, similar to GPS integration. This is more principled but requires modifying the measurement model.
- **Map correction**: Apply drift corrections to the ikd-Tree by transforming its points.

**Use cases**:
- Repeated mapping of the same environment (warehouse, office)
- Using a BIM model or architectural plan as a reference
- Long-term operation where drift accumulates

---

## 7. Applicability to Livox Mid-360

- **360° FoV**: The Mid-360's full coverage provides good overlap with reference maps from any orientation.
- **Indoor mapping**: The primary use case (repeated indoor mapping) aligns with the Mid-360's typical deployment.
- **Map aging**: SLAM2REF addresses the long-term map consistency problem that the ikd-Tree's bounded map doesn't fully solve.
- **Practical value**: For production deployments where the same area is mapped repeatedly, integrating a reference map provides absolute accuracy that pure odometry cannot achieve.
