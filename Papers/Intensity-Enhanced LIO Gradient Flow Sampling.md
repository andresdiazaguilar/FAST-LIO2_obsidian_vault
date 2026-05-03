# Intensity-Enhanced LiDAR-Inertial Odometry with Gradient Flow Sampling

**Category**: Intensity features  
**Failure Modes Addressed**: FM2 (Insufficient Features)

---

## 1. Problem Statement

Standard point sampling for LiDAR odometry (voxel grid, random) ignores the informational content of intensity. This paper proposes **gradient flow sampling** — a sampling strategy that selects points along intensity gradient flow lines, ensuring that the selected points maximally exploit intensity information for registration.

---

## 2. Core Methodology

### 2.1 Gradient Flow Concept

The intensity field $I(x, y, z)$ on surfaces defines a gradient field $\nabla I$. **Gradient flow lines** are curves that follow the steepest intensity ascent:

$$
\frac{d\gamma(s)}{ds} = \frac{\nabla I(\gamma(s))}{\|\nabla I(\gamma(s))\|}
$$

Points along these flow lines are maximally informative for intensity-based registration because:
- They sample across intensity boundaries (high gradient → more constraint)
- They provide directional diversity in the intensity gradient space

### 2.2 Sampling Strategy

Instead of uniform or random sampling, gradient flow sampling:

1. **Compute intensity gradients** at all points (using kNN-based estimation)
2. **Seed flow lines** from local intensity maxima/minima
3. **Trace flow lines** in both directions (ascent and descent)
4. **Sample points** at regular intervals along each flow line
5. **Diversity enforcement**: Ensure flow lines cover all spatial regions and gradient directions

The result is a point subset that:
- Over-samples intensity-rich regions (boundaries, textured areas)
- Under-samples intensity-flat regions (uniform surfaces)
- Provides balanced directional coverage of intensity gradients

### 2.3 Integration with Geometric Sampling

The gradient flow sampling is combined with geometric sampling:
- A fraction of points are selected for geometric diversity (normal diversity, as in Geometrically Stable Sampling)
- The remaining fraction is selected for intensity diversity (gradient flow)
- The combined set provides both geometric and intensity constraints

---

## 3. Key Equations

### Gradient flow ODE
$$
\dot{\gamma}(s) = \nabla I(\gamma(s)), \quad \gamma(0) = p_0
$$

Discretized:
$$
p_{k+1} = p_k + \Delta s \cdot \frac{\nabla I(p_k)}{\|\nabla I(p_k)\|}
$$

### Information-theoretic sampling criterion
$$
j^* = \arg\max_j \; v_{\min}^T (J_{\text{geo},j}^T J_{\text{geo},j} + J_{\text{int},j}^T J_{\text{int},j}) v_{\min}
$$

where $v_{\min}$ is the current minimum eigenvector of the combined information matrix.

### Sampling density proportional to gradient magnitude
$$
\rho_{\text{sample}}(p) \propto \|\nabla I(p)\|^\alpha
$$

---

## 4. Assumptions

1. **Smooth intensity field**: The intensity varies smoothly enough for gradient computation and flow line tracing. Discontinuities (material edges) create singular points in the flow field.
2. **kNN neighborhoods are locally planar**: Gradient estimation projects neighbors onto the tangent plane, assuming local planarity.
3. **Gradient flow is stable**: The flow lines don't oscillate or diverge. In practice, noisy intensity data can cause unstable flows.

---

## 5. Limitations

1. **Computational overhead**: Tracing gradient flow lines across the point cloud is significantly more expensive than standard sampling methods.
2. **Sensitivity to intensity noise**: Gradient estimation from noisy intensity data produces noisy flow directions, especially at long range.
3. **Singular points**: At intensity maxima, minima, and saddle points, the gradient is zero, and flow lines become singular.
4. **Scan-pattern dependency**: The gradient estimation quality depends on the local point density. For Livox's non-uniform density, gradient flow may be unreliable in sparse regions.
5. **Limited benefit in uniform environments**: Like all intensity methods, provides no benefit when intensity variation is absent.

---

## 6. Applicability to FAST-LIO2 / IEKF

**The concept is more applicable than the full algorithm**:

- **Gradient flow sampling overhead is too high for FAST-LIO2's real-time budget**: Tracing flow lines adds significant computation.
- **Simplified version**: Instead of full gradient flow, use **gradient-magnitude-weighted sampling**: sample points with probability proportional to $\|\nabla I\|$. This achieves similar diversity with much lower cost.
- **Integration with existing pipeline**: After FAST-LIO2's voxel grid downsampling, apply gradient-magnitude-weighted resampling to select the final point subset. This is a preprocessing step.
- **Intensity residuals**: The selected points can be used for both geometric and intensity residuals in the IEKF measurement update.

---

## 7. Applicability to Livox Mid-360

- **Gradient flow is scan-pattern agnostic** in principle, but the quality depends on point density.
- The **simplified gradient-magnitude-weighted sampling** is more practical and density-robust.
- The Mid-360's 360° coverage provides more diverse gradient directions per scan than limited-FoV solid-state LiDARs.
- **Intensity noise characterization** for the Mid-360 is a prerequisite for any intensity-based sampling strategy.
