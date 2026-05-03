# Thesis: Degeneracy-Aware LiDAR Odometry and Mapping

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy)

---

## 1. Overview

This is a comprehensive thesis covering the full spectrum of degeneracy in LiDAR odometry: detection, characterization, mitigation, and evaluation. It synthesizes and extends multiple conference/journal papers into a coherent framework.

---

## 2. Core Contributions

### 2.1 Unified Degeneracy Framework

The thesis provides a unified treatment of degeneracy across different registration paradigms:

- **Point-to-point ICP**: Degeneracy when points are collinear or coplanar with specific orientations
- **Point-to-plane ICP**: Degeneracy when surface normals are insufficient to span $\mathbb{R}^3$
- **GICP**: Degeneracy depends on the local covariance structure
- **Feature-based methods**: Degeneracy when feature distributions are degenerate

### 2.2 Detection Methods Compared

The thesis compares multiple detection approaches:

1. **Absolute eigenvalue threshold**: $\lambda_{\min} < \tau$
   - Simple but environment-dependent
   
2. **Relative eigenvalue threshold**: $\lambda_{\min} / \lambda_{\max} < \tau_r$
   - Scale-invariant but still arbitrary

3. **Eigenvalue gap**: $\lambda_k / \lambda_{k+1} < \gamma$
   - Detects the natural separation between degenerate and non-degenerate subspaces
   - Recommended as the most robust criterion

4. **Condition number**: $\kappa = \lambda_{\max} / \lambda_{\min}$
   - Global measure, doesn't identify which directions are degenerate

5. **D-optimality**: $\det(A)$
   - Sensitive to all eigenvalues but doesn't identify degenerate directions

### 2.3 Mitigation Strategies

Organized from least to most aggressive:

1. **Covariance inflation**: Inflate the solution covariance along poorly-conditioned directions. Doesn't change the solution but signals uncertainty to downstream systems.

2. **Adaptive weighting**: Weight correspondences based on their contribution to well-conditioned directions. Continuous, smooth, preserves partial information.

3. **Solution remapping**: Project the solution onto the well-conditioned subspace. Removes degenerate components entirely.

4. **Constrained optimization**: Prevent the optimizer from exploring degenerate directions. Equivalent to solution remapping when applied correctly.

5. **State reset**: In extreme cases, reset the degenerate state components to the prior estimate and increase their covariance.

### 2.4 Multi-Environment Evaluation

The thesis evaluates degeneracy-aware methods across:
- Indoor corridors (translational degeneracy)
- Tunnels (translational degeneracy)
- Large open areas (vertical + yaw degeneracy)
- Staircases (rotational degeneracy around vertical)
- Mixed environments (transitional degeneracy)

---

## 3. Key Insights

### Scale Mismatch
The thesis confirms and quantifies the translation-rotation scale mismatch (FM10): rotation Jacobian columns are 3-7× larger than translation columns in norm. This affects:
- Combined eigenvalue analysis (mixing translation and rotation)
- Threshold selection (must be separate for translation and rotation)
- Solution remapping (degenerate direction may appear well-conditioned in the combined analysis)

### Transitional Degeneracy
A key insight is that **transitions between degenerate and non-degenerate environments** are often more dangerous than sustained degeneracy:
- During sustained degeneracy, the system knows not to trust LiDAR
- During transition, the system may partially trust LiDAR when it's still partially degenerate, leading to subtle errors

### Degeneracy Propagation
In a SLAM system, degeneracy affects not just the current pose but all future poses through:
- Map quality degradation (corrupted map points from drifted registrations)
- Covariance optimism (filter becoming overconfident after a degenerate period)
- Loop closure sensitivity (incorrect loop closures due to map distortion)

---

## 4. Applicability to FAST-LIO2 / IEKF

**Directly applicable** — the thesis's recommendations are framework-agnostic:

- **Eigenvalue gap detection**: Use $\lambda_k / \lambda_{k+1} < \gamma$ for degeneracy detection in the IEKF. Apply separately to translation and rotation blocks.
- **Adaptive weighting + constrained fallback**: Use continuous weighting during mild degeneracy, switch to zero-update (constrained) during severe degeneracy.
- **Covariance management**: During degeneracy, don't let the IEKF posterior covariance shrink along degenerate directions. The Schmidt-Kalman approach (LODESTAR) naturally handles this.
- **Transitional awareness**: Implement smooth transitions between degeneracy states to avoid oscillation.

---

## 5. Applicability to Livox Mid-360

- All recommendations are sensor-agnostic.
- The thesis does not specifically address solid-state LiDAR patterns, but the framework applies to any point cloud registration.
- The Livox's per-scan variability makes transitional degeneracy more relevant — the degeneracy state may change more frequently than with spinning LiDARs.
