# On Degeneracy of Optimization-based State Estimation Problems

**Authors**: Zhang, Kaess, Singh  
**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy) — foundational theory

---

## 1. Problem Statement

This is a **foundational theory paper** that formally analyzes when and why optimization-based state estimation problems degenerate. It provides rigorous mathematical conditions for degeneracy, categorizes degeneracy types, and establishes the theoretical framework that subsequent papers (D²-LIO, LP-ICP, etc.) build upon.

---

## 2. Core Methodology

### 2.1 Problem Formulation

Consider a general nonlinear least-squares state estimation problem:

$$
\hat{x} = \underset{x}{\arg\min} \sum_{i=1}^{M} \|r_i(x)\|_{\Sigma_i}^2
$$

where $r_i(x)$ is the $i$-th residual and $\Sigma_i$ is its covariance. The solution is found by iterating:

$$
\delta x^* = -(J^T \Sigma^{-1} J)^{-1} J^T \Sigma^{-1} r
$$

where $J$ is the stacked Jacobian of all residuals.

### 2.2 Degeneracy Condition

The problem is degenerate when the **normal matrix** (Hessian approximation):

$$
A = J^T \Sigma^{-1} J
$$

is **rank-deficient** or **ill-conditioned**. Formally:

- **Exact degeneracy**: $\text{rank}(A) < \dim(x)$. The system of normal equations has infinitely many solutions.
- **Near-degeneracy**: $\kappa(A) = \lambda_{\max}/\lambda_{\min} \gg 1$. The solution exists but is highly sensitive to noise.

### 2.3 Types of Degeneracy

The paper categorizes degeneracy into:

1. **Structural degeneracy**: Arising from the problem structure itself (e.g., insufficient measurements, unobservable states). This is independent of the data values.

2. **Data-dependent degeneracy**: Arising from the specific geometric configuration of the data (e.g., all surface normals being parallel). The problem is structurally well-posed but the specific data instance makes it ill-conditioned.

3. **Numerical degeneracy**: Arising from finite precision arithmetic or poor numerical conditioning (e.g., scale mismatch between state components).

### 2.4 Degeneracy Analysis for Point-to-Plane Registration

For the specific case of point-to-plane ICP:

$$
r_j = n_j^T (R p_j + t - q_j)
$$

The Jacobian w.r.t. pose $\xi = [t^T, \omega^T]^T$ (using small-angle approximation for rotation) is:

$$
J_j = [n_j^T, \quad n_j^T [p_j]_\times]
$$

where $[p_j]_\times$ is the skew-symmetric matrix of $p_j$.

The normal matrix is:

$$
A = \sum_{j=1}^{N} J_j^T J_j = \sum_{j=1}^{N} \begin{bmatrix} n_j n_j^T & n_j (n_j^T [p_j]_\times)^T \\ n_j^T [p_j]_\times n_j^T & ([p_j]_\times^T n_j)(n_j^T [p_j]_\times) \end{bmatrix}
$$

**Degeneracy occurs when**:
- All normals $n_j$ are parallel → translation along any direction perpendicular to the normals is unconstrained
- All points $p_j$ are collinear → rotation about the line through the points is unconstrained
- Combinations of these conditions

### 2.5 Sufficient Conditions for Non-Degeneracy

The paper establishes that point-to-plane registration is non-degenerate if and only if:

1. The normals $\{n_j\}$ span $\mathbb{R}^3$ (at least 3 linearly independent normal directions)
2. For each pair of linearly independent normals, the corresponding points are not all on a line perpendicular to both normals

These are **necessary and sufficient conditions** for full-rank $A$.

---

## 3. Key Equations

### Normal matrix structure
$$
A = \begin{bmatrix} A_{tt} & A_{tr} \\ A_{rt} & A_{rr} \end{bmatrix}
$$

where:
- $A_{tt} = \sum n_j n_j^T$ (translation block)
- $A_{rr} = \sum [p_j]_\times^T n_j n_j^T [p_j]_\times$ (rotation block)
- $A_{tr} = \sum n_j n_j^T [p_j]_\times$ (cross-coupling block)

### Degeneracy index
$$
\mathcal{D} = \frac{\lambda_{\min}(A)}{\text{tr}(A)/6}
$$

Normalized to $[0, 1]$: $\mathcal{D} = 0$ means fully degenerate, $\mathcal{D} = 1$ means isotropically well-conditioned.

---

## 4. Assumptions

1. **Correct correspondences**: The analysis assumes correspondences are correct. Wrong correspondences change the effective geometry, potentially creating artificial degeneracy or masking real degeneracy.
2. **Linear approximation**: The normal matrix analysis is valid at the linearization point. Far from the true solution, the actual degeneracy profile may differ.
3. **Small-angle rotation**: The Jacobian derivation uses the small-angle approximation for rotation, which is valid near the true solution.

---

## 5. Limitations

1. **Theoretical — no implementation**: This paper provides analysis, not algorithms. It doesn't propose a specific method for handling degeneracy.
2. **Local analysis**: The degeneracy conditions are evaluated at a specific linearization point. The global landscape of the cost function may have different degeneracy characteristics.
3. **Static analysis**: Doesn't consider how degeneracy evolves over time in a SLAM system or how the filter's prior affects effective observability.
4. **No IMU integration**: The analysis is purely for registration, not for state estimation with motion priors.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Highly relevant as theoretical foundation**:

- **Understanding when FAST-LIO2 will fail**: The degeneracy conditions directly predict when the LiDAR measurement update will be unreliable.
- **Information matrix analysis**: The normal matrix $A$ in the paper is exactly the information matrix $H_{\text{pose}}^T R^{-1} H_{\text{pose}}$ in FAST-LIO2's IEKF (up to the noise weighting).
- **Scale mismatch explanation**: The paper's block structure analysis ($A_{tt}$ vs $A_{rr}$) directly explains FM10 — the scale mismatch between translation and rotation blocks.
- **Cross-coupling analysis**: The off-diagonal block $A_{tr}$ explains how translation and rotation degeneracy interact — something that D²-LIO's block-separated analysis ignores.
- **Design guidance**: The sufficient conditions for non-degeneracy inform what geometric diversity is needed in the point-to-plane correspondences: at least 3 independent normal directions with appropriate spatial distribution.

---

## 7. Applicability to Livox Mid-360

- **Normal distribution analysis**: The Livox's non-repetitive scan pattern may produce different normal distributions than spinning LiDARs. In corridor environments, both sensor types see similar normals, so the degeneracy conditions are the same.
- **Point spatial distribution**: The Livox's irregular point spacing means the rotation block $A_{rr}$ (which depends on point positions $p_j$) may have different conditioning than spinning LiDARs.
- **Theoretical prediction of failure**: The paper's conditions can be evaluated on Livox scans to predict when FAST-LIO2 will degenerate, enabling proactive mitigation.
