# Informed, Constrained, Aligned: A Field Analysis on Degeneracy-aware Point Cloud Registration in the Wild

**Category**: Degeneracy-aware  
**Failure Modes Addressed**: FM1 (Geometric Degeneracy)

---

## 1. Problem Statement

Multiple degeneracy-aware registration strategies have been proposed in the literature, but there is no comprehensive comparison in real-world, diverse environments. This paper provides a **field analysis** comparing three main categories of degeneracy mitigation:
1. **Informed**: Eigenvalue-based detection with information-aware weighting
2. **Constrained**: Direction-locking that prevents updates along degenerate directions
3. **Aligned**: Subspace projection that remaps the solution to the well-conditioned subspace

---

## 2. Core Methodology

### 2.1 Taxonomy of Approaches

The paper categorizes degeneracy mitigation strategies into three families:

**Informed approaches**:
- Modify weights or trust levels based on eigenvalue analysis
- Example: adaptive covariance scaling, per-direction information weighting
- Effect: The registration still tries to find a solution in all 6 DoFs, but with adjusted confidence

$$
\Sigma_{\text{adapted}} = V \text{diag}\left(\frac{1}{\max(\lambda_i, \lambda_{\text{min}})}\right) V^T
$$

**Constrained approaches**:
- Prevent the solution from moving along degenerate directions
- Example: constrained optimization, penalty methods
- Effect: The degenerate components of the solution are locked to zero (or to the prior)

$$
\xi^* = \arg\min E(\xi) \quad \text{s.t.} \quad V_d^T \xi = 0
$$

**Aligned approaches**:
- Project the solution onto the well-conditioned subspace
- Example: solution remapping (Hinduja et al.), subspace projection
- Effect: Post-hoc removal of the degenerate components from the solution

$$
\xi_{\text{aligned}} = V_g V_g^T \xi_{\text{full}}
$$

### 2.2 Evaluation Framework

The paper evaluates each approach across:
- **Accuracy**: Translational and rotational error vs ground truth
- **Consistency**: Whether the reported uncertainty matches the actual error
- **Robustness**: Performance across diverse environments (indoor, outdoor, underground, structured, unstructured)
- **Computational cost**: Runtime overhead

### 2.3 Key Findings

1. **Constrained approaches are most robust** in severe degeneracy: When the problem is truly rank-deficient, preventing movement along degenerate directions is the safest strategy.

2. **Informed approaches are best in mild degeneracy**: When partial information exists along a "degenerate" direction, informed approaches (down-weighting rather than zeroing) preserve useful information.

3. **Aligned approaches have consistency issues**: Solution remapping can produce inconsistent covariance estimates because the remapping operation is not properly reflected in the covariance update.

4. **Threshold sensitivity is universal**: All approaches suffer from threshold sensitivity. The paper recommends **adaptive thresholds** based on the eigenvalue gap (ratio between consecutive eigenvalues) rather than absolute values.

5. **Combined approaches work best**: The best performance comes from combining detection (informed) with mitigation (constrained or aligned) — detect the degree of degeneracy, then apply the appropriate mitigation level.

---

## 3. Key Findings in Equations

### Eigenvalue gap criterion
$$
\text{degenerate}_i = \begin{cases} 1 & \text{if } \frac{\lambda_i}{\lambda_{i+1}} < \gamma_{\text{gap}} \\ 0 & \text{otherwise} \end{cases}
$$

This is more robust than absolute thresholds because it detects the **separation** between well-conditioned and degenerate subspaces.

### Consistency metric (Normalized Estimation Error Squared)
$$
\text{NEES} = \epsilon^T \Sigma_{\text{est}}^{-1} \epsilon
$$

where $\epsilon$ is the true error and $\Sigma_{\text{est}}$ is the estimated covariance. For a consistent estimator, NEES should follow a $\chi^2$ distribution.

---

## 4. Assumptions

1. **Ground truth availability**: The evaluation requires ground truth poses, limiting the analysis to environments with external references.
2. **Isolation of degeneracy effect**: The comparison assumes that differences in performance are due to degeneracy handling, not other system differences (feature extraction, correspondence search, etc.).

---

## 5. Limitations

1. **Evaluation paper, not a novel method**: This paper provides analysis and recommendations, not a new algorithm.
2. **ICP/optimization focus**: The evaluation is in the context of ICP-based registration, not Kalman filter-based fusion.
3. **Limited sensor diversity**: Primarily spinning LiDARs; solid-state LiDAR evaluation is minimal.
4. **No dynamic environments**: All evaluations are in static environments.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Highly applicable as design guidance**:

- **Recommends combined approaches**: For FAST-LIO2, this suggests combining eigenvalue-based degeneracy detection (informed) with a smooth mitigation strategy that transitions between full update (well-conditioned) and constrained update (degenerate).

- **Eigenvalue gap criterion**: The recommended gap-based threshold ($\lambda_i / \lambda_{i+1} < \gamma$) is directly usable in FAST-LIO2's IEKF as a degeneracy detection criterion.

- **Consistency warning for aligned approaches**: If using solution remapping (D²-LIO-style), the covariance must be adjusted accordingly. The LODESTAR Schmidt-Kalman approach naturally handles this.

- **Practical recommendation**: Use an **informed** approach (continuous weighting) during mild degeneracy and switch to a **constrained** approach (zero update along degenerate directions) during severe degeneracy. The eigenvalue gap quantifies the severity.

---

## 7. Applicability to Livox Mid-360

- **Eigenvalue gap criterion is scan-pattern agnostic**: Works regardless of the LiDAR's scan pattern.
- **The recommendation for adaptive thresholds is especially relevant**: The Livox's varying per-scan point distribution means fixed thresholds are even more unreliable than for spinning LiDARs.
- **Field evaluation environments overlap with failure-mode scenarios**: Underground, corridor, and indoor environments in the paper's evaluation match the Cameroon, Senegal, and Valis failure environments.
