# HCTO: Optimality-Aware LiDAR Inertial Odometry with Hybrid Continuous Time Optimization for Compact Wearable Mapping System

**Category**: Hand-held / Continuous-time  
**Failure Modes Addressed**: FM6 (Motion Distortion / Deskew Errors)

---

## 1. Problem Statement

Full continuous-time optimization (like "Eigen Is All You Need") is computationally expensive and requires a complete system redesign. HCTO proposes a **hybrid** approach: use a standard discrete IEKF as the primary estimator, but selectively apply continuous-time (CT) refinement for **critical frames** where motion distortion is most severe.

---

## 2. Core Methodology

### 2.1 Hybrid Architecture

HCTO operates in two modes:

**Normal mode**: Standard IEKF-based LIO (like FAST-LIO2)
- IMU-based deskewing
- Standard IEKF update with point-to-plane residuals
- Low computational cost

**CT refinement mode**: Activated for selected frames
- The IEKF estimate serves as the initial guess
- A continuous-time B-spline is fitted to the intra-scan trajectory
- Point-to-map residuals and IMU constraints jointly optimize the spline
- The refined trajectory replaces the IEKF estimate for this frame

### 2.2 Optimality-Aware Frame Selection

Not every frame needs CT refinement. HCTO selects critical frames using an **optimality criterion**:

$$
\text{needs\_CT}(k) = (\|\omega_k\| > \omega_{\text{thresh}}) \vee (\|a_k - g\| > a_{\text{thresh}}) \vee (\text{IEKF residual}_k > r_{\text{thresh}})
$$

Frames with high angular velocity, high acceleration, or high post-IEKF residuals are selected for CT refinement. This targets the frames where deskewing errors are most significant.

### 2.3 Continuous-Time Refinement

For selected frames, a cubic B-spline is fitted:

$$
T(t) = \text{B-spline}(\{c_i\}_{i=1}^{N_c}, t)
$$

The control points $\{c_i\}$ are initialized from the IMU-propagated trajectory and optimized using:

$$
\min_{\{c_i\}} \sum_j r_{\text{LiDAR},j}^2 + \lambda_\omega \sum_m r_{\omega,m}^2 + \lambda_a \sum_m r_{a,m}^2 + \lambda_{\text{smooth}} \sum_i \|\Delta^2 c_i\|^2
$$

The smoothness regularizer prevents overfitting to noisy LiDAR residuals.

### 2.4 Wearable System Design

HCTO is designed for compact wearable mapping systems where:
- Motion is aggressive and unpredictable (body-mounted, head-mounted)
- Computational budget is limited (embedded processor)
- The hybrid approach balances accuracy and speed

---

## 3. Key Equations

### Frame selection criterion
$$
\text{CT\_score}(k) = \alpha_\omega \frac{\|\omega_k\|}{\omega_{\max}} + \alpha_a \frac{\|a_k - g\|}{\Delta a_{\max}} + \alpha_r \frac{r_k}{r_{\max}}
$$

CT refinement is triggered when $\text{CT\_score} > \tau_{\text{CT}}$.

### B-spline control point initialization
$$
c_i^{(0)} = T_{\text{IMU}}(t_i)
$$

where $T_{\text{IMU}}(t_i)$ is the IMU-propagated pose at the $i$-th knot time.

### CT-refined IEKF state feedback
After CT refinement, the IEKF state is updated:
$$
\hat{x}_k^{\text{CT}} = T_{\text{spline}}(t_{\text{end}})
$$

The covariance is approximated from the CT optimization Hessian:
$$
P_k^{\text{CT}} \approx (J_{\text{CT}}^T R_{\text{CT}}^{-1} J_{\text{CT}})^{-1}
$$

---

## 4. Assumptions

1. **Frame selection is accurate**: The optimality criterion correctly identifies frames that benefit from CT refinement. Missed frames → deskew errors propagate; unnecessary CT frames → wasted computation.
2. **IEKF provides good initialization**: The CT optimization starts from the IEKF estimate. If the IEKF is already diverged, CT refinement may not recover.
3. **Short CT windows**: CT refinement operates on single frames (one scan interval), not extended windows. This limits the trajectory smoothing to intra-frame.

---

## 5. Limitations

1. **Complexity**: Maintaining two estimation modes (IEKF + CT optimizer) adds implementation complexity.
2. **CT refinement latency**: The B-spline optimization adds latency for selected frames. In a real-time system, this may cause processing delays.
3. **State consistency**: Feeding the CT-refined state back into the IEKF requires careful covariance handling. The CT covariance may not be consistent with the IEKF's prior, causing filter inconsistency.
4. **Threshold tuning**: Three thresholds ($\omega_{\text{thresh}}, a_{\text{thresh}}, r_{\text{thresh}}$) and their weights need tuning.
5. **Not all deskew errors are frame-specific**: Some deskew errors are systematic (IMU bias) and not captured by per-frame CT refinement.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable as an optional enhancement**:

- **FAST-LIO2 as the base**: HCTO's normal mode is essentially FAST-LIO2. The CT refinement is an optional overlay.
- **Selective activation**: Only activated during aggressive motion, adding minimal average computational cost.
- **State feedback**: The CT-refined pose replaces the IEKF state at the current step. The subsequent IEKF prediction starts from this refined state.
- **Implementation path**:
  1. Monitor $\omega, a$ from IMU and post-IEKF residuals
  2. When thresholds are exceeded, fit a B-spline to the current scan interval
  3. Optimize using the same map (ikd-Tree) and residual types
  4. Feed the refined end-pose back to the IEKF

**Key advantage**: Avoids the full system redesign of pure CT approaches while capturing most of the benefit during critical frames.

---

## 7. Applicability to Livox Mid-360

- **Hand-held relevance**: HCTO's wearable/hand-held focus directly matches the use case.
- **100ms scan window**: The Mid-360's ~100ms scan interval is long enough that CT refinement provides meaningful improvement during aggressive motion.
- **Point timestamps**: Per-point timestamps from the Mid-360 enable accurate continuous-time association within the B-spline.
- **Computational budget**: The selective activation means CT refinement is only used when needed, preserving the real-time budget.
- **Non-repetitive pattern**: The B-spline trajectory handles the non-repetitive scan pattern naturally — each point gets its own pose regardless of spatial pattern.
