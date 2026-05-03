# De-Skewing Point Clouds Based on Gaussian Process Regression

**Authors**: Zheng et al., 2025  
**Category**: Motion distortion (suggested paper)  
**Failure Modes Addressed**: FM6 (Motion Distortion / Deskew Errors)

---

## 1. Problem Statement

Standard deskewing assumes either constant velocity (linear interpolation) or uses IMU data directly. Both approaches have limitations: constant velocity fails during aggressive maneuvers, and IMU-based deskewing is sensitive to IMU noise and bias errors. This paper proposes **Gaussian Process (GP) regression** to model the sensor trajectory within a scan, providing a smooth, uncertainty-aware interpolation that naturally handles varying motion dynamics.

---

## 2. Core Methodology

### 2.1 Trajectory as a Gaussian Process

The sensor trajectory $T(\tau) \in SE(3)$ over the scan period $[t_0, t_1]$ is modeled as a GP on the Lie algebra:

$$
\xi(\tau) \sim \mathcal{GP}(m(\tau), k(\tau, \tau'))
$$

where $\xi(\tau) = \log(T_0^{-1} T(\tau)) \in \mathfrak{se}(3)$ is the relative pose in the Lie algebra.

**Mean function**: From IMU integration (or constant velocity):
$$
m(\tau) = \int_{t_0}^{\tau} u(\tau') d\tau'
$$

**Kernel function**: Encodes the smoothness prior:
$$
k(\tau, \tau') = \sigma_f^2 \exp\left(-\frac{(\tau - \tau')^2}{2l^2}\right) \cdot I_6
$$

where $l$ is the length scale (controls smoothness) and $\sigma_f^2$ is the signal variance.

### 2.2 Training Points

The GP is trained on:
1. **IMU measurements**: Each IMU measurement at time $\tau_i$ provides a velocity observation:
   $$
   \dot{\xi}(\tau_i) = \begin{bmatrix} \omega_i - b_g \\ R(\tau_i)^T(a_i - b_a) + g \end{bmatrix}
   $$

2. **Endpoint constraints**: The estimated pose at the scan boundaries:
   $$
   \xi(t_0) = 0, \quad \xi(t_1) = \log(T_0^{-1} T_1)
   $$

### 2.3 GP Prediction for Deskewing

For each point at time $\tau_j$, predict the pose:
$$
\hat{\xi}(\tau_j) = m(\tau_j) + k(\tau_j, \boldsymbol{\tau})^T [K(\boldsymbol{\tau}, \boldsymbol{\tau}) + \sigma_n^2 I]^{-1} (\boldsymbol{y} - m(\boldsymbol{\tau}))
$$

$$
T(\tau_j) = T_0 \cdot \exp(\hat{\xi}(\tau_j))
$$

The uncertainty in the deskewing:
$$
\text{Var}[\xi(\tau_j)] = k(\tau_j, \tau_j) - k(\tau_j, \boldsymbol{\tau})^T [K + \sigma_n^2 I]^{-1} k(\boldsymbol{\tau}, \tau_j)
$$

This uncertainty can propagate to the point's position, providing **per-point uncertainty from deskewing**.

### 2.4 Adaptive Kernel Parameters

The kernel length scale $l$ adapts to the motion dynamics:
- **Smooth motion**: Large $l$ → smooth interpolation (few IMU measurements suffice)
- **Aggressive motion**: Small $l$ → allows faster-varying trajectory (follows IMU closely)

The length scale is set based on the IMU angular rate variance:
$$
l = l_0 / \sqrt{1 + \text{Var}[\omega] / \sigma_\omega^2}
$$

---

## 3. Key Equations

### GP predictive mean (compact form)
$$
\hat{\xi}(\tau) = m(\tau) + \mathbf{k}_*^T \alpha
$$

where $\alpha = (K + \sigma_n^2 I)^{-1}(\mathbf{y} - \mathbf{m})$ is precomputed once per scan.

### GP predictive variance
$$
\sigma^2(\tau) = k(\tau, \tau) - \mathbf{k}_*^T (K + \sigma_n^2 I)^{-1} \mathbf{k}_*
$$

### Deskewed point position
$$
p_j^{\text{deskewed}} = T(t_0)^{-1} T(\tau_j) p_j^{\text{raw}}
$$

### Per-point position uncertainty from GP
$$
\Sigma_{p_j} = J_{\xi \to p} \cdot \text{Var}[\xi(\tau_j)] \cdot J_{\xi \to p}^T
$$

---

## 4. Assumptions

1. **Trajectory is smooth**: The GP prior assumes smoothness via the kernel. Discontinuous motion (impacts, sudden stops) violates this.
2. **IMU measurements are noisy but unbiased**: The GP treats IMU measurements as noisy observations of the true trajectory derivative. Systematic bias should be estimated and removed beforehand.
3. **Kernel is appropriate**: The squared exponential kernel encodes a specific smoothness assumption. Other kernels (Matérn, etc.) may be more appropriate for different motion profiles.

---

## 5. Limitations

1. **Computational cost**: GP regression involves inverting a $N_{\text{train}} \times N_{\text{train}}$ matrix. For $N$ IMU measurements in a scan (typically 20 at 200 Hz × 100ms), this is $O(20^3) = O(8000)$ — negligible. But querying $M$ points is $O(M \cdot N^2)$.
2. **Global kernel parameters**: A single length scale for the entire scan may not capture varying dynamics within the scan (smooth then sudden turn).
3. **Lie algebra linearization**: Representing poses in the Lie algebra is an approximation that degrades for large rotations within a scan.
4. **Sparse IMU for short scans**: For very short scan periods or low IMU rates, the GP has few training points and the prediction uncertainty is large.
5. **Endpoint dependency**: The GP requires endpoint poses, which are the result of registration. This creates a circular dependency that must be resolved iteratively.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Applicable as a deskewing replacement**:

- **Replace linear IMU integration**: Instead of integrating IMU forward from the scan start, fit a GP to the IMU measurements and predict poses. This provides smoother, uncertainty-aware deskewing.
- **Per-point uncertainty**: The GP provides per-point deskewing uncertainty, which can be incorporated into the measurement noise $R_{jj}$ of the IEKF. Points with high deskewing uncertainty (fast motion, sparse IMU) get higher measurement noise.
- **Moderate implementation effort**: Requires implementing GP regression (small kernel matrix) and querying per point.

**Key benefit for FAST-LIO2**: The per-point uncertainty from deskewing provides a principled way to adjust the IEKF's measurement noise per point, rather than using a single constant $R$ for all points.

---

## 7. Applicability to Livox Mid-360

- **Per-point timestamps**: The Livox's microsecond timestamps enable precise GP querying at each point's exact time.
- **ICM40609 at 200 Hz**: With ~20 IMU measurements per 100ms scan, the GP has sufficient training data for smooth interpolation.
- **Non-repetitive pattern**: Different scan patterns may have different motion sensitivity. The GP's uncertainty naturally increases for time periods with less IMU support.
- **Vibration resilience**: The GP smooths over IMU vibration noise, providing a more stable deskewing trajectory than raw IMU integration.
