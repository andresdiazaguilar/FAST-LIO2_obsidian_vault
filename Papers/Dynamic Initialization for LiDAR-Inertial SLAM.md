# Dynamic Initialization for LiDAR-Inertial SLAM

**Authors**: Xu et al., 2025, IEEE/ASME Transactions on Mechatronics  
**Category**: Initialization (suggested paper)  
**Failure Modes Addressed**: FM8 (Map-History / Initialization Sensitivity)

---

## 1. Problem Statement

FAST-LIO2's initialization assumes the system starts stationary (to estimate gravity direction and initial gyroscope bias from the accelerometer reading). This fails when:
1. The robot is already moving at startup
2. The platform experiences vibration at startup (vehicle, drone)
3. The initial IMU bias is large and corrupts the gravity estimate
4. The system restarts mid-mission (e.g., after a software crash)

This paper proposes a **dynamic initialization** method that can estimate initial states (pose, velocity, IMU biases, gravity) while the system is in motion.

---

## 2. Core Methodology

### 2.1 Multi-Frame Initialization

Instead of using a single stationary period, collect $K$ frames of LiDAR data and associated IMU measurements:

$$
\{(S_1, U_1), (S_2, U_2), \ldots, (S_K, U_K)\}
$$

where $S_k$ is the $k$-th LiDAR scan and $U_k$ is the IMU data between scans $k-1$ and $k$.

### 2.2 Frame-to-Frame Registration

Register consecutive scans to get relative pose estimates:
$$
\hat{T}_{k,k+1} = \text{ICP}(S_k, S_{k+1})
$$

These provide **LiDAR-only pose estimates** that don't depend on IMU initialization.

### 2.3 Joint Optimization

Solve for initial states by jointly optimizing over all $K$ frames:

$$
\{v_0, b_g, b_a, g\} = \arg\min \sum_{k=1}^{K-1} \|T_{k,k+1}^{\text{LiDAR}} - T_{k,k+1}^{\text{IMU}}(v_0, b_g, b_a, g)\|^2
$$

The LiDAR provides relative pose constraints; the IMU provides dynamic constraints parameterized by the initial velocity, biases, and gravity.

### 2.4 Gravity Estimation

The gravity direction $g$ is parameterized as a 2-DOF quantity on $S^2$ (the unit sphere):
$$
g = \|g\| \begin{bmatrix} \sin\theta \cos\phi \\ \sin\theta \sin\phi \\ \cos\theta \end{bmatrix}
$$

where $\|g\| = 9.81$ m/s² is assumed known, and $\theta, \phi$ are estimated.

### 2.5 Observability Analysis

The paper analyzes the observability of the initialization parameters:
- **$b_g$**: Observable from the discrepancy between LiDAR-estimated rotation and IMU-integrated rotation
- **$b_a$**: Observable from the discrepancy between LiDAR-estimated translation and IMU-integrated translation, given known gravity
- **$g$**: Observable from the accelerometer-velocity relationship over multiple frames
- **$v_0$**: Observable from the position changes over the initialization window

**Minimum frames**: At least $K = 3$ frames are needed for full observability. In practice, $K = 10-20$ provides robust initialization.

---

## 3. Key Equations

### IMU pre-integration between frames $k$ and $k+1$
$$
\Delta R_{k,k+1} = \prod_{i \in [k,k+1]} \exp((\omega_i - b_g) \Delta t)
$$
$$
\Delta v_{k,k+1} = \sum_{i \in [k,k+1]} \Delta R_{k,i} (a_i - b_a) \Delta t
$$
$$
\Delta p_{k,k+1} = \sum_{i \in [k,k+1]} (\Delta v_{k,i} \Delta t + \frac{1}{2} \Delta R_{k,i} (a_i - b_a) \Delta t^2)
$$

### Rotation residual
$$
r_R^{(k)} = \log(\Delta R_{k,k+1}^T R_k^{L,T} R_{k+1}^L)
$$

### Translation residual
$$
r_t^{(k)} = R_k^L (p_{k+1}^L - p_k^L) - v_k \Delta t - \frac{1}{2} g \Delta t^2 - \Delta p_{k,k+1}
$$

### Velocity propagation
$$
v_{k+1} = v_k + R_k^L \Delta v_{k,k+1} + g \Delta t
$$

---

## 4. Assumptions

1. **Sufficient LiDAR registration quality**: Frame-to-frame ICP must succeed during initialization. In featureless environments, this may fail.
2. **IMU data is available**: Continuous IMU measurements during the initialization window.
3. **Gravity magnitude is known**: $\|g\| = 9.81$ m/s² is assumed. At high altitudes or on other planets, this must be adjusted.
4. **Linear acceleration is bounded**: The initialization assumes the acceleration is bounded so that the optimization converges.

---

## 5. Limitations

1. **Initialization delay**: Requires $K$ frames (~1-2 seconds at 10 Hz), during which the system cannot provide poses. For applications requiring instant-on, this is a delay.
2. **Frame-to-frame registration noise**: ICP between consecutive frames is noisier than scan-to-map registration. The initialization may be less accurate than a well-initialized system after convergence.
3. **Degenerate motion during initialization**: If the system moves with constant velocity (no acceleration) during initialization, $b_a$ and $g$ are not independently observable.
4. **Computational overhead**: The joint optimization over $K$ frames is a one-time cost but can be significant for large $K$.
5. **No online re-initialization**: The paper addresses startup initialization, not mid-operation re-initialization after tracking failure.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable as an initialization module**:

- **Replace stationary initialization**: FAST-LIO2's current initialization collects IMU measurements while stationary to estimate gravity and gyro bias. Replace with the dynamic initialization that works during motion.
- **Pre-IEKF module**: Run the dynamic initialization for the first $K$ scans, then switch to the standard IEKF with the estimated initial states.
- **Bias seeding**: The estimated biases from initialization provide a much better starting point for the IEKF's online bias estimation.

**Integration approach**:
1. At startup, run frame-to-frame ICP for $K$ scans
2. Solve the joint optimization for $v_0, b_g, b_a, g$
3. Initialize the IEKF state with these values
4. Switch to standard scan-to-map IEKF processing

---

## 7. Applicability to Livox Mid-360

- **Mobile robots**: Robots equipped with the Mid-360 often start while already moving (e.g., drones, delivery robots). Dynamic initialization eliminates the need for a stationary startup.
- **ICM40609 bias**: The initial gyro and accel biases of the ICM40609 can be significant. The dynamic initialization estimates them jointly with gravity, providing better initial values.
- **Frame-to-frame ICP**: The Mid-360's 360° FoV provides good frame-to-frame overlap, supporting reliable ICP during initialization.
- **Fast deployment**: Enables "instant-on" operation — no need for a calibration/stationary phase before the robot can navigate.
