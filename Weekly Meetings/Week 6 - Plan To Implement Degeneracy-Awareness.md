---

---

# New datasets

### 1. Valis

Full on geometric degeneracy (due to tunnels!)

![[image 230.png]]

![[image 231.png]]

![[image 232.png]]

### 2. Senegal

Same type of issue:

![[image 233.png]]

![[image 234.png]]

---

# Intensity Features

### Visualizing intensities

Changed script to do “auto exposure” while removing very high/low intensity points:

- Ignore outliers when making the intensity colormap
    - lower bound = 2nd percentile
    - upper bound = 98th percentile
- Normalize using this range
- Anything outside the range gets pushed into the range through clipping

There seems to be some informative intensity information in the LiDAR intensities:

![[image 235.png]]

![[image 236.png]]

Spherical projection:

![[image 237.png]]

We can see geometrical features through our intensities!

- lines on the roof
- lines and corners on the piles of bags

We have an unused avenue for more features!

→ Caveat: these visualizations are for the full map that was built. Per scan, it will be much more sparse

---

# Degeneracy Awareness

Geometrically Stable Sampling for the ICP Algorithm

![[image 238.png]]

Methods that handle degeneracy by modulating LiDAR measurement trust through the covariance matrix can be split into two families:

**A. Direct covariance / information shaping of LiDAR measurements**

**B. Covariance-like weighting / regularization that changes LiDAR-vs-prior balance**

We can also subdivide them into:

**1. Global Uncertainty Handling (direction-agnostic)**

**2. Locally Directional Uncertainty Handling (anisotropic)**

Let’s look at each of these to decide what to implement on our FAST-LIO pipeline:

---

### D²-LIO: Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy
[[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf]]

- **Covariance-like weighting / regularization that changes LiDAR-vs-prior balance**
- **Locally Directional Uncertainty Handling (anisotropic)**

They separate degeneracy as coming from 3 different cases:

1. Low observability of state variables: As illustrated in Fig. 3 of [23], certain environment types lead to unobservable variables that degrade SLAM performance
    - This means “the environment does not give you enough information to estimate all motion directions”
    - Geometrically degenerate environments where LiDAR measurements simply do not constrain certain degrees of freedom (DoF), and some parts of the state (translation or rotation) become unobservable
2. Point cloud mismatches: Unstructured environments tend to cause point cloud mismatches during the ICP process, resulting in errors in the formulation of the optimization
    - SLAM can fail due to incorrect correspondences, which can come from:
        - noise
        - sparsity
        - motion distortion
        - dynamic objects
→ causes **incorrect point matches**
3. Ill-conditioned optimization: In these scenarios, the ICP cost function tends to exhibit flat minima, making the optimization process highly sensitive and prone to error
    - Where the ICP cost function is a “flat valley”, with an ambiguous minimum (not the same as low observability, where no information exists)

Their main contributions are:

<u>**1. Per-point distance thresholding mechanism that selectively filters out invalid correspondences**</u>

- Rejects points whose residuals are above a certain threshold (as we discussed before)
- But the threshold itself is computed **dynamically **and **per-point**
- This is because standard ICP threshold $||r_j||<\tau$ 
    - Far points are rejected much more than nearby points due to higher penalty (as the spatial deviation due to motion grows with distance)
    - Ignores how much motion happened between scans (larger motion causes larger residuals, which needs higher thresholding)

![[image 239.png]]

Translation term: 

- How much the robot moved linearly between scans → **displacement due to translation**
- Ex: If the robot moved 1 meter, then a point could legitimately shift up to ~1 meter

Rotation term = chord length of a rotation: 

- If a point is at distance $r = \|p_j\|$, and you rotate by angle $\theta = \|\Delta R\|$
- The displacement of that point is computed with $2 r \sin(\theta/2)$ → **displacement due to rotation**

→ **upper bound on how much a point could move due to the motion**

(Translation and rotation estimates come from IMU pre-integration)

ATE Results (Average Trajectory Error):

![[image 240.png]]

<u>**2. Degeneracy-Aware Regularization Strategy**</u>

ICP solves the following optimization problem:

$$
c(\mathbf{T}) = \sum_{j=1}^{N} \rho\left(\left\| e_j(\mathbf{T}) \right\|^2\right)
$$

where

- $e_j(\mathbf{T}) = \mathbf{T}\,\hat{\mathbf{p}}_j^{\ell_k} - \hat{\mathbf{q}}_j$: matching error (residual) between the transformed input point $\hat{\mathbf{p}}_j^{I_k}$ and the corresponding reference point $\hat{\mathbf{q}}_j$
- $\rho(\cdot)$: robust kernel function to reduce the influence of outliers

From the linearized version:

$$
\min_x ||Ax-b||^2
$$

From the eigenvalue analysis of $A^TA$, a regularization penalty is added for each direction of the state-space (each eigen-direction of the Hessian):

$$
\frac{1}{\lambda_i} \left\| (\mathbf{x} - \hat{\mathbf{x}}_k) \cdot \mathbf{v}_i \right\|^2
$$

- Ensures that the optimal solution is “not far away” from the initial guess $\hat{x}_k$ (from the IMU pre-integration)
- Regularization strength varies inversely with $\lambda_i$ → stronger constraints for more degenerate directions

Summing over all directions gives:

$$
\sum_{i=1}^N \frac{1}{\lambda_i} \left\| (\mathbf{x} - \hat{\mathbf{x}}_k) \cdot \mathbf{v}_i \right\|^2
= \left\| \mathbf{x} - \hat{\mathbf{x}}_k \right\|^2_{(\mathbf{A}^\top \mathbf{A})^{-1}}
$$

This is great, but we are not taking into account the uncertainty in IMU. We can do so by taking the norm over the combined terms:

$$
\mathbf{W} = \left(\mathbf{A}^\top\mathbf{A} \mathbf{\hat{P}}_{\mathbf{T},k} \right)^{-1}
$$

where:

- $\mathbf{A}^\top\mathbf{A}$ is the LiDAR information matrix
- $\mathbf{\hat{P}}_{\mathbf{T},k}$ is the covariance matrix associated with the pose estimate obtained via IMU pre-integration

They also do a separate eigenspace analysis for the rotational and translational components of the Hessian due to the scale differences between both, yielding in the final objective function for LiDAR scan matching as follows:

$$
\arg\min_{\mathbf{T}} 
\sum_{j=1}^{N} \rho\left( \left\| \mathbf{e}_j(\mathbf{T}) \right\|^2 \right)
+ w \left( 
\left\| \mathbf{e}_r(\mathbf{T}) \right\|^2_{\mathbf{W}_r}
+ \left\| \mathbf{e}_t(\mathbf{T}) \right\|^2_{\mathbf{W}_t}
\right)
$$

where $\mathbf{e}_r(\mathbf{T})$ and $\mathbf{e}_t(\mathbf{T})$ denote the rotational and translational residuals, and where the translation and rotation norm components are:

$$
\mathbf{W}_r = \left( \mathbf{A}_r^\top \mathbf{A}_r \, \hat{\mathbf{P}}_{r,k} \right)^{-1}
$$

$$
\mathbf{W}_t = \left( \mathbf{A}_t^\top \mathbf{A}_t \, \hat{\mathbf{P}}_{t,k} \right)^{-1}
$$

Results:

![[image 241.png]]

![[image 242.png]]

Important: this has been made for an optimization-based point-cloud registration algorithm, not for a Kalman filter framework. Adapting such a regularization is not 100% straight forward:

→ We already have the IMU prior embedded into the problem. We would simply need to down-weigh the LiDAR “trust” through the covariance matrix

---

### Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization
[[Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization.pdf]]

- **Direct covariance / information matrix shaping**
- **Locally Directional Uncertainty Handling (anisotropic)**

Open-source:

[https://github.com/ntnu-arl/drpm](https://github.com/ntnu-arl/drpm)

“The implementation can be used in your point cloud registration pipeline by including the `degeneracy.h` file in your project.”

They argue that dealing with degeneracy through eigenvalue analysis is fragile:

- Proposed methods require careful fine-tuning tailored to the robot, sensor, environment and particulars of the localization pipeline they are used in
- The eigenvalues of the Hessian depend on the number of measurements, weights, the scale of the environment and sensor noise
- Tuning parameters cannot be expected to generalize across heterogeneous scenarios
- [15], [19] propose dynamic parameters for [14], but offer limited generalizability as only a subset of the sources of variation in eigenvalues is accounted for
- The localizability metric of [20] accounts for scale. However, the method considers the rotational and translational blocks of the Hessian separately and may fail when the full Hessian is degenerate while the individual blocks are not.

This paper reframes degeneracy as:

- Not “small eigenvalue = bad”
- But “low signal-to-noise = bad” → potentially much more robust

Proposal:

- probabilistic characterization of the noise entering the Hessian
- probability of a direction being effectively non-degenerate = probability of the signal being at least an order of magnitude higher than the noise
- method explicitly accounts for environment scale, weights and measurement count
- The noise models used are based on the LiDAR’s datasheet
- real-time compatible plugin step and can be incorporated in common non-linear least squares point-to-plane optimizers while retaining the overall worst-case computational complexity

From the derivation (Sec. II-D):

$$
(H + H_N)\hat{x} = Hx^* + H_N x_N
$$

- $H$: true geometry (signal)
- $H_N$: noise-induced structure (fake information)

→ Degeneracy is fundamentally a signal-to-noise problem:

- The Hessian $H$ from the least-squares problem contains true geometric information
- Noise introduces a spurious Hessian component

Noise models:

- Points: Gaussian noise
- Normals: small rotational noise

By propagating noise, they derive:

$$
\mathbb{E}[\hat{H}] = H + \Sigma
$$

For each direction $u$, compute:

- Signal: $a_u = u^T H u$
- Noise: $\xi_u \sim \mathcal{N}(\mu_u, \sigma_u^2)$

Probability of being non-degenerate:

$$
p_u = P(a_u \ge s \cdot \xi_u)
$$

- s = 10s→ signal must be 10× noise
- So:
    - $p_u \approx 1$ → reliable direction
    - $p_u \approx 0$ → degenerate direction

**Using this in ICP**

After linearization, ICP gives:

$$
\min_x \|Jx - b\|^2
$$

Taking the gradient and setting to zero gives:

$$
J^T J x = J^T b
$$

Let the Hessian be $H=J^T J$. We thus get:

$$
x=H^{-1}J^T b
$$

We can diagonalize the Hessian:

$$
H=U\Lambda U^T
$$

- $U$: eigenvectors (directions in state space)
- $\Lambda = \text{diag}(\lambda_1, ..., \lambda_6)$

Thus we get

$$
x=U\Lambda^{-1} U^T J^T b
$$

Each component of $\Lambda$ controls how much you move along direction $u_k$ by factor $\frac{1}{\lambda_k}$.

- large $\lambda_k$ → divide by big number → small update → stable
- small $\lambda_k$ → divide by small number → **huge update → dangerous**

**Hard thresholding**

Define threshold $\lambda_{\min}$

Then:

$$
\lambda_k^{-1} =\begin{cases}\frac{1}{\lambda_k} & \lambda_k > \lambda_{\min} \\0 & \text{otherwise}\end{cases}
$$

where the update becomes

$$
x = U \Lambda^{\dagger} U^{T} J^{T} b
$$

where

$$
\Lambda^{\dagger} = \operatorname{diag}\left(\begin{cases}\frac{1}{\lambda_k} & \text{if safe} \\0 & \text{if degenerate}\end{cases}\right)
$$

**Their proposal**

They substitute $\lambda_k^{-1}$ with

$$
\lambda_k^{-1} \rightarrow p_k \cdot \frac{1}{\lambda_k}
$$

The final update becomes:

$$
x^* = U P \Lambda^{-1} U^T J^T b
$$

where

$$
P = \mathrm{diag}(p_1, \ldots, p_6)
$$

This does two things simultaneously:

4. Attenuates unreliable directions: If $p_k \approx 0$→ no update
5. Adds implicit regularization: Equivalent to $\text{least squares} + \text{Tikhonov (L2) regularizer}$
= From a least-squares perspective, adding a zero-prior in directions where $p_{u_k} < 1$

![[image 243.png]]


---

### Discussion: What to implement on FAST-LIO2?

**Main differences:**

- Probabilistic Degeneracy Detection
    - Down-weighing measurements in degenerate directions
    - Adds a zero prior: penalizes motion magnitude in directions judged degenerate
    - weak LiDAR direction → suppress LiDAR-driven motion
- D²-LIO 
    - Adds regularization term that can be interpreted as a “stay-close-to-the-IMU-prediction prior”
    - weak LiDAR direction → lean harder on IMU prior


The probabilistic paper:

- stronger at detecting whether a direction is genuinely trustworthy as it reasons about signal vs noise rather than just raw Hessian magnitude
- explicitly tries to avoid being fooled by eigenvalues that change with scale, count, and weighting. 

D²-LIO:

- stronger at keeping the optimizer well anchored once a direction is weak, because it does not merely suppress motion
- “if LiDAR is weak here, use the IMU-predicted pose as the anchor”
-  Useful as we do not want a direction to just freeze

While what D²-LIO does is interesting for ICP, we already have the IMU as our prior due to the IEKF formulation of FAST-LIO2. So an explicit regularizer is not needed for us.

From the probabilistic paper, we can use the $P$ matrix to weigh the LiDAR contribution in the IEKF.

6. estimate directional degeneracy probabilities $p_k$ 
7. use these to attenuate IEKF measurement update so that weak LiDAR directions automatically fall back to the IMU prior
8. add D²-LIO’s robust correspondence filtering in front of this, because that part is complementary, and deals with dynamic/unreliable points (low hanging fruit)

### Proposed pipeline:

We would not want to scale all states. Only the translations and rotations. From the stacked FAST-LIO2 LiDAR Jacobian $H \in \mathbb{R}^{m\times 24}$, extract only the columns corresponding to translation and rotation. Call this:

$$
H_{\text{pose}} \in \mathbb{R}^{m\times 6}
$$

Then build the 6D LiDAR pose information matrix

$$
\Lambda_{\text{pose}} = H_{\text{pose}}^T R^{-1} H_{\text{pose}}
$$

This is the FAST-LIO2 equivalent of the point-to-plane Hessian used in the degeneracy paper. Now eigendecompose:

$$
\Lambda_{\text{pose}} = U \Lambda U^T
$$

where:

- $U\in \mathbb{R}^{6\times 6}$ eigenvectors
- $\Lambda = \mathrm{diag}(\lambda_1,\dots,\lambda_6)$

Then compute your degeneracy weights/probabilities:

$$
P = \mathrm{diag}(p_1,\dots,p_6), \qquad p_k \in [0,1]
$$

Then comes the novelty. To inject matrix $P$ into the IEKF, we use the following matrix

$$
A = UP^{1/2}U^T,
$$

to replace the pose part of the LiDAR Jacobian by

$$
H_{pose} \rightarrow H_{pose}A.
$$

So:

$$
H'=\begin{bmatrix}H_{\text{pose}} A & H_{\text{other}}
\end{bmatrix}
$$

To show that this works, let’s compute the full modified information matrix $(H')^T R^{-1}H'$:

$$
\begin{bmatrix}
A^T H_{\text{pose}}^T \\
H_{\text{other}}^T
\end{bmatrix}
R^{-1}
\begin{bmatrix}
H_{\text{pose}} A & H_{\text{other}}
\end{bmatrix}
$$

$$
= \begin{bmatrix}
A^T H_{\text{pose}}^T R^{-1} H_{\text{pose}} A &
A^T H_{\text{pose}}^T R^{-1} H_{\text{other}} \\
H_{\text{other}}^T R^{-1} H_{\text{pose}} A &
H_{\text{other}}^T R^{-1} H_{\text{other}}
\end{bmatrix}
$$

Since $A = U P^{1/2} U^T$ is symmetric, $A^T = A$, so this becomes:

$$
= \begin{bmatrix}
A H_{\text{pose}}^T R^{-1} H_{\text{pose}} A &
A H_{\text{pose}}^T R^{-1} H_{\text{other}} \\
H_{\text{other}}^T R^{-1} H_{\text{pose}} A &
H_{\text{other}}^T R^{-1} H_{\text{other}}
\end{bmatrix}
$$

Now substitute the eigendecomposition of the pose block:

$$
\Lambda_{\text{pose}}=H_{\text{pose}}^T R^{-1} H_{\text{pose}} = U \Lambda U^T
$$

Then the top-left block becomes:

$$
A H_{\text{pose}}^T R^{-1} H_{\text{pose}} A = 
(U P^{1/2} U^T)(U \Lambda U^T)(U P^{1/2} U^T)
\\= U P^{1/2} \Lambda P^{1/2} U^T \\= UP\Lambda U^T
$$

So the final result is:

$$
\boxed{
(H')^TR^{-1}H'
=
\begin{bmatrix}
U P \Lambda U^T &
A H_{\text{pose}}^T R^{-1} H_{\text{other}} \\
H_{\text{other}}^T R^{-1} H_{\text{pose}} A &
H_{\text{other}}^T R^{-1} H_{\text{other}}
\end{bmatrix}
}
$$

→ We need to make sure that the properties of the information matrix are kept with our transformation

We can also we-write this as a full state transform matrix:

$$
A_{\text{full}} =
\begin{bmatrix}
A & 0 \\
0 & I
\end{bmatrix} = \begin{bmatrix}
U P^{1/2} U^T & 0 \\
0 & I
\end{bmatrix}
$$

Then:

$$
H'=HA_{\text{full}}
$$

and therefore:

$$

(H')^T R^{-1} H'
=
A_{\text{full}}^T H^T R^{-1} H A_{\text{full}}
$$

Since $A_{\text{full}}$ is symmetric:

$$


(H')^T R^{-1} H'
=
A_{\text{full}} H^T R^{-1} H A_{\text{full}}
$$

### Comparison

Instead of computing the probabilities, we could also use a more heuristic downscaling estimate to populate matrix $P$ by using eigenvalues directly, instead of the full probabilistic description. Let’s call the matrix $P'$:

$$
P' = \mathrm{diag}(p'_1,\dots,p'_6), \qquad p'_k \in [0,1]
$$

For example:

<u>Option 1: Hard threshold</u>

$$
p'_k =
\begin{cases}
1, & \lambda_k > \lambda_{\min}\\
0, & \lambda_k \le \lambda_{\min}
\end{cases}
$$

<u>Option 2: Soft threshold</u>

$$
p'_k = \frac{\lambda_k}{\lambda_k+\alpha}
$$

- if $\lambda_k \gg \alpha$ → $p_k \approx 1$
- if $\lambda_k =\alpha$ → $p_k = 0.5$
- if $\lambda_k \ll \alpha$ → $p_k \approx 0$

Here is an example for $\alpha = 2$:

![[image 244.png]]

Could also do a sigmoid:

$$
p_k = \sigma\left(\frac{\lambda_k - \tau}{s}\right)
$$

where $\tau$ is the threshold at which the probability goes above 0.5, and $s$ dictates how quickly the probability rises. Here is an example for $\tau = 5$ and $s = 1$:

![[image 245.png]]

For these, we should compute the probability components of the translation and rotation separately due to their difference in scale. 

<u>Alternative: Eigenvalues on prior-whitened LiDAR information:</u>

We extract the LiDAR pose information as before with

$$
\Lambda_{\text{pose}} = H_{\text{pose}}^T R^{-1} H_{\text{pose}}
$$

But from here, we whiten it with the IEKF predicted pose covariance:

$$
\tilde{\Lambda}_{\text{pose}} = P_{\text{pose}}^{1/2}{\Lambda}_{\text{pose}}P_{\text{pose}}^{1/2}
$$

Whitening expresses LiDAR information in units of the prior uncertainty. This is because whitening is a change of coordinates that transforms the state such that prior covariance becomes identity. It tells us whether LiDAR is weak or strong relative to what the IMU already knows!

Hence, we can eigendecomponse this matrix to get:

$$
\tilde{\Lambda}_{\text{pose}} = U\tilde{\Lambda} U, \qquad
\tilde{\Lambda}=\mathrm{diag}(\tilde\lambda_1,\dots,\tilde\lambda_6).
$$

Interpretation of $\tilde\lambda_k$:

- $\tilde\lambda_k \ll 1$: LiDAR adds much less information than the prior along eigen-direction $u_k$
- $\tilde\lambda_k \approx 1$: LiDAR and prior have comparable strength
- $\tilde\lambda_k \gg 1$: LiDAR is much stronger than the prior in that direction

Set:

$$
p'_k=\frac{\tilde\lambda_k}{1+\tilde\lambda_k}
$$

Form:

$$
P'=\mathrm{diag}(p'_1,\dots,p'_6).
$$

---

### SLAM evaluation tools

Option 1: “evo”

[https://github.com/michaelgrupp/evo](https://github.com/michaelgrupp/evo)

→ What D²-LIO used to compute ATE (average trajectory error)

Option 2: From Prof. Scaramuzza’s lab

[https://github.com/uzh-rpg/rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation)

---

### Principled ICP Covariance Modelling in Perceptually Degraded Environments for the EELS Mission Concept

- **Covariance-like weighting / regularization that changes LiDAR-vs-prior balance**
- **Locally Directional Uncertainty Handling (anisotropic)**

Different from the other papers: 

- they compute the pose covariance of the registration result of ICP 
- this is to be then used during fusion with other sensors trough a tightly-coupled factor graph (GTSAM), where the ICP covariance is used as a Gaussian pose-to-pose factor/weight

→ much less translatable to FAST-LIO

They compare:

- Constant covariance
- Hessian-based covariance $\Sigma = \sigma^2 (A^T A)^{-1}$
- Proposed (Censi-based) covariance with
    - isotropic noise
    - range-based sensor noise

Importantly, they say that the final registration uncertainty should depend on both

9. the scene geometry / conditioning (what we have been looking at)
10. the underlying LiDAR point measurement noise

They specifically add LiDAR sensor noise modeling and achieve slightly higher performance with it. However, this is much less important than the degeneracy-awareness we need, and as such will be left at that.


- think of/see how you will implement the probabilistic one
- also look at D2-LIO and see how the regularization term works once again and see if it can have a MAP interpretation, and maybe if we can weigh directly using eigenvalues instead of the probabilities for a first implementation (or just to compare)

For next time:

1. Try to implement outlier removal
2. Understand FAST-LIO2 paper
3. Try to implement degeneracy probability thing
    1. With heuristic probs
    2. With proper probs ([GitHub - ntnu-arl/drpm](https://github.com/ntnu-arl/drpm/tree/main))
4. Work on the intensities
    3. Figure out how to use COIN-LIO style intensity handling with solid-state Livox Mid-360 LiDAR
