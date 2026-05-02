---

---

**Goals for this week: **

1. Understand what is being done to tackle the problems we want to improve from FAST-LIO2:
    - Handling geometric degeneracy 
    - Additional feature modalities
    - Hand-held LIO systems
    - “Ghosting” of dynamic objects
2. Proposing potential solution/pipeline to be implemented over the upcoming weeks, which tackles
    1. Robust to geometrically degenerate scenes
    2. Robust to hand-held use
    3. Suitable for solid-state LiDARs (Livox Mid360, non-spinning & irregular scan pattern)

# Literature Review

Lots of papers are using FAST-LIO2 as a basis for their developments (ex: COIN-LIO, AKF-LIO…), replacing the trend of using LOAM or LIO-SAM as the baseline. As such, this further enforces FAST-LIO2 being seen as the state-of-the-art LIO algorithm at the moment, “the one to beat”.

<u>Overview</u>

3. **Degeneracy-Aware LIO**
    1. On Degeneracy of Optimization-based State Estimation Problems
    2. Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization
    3. A Real-time Degeneracy Sensing and Compensation Method for Enhanced LiDAR SLAM
    4. D²-LIO
    5. AdaLIO
    6. DAMM-LOAM
    7. GenZ-ICP
    8. LP-ICP
    9. LODESTAR
    10. AKF-LIO
    11. X-ICP
    12. Principled ICP Covariance Modelling in Perceptually Degraded Environments for the EELS Mission Concept
4. **Dealing with dynamic objects/hand-held motion**
    13. RF-LIO
    14. AS-LIO
    15. HTCO
5. **Multi-Modal LIO (additional feature modalities)**
    16. **Intensity/reflectivity features**
        1. COIN-LIO
        2. Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping
        3. Intensity-enhanced LiDAR-inertial odometry with gradient flow sampling
        4. Enhanced LiDAR-inertial SLAM with adaptive intensity feature extraction
        5. IGE-LIO - Intensity Gradient Enhanced Tightly Coupled LiDAR–Inertial Odometry
    17. **Edge features**
        6. LP-ICP
    18. **Other features**
        7. Extracting general-purpose features from LIDAR data
        8. NV-LIO - LiDAR-Inertial Odometry using Normal Vectors Towards Robust SLAM in Multifloor Environments
    19. **Semantic-aware**
        9. SE-LIO - Semantic-Enhanced Solid-State-LiDAR–Inertial Odometry for Tree-Rich Environments
6. **Continuous-time**
    20. Eigen is all you need

---

## 1. Multi-Modal LIO (additional feature modalities)

### a. Intensity/reflectivity features

### COIN-LIO

- From 2024
- From ASL at ETH
- Open-source

[https://arxiv.org/abs/2310.01235](https://arxiv.org/abs/2310.01235)

[https://github.com/ethz-asl/COIN-LIO](https://github.com/ethz-asl/COIN-LIO)


![[image 220.png]]

- Based on FAST-LIO2 (keeps the ikd-tree, point-to-line residuals and IEKF formulation), but adds intensity modality specifically to **provide information in uninformative directions of the point cloud geometry**
- Projects LiDAR points onto an image through spherical projection
- Processes the image through a pipeline to reduce line artifacts that come from the different scans, give consistent brightness accross the image, etc. to end up with usable images for photometric residuals

![[image 221.png]]

- Perform information analysis of the LiDAR geometry to detect uninformative directions:
    - Inspired from X-ICP
    - Calculates principal components of Hessian $H^{geo^T}H^{geo}$ of the point-to-plane terms → direction is uninformative if the accumulated filtered contribution is below a threshold
    - If uninformative, they analyze the **translational components** and store the uninformative directions
- **Select intensity photometric features where shifting the feature point along an uninformative 3D direction results in a 2D coordinate shift in an informative image direction** (details omitted for consciences) 
    1. **They use of photometric intensity features is only done when in an uninformative (degenerate) case**
    2. **The photometric features are chosen such that they are informative in the uninformative directions!**
- They stack the point-to-plane and photometric terms into a combined residual vector and Jacobian for the IEKF, with the photometric terms being scaled to compensate for the difference in magnitudes between the geometric and photometric residuals:

![[image 222.png]]

**Performance results:**

<!-- Column 1 -->
They also made a dataset consisting of degenerate scenarios, can be very useful for our own testing:

[https://projects.asl.ethz.ch/datasets/enwide/](https://projects.asl.ethz.ch/datasets/enwide/)


<!-- Column 2 -->
![[image 223.png]]


And clearly it shows that where other LIO algorithms fail (including FAST-LIO), COIN-LIO doesn’t:

![[image 224.png]]

**Problem: **not compatible with solid-state LiDARs! Relies on regular angular scanning like a spinning LiDAR to project the LiDAR scan into an intensity image, as it uses a spherical projection model:

- the image resolution corresponds to LiDAR horizontal and vertical resolution
- rows correspond to laser beams
- columns correspond to azimuth angle

**Research opportunity:** integrate intensity features in degenerate directions similar to COIN-LIO, but for solid-state LiDARs

However, it is important to remember:

- Flower / rosette scan pattern, irregular angular sampling → unstable pixel correspondences!

I still wanted to test it with our data just to see, but unfortunately:

![[image 225.png]]


---


Other intensity-augmented LIO papers:

- Intensity enhanced for **solid-state-LiDAR** in simultaneous localization and mapping
    - Focused on preprocessing for irregular scan patterns
    - Uses intensity normalization to make intensity measurements usable (as they are affected by reflectivity, distance, angle…)
    - Key idea: compensate sensor limitations (small FoV)
- IGE-LIO - Intensity Gradient Enhanced Tightly Coupled LiDAR–Inertial Odometry
    - Extracts **intensity edge points**, and combines with geometric edge and planar point features in a weighted residual fusion
- Enhanced LiDAR-inertial SLAM with adaptive intensity feature extraction
    - Edge and planar features (curvature-based) + intensity edge features in an **adaptive **weighted fusion
    - Adaptive weights based on
        - Geometric features → based on curvature
        - Intensity features → based on intensity gradient strength
        - Residuals (small → higher confidence)
- Intensity-enhanced LiDAR-inertial odometry with gradient flow sampling
    - More degeneracy-aware handling of intensity features
    - Uses point-wise photometric error, which gets around the issue we would have if we want to implement something like COIN-LIO but adapted for Livox Mid-360 (but needs good quality correspondences)


---

### b. Other features

**NV-LIO (Normal Vector features)**

- Uses surface normal vectors (local orientation of surfaces) extracted from LiDAR scans
- Normals encode geometric structure beyond just point positions
- Improves correspondence by enforcing consistency in both position *and* orientation
- Analyzes distribution of normal directions to detect degeneracy and adjust uncertainty
- Especially useful in indoor / repetitive environments (walls, corridors, stairs) where geometry is ambiguous

**LP-ICP (Point-to-line features)**

- Extends ICP by incorporating point-to-line constraints (edge features) in addition to planes, which is what ICP normally uses (similar to how FAST-LIO2 only uses point-to-plane)
- Improves observability in degenerate environments (e.g. tunnels, flat areas)
- Could be argued that FAST-LIO2 could also benefit from point-to-line residuals
- Also performs localizability analysis per constraint to detect weak directions, and adds constraints in optimization to stabilize ill-conditioned directions

### c. Semantic-Aware LIO

**SE-LIO (Semantic features)**

- Uses semantic segmentation of point clouds (e.g. ground, trees, buildings, dynamic objects)
- Introduces semantic geometric models (e.g. tree trunks → cylinders)
- Enhances estimation via class-specific constraints (point-to-cylinder, point-to-plane)
- Particularly effective in unstructured environments where classical features fail


---

## 2. Degeneracy-Aware LIO

### Summary: techniques used

7. **Detect bad directions**
    - eigenvalues / eigenvectors / singular values / condition number
    - probabilistic degeneracy probability
    - scene-type degeneracy detection
8. **Global uncertainty handling: Change sensor trust / covariance if in a degenerate case (degeneracy direction agnostic)**
    - adaptive R
    - adaptive Kalman gain
    - Schmidt-Kalman structure
9. **Subspace / directional observability handling: Modify the update in degenerate directions (degeneracy direction aware)**
    - solution remapping
    - soft attenuation (equivalent to downweighing degenerate directions on Kalman gain)
    - regularization / TSVD / Tikhonov
10. **Point-wise handling: Weight or prune points / residuals**
    - point-wise observability weighting (to strengthen good constraints)
    - point-wise pruning for unreliable points (redundant, inconsistent…)
    - robust outlier thresholds (remove points if residual too high)
    - residual reweighting (down-weighting points with large residuals)
11. **Use better geometry**
    - point-to-line
    - point-to-point vs point-to-plane switching
    - semantic / structural classification
12. **Borrow information from time/other sensors**
    - IMU stronger weighting
    - past-state constraints (fixed-state anchors)
13. **Change front-end parameters when degeneracy is likely**
    - voxel size (reduced in degenerate cases)
    - normal estimation parameters

### On Degeneracy of Optimization-based State Estimation Problems

[https://frc.ri.cmu.edu/~zhangji/publications/ICRA_2016.pdf](https://frc.ri.cmu.edu/~zhangji/publications/ICRA_2016.pdf)

- The “original” paper that tackles degeneracy-aware odometry
- Argues that state state estimation (both visual and LiDAR odometry) can fail in degeneracy environments, where some directions of motion are poorly constrained:
    - Vision: low texture, poor lighting
    - LiDAR: planar or geometrically simple scenes
- However, they already note that even when a problem is degenerate, it is not degenerate in all directions
- Instead of fixing the whole problem, only solve it in well-conditioned directions and handle the rest differently

→ Really helps understand the intuition behind degeneracy detection and handling. Very well written paper, cited by many of the other degeneracy-aware LIO methods cited above.

<u>Problem statement:</u>

For linear problem (whether it has been linearized or is inherently linear):

$$
\arg \min_x ||Ax-b||^2
$$

- They consider each row as a (hyper-) plane in the space of x → each row represents a unique constraint
- They study the geometric distribution of these planes to determine degeneracy

![[image 226.png]]

- Problem: 
    - Given a linearized system as the equation above, determine degeneracy and corresponding degenerate directions in the state space. 
    - In the case of degeneracy, prevent faulty solutions from occurring in the degenerate directions.

<u>Main contributions:</u>

**Degeneracy factor **$D$

![[image 227.png]]

- Measures how unstable the solution is to perturbations: $D = \frac{\delta d}{\delta x_c^*}$
- Derived from the system matrix $A$:
$$
D = \lambda_{\min} + 1
$$
- Where $\lambda_{\min}= \text{smallest eigenvalue of } A^T A$
- The associated eigenvector, denoted as $v_{min}$, represents the first degenerate direction
- For the eigenpairs of $A^T A$:

$$
(\lambda_1,v_1),(\lambda_2,v_2),...,(\lambda_n,v_n)
$$

        each eigenvalue corresponds to how well constrained that specific direction is

- They also derive a proof stating that degeneracy is purely about constraints, not measurements (only depends on $A$ (constraint directions), not $b$)

**Solution remapping**

![[image 228.png]]

- They threshold eigenvalues to detect which directions are degenerate or not. Those with eigenvalues below the threshold are degenerate

![[image 229.png]]

- You have two candidate solutions: prediction (prior) $x_p$ and update (optimizer result) $x_u$
- You decompose both solutions into components along eigenvectors
- For degenerate directions, you use the prediction $x_p$
- Otherwise, you use the update $x_u$
- You reconstruct the final result with $x_f = x_p' + x_u'$

### **AKF-LIO**

14. <u>Adaptive Kalman Filter</u>
Instead of fixed measurement noise R, it is adapted online based on measurement quality. If LiDAR constraints are:
    - good: trust LiDAR more
    - bad: trust IMU more
“Quality” is measured through residual statistics and innovation consistency:
    - if residuals are large → measurement unreliable → increase R
    - if residuals are small → measurement reliable → decrease R

This is a filter-level solution. It directly modifies:
$$
K= P H^T (H P H^T + R)^{-1}
$$
Larger R → smaller Kalman gain → less LiDAR influence
15. <u>Represent local structure using Gaussian distributions, so each voxel stores mean and covariance</u>
16. <u>Measurement uncertainty modeling</u>

They explicitly model uncertainty of LiDAR measurements dependent on

**Limitations:**

17. No explicit degeneracy modeling/detection, they just react to bad residuals
- Residual-based adaptation is noisy. Residuals can spike due to:
    - dynamic objects
    - outliers
    - bad correspondences
- This can lead to unstable R updates
- No directional awareness
    - R is adapted globally, but degeneracy is directional (e.g., corridor → yaw unobservable)


### **GenZ-LIO**

- Adapts voxel size dynamically based on scene scale
- Handles transitions like indoor → outdoor
- Uses hybrid residuals (point-to-plane + point-to-point) to reduce degeneracy

---

## 3. Dealing with dynamic objects (ghosting)

### **RF-LIO: Removal-First Tightly-coupled Lidar Inertial Odometry in High Dynamic Environments**

[https://arxiv.org/abs/2206.09463](https://arxiv.org/abs/2206.09463)

### Different idea

Lightweight implementation strategy for FAST-LIO2, using the already computed residuials:

- **Calculate Residual**: $r_i = \text{point-to-plane distance}$.
- **Filter State**: Use $w_i = \exp(-k \cdot r_i^2)$ for the IEKF update.
- **Filter Map**: `if (r_i < static_threshold) ikdtree.Add_Points(pt_i);`


---

# 2. Proposed high-level pipeline

### Step 1: detect degeneracy

- eigen-analysis
- get weak directions

### Step 2: improve measurements with intensity features

- select intensity features that maximize contribution along weak directions

### Step 3: fuse measurements

- geometry + selected intensity features in the IEKF

### Step 4: adaptive covariance/directional attenuation

- adjust global trust based on fused measurements

AND/OR?

- attenuate update along remaining weak directions

### Step 5: don’t map unreliable points

- if residuals are too high for certain points, don’t include them in the map

We actively reshape the information matrix with our extra sensing modality (intensity)! 