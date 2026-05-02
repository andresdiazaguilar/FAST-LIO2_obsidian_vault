---

---
# a) Extracting data to understand our failure modes

Summary:

1. Extracting corner features
2. SVD of Information Matrix
3. Plotting acceleration

1 and 2 will let us know more about the LiDAR sensor/geometry information, 3 will let us know about the Kalman filter estimates.

# 1. Extracting corner features

### The solid-state Livox LiDAR problem

Livox LiDAR data output look like so:

```c++
offset_time: 99506047
x: -1.0850000381469727
y: 0.6399999856948853
z: 0.27900001406669617
reflectivity: 8
tag: 0
line: 3
```

Problem:

- LOAM/LIO-SAM corner feature extraction relies on having information about which scan line the LiDAR points come from
- The Livox Mid-360 Lidar is a solid-state LiDAR with non-repetitive scanning. This means that it does not have scan lines
- Line ≠ scan line, it varies between 0 and 3, and is the only output not mentioned in the Livox wiki

We need an alternative approach to extracting corner features. After looking at the literature, conclusions are:

- Curvature-based methods like LOAM are still possible with some workarounds. 
    - `loam_livox` specifically adapts the LOAM algorithm to tackle the issue of the non-repetitive, irregular scanning pattern of Livox LiDARs. 
    - We can thus adapt their feature detection pipeline to FAST-LIO to extract edge and planar features
- Alternative methods exist. An interesting one is a PCA-based edge and plane detection algorithm, using nearest neighbors to analyze the local geometry
- We will use a PCA-based edge detection method for now due to its easier implementation, to allow us to have an estimate on the number of edge features for our failure case study

## PCL-based corner feature extractor

### Idea comes from this paper:

> The eigenvalues are arranged in ascending order $\lambda_1$, $\lambda_2$, and $\lambda_3$. If the set of these points satisfies the geometry of the plane, then $\lambda_1/\lambda_2\leq\alpha$. If the set of these points satisfies the geometric characteristics of the edge, then $\lambda_2/\lambda_3\leq\beta$. In this study,  $\alpha$ is set to 0.22 and $\beta$ is set to 0.2 through experimental testing.
> [[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf]]

My implementation is not exactly the same as what they do, because they only use this as a check after doing a curvature analysis. Instead, what I do is the following:

For each point of the already down-sampled and motion-compensated point cloud:

4. Find its k nearest neighboring points using a KD-tree of the point cloud of the current frame
5. Compute the 3D covariance matrix of those neighbor coordinates
    - Compute centroid of cluster of points $\mu$
    - Compute vector $d_i = p_i - \mu$, where $p_i$ is the position of each point in the cluster
    - Average over covariance of all points to get the covariance matrix:
$$
C = \frac{1}{N}\sum_{i=1}^N d_id_i^T \in \R^{3\times3} 
$$
This represents the shape of the local point distribution.
6. Do eigenvalue decomposition of that covariance: $\lambda_1 \leq \lambda_2 \leq \lambda_3$
7. Classify local shape from eigenvalue ratios:
    - Edge (where $\lambda_1 \approx \lambda_2 \ll \lambda_3$):
        - $\lambda_2/\lambda_3 < \alpha$ , as variance mostly along one direction.
    - Plane (where $\lambda_1 \ll \lambda_2 \approx \lambda_3$):
        - $\lambda_1/\lambda_2 < \beta$ 
        - $\lambda_2/\lambda_3 > \gamma$, variance lies in a 2D sheet. 
    - Otherwise unstructured
    - Current parameters: $\alpha = 0.1$, $\beta = 0.2$, $\gamma = 1/3$

**Caveat:** this method cannot extract an intersection between two planes as an edge! This will only be possible if we move on to a curvature-style feature extractor method.

### Results

![](https://youtu.be/ALxvBG2vazg)

![](https://youtu.be/LgsYdOtjc2s)

Link: [PCL LiDAR Feature Extraction](https://youtu.be/LgsYdOtjc2s)

If the plane feature extraction is correctly implemented, the trend in the number of features over time should be similar to the effective number of point-to-plane features we see used in FAST-LIO2. This is indeed the case:

![[image 212.png]]

---

## LOAM-Livox

[https://arxiv.org/abs/1909.06700](https://arxiv.org/abs/1909.06700)

### Motivation:

Standard LOAM algorithms were designed for mechanical spinning LiDARs (e.g., Velodyne), which have:

- Wide 360° field of view
- Regular scan patterns (rings)

However, solid-state LiDARs introduce new challenges:

8. Small field of view (not a concern for us)
9. Irregular scanning pattern - Rosette-like pattern rather than rings
10. Non-repetitive scanning - Scan pattern changes every frame
11. Motion distortion - Points are captured at different times during scanning

### Point Selection:

Before feature extraction, the algorithm filters unreliable points.

For each point $\mathrm{P}=[x,y,z]$, the following attributes are computed:

12. Depth:

$$
D(\mathrm{P})=\sqrt{x^2 + y^2 + z^2}
$$

13. Deflection angle: Angle between laser ray and x-axis

$$
\phi(\mathrm{P}) = \tan^{-1}\left({\sqrt{y^2 + z^2/x^2}}\right)
$$

14. Intensity: Reflectivity normalized by distance

$$
I(\mathrm{P}) = R/D(\mathrm{P})^2
$$

15. Incident angle: Angle between the laser ray and the surface

$$
\theta(P_b) =
\cos^{-1}\!\left(
\frac{(\mathrm{P}_a - \mathrm{P}_c)\cdot \mathrm{P}_b}
{\lvert \mathrm{P}_a - \mathrm{P}_c \rvert \, \lvert \mathrm{P}_b \rvert}
\right)
$$

Unreliable points are then filtered to increase the localization and mapping accuracy. Points are removed if they are:

16. Near the FoV boundary (scanning geometry unreliable)
17. Intensity too high or too low
    - high → sensor saturation
    - low → noise
18. Incident angle too small or large (as the laser spot will be elongated, and the measured range becomes the average of the area covered by the large spot instead of a specific point)
19. Occluded (hidden behind objects), which would cause a false edge feature otherwise

### Feature extraction

- Original LOAM assumes points come in ordered scan lines, neighbors are the nearby points in the same scan
- LOAM-Livox approximates this by segmenting “pseudo-scan lines”
- They use the temporal order + rosette pattern geometry of Livox LiDARs to create these sequences
- They then run LOAM feature extraction on the segmented sequences

### ROS Implementation

Open-source ROS package, easier for future implementation if we want to use edge features

[https://github.com/hku-mars/loam_livox#](https://github.com/hku-mars/loam_livox#)

---

### Discussion: does it make sense to use corner features?

- FAST-LIO2 already uses all raw points directly for registration rather than only corner/plane features, so adding LOAM-Livox-style edge feature extraction would not give it new geometric information
- FAST-LIO2’s main design choice is to avoid hand-engineered feature extraction and “directly register raw points to the map” because this can exploit subtle geometry and remain robust across different LiDAR scan patterns
- However, FAST-LIO2 uses point-to-plane residuals, which constrain the motion less than point-to-line residuals would
- So in a sense, while it wouldn’t add any additional geometry (no extra LiDAR points), having an additional point-to-line residual with edge features could further constrain our motion
- This would mean:
    - keep existing direct planar update
    - extract corner features only when available
    - compute point-to-line correspondences
    - append those residuals to the IEKF update

---

### Alternative methods to compute corner features

If we want to actually use corner features, we could consider alternative methods of computing them. 

20. Map the point cloud onto a 2D grid using range and azimuth angles, compute local curvature, and extract features using covariance eigenvalues (where eigenvalues describe local point distribution). See paper below:
[[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf]]
21. FAST-LIO1 edge and planar feature extraction. Also available under the old commits of the FAST-LIO open-source codebase

Other alternatives exist in the literature, may be interesting to take a look.

# Conclusion: 

If we want to use corner features, a LOAM-Livox approach would be a good idea given its workarounds for being compatible with Livox-style LiDARs. However, a deeper look into other methods of computing corner features could also be warranted.

---

### Alternatives to corner features

It is important to note that edge and planar features are not the only options. It could be interesting to look at other alternatives:

- Intensity features (reflectivity changes), which are used in combination with edge and planar points to add an additional feature constraint due to the challenges that come from solid-state LiDARs:
[[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf]]
- General-purpose features:
[[Extracting general-purpose features from LIDAR data.pdf]]



---


# 2. Eigenvalues & Eigenvectors of Information Matrix

- Before, we were looking at plots of the minimum eigenvalue and condition number of the “geometric” information matrix and of the “posterior” information matrix. 
- Problem: This does not take into account scale differences between position and rotation
- We will thus plot these separately to have better insight. Furthermore, we will look at all of the eigenvalues separately to see how many directions become poorly constrained at the moment of failure
- Final plots
    - **Figure A:** pre translation eigenvalues (3 curves)
    - **Figure B:** pre rotation eigenvalues (3 curves)
    - **Figure C:** post translation eigenvalues (3 curves)
    - **Figure D:** post rotation eigenvalues (3 curves)
    - **Figure E: **pre translation condition number
    - **Figure F: **pre rotation condition number
    - **Figure G: **post translation condition number
    - **Figure H: **post rotation condition number

Note: Looking at condition numbers separately really clear up what’s going on!!

- Also added a visualization for the weakest eigenvector direction (associated with the smallest eigenvalue) on RVIZ to see which direction is most poorly constrained geometrically. This was done for the 
    - pre translation eigenvalues
    - pre rotation eigenvalues

![](https://www.youtube.com/watch?v=Rt048oL92QQ&feature=youtu.be)

Translation is red, rotation is blue

---

# 3. Visualizing acceleration

In order to see if there is an attitude estimation problem, we can plot the IMU linear acceleration compensated by the computed bias $b$, and rotated according to the computed rotation matrix $R$:

$$
R(a_m−b_a)
$$

Both $b$ and $R$ are computed by the Kalman filter.

On top of this, we can compare this to the estimated gravity vector. Both have been added as RVIZ markers for visualization.

![](https://youtu.be/ZaH500n61nw)

[Cameroon acceleration and gravity rviz plots](https://www.youtube.com/watch?v=ZaH500n61nw&embeds_referring_euri=https%3A%2F%2Fwww.notion.so%2F)

To more easily see the discrepancy between the gravity vector and the rotation- and bias-compensated acceleration, the following plots were added:

- Residual between the gravity estimate and the component of the acceleration that is aligned with gravity
- Magnitude of the component of the acceleration vector that is orthogonal to gravity 

The expressions were derived as follows:

![[image 213.png]]

You can then get the residual between $|g|$ and $|a_\parallel|$ through $r=|g|-|a_\parallel|$ .

An increase in both $r$ and $|a_\perp|$ means that either:

- The sensor rig is undergoing acceleration not caused by gravity
- The rotation (and possibly bias) are not well estimated by the IEKF, causing a belief of sideways acceleration, which may worsen the potential for drift → can be improved with attitude estimation

---

# b) Failure mode analysis

# 1. Cameroon dataset

![](https://www.youtube.com/watch?v=ZaH500n61nw&embeds_referring_euri=https%3A%2F%2Fwww.notion.so%2F)

![[02_feature_count_diagnostics.png]]

- Planar features (and thus effective feature number) dips around failure time
- Number of edge features is consistently lower than planar ones (partly due to the parameter choice, but in practice will still remain lower), and is particularly low right before failure time. However, quick spike in edge features right after failure, could potentially help guard against the drift starting

![[01_imu_diagnostics.png]]

- There is an increase in rotation velocity around the failure point, but not unusual compared to elsewhere in the dataset

![[04_gravity_diagnostics.png]]

- Compensated linear acceleration remains pretty consistent around failure time when compared to earlier in the dataset (vertical component is still similar to gravity in magnitude, horizontal component remains close to 0). It is only after the initial drift that the acceleration estimates start to change

![[03_residuals.png]]

- Residuals immediately start rising as soon as we start drifting. However, this is a symptom of the drift, cannot help us predict it
![[05_eigenvalues_prepost_translationrotation.png]]

![[06_condition_numbers_and_minimum_eigenvalues.png]]

- During the initial drift, the **translation **information matrix seems ill-conditioned
    - Huge spike in condition number/dip in min eigenvalue
    - Particularly in the pre/geometric info matrix, meaning that the environment geometry is **degenerate** and does **not constrain translation **well
- During initial drift, the **rotation** information matrix is also not very well conditioned, but with nowhere near as much of a spike as with **translation**
- It seems that the degeneracy comes mostly from **translation**, and not from **rotation** in this case

### Conclusion: 

Failure seems to come from poorly constrained translation!

- Low number of effective features
- Spike in condition number of translation information matrix
- Better conditioned rotation information matrix, well behaved compensated acceleration readings (meaning that $R$ and $b$ estimates are good)

# 2. Silo dataset

2 different failures! Both due to the same thing: drift during the beginning of the map given a low number of features

- Case 1: drift forwards, both maps don’t overlap at all - fail at ~21s (starts drifting slowly at ~21s, gets bad at ~26s onwards)
- Case 2: drift backwards, both maps almost overlap, but mismatch on one side - fail at ~36s, when the person does the full turn of the silo and the maps try to match

<!-- Column 1 -->
![[image 214.png|Case 1]]

<!-- Column 2 -->
![[e9662b11-2231-4549-ba86-49009739a1f9.png|Case 2]]

→ Once again, failure seems to be feature-related. Let’s look at the plots:

## Case 1

![[01_imu_diagnostics 1.png]]

- Large-ish angular velocity around failure time

![[02_feature_count_diagnostics 1.png]]

- Super low (almost none) # features during 8-18s! Cause of our issues

![[03_residuals 1.png]]

![[04_gravity_diagnostics 1.png]]

- Acceleration components get worse after drift starts, do not seem to be the root cause

![[05_eigenvalues_prepost_translationrotation 1.png]]

![[06_condition_numbers_and_minimum_eigenvalues 1.png]]

- Rotation seems to be more poorly conditioned than translation in this case! 

## Case 2

![](https://www.youtube.com/watch?v=fkr8mGyuPz4)

[Silo dataset failure case 2](https://www.youtube.com/watch?v=fkr8mGyuPz4)

![[01_imu_diagnostics 2.png]]

![[02_feature_count_diagnostics 2.png]]

![[03_residuals 2.png]]

![[04_gravity_diagnostics 2.png]]

![[05_eigenvalues_prepost_translationrotation 2.png]]

![[06_condition_numbers_and_minimum_eigenvalues 2.png]]

### Conclusion:

This is a classic case of geometric degeneracy:

- Degeneracy doesn’t happen right at the time of the drift, but instead towards the beginning of the dataset (8-18s roughly). 
- LiDAR seems to be covered during this time, giving very few features during this time, which is when an initial drift happens. 
- The map that is built becomes inaccurate when the LiDAR is uncovered again (previous points don’t line up with current points)

# 3. Drift_gtc dataset

Drift from 0-25s, 42-50s, 90-95 → all happening at narrow corridors

0-25s:

![[image 215.png|Initial drift (in red)]]

42-50s:

![[image 216.png|Ghosting (2 instances of the same floor)]]

![[image 217.png|This was due to a tight corridor]]

![[image 218.png|Combined by an extremely narrow FoV]]

![](https://www.youtube.com/watch?v=yjQiv5JLnK8)

![[02_feature_count_diagnostics 3.png]]

- Low number of features preceeding/during the failure modes once again, due to extremely narrow LiDAR FoV. Edge features won’t help here either.

![[01_imu_diagnostics 3.png]]

- Peak in $\omega_z$ right before second failure mode
- Linear acceleration readings seem pretty consistent

![[03_residuals 3.png]]

- Residuals are high during failure modes once again

![[04_gravity_diagnostics 3.png]]

- Acceleration components are extremely jittery → compensated acceleration vector is very jittery (see video) - perhaps due to rotation matrix estimate

![[05_eigenvalues_prepost_translationrotation 3.png]]

![[06_condition_numbers_and_minimum_eigenvalues 3.png]]

### Conclusion:

- Very low number of features due to very narrow FoV seems to cause the drifts
- However, jitter in compensated acceleration is unusual compared to previous datasets

***Sidenote: How is the gravity estimate computed?***

The reason why the gravity estimate norm stays constant over time is due to the assumption in the state transition model that $^G\dot{g}=0$ (see below):

![[image 219.png]]

# Conclusion

### Main problem

When the number of features is too low:

- Narrow FoV
- Covered LiDAR
- Very narrow corridors

However, other potential issues noticed:

- Jittery compensated acceleration (rotation matrix/bias estimates are jittery?)
- Biases not yet well calibrated
- Certain component of high angular velocity

### Ideas

- Adding degeneracy-awareness will be helpful
    - use min eigenvalue/condition number of geometric information matrix to either
        - weight the contribution of the LiDAR in the estimate
        - fully remove the contribution of the LiDAR in the estimate
- Adding extra feature method will be helpful
    - edge features?
    - camera features
- Unsure how much attitude estimation will help 


# Overall view

### Problems

Seem to be map related:

- Lack of constraining features (ex: narrow corridors)
- Ghosting

Fixes:

- Uncertainty weighting (based on the condition number/eigenvalue) → degeneracy-awareness
    - Look into degeneracy-aware SLAM/odometry papers, look into how they do it 
    - Look into how FAST-LIO2 weighs translation and rotation uncertainty/covariance, as this is what we will be modifying
- Additional features
    - Edge features with point-to-line residuals
        - What’s the real differences between LOAM-style methods and FAST-LIO style methods? (Compare FAST-LIO2 to LOAM-LIVOX)
    - Intensity features
    - General-purpose features
- Fixing ghosting
    - Remove all points from within 1 or 2m from the LiDAR
    - Problem: gets rid of nearby features, problematic in narrow corridors
    - Could look into how people deal with this

Overall problem: want to get robust LiDAR-IMU odometry method for solid-state LiDARs. 

- Solid-state LiDAR: constraint in terms of scanning pattern, traditional LOAM methods don’t work
- Direct point-to-plane methods like FAST-LIO2 that don’t require scan lines lack constraining features in corridor-like cases, causing drift

To do:

22. Uncertainty weighting
    1. Look into degeneracy-aware SLAM/odometry papers, look into how they do it 
    2. Look into how FAST-LIO2 weighs translation and rotation uncertainty/covariance, as this is what we will be modifying
23. Additional features
    - Edge features with point-to-line residuals
        - Compare FAST-LIO2 to LOAM-LIVOX to understand what the real differences between LOAM-style methods (feature-based) and FAST-LIO2 style (”direct”) methods are. 
        - How does FAST-LIO2 do point-to-plane without detecting planar features? Which gives better performance, FAST-LIO2 style or LOAM-style?
        - Compare FAST-LIO to FAST-LIO2 → Is the feature extraction in FAST-LIO compatible with solid-state LiDARs?
    - Intensity features
        - Look into how people use this in the literature
    - General-purpose features
        - Look into how people use this in the literature
    - Look into other alternative features with LiDAR or IMU
    - Look into how people deal with LiDAR-inertial odometry with solid-state LiDARs in general
- Fixing ghosting
    - Look into how people deal with ghosting for LiDAR-IMU odometry
