
Next steps:
1. Quantify the performance of our added features
	1. Simple comparison of our output to ground truth by plotting both trajectories on the same plot
	2. Later, look into metrics (EPE, ATE) to test the performance of the odometry relative to ground truth, as well as open-source datasets with ground truth to test these with
	3. Conduct these tests for individual features by themselves and also for combined features (for later, when individual features already work as desired)
2. Work on probabilistic degeneracy awareness
	1. Look into whether the implementation is correct
	2. Look into the math and see if it's correct or if it needs modification
		%%
		- Heuristic implementation: For the heuristic case, we want to just extract eigenvalues from the separate info matrices (one translation, one rotation), compute the probabilities from here, but then use the normal probabilistic formulation as derived last week (which combines both probabilities into a single P matrix). If Claude did it differently, try this out
		- Probabilities injected into IEKF: see if the additional RHS term makes any sense. Also look into our derivation last week and see if it makes sense. Why are we not directly working with the covariance matrix? 
		%%
	3. Test the IMU dead reckoning starting right before failure
	4. Test probabilities set to 0 starting right before failure and compare to IMU dead reckoning
%% 
3. Background
	1. Watch recursive estimation lectures
	2.  Read and understand [[FAST-LIO2 - Fast Direct LiDAR-inertial Odometry.pdf|FAST-LIO2]] and [[FAST-LIO - A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter.pdf|FAST-LIO]] papers 
%%
4. Work on adding intensity features
	1. Read [[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf|Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping]]
	2. Ideate how to implement intensities for the Livox-Mid360 (i.e. COIN-LIO style intensity handling with solid-state Livox Mid-360 LiDAR)
	3. Make an initial implementation

First:
- Test current implementation with 100-400 (with probs)
	- Done, turns out even with everything off it doesn't fail
- Make a script to extract trajectories

---
# Recall: proposed high-level pipeline

#### Step 1: outlier removal

- Per-point distance thresholding that selectively filters out invalid correspondences
- Outlier removal from [[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf|D²-LIO]]

#### Step 2: detect degeneracy 

- eigen-analysis
- get weak directions
- [[COIN-LIO - Complementary Intensity-Augmented LiDAR Inertial Odometry.pdf|COIN-LIO]]

### Step 3: improve measurements with intensity features

- select intensity features that maximize contribution along weak directions
- [[COIN-LIO - Complementary Intensity-Augmented LiDAR Inertial Odometry.pdf|COIN-LIO]]

#### Step 4: fuse measurements

- geometry + selected intensity features in the IEKF
- [[COIN-LIO - Complementary Intensity-Augmented LiDAR Inertial Odometry.pdf|COIN-LIO]]

#### Step 5: adaptive covariance/directional attenuation

- attenuate update along remaining weak directions
- done through probabilistic degeneracy mitigation as derived on [[Week 6 - Plan To Implement Degeneracy-Awareness]]
- With probabilities as derived on [[Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization.pdf|Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization]] 

#### Bonus step: adaptive parameter tuning based on environment detection 

- Adapt voxel size, search radius, and residual margin based on detected environment (if degenerate)
- [[AdaLIO - Robust Adaptive LiDAR-Inertial Odometry in Degenerate Indoor Environments.pdf|AdaLIO]]



---

# 1. Performance Metrics

Using end-pose-error is no longer the standard. We need
1. One global trajectory error metric
2. One local trajectory metric

This came from Prof. Scaramuzza's lecture in the *Vision Algorithms for Mobile Robotics* course:

## `rpg_trajectory_evaluation` (from RPG, Prof. Scaramuzza's lab)

![[10_multiple_view_geometry_4-67-69.pdf]]

![[Pasted image 20260420223745.png]]
https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf
[https://github.com/uzh-rpg/rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation)

As mentioned in Scaramuzza's lecture: to deal with a different number of ground truth vs. estimated poses, you can interpolate the trajectory with less poses with cubic splines. Alternatively, just use the lesser number of poses.

## `evo` (from Michael Grupp, most popular repo)

Computes:
- APE (absolute pose error)
- RPE (relative pose error)

→ What [[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf|D²-LIO]] uses

### APE: Absolute Pose Error
***The absolute pose error is a metric for investigating the global consistency of a SLAM trajectory***

APE is based on the absolute relative pose between two poses $P_{ref,i}, P_{est,i} \in \mathrm{SE}(3)$ at timestamp $i$:
$$
E_i = P_{est,i} \ominus P_{ref,i} = P_{est,i}^{-1} P_{ref,i} \in \mathrm{SE}(3)
$$
where $\ominus$ is the inverse compositional operator, which takes two poses and gives the relative pose [Lu-1997].

##### Intuitive Explanation:

In Euclidean space, error is computed as:
$$
x_{\text{est}} - x_{\text{gt}}
$$

This works because points lie in $\mathbb{R}^3$. However, a pose is:
$$
P =
\begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}
\in SE(3)
$$

It contains both rotation and translation, so subtraction is not meaningful:
$$
P_{\text{est}} - P_{\text{gt}} \quad \text{(invalid)}
$$

The correct way to compute pose error is to use inverse + composition:
$$
E = P_{\text{est}}^{-1} P_{\text{gt}}
$$

%% 
Interpretation:
- $P_{\text{est}}^{-1}$: transforms from estimated frame → world  
- $P_{\text{est}}^{-1} P_{\text{gt}}$: transforms from estimated frame → ground truth   
%%
 $E$ (pose error) is the **rigid transformation that maps the estimate to the ground truth**

The pose error is defined for one pose only (for one unit of time only)!

##### What `evo` includes:
You can use different pose relations to calculate the APE:
* **`metrics.PoseRelation.translation_part`**
    * this uses the translation part of $E_i$
    * $APE_i = \| \mathrm{trans}(E_i) \|$
    * since this is the distance measured in the shared inertial frame, it is equal to `metrics.PoseRelation.point_distance`
* **`metrics.PoseRelation.rotation_angle_(rad/deg)`**
    * uses the rotation angle of $E_i$
    * $APE_i = |( \mathrm{angle}(\log_{\mathrm{SO}(3)}(\mathrm{rot}(E_i)) )|$
    * $\log_{\mathrm{SO}(3)}(\cdot)$ is the inverse of $\exp_{\mathfrak{so}(3)}(\cdot)$ (Rodrigues' formula)
* **`metrics.PoseRelation.rotation_part`**
    * this uses the rotation part of $E_i$
    * $APE_i = \| \mathrm{rot}(E_i) - I_{3 \times 3} \|_F$
    * unit-less
* **`metrics.PoseRelation.full_transformation`**
    * this uses the full relative pose $E_i$
    * $APE_i = \| E_i - I_{4 \times 4} \|_F$
    * unit-less
    
Then, different statistics can be calculated on the APEs of all timestamps, e.g. the RMSE:
$$
\mathrm{RMSE} = \sqrt{ \frac{1}{N} \sum_{i=1}^N APE_i^2 } 
$$
### RPE: Relative Pose Error
***The relative pose error is a metric for investigating the local consistency of a SLAM trajectory***

RPE compares the relative poses along the estimated and the reference trajectory. This is based on the delta pose difference: 
$$E_{i,j} = \delta_{est_{i,j}} \ominus \delta_{ref_{i,j}} = (P_{ref,i}^{-1}P_{ref,j})^{-1} (P_{est,i}^{-1}P_{est,j}) \in \mathrm{SE}(3) $$

##### Intuitive explanation: 

Now instead of comparing **absolute poses** (only at timestamp $i$), we compare **motion between poses $i$ and $j$**:
###### Step 1: ground truth motion
$$
\delta_{\text{ref}} = P_{\text{ref},i}^{-1} P_{\text{ref},j}
$$
"How did the real system move from $i \rightarrow j$?"
###### Step 2: estimated motion
$$
\delta_{\text{est}} = P_{\text{est},i}^{-1} P_{\text{est},j}
$$
"How does SLAM think it moved?"
###### Step 3: compare them
$$
E_{i,j} = \delta_{\text{ref}}^{-1} \delta_{\text{est}}
$$
"What is the error in motion?"

RPE measures drift per segment (not total drift).

##### What `evo` includes:
You can use different pose relations to calculate the RPE from timestamp $i$ to $j$:
* **`metrics.PoseRelation.translation_part`**
    * this uses the translation part of $E_{i,j}$
    * $RPE_{i,j} = \| \mathrm{trans}(E_{i,j}) \|$
* **`metrics.PoseRelation.rotation_angle_(rad/deg)`**
    * uses the absolute angular error of $E_{i,j}$
    * $RPE_{i,j} = |( \mathrm{angle}(\log_{\mathrm{SO}(3)}(\mathrm{rot}(E_{i,j})) )|$
    * $\log_{\mathrm{SO}(3)}(\cdot)$ is the inverse of $\exp_{\mathfrak{so}(3)}(\cdot)$ (Rodrigues' formula)
* **`metrics.PoseRelation.rotation_part`**
    * this uses the rotation part of $E_{i,j}$
    * $RPE_{i,j} = \| \mathrm{rot}(E_{i,j}) - I_{3 \times 3} \|_F$
    * unit-less
* **`metrics.PoseRelation.full_transformation`**
    * this uses the full delta pose difference $E_{i,j}$
    * $RPE_{i,j} = \| E_{i,j} - I_{4 \times 4} \|_F$
    * unit-less
    
Then, different statistics can be calculated on the RPEs of all timestamps, e.g. the RMSE:
$$
\mathrm{RMSE} = \sqrt{ \frac{1}{N} \sum_{\forall ~i,j} RPE_{i,j}^2 } 
$$
---

## Comparing `evo` and `rpg_trajectory_evaluation`

`rpg_trajectory_evaluation`
- ATE: aligns full trajectory as best as possible using similarity transform, and computes RMSE of between all poses after alignment
- RTE/RE: Align initial pose of trajectories and compute EPE for short trajectory segments

`evo`
- APE: computes rigid transform between estimate and ground truth at a single point in time. Can be accumulated over full trajectory by taking RMSE and other statistics.
- RPE: computes motion between 2 selected poses (estimated and ground truth), and compares them. Again, can be accumulated over full trajectory by taking RMSE and other statistics.

These are not exactly the same metrics. ATE is the only truly global estimate. But since APE can be taken after alignment, the RMSE of APE after alignment is essentially ATE. This is probably why [[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf|D²-LIO]] mentions computing ATE with `evo`!

RPE and RTE are also similar. However, RTE also has a specific evaluation protocol, where one chooses a specific interval length and computes statistics for all sub-trajectories of that length. Here, it would be easier to use RTE directly, as RPE is very general and requires the explicit choosing of start and end time.

Still, `evo` is more widely used, supports many dataset formats, and very intuitive. So we will start with it.

---

### Data Formats supported by `evo`

Supports:

- TUM format

```
timestamp x y z q_x q_y q_z q_w
```

- KITTI format

*Remark:* This is actually not a real trajectory format because it has no timestamps - it only contains the poses in a text file. This means that you have to be careful when you want to compare two files in this format with a metric because the number of poses must be exactly the same.

Every row of the file contains the first 3 rows of a 4x4 homogeneous pose matrix (SE(3) matrix) flattened into one line, with each value separated by a space. For example, this pose matrix:

```
a b c d
e f g h
i j k l
0 0 0 1
```

- bag & bag2 (ROS/ROS2 bagfiles) that contain either
	- `geometry_msgs/PoseStamped`
	- `geometry_msgs/TransformStamped`
	- `geometry_msgs/PoseWithCovarianceStamped`
	- `geometry_msgs/PointStamped`
	- `nav_msgs/Odometry`
	- `tf2`

**Takeaway:** To build our ground truth, we need:
- timestamps
- positions
- orientations

→ Recommendation: let's stick to TUM format
- easy to make and understand
- used by quite a few ground truth datasets

---
## Tinamu ground truth

Currently, the ground truth I have from the globally optimized data includes
- Point cloud of the corrected map
- Point cloud with corrected poses collected at a 4 second interval
![[Pasted image 20260421115740.png]]
*Tinamu ground truth trajectory point cloud (Senegal Dataset)*

However, we cannot recover the metrics discussed above with this only, as
- Each pose does not have an associated timestamp
- Thus the estimated poses do not allow for the trajectory to be extracted
- We are also missing the orientations of each position, which is used in the computation of the performance metrics

Also, extraction every 4 seconds is perhaps too sparse, as handheld trajectories are quite erratic and unstable.

Thus we have two options
- Rely on open-source ground truth for performance quantification
- Extract trajectory from globally optimized map that
	- Includes timestamps for each extracted pose
	- Includes orientation for each extracted pose
	- Is ideally less sparse

---
## Open-source degeneracy datasets

https://github.com/thisparticle/Awesome-Algorithms-Against-Degeneracy 

[ENWIDE LiDAR Inertial Dataset](http://projects.asl.ethz.ch/datasets/enwide/)
- **Ouster OS0 128 (Rev D) LiDAR** (which includes IMU)
- Ground-truth position recorded from a **Leica MS60 Total Station** using a prism mounted on top of the LiDAR and time-synced accordingly.
- Used in COIN-LIO

[GEODE_dataset](https://github.com/PengYu-Team/GEODE_dataset)
- Device α:
    - **Velodyne VLP-16 ;**
    - Stereo HikRobot MV-CS050-10GC cameras;
    - Xsens MTi-30 IMU;
- Device β:
    - **Ouster OS1-64;**
    - Stereo HikRobot MV-CS050-10GC cameras;
    - Xsens MTi-30 IMU;
- Device γ:
    - **Livox AVIA;** (70.4° * 77.2°)
    - Stereo HikRobot MV-CS050-10GC cameras;
    - Xsens MTi-30 IMU;

**[lidar_degeneracy_datasets](https://github.com/ntnu-arl/lidar_degeneracy_datasets)**
- [Ouster OS0-128 LiDAR](https://ouster.com/products/scanning-lidar/os0-sensor/)
- [VectorNav VN100 IMU](https://www.vectornav.com/products/detail/vn-100)
- [Texas Instruments IWR6843AOP-EVM Radar](https://www.ti.com/tool/IWR6843AOPEVM)

**Problem:** 
- Most datasets use spinning LiDARs, and for the one that does use a Solid-State LiDAR, it's a narrow FoV one (Livox AVIA). 
- Thus for Tinamu, it does make sense to make our own ground truth data. 
- Nevertheless, for general comparison to other SLAM pipelines, it could also be nice to quantify error in one of these datasets, as FAST-LIO is compatible with any type of LiDAR anyways (as long as our intensity handling stays compatible with spinning LiDARs, otherwise only the GEODE dataset with the Livox AVIA will be usable).

---

# Using `evo`

## 1. Implementing pose extraction in TUM format

I have implemented a writer that creates a trajectory output file at node startup, and appends one TUM row after the posterior pose is computed
#### Dummy example: `AST_LIO_trajectory_everything_false` vs. `AST_LIO_trajectory_everything_true`

Trajectory plots:
![[Pasted image 20260421125816.png]]
![[Pasted image 20260421125838.png]]
![[Pasted image 20260421125850.png]]
![[Pasted image 20260421125858.png]]

APE translation, not aligned:

```
(.venv) andres@LaptopAndres:~/semester_project/data$ evo_ape tum /home/andres/semester_project/data/estimated_trajectories/FAST_LIO_trajectory_e
verything_false.txt /home/andres/semester_project/data/estimated_trajectories/FAST_LIO_trajectory_everything_true.txt

APE w.r.t. translation part (m)
(not aligned)

       max      0.054014
      mean      0.007389
    median      0.005209
       min      0.000000
      rmse      0.010051
       sse      0.100123
       std      0.006815
```

RPE translation, not aligned:

```
(.venv) andres@LaptopAndres:~/semester_project/data$ evo_rpe tum /home/andres/semester_project/data/estimated_trajectories/FAST_LIO_trajectory_e
verything_false.txt /home/andres/semester_project/data/estimated_trajectories/FAST_LIO_trajectory_everything_true.txt

RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      0.013831
      mean      0.002493
    median      0.001963
       min      0.000084
      rmse      0.003145
       sse      0.009792
       std      0.001917
```

**Conclusion:** easy to use, intuitive, great visualizations!

#### Sanity checks
For baseline, comparing 2 trajectories from the same exact settings (everything set to false):

```
APE w.r.t. translation part (m)
(not aligned)

       max      0.000000
      mean      0.000000
    median      0.000000
       min      0.000000
      rmse      0.000000
       sse      0.000000
       std      0.000000

RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      0.000000
      mean      0.000000
    median      0.000000
       min      0.000000
      rmse      0.000000
       sse      0.000000
       std      0.000000
```

Comparing original FAST-LIO (with no modifications) with my version where everything is set to false:

```
APE w.r.t. translation part (m)
(not aligned)

       max      0.000000
      mean      0.000000
    median      0.000000
       min      0.000000
      rmse      0.000000
       sse      0.000000
       std      0.000000
       
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max      0.000000
      mean      0.000000
    median      0.000000
       min      0.000000
      rmse      0.000000
       sse      0.000000
       std      0.000000
```

%% 
## 2. Comparing original FAST-LIO vs. FAST-LIO with outlier removal
- Make one general script for comparing trajectories that we can use fully until the end of the project! 
%%

---
### Side note: LiDAR intensity

The reported intensity is mainly influenced by:
- Surface reflectivity: e.g. white walls vs. asphalt
- Distance to the object : longer distance = weaker return
- Incidence angle: grazing angles reflect less energy back
- Material properties: specular vs. diffuse reflection

So fundamentally, intensity is a property of the laser-object interaction, not the environment lighting. This is because
- LiDAR uses narrow-band laser wavelengths (near-infrared)
- Sensors have optical filters tuned to that wavelength

However, light can matter under some specific cases
- Strong sunlight outdoors: sun emits in near-infrared which overlaps with LiDAR wavelength, adds background noise to the detector (lowers SNR)
- Low reflectivity + long range: similar effect - with an already weak return signal, ambient light can increase the effect of noise

→ Even in complete darkness, you will still get range + intensity returns

---

# 2. IMU Dead Reckoning

Implement IMU-only dead reckoning by:
- Keeping the **IEKF prediction (IMU propagation)** active  
- Disabling the **LiDAR measurement update** after a chosen start time 

→ This corresponds to **IEKF prediction-only mode**, which is effectively **IMU-only dead reckoning**

**Key nuance**
- **IEKF prediction-only**: propagates state *and covariance* using IMU  
- **IMU dead reckoning**: motion estimated purely from IMU, no external corrections  

→ For this use case, they are equivalent at the **state estimate level**, with IEKF additionally tracking uncertainty

This is implemented in a switch manner. The system behavior is:
- Before switch time
	- IMU used for state propagation  
	- LiDAR used for drift correction (IEKF update)  
- After switch time
	- IMU continues to propagate state  
	- LiDAR updates are disabled  
	- Disable map insertion / scan matching

The switch time is chosen manually. This will allows us to test
- Whether the IMU is the cause of the increase in drift when using probabilities to mitigate the effect of LiDAR in degenerate directions
	- Test for the whole trajectory
	- Test only near failure mode

Nuance: We wait for IMU to be initialized before starting dead-reckoning

### FAST-LIO2 vs. Dead-Reckoning

- Senegal
- Dead reckoning starting at t = 0 (green)
- Original FAST-LIO2 (blue)

![[Pasted image 20260421143850.png]]
![[Pasted image 20260421143937.png]]
![[Pasted image 20260421143946.png]]
![[Pasted image 20260421143953.png]]

- Cameroon
- Dead reckoning from t = 145 (~5s before failure) to t = 150 (green)
- Original FAST-LIO2 (blue)

![[Pasted image 20260421145835.png]]
- It actually saves itself! But still drifts
- Probably manages to stop drifting as it essentially "starts a new map" after drifting a bit for those 5 seconds
![[Pasted image 20260421150115.png]]
![[Pasted image 20260421150127.png]]
![[Pasted image 20260421150135.png]]
![[Pasted image 20260421150147.png]]

Senegal:
- put small cuts of 1-2 seconds when in the tunnel

New dataset in the office:
- do small cuts in a similar fashion so that you can see if it's worth just trusting IMU

Looking at how long can IMUs do dead reckoning for normally 





%% 
# 2. Intensities

Relevant papers:
- [[COIN-LIO - Complementary Intensity-Augmented LiDAR Inertial Odometry.pdf]]
- [[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf]]


---

 %%
