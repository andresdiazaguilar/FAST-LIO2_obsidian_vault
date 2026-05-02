
---

# 1. Internal Datasets

## a. Valis

![[Pasted image 20260421190448.png]]

Initial drift:
- Traditional drift due to degenerate geometry

Extreme drift due to very low FOV:
- Having almost no LiDAR features cause us to rely on IMU more.
- But due to the angular drift of the map, the **attitude is potentially mis-estimated**. 
- This incorrect attitude caused a **sideways acceleration** that does not exist in real life, which then caused **sideways motion** when the pose was not constrained by LiDAR features (moment of very low FOV, very few LiDAR features). 
- On top of this, our IMU is low-cost and thus can only handle a few seconds of dead-reckoning before serious drift, which does not help with the pose estimation during these low feature moments.

>**Note to Stefan**: Please let me know if you find this to be a plausible explanation to this drift during very low FOV!

Notes:
- Mild drift starts from ~85-90s (enters hallway)
- Strong drift between 145-160 due to very low FOV, up to 190s still very low FOV but pose has stabilized

---
## b. Senegal

![[Pasted image 20260421190505.png]]

- More classic hallway drift due to degenerate geometry (hallway)

**Question:** Why do we see a drift laterally and vertically in the hallway scenario, even though only the forward motion is supposed to be poorly constrained?

---
## c. Ground Truth for Internal Datasets

%% *Get ground truth in TUM format from https://gitlab.com/tinamu-labs/mapping_fg/-/blob/main/README.md?ref_type=heads, using run.launch,* 

*need to install GTSAM*

*extract the poses from `_optimizedPoses` which are in `publishTrajectory(_optimizedPoses, msg->header.stamp, _odometryFrame);`. You can just record a new bag file for example. And then from there using a python script extract the TUM format ground truth. Keep in mind you will still only have data every 4 seconds.* %%
### Overview of Mapping FG (Mapping with Factor Graphs)
https://gitlab.com/tinamu-labs/mapping_fg/

**Mapping FG is a global optimization framework (made by Stefan) that optimizes both the trajectory (pose estimates) and the map simultaneously using factor graphs.** It takes odometry estimates from a lidar odometry package (like FAST-LIO) and globally refines them through loop closure detection and graph optimization.

### Changes made & usability of generated trajectories

- Added TUM format `.txt` file export to extract the optimized trajectory with the offline optimizer, run with the following command:
```
roslaunch mapping_fg_ros run_offline.launch bag_path:=/home/andres/semester_project/data/usbdata/valis_fail_2/chunks_bag.bag
```

>#### For Stefan:
 This was added in the `OptOfflineNode.cpp` file. The pose is extracted as `pose : optimizedPoses->points`, with the outputs for the TUM file defined as:
>```
  posesTumFile << pose.time << " " << pose.x << " " << pose.y << " "
               << pose.z << " " << q.x() << " " << q.y() << " "
               << q.z() << " " << q.w() << std::endl;
>```

- Done so that the optimized trajectory can be used as ground truth to evaluate the FAST-LIO generated trajectories, gives a reference to compare them to each other
- Should not be interpreted as a real ground, but more so as a reference that we want to get closer to
	- We get one pose for every ~4 seconds
		- Good for low-frequency drift/global alignment quality check (ATE works well)
		- Weak for high-frequency motion quality (short-window RPE (e.g., 0.1–1 s) will not work well)
	- While the global optimization provides a much more accurate map/trajectory, it is not guaranteed to match the ground truth
	- Should thus be used to see how much we are improving the drift, but an exact 0 error is not necessarily what we are aiming for

$\rightarrow$ We will call these generated trajectories "reference trajectories", unlike those from GEODE which can be called "ground truth trajectories"

### Extracting reference trajectories for Valis and Senegal

Let's look at the process for Valis as an example. Sanity check: look at generated pose point clouds:

Newly generated point cloud:
![[Pasted image 20260428114610.png]]

Point cloud provided by Stefan:
![[Pasted image 20260428114732.png]]

While not a very scientific test, they look pretty much identical. We have now a generated TUM file for these poses!

The same was done with the Senegal dataset.

These ground truths are quite long, but this is not a problem as `evo` only compares the time-overlapping, associated poses. For example, if
- ground truth = 0 to 100 s
- estimated trajectory = 0 to 10 s
then `evo_ape` / `evo_rpe` will only evaluate roughly **0 to 10 s**, assuming timestamps overlap.

### Dummy test: comparing ground truth to generated poses

For the Senegal dataset, comparing reference trajectory to FAST-LIO for the first 100 seconds:
![[Pasted image 20260428130113.png]]
![[Pasted image 20260428130158.png]]
![[Pasted image 20260428130213.png]]
**This might be in radians, may be a mismatch in units**

Roll, pitch, yaw are not a good reference here, similarly to what we will see in GEODE (there it is just constant). My guess is that this is due to the sparsity of our data. In any case, I think just looking at position RMSE is good enough for our error computation.
![[Pasted image 20260428130323.png]]

Otherwise, this all looks good! We can now use our globally optimized trajectories as reference :)

---
# 2. IMU

## a. Model Specifications

Livox Mid-360 built-in IMU Model: ICM40609

![[Pasted image 20260421190910.png]]
![[Pasted image 20260421190936.png]]

- Low-cost/high-end consumer grade MEMS IMU
- Not close to the level of industrial MEMS IMUs or FOG IMUs
- Good news: by using open-source datasets, we can also have better IMUs and make sure that the weakness in our performance does not come from here!
## b. Testing capabilities

![[loam_livox.rviz_ - RViz (Ubuntu-20.04) 2026-04-21 17-46-51.mp4]]
- Dead reckoning 3 seconds on, 2 seconds off
- Still allows for correct pose estimation, LiDAR data for 2 seconds re-stabilizes IMU before it starts to drift again
- Means that even with the ICM40609 IMU included in the Livox Mid-360, it should be possible to rely more heavily on LiDAR for short bursts of time
- Nevertheless, our approach is
	- Reducing trust on not very good LiDAR during degeneracy (good)
	- Incidentally relying more on not very good IMU (ok for short amounts of time, but may not be preferential over our not very good LiDAR + IMU!)
- So while our approach is theoretically sound, it does rely on the fact that our IMU is better than our degenerate LiDAR data to some extent, which is not necessarily true (particularly if degenerate LiDAR data is present for longer than very short burst of time, i.e. ~3s)
%%
*Also test this few seconds on and off in the tunnel of Senegal!*

#### Ex: Valis dataset during extreme drift due to low FoV

Normal FAST-LIO:
*Add video*

Probabilistic degeneracy:
*Add video*

Dead-reckoning:
*Add video*

Probabilities = 0:
*Add video*

Probabilistic degeneracy: We drift more than the original FAST-LIO because we have low probabilities for a long amount of time (show probabilities)

We can see this because the dead-reckoning performs even worse, drifts in the same direction even further away!
 %%
%% ## c. New Dataset for further tests

*Experiments to test the dead-reckoning capabilities of our IMU*
- *Leaving the rig stationary on the floor*
- *Moving it only sideways*
 %%
---
# 3. Open-Source Datasets

## a. Choosing which open-source degeneracy dataset to use

https://github.com/thisparticle/Awesome-Algorithms-Against-Degeneracy 

[ENWIDE LiDAR Inertial Dataset](http://projects.asl.ethz.ch/datasets/enwide/)
- **Ouster OS0 128 (Rev D) LiDAR**
- InvenSense IAM-20680HT IMU included in the LiDAR system
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

Available IMUs:
- ENWIDE: InvenSense IAM-20680HT
- GEODE: Xsens MTi-30 IMU
- lidar_degeneracy_datasets: [VectorNav VN100 IMU](https://www.vectornav.com/products/detail/vn-100)

Comparison:
- Vectornav and Xsens are a big jump in performance (also in cost, ~$500-$2000 range)
- InvenSense is still slightly better performance than ours

**Choice: [GEODE_dataset](https://github.com/PengYu-Team/GEODE_dataset)**
- Has higher performance IMU $\rightarrow$ Benefit: better IMUs, so methods reducing trust in LiDAR (e.g. probabilistic methods) may fair better
- Has 3 types of LiDARs to choose from, including both spinning and solid-state LiDAR kinds

## b. GEODE (Heterogeneous LiDAR Dataset for Benchmarking Robust Localization in Diverse Degenerate Scenarios) dataset

![[Pasted image 20260424131553.png]]
*Varied LiDAR types*

![[Pasted image 20260424183947.png]]
![[Pasted image 20260424131536.png]]
*Multi-degenerate scenarios*

- Website: https://thisparticle.github.io/geode/
- Paper: https://arxiv.org/abs/2409.04961
- Dataset: https://drive.google.com/drive/folders/1hEn3sBAvQhSdUFnGMZCCv-W0Ynj2rWBs

$\rightarrow$ Requires estimated data in TUM format! :)

---
### Some details

#### 1. LiDAR topic (for personal reference, not important)
Velodyne provides _[x, y, z, intensity, ring, time]_; Ouster includes _[x, y, z, intensity, t, reflectivity, ring, ambient, range]_; and Livox offers _[x, y, z, intensity, tag, line]_.

From our data, Livox LiDAR data output look like so:

```c++
offset_time: 99506047
x: -1.0850000381469727
y: 0.6399999856948853
z: 0.27900001406669617
reflectivity: 8
tag: 0
line: 3
```

The only difference between theirs and ours is _offset_time_. 

Rosbag topics:
![[Pasted image 20260424141719.png]]

---
#### 2. AVIA yaml config file

#### a. Intrinsics
New yaml config has been made using external IMU intrinsics w.r.t. LiDAR.

Parameters provided by GEODE dataset:
```yaml
T_IMU_LiDAR: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [0.999620,  0.027463,  0.002445,  0.049258,
         -0.027517,  0.999299,  0.025398, -0.012500,
         -0.001746, -0.025456,  0.999674,  0.026946,
          0.000000,  0.000000,  0.000000,  1.000000]
```

FAST-LIO:
>The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.

Given the naming convention of T_IMU_LiDAR, as well as the placement of the LiDAR clearly being above the IMU (see picture below), it is fair to say that the GEODE extrinsic parameters are also defined as the LiDAR's pose in the IMU body frame.

![[Pasted image 20260427185644.png]]

Hence, these are the new parameters in FAST-LIO:
```yaml
    # For external IMU from the GEODE dataset:
    extrinsic_T: [ 0.049258, -0.012500,  0.026946 ]
    extrinsic_R: [ 0.999620,  0.027463,  0.002445,
                  -0.027517,  0.999299,  0.025398,
                  -0.001746, -0.025456,  0.999674 ]
```

Running with these updated parameters improves performance dramatically compared to if we run with the mid_360 launch file (the fov is also reduced from 360 to 90 degrees in the parameters)

---
#### b. IMU covariances

Similarly, we might also want to change the IMU covariance values with the provided parameters:
```yaml
#----------------------------------------------------------------------------------
# IMU noise
#----------------------------------------------------------------------------------

imu_topic: "/imu/data"

# Standard deviation of the acceleration noise
accel_std: 5.5741025513357766e-03

# Standard deviation of the acceleration random walk noise
accel_rw:  1.2078671772058479e-04

# Standard deviation of the angular velocity noise
gyro_std: 5.1441059541810427e-03

# Standard deviation of the angular velocity random walk noise
gyro_rw: 1.0881627554710405e-04

pointcloud_topic: "/livox/lidar"    # Pointcloud topic
```

Default FAST-LIO covariances:
```yaml
mapping: 
    acc_cov: 0.1
    gyr_cov: 0.1
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
```

My interpretation is that:
```yaml

# This is the conversion:
acc_cov = accel_std^2
gyr_cov = gyro_std^2
b_acc_cov = accel_rw^2
b_gyr_cov = gyro_rw^2

# This yields:

# Calibrated noise parameters for GEODE external IMU:
# acc_cov = (5.5741025513357766e-03)^2
acc_cov: 3.1070619252808014e-05

# gyr_cov = (5.1441059541810427e-03)^2
gyr_cov: 2.6461826067840857e-05

# b_acc_cov = (1.2078671772058479e-04)^2
b_acc_cov: 1.4589431177712233e-08

# b_gyr_cov = (1.0881627554710405e-04)^2
b_gyr_cov: 1.1840981823943275e-08
```

However, switching to these parameters increases the trust in the IMU too much, causing much more drift. See the images below for an example

![[Pasted image 20260428141909.png]]
*With old covariances*

![[Pasted image 20260428141219.png]]
*With new covariances (killed as it was continuously drifting)*

Possible reasons why:
1. FAST-LIO uses these as EKF process-noise tuning terms, not as a faithful sensor model.  
    They go straight into Q for each predict step. In practice, these numbers are often tuned larger than the physical IMU noise to absorb modeling errors, timing error, motion distortion residuals, and imperfect extrinsics.

2. Your GEODE values are very small compared with FAST-LIO defaults.  
    These reduce `acc_cov` and `gyr_cov` from `1e-1` scale to about `1e-5`, and bias terms from `1e-4` to about `1e-8`. That makes the filter far more confident in IMU propagation and far less willing to attribute error to bias drift or process uncertainty. If there is any timestamp offset, extrinsic error, or unmodeled IMU behavior, the estimate can run away.

3. `accel_rw` and `gyro_rw` may not map directly to `b_acc_cov` and `b_gyr_cov`.  
    Dataset calibration files often report continuous-time random-walk densities, while FAST-LIO is expecting discrete-time process covariance tuning. Squaring the number is not always the right conversion by itself; dt and model convention matter.

Even just reducing the covariances by one order of magnitude 
- `acc_cov` and `gyr_cov` from `1e-1` to `1e-2`
- bias terms from `1e-4` to `1e-5`
also causes infinite drift:
![[Pasted image 20260428142900.png]]

Hence, we will keep the original covariance parameters from FAST-LIO. Another reason to keep this is because all yaml files have the same covariance values for the IMU, despite the use of different IMUs for all of those setups!

---
#### 3. Chosen scenes from GEODE
Because the dataset is ridiculously huge, a few scenes were picked for evaluation. These are:

γ (Handheld):
- Flat_Surfaces_Smooth
- Stairs_Gamma
- Shield_tunnel3_gamma
- Shield_tunnel5_gamma
$\rightarrow$ All considered hard

γ (UGV)
- Tunneling_tunnel1_gamma (easy)
- Offroad1_gamma (medium)

---

## c. Testing with GEODE

These are the APEs w.r.t. translation part (m) with SE(3) Umeyama alignment:
%% Before when using the wrong launch file:
These are the APEs w.r.t. translation part (m) with SE(3) Umeyama alignment:
#### Base FAST-LIO
```
       max      153.821755
      mean      56.449644
    median      36.466478
       min      0.461365
      rmse      70.882781
       sse      763704.041914
       std      42.869645
```
![[Pasted image 20260424164419.png]]![[Pasted image 20260424164423.png]]![[Pasted image 20260424164428.png]]
#### FAST-LIO with outlier rejection
```
       max      60.833496
      mean      30.648774
    median      31.135909
       min      0.501739
      rmse      33.261592
       sse      168162.692042
       std      12.922313
```
Incredible improvement! Much less drift!
![[Pasted image 20260424164451.png]]![[Pasted image 20260424164454.png]]

#### FAST-LIO with probabilistic increase covariance + outlier rejection:
```
       max      91.965331
      mean      39.179573
    median      35.683958
       min      6.395861
      rmse      45.645846
       sse      316698.577825
       std      23.420169
```

#### FAST-LIO with heuristic increase covariance + outlier rejection:
```
       max      60.825251
      mean      30.201425
    median      29.571991
       min      0.808207
      rmse      32.419173
       sse      159752.425053
       std      11.784597
```
 %%

#### Base FAST-LIO

![[Pasted image 20260428143540.png]]
   
APE:
```
	   max      93.854432
      mean      30.260228
    median      29.504421
       min      1.287654
      rmse      36.365214
       sse      198364.316732
       std      20.167980
```

![[Pasted image 20260428135133.png]]
![[Pasted image 20260428143242.png]]
![[Pasted image 20260428143309.png]]

#### FAST-LIO with outlier rejection
```
       max      91.727163
      mean      28.834603
    median      28.139292
       min      0.646961
      rmse      35.283056
       sse      186734.110572
       std      20.333710
```
![[Pasted image 20260428144516.png]]
Slight improvement, not much. However, when I was originally using the wrong launch file (the one for the mid-360), it made a huge difference, going from mean of 70 to 33. The point is that it helps, particularly to help slow down/stop large quick drifts, but does not change the world when the drift is slow.

#### FAST-LIO with heuristic increase covariance + outlier rejection:

![[Pasted image 20260428144942.png]]
AVE:
```
       max      64.434549
      mean      26.683373
    median      22.175381
       min      0.554969
      rmse      32.526733
       sse      158698.252181
       std      18.600698
```

![[Pasted image 20260428145203.png]]

Suddenly drifted backwards at the end! Otherwise seemed more stable at the beginning.
Need to make plots of the probabilities to compare to how it relates to the drift!

RPE:
```
       max      13.823422
      mean      2.052056
    median      1.566383
       min      0.031860
      rmse      2.874782
       sse      1231.391357
       std      2.013315
```
![[Pasted image 20260428145241.png]]

![[Pasted image 20260428145411.png]]
Doesn't seem to fair much better at the moment. Nevertheless, the probabilities seem to somehow all stay quite high during the dataset. So this needs to be analyzed further.

%% Much better than probabilistic, and slightly better than just heuristic. we need to look at
1) How are the probabilities being used in the filter?
2) How are the probabilities being computed?
3) Also test just heuristic without outlier rejection, and different probabilities too. %%

One conclusion: these covariance increase methods work, but the IMU needs to be good enough to be trusted for a long enough amount of time. 

Pure IMU dead-reckoning:
Still drifts quite a lot, similar to in our own datasets. However, somehow it seems that the covariance in x and y takes longer to increase than in z?

%% Command:
(.venv) andres@LaptopAndres:~/semester_project/data$ evo_ape tum /home/andres/semester_project/data/usbdata/GEODE/groundtruth/traj/Shield_tunnel6.txt /home/andres/semester_project/data/estimated_trajectories/GEODE/Shield_tunnel6/FAST_LIO_trajectory_pprob_outlierrej.txt -a %%




---

%% # 4. Probabilistic Degeneracy Mitigation

*Work on probabilistic degeneracy awareness*
1. *Look into whether the implementation is correct*
2. *Look into the math and see if it's correct or if it needs modification*
	- *Heuristic implementation: For the heuristic case, we want to just extract eigenvalues from the separate info matrices (one translation, one rotation), compute the probabilities from here, but then use the normal probabilistic formulation as derived last week (which combines both probabilities into a single P matrix). If Claude did it differently, try this out*
	- *Probabilities injected into IEKF: see if the additional RHS term makes any sense. Also look into our derivation last week and see if it makes sense. Why are we not directly working with the covariance matrix?* 
3. *Test the IMU dead reckoning starting right before failure*
4. *Test probabilities set to 0 starting right before failure and compare to IMU dead reckoning* 
%%

---

# Meeting Highlights


- Showing errors (will be in the final report)
	- We want to have a big table with ATE and RTE for all proposed methods as well as regular FAST-LIO for a variety of datasets (internal ones and GEODE). This will help us see which method helps with what cases (ex: one method might help with hallways, one with flat ground...)
	- For tunnel dataset from GEODE we saw this week, RPE is perhaps more interesting to look at than APE, because on the second half of the dataset the trajectory is tracked quite well by the base FAST-LIO (and there the RPE would be quite low), whereas the probabilistic method had a trajectory that matched less well but had lower APE still. APE really depends on the alignment.
	- For the internal Tinamu dataset reference trajectories that were computed (the "pseudo ground truth"), the roll, pitch, yaw may have been wrong due to incorrect units (perhaps in rad instead of degrees)

- Framing thesis
	- Framing the semester thesis with a story that makes the results useful and interesting
	- Ex: a study of different methods to deal with degeneracy of LIO when using a Kalman filter-based approach
	- Which methods help, and which don't? In which use cases are which methods useful?

- Probabilistic method: is it correct?
	- Does the math make sense? Would it make more sense to just play with the covariance matrix directly?
	- Is it implemented correctly in the code?
	- Heuristic probs: we may need to tune our probability function. The probabilities in the tunnel were quite high, so function may need to be changed.

-  Look at D$^2$-LIO
	- They got an improvement in performance by falling back to the IMU with the same dataset as us!
	- This is unlike the probabilistic paper that used a zero-prior instead, not IMU prior
	- Why did D$^2$-LIO get better results? Can we replicate this for a Kalman filter?
	- What scenes from GEODE did they use? Perhaps these are better suited for covariance shaping

- Main next step: implement intensities due to getting new data without new sensors. Should help because in a degenerate case where LiDAR data is bad and IMU cannot be trusted for a long time, you can get "trash in, trash out" situations.