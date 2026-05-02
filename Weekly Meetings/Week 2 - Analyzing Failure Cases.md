---

---

## 1. Issue with jittery IMU data plots

Plotting the IMU data (here the angular velocity x) with rqt_plot (left) and PlotJuggler (right) shows that PlotJuggler is definitely doing something with the data, perhaps some sort of interpolation. 

![[image 180.png]]

We can dissect this issue further when plotting the bag file data using a python script. We can use either the bag timestamp (when the message was recorded) or the header timestamp (when the sensor actually produced the measurement). When using the bag timestamp (`stamp.to_sec`) we get a plot like this:

![[image 181.png]]

But if we use the header timestamp (`msg.header.stamp.to_sec()`), the jitter in the plot is fixed:

![[image 182.png]]


## 2. Looking at failure cases

### a) Silo dataset

This is the generated point cloud for the “silo” dataset. While the map works somewhat as expected at the beginning, the mapping breaks after a little bit of time and starts to drift without stopping:

![[image 183.png]]

To avoid plotting issues and to have better control over the plots, we export the data as a new bag file that includes the degeneracy analysis. Here is the raw IMU data, as well as the smallest eigenvalue and condition number from $A = H^{T}H/\sigma$:

![[image 184.png]]

Clearly, the smallest eigenvalue drops to 0 once the pose estimate becomes ill-conditioned. 

Important to note: the sensor rig is mostly stationary for the first 20s, then starts moving. From 20-40s, the mapping works relatively well, and breaks at around the 40s mark onwards.

The smallest eigenvalue drops at around the 10s mark due to a low number of LiDAR features, as well as a slow-moving rig (so IMU data isn’t as informative with small accelerations)

![[image 185.png]]

Zooming into 20-100s interval

![[image 186.png]]

And further zooming into the 20-40s, the interval during which the mapping is working well:

![[image 187.png]]

clearly the minimum eigenvalue is larger, and the condition number is lower, meaning that the problem is much better conditioned. However, it is still interesting to notice that the condition number never drops below 200.

### Question: why did SLAM break?

It’s nice to know that the minimum eigenvalue and condition number of $A$ are informative about when the FAST-LIO2 IEKF is working well and when it is not due to ill-conditioning. However, we want to understand why the problem became ill-conditioned. For this, we need to analyze the period of time during which it first became ill-conditioned, around the 30-50s mark:

![[image 188.png]]

1. **Big drop in the number of effective feature points!**
2. **Peak in linear acceleration in z direction without much of a change in angular velocity!**

Also, why is ax always pinned at 1m/s2? Gravity?

### b) Cameroon dataset (~400s)

Initially, the scan of the warehouse is working well. However, at around 200 seconds, the SLAM algorithm breaks and creates the same type of output as before:

![[image 189.png]]

However, if you run using the same data but starting from 100s onwards, the scan doesn’t break at all! Here is the final point cloud of the full scan starting from 100s:

![[image 190.png]]

We need to analyse both of these cases independently. Let’s call them the 0-400s case (which breaks) and the 100-400s case (which doesn’t break). Notice that in the following plots, a vertical red line was added to signify where the failure occurs.

Overall plots for the 0-400s case:

![[image 191.png]]

Overall plots for the 100-400 case:

![[image 192.png]]

As mentioned before, the SLAM algorithm fails at around 200s of the 0-400s case, which is equivalent to ~100s of the 100-400s case. You can notice that in both plots, the effective number of features drops significantly at this time as well, similar to what we saw in the silo dataset!

Let’s zoom into the time where the failure appears:

Plot of 0-400s case:

![[image 193.png]]

As expected, condition number blows up and lambda_min goes to 0. This occurs when the effective feature number is low! (~100)

Plot of 100-400s case:

![[image 194.png]]

Here, the feature number goes down again but given that the map does not begin drifting, as the number of features increases again the problem becomes better conditioned (although still not very well conditioned for the next 40 seconds after the failure case!) 


Once again:

3. **Big drop in the number of effective feature points!**
4. **Peak in linear acceleration in z direction without much of a change in angular velocity!**

→ Notice that in both examples, the failure modes seem to be associated with an increase of the linear acceleration in one direction (z component for both cases) but no significant change in the angular velocity, meaning that the sensor rig was moved in a jerking motion in one specific direction.

## 3. Checking why the number of effective features dips

Let’s check the failure modes more closely. Looking a the LiDAR point cloud at the failure mode times:

<u>Silo dataset:</u>

<!-- Column 1 -->
![[image 195.png]]

<!-- Column 2 -->
![[image 196.png]]

Not very different from data in previous scans. This is different to the cameroon dataset. Why does the effective number of features drop here? Perhaps due to the symmetry of the silo? 

<u>Cameroon dataset:</u>

At 160s:

<!-- Column 1 -->
![[image 197.png]]

<!-- Column 2 -->
![[image 198.png]]

At 180s:

<!-- Column 1 -->
![[image 199.png]]

<!-- Column 2 -->
![[image 200.png]]

At 200s (when it fails):

<!-- Column 1 -->
![[image 201.png]]

<!-- Column 2 -->
![[image 202.png]]

At 200s (failure mode), point cloud just focused on one small section of one single wall! Can cause ambiguities. Compare this to at 160/180s, where you have a much more spread out point cloud, more constraints.

**Conclusion: The second example shows a more classic case of degenerate data, but the first one not so much. **

Another thing to note: ghosting on Cameroon dataset → the person recording the dataset appears multiple times in the final scan:

![[image 203.png]]

This can also cause issues as the map won’t match the measurements. As the map is assumed to the be truth, this will cause our filter to have little confidence in data that is potentially correct, which can cause drift.

## Checking why the condition number is high

In earlier plots we have seen that the condition number can be in the order of 200 even when the pipeline is working. 

A potential reason is because out of the first 6 columns of $H \in \R^{N \times 12}$ that we are extracting:

- 3 are for translation residual sensitivity (unitless)
- 3 are for rotational residual sensitivity (in m)

The differing units can cause a very large difference in eigenvalue dimensions. We can see if this is the issue by printing out the 2-norm of each of the 6 columns. After printing out a few sample $H_{pose}$ matrices (for different timestamps), it can be noted that, as expected, the translation norm magnitude is much smaller than the rotation (around 7-8x). One example output:

```c++
Hpose rows=851 

col_norms=[t_x t_y t_z r_x r_y r_z]=19.7365 10.398 18.7977 115.377 71.0854 105.326 
```

We can further test to see if this is correct by looking at the Frobenius norms of $t = H_{pose}[1:3]$ and $r = H_{pose}[4:6]$. The ratio $r/t$ sits consistently between 3-7, meaning that the columns for rotation are aroud 3-7x larger in the Frobenius sense than the columns for translation. 

As an “easy dirty” fix, we can just scale down the rotation residual columns to make them comparable in magnitude. For example, when scaling them with 1/10, we get a condition number that sits consistently at around 20-25 when the pipeline is working, instead of 200 as before.

(Will also check what happens when we use all of the columns)

## New degeneracy parameter: posterior pose information matrix

The pose information matrix we have been looking at so far computes “how well does the current scan geometry alone constrain the pose?”, measuring pure geometric observability of this scan. This is useful in understanding how the geometry of the current scan affects observability as a whole, giving insight into what types of geometry can be associated with degeneracy cases.

However, we can also use the *posterior* pose information matrix after the EKF update:

$$
\Lambda_{\text{post}} = P_{\text{pose, post}}^{-1}.
$$

If we want the equivalent information-form update explicitly, it is

$$
\Lambda_{\text{post}} \approx \Lambda_{\text{prior}} + H^\top R^{-1}H,
\quad \Lambda_{\text{prior}}=P_{\text{pose, prior}}^{-1},
$$

which is only valid when using the **prior** covariance.

In our implementation, $P_{\text{pose}}$ is taken after `update_iterated_dyn_share_modified(...)`, so it is already posterior; adding $H^\top R^{-1}H$ again would double-count the measurement.

Using this is useful as degeneracy in LiDAR does not necessarily mean degeneracy of the Kalman filter, as the propagated prior information (i.e. IMU) may still constrain the weak direction. In a sense, this computes "given prior uncertainty and the current scan, how well constrained is pose after fusion?", rather than LiDAR geometry alone.


For the Cameroon dataset, we see the following:

![[image 204.png]]

The condition number and lambda_min follow similar curves for the geometric information matrix (pre) and the posterior information matrix (post). However, the posterior one is better conditioned (smaller condition number, larger lambda_min) overall, as expected.

The failure time occurs when the condition number spikes and the lambda_min falls drastically.

<u>To do:</u>

- Finish the work from above: check additional columns of H matrix, look at the new degeneracy parameter
- Analyze data from new failure case dataset, see what went wrong there
- Determine what are the failure cases of FAST-LIO2
    - Try to understand why the SLAM breaks for longer datasets (more map prior data), but works if you remove some of the data from the beginning
- Start looking into “hacky” solutions. For example, if the effective number of features is below a certain number, or if the condition number of the geometric information matrix is too high, give a lower weight/ignore LiDAR information and trust IMU more. This will help us determine whether the problems lie where we think they do.


### c) Drift dataset

![[image 205.png]]

The map breaks pretty much immediately.


# Failure modes

### **1. Insufficient information** 

**Cause:** system drifts slowly in weak directions until it becomes unstable, caused by weak observability 

- Geometrically degenerate environments (feature-poor/repetitive scenes)
- Weak IMU observability: weakly constrained yaw, bias, no excitation/constant velocity motion)

**Symptoms:** 

- Small eigenvalues
- Large condition number
- Specific state directions weakly constrained

**→ What we are already plotting**

### **2. Bad measurements**

**Cause:** filter updates in wrong direction due to incorrect or distorted measurements

- Wrong scan-to-map correspondences (wrong kNN neighbors / plane fits / outliers)
- Deskew errors (bias drift, gravity error, time sync issues)
- Poor spatial feature distribution (all points on one surface, low angular coverage)

**Symptoms:**

- **Correspondence quality**
    - number of effective points used in update
    - number of rejected planes / failed fits (not enough neighbors)
    - kNN success rate / NN radius
    - mean/median point-to-plane residual
    - Inlier ratio dropping
- **IMU consistency (deskew-related)**
    - estimated gyro/accel biases over time (rapid drift/jumps)
    - residuals increasing even though conditioning is okay

What I added:

5. We already do the number of effective points (= # inliners). I added an inlier ratio to give a bit more context to the # of inliers
6. point-to-plane residual: median and 95th percentile
7. gyro bias over time
8. accel bias over time

Could add eigenvectors with smallest eigenvalues to see least constrained directions

> [!note]+ ### Less likely contenders
> ### **3. Filter inconsistency** (EKF breakdown)
> 
> **Cause:** estimator becomes internally inconsistent or overconfident
> 
> **Symptoms:**
> 
> - Iteration count spikes / stops converging
> - Post-update residual not improving (or gets worse)
> 
> ### **4. Map-history effects** (local map changes)
> 
> **Cause:** estimation depends on how the local map was built over time
> 
> - Map sliding / point deletions
> - Sudden change in map composition
> - Early bias convergence influencing later mapping
> 
> **Symptoms:**
> 
> - Map point count in ikd-tree
> - Number of points deleted at “map move”
> 

