---

---

**Goal:** Streamline data analysis to have a full understanding of what the failure modes are to better inform us what fixes we should try to do from here

### Updates

- Added initialization fix for the gyro (taken from the implementation that was already done on the Tinamu branch): checks the angular velocity (gyro readings) during IMU initialization. If the norm of the angular velocity is above 0.1 rad/s, it skips initialization for that frame, and tries again at the next frame
- Looked into IMU units. As it turns out, the readings are in
    - Accelerometer: g
    - Gryo: rad/s

![[image 206.png]]

- Add plotting feature for
    - Norm of linear acceleration with no gravity component (to see if there is any actual accelerations during the failure)
    - Gravity estimate from the filter (to see if the failure comes from a bad gravity estimate/attitude estimation problem)
    - Gyro angular velocity with running average (to better see the graph curves, otherwise the readings are too jittery)
    - Inlier ratio (number of inliers/total number of points after downsampling)
- Added an initial version of a corner (and also surface) feature detector based on the LIO-SAM implementation
    - For now, the idea is to use this to see if when the current number of point-to-plane features drops, which we have seen can be correlated with the failure modes, there are enough corner features to better constrain the estimates during these times

### Plots/parameters to analyze failure modes with the new implementation

Interesting to notice: the initialization fix for the gyro has made it such that the map starts building slightly later (around 10 seconds later) for the Cameroon dataset. This slightly different map sees failure at a different time than the original code which does not include the initialization fix for the gyro. More specifically, the original one breaks at around 200s, and the version with the gyro initialization fix breaks at around 150s. Thus, we can extract two failure points from this one dataset.

With gyro initialization fix:

![[image 207.png]]

![[image 208.png]]

This is a bit of a weird one, as the proper drift starts visibly happening at 145s, and at 150s the estimated pose just leaves the building (hence the decrease in the number of features afterwards). 

**Take a look at how the gravity estimation is being done in the filter. Why is it super flat? Weighted averaging with a large window?**

Important: the only plot missing is the number of corner features. Once this is done, will make the final plots of all of the failure cases, such that we can properly analyze all of them and see what the failure modes we want to tackle are.

Plot of current corner feature counter on Cameroon dataset (compared to point-to-plane feature count):

![[image 209.png]]

### Corner feature detector

- Added a new CornerFeatureExtractor class to handle corner feature detection and extraction. Has been made such that it will be easy to then integrate into the Kalman filter as well
- At the moment, the estimated look like so:

![[image 210.png]]

- In white you see the corner features, in purple the planar features, and in blue the full point cloud
- You may notice that features are only being extracted from the bottom of the point cloud. I have been trying to fix this, but have not gotten there by the time of this meeting. Will be focusing on this to have an estimate of the number of corner features to make the final plots of all the failure cases

### To do

- Finish corner feature detector implementation
- Plot eigenvectors of information matrix to see which direction is least constrained and may have been the source of the initial failure **(this is not really right, look below)**
    - **Keep track of SVD singular values that are correlated with largest values, will give you information of where in the space there is information and where in the space there is no information. Could do a diagonal scaling /softmax / make them sum up to 1 to make them consistent while updating.**
    - **Do we see a direction where we cannot update? → add other types of features (corner/camera)**
    - **Do we do something with gravity - fast motions that are breaking this? → fix variances for things that we know are not good**
- Make final plots for all of the failure datasets to understand failure modes
- Make initial fixes to try to tackle these failure modes (ex: uncertainty weighting, or shutting down LiDAR features when uncertainty/number of features is too low)

# To do:

### Understand how gravity estimation is being done/computed:

- Take a look at how the gravity estimation is being done in the filter. Why is it super flat? Weighted averaging with a large window?

### To see if we have a problem with LiDAR measurement information:

1. Corner features
    - See if they correlate with the failure modes. Ideally they don’t, and if they don’t, we can just implement these in the Kalman filter to give us more features during failure modes. 
2. SVD
    - Take the SVD of the information matrix (just position - x,y,z) , look at the singular values (plot the 3 singular values - if large, that direction is not well constrained!). Like this we can see which directions are well constrained and which are not.
    - can then do it separately for rotations as well (but not do it together bcs of difference in scales). So this would be with the information matrix but just the rotation components (first 3 according to the paper).

### To see if the problem comes from the Kalman filter estimation:

3. Gravity vector
    - Plot gravity vector and acceleration from accelerometer (in global frame, so accelerometer reading needs have bias subtracted and rotation multiplied). 
    - We expect to see the accelerometer reading to be up, and gravity down (opposing each other). 
    - However, this accelerometer reading is rotated by the rotation matrix R which is computed from the Kalman filter. So if the Kalman filter is having trouble computing the rotation matrix (which comes from the angular velocity readings), this could cause the filter to think that there is a sideways acceleration and cause a drift to start. This could be seen as a jerky acceleration vector (swinging back and forwards), or just pointing the wrong way… Plot these vectors on rviz!

![[image 211.png]]

