---

---
## 1. Installing and testing FAST-LIO

I got FAST-LIO up and running. I tried it out with the example bag files they supply, you can find a screenshot here of the resulting map from one of these bag files

![[image 172.png]]

![[image 173.png]]

![[image 174.png]]

![[image 175.png]]

![[image 176.png]]

## 2. Tools to analyze degeneracy

To analyze when we have degeneracy cases, I added a few metrics to plot. More specifically, after building the LiDAR measurement Jacobian $H$ (which comes from the linearization step of the measurement model, and tells us how sensitive the measurement is to small changes in each state variable), we extract its pose-related columns (first 6) to get $H_{pose}$. 

When then use this to compute the pose information matrix

$$
A = \frac{H_{pose}^T H_{pose}}{\sigma^2},
$$

which tells us how well constrained the different directions in pose space are. Small eigenvalues of $A$tell us that this is a weakly constrained direction, and vice-versa. The eigenvalues of $A$ thus serve as a degeneracy indicator. 

As such, we perform an eigenvalue decomposition of $A$ and publish under the topic `/fastlio/degeneracy` for each LiDAR scan: 

We can thus plot the smallest eigenvalue and the condition number as signs of per-scan degeneracy, where very small eigenvalues indicate degeneracy cases, and the condition number describes “how ill-conditioned” it is.

On top of this, we compute the angular velocity magnitude:

$$
∣\omega∣=\sqrt{\omega_x^2 + \omega_y^2 + \omega_z^2}
$$

and the acceleration magnitude:

$$
∣a∣=\sqrt{a_x^2 + a_y^2 + a_z^2}
$$

to see when degeneracy cases occur. Potential cases are high angular velocity, or high linear acceleration. This will have to be tested with highly dynamic data, which is not possible with the data from the example bag files.

(There exists a question as to whether the effect of gravity needs to be subtracted from the acceleration magnitude. As we mostly only care about spikes in the magnitude, this might not matter)

Here are some sample plots of these metrics from the example bag files (all at the same timestamp):

<!-- Column 1 -->
![[image 177.png]]

Beige (data[5]): magnitude of angular velocity

<!-- Column 2 -->
![[image 178.png]]

Pink (data[7]): magnitude of linear acceleration

![[image 179.png]]

Blue (data[2]): min eigenvalue

Red (data[4]): condition number

## 3. Literature Review

The goal of the literature review is to understand how FAST-LIO2 works. For this, the following approach is being followed:

- [LOAM](https://www.roboticsproceedings.org/rss10/p07.pdf)
- [FAST-LIO](https://arxiv.org/abs/2010.08196)
- [FAST-LIO2](https://arxiv.org/abs/2107.06829)
- (eventually [FAST-LIVO](https://arxiv.org/abs/2203.00893) and [FAST-LIVO2](https://arxiv.org/abs/2408.14035) to see how they implement visual data, but not necessary at the moment)

### Current progress:

<u>LOAM:</u>

I finished reading the LOAM paper to have a better understanding of LiDAR odometry/SLAM as a whole, as it is the LiDAR odometry algorithm that most current algorithms seem to be based on. More specifically, this was to understand what are LiDAR features, how they are extracted, how we do feature matching with LiDAR data, and eventually how the transformation matrix is computed between poses. This was highly useful as I have only have background on visual SLAM, and haven’t worked with LiDAR SLAM previously. If you want to take a look at my notes on the paper (and also for my own future reference), the pdf is attached below:

[[With Notes - LOAM - Lidar Odometry and Mapping in Real-time.pdf]]

<u>FAST-LIO:</u>

I began taking a look at the FAST-LIO paper to understand the underlying algorithm behind FAST-LIO2. The main relevant contributions are:

- Tightly-coupled IEKF formulation to fuse LiDAR & IMU
- New formula for Kalman gain w/ computational complexity depending on state dimension instead of measurement dimension

Before getting into the math of the IEKF algorithm, I need to get up to speed on Kalman filtering. To do so, I started studying the content from the “Recursive Estimation” course (which I am taking this semester). I’m already about halfway through with the relevant content, so hopefully by next week I should be caught up.

<u>FAST-LIO2:</u>

I also took a brief look at the FAST-LIO2 paper. I noticed that the main 2 contributions are:

- ikd-Tree structure (not super relevant for the scope of our project)
- “direct” odometry approach compared to the original FAST-LIO paper, registering raw LiDAR points to the map instead of feature points. This part is important to understand.

Nevertheless, I want to get a good understanding of the original FAST-LIO paper before diving further into this one. 

## Next steps:

<u>Literature Review</u>

- Finish catching up on Recursive Estimation for Kalman Filtering and EKF background
- Finish reading FAST-LIO and FAST-LIO2 papers

<u>Implementation</u>

- Test degeneracy cases with Tinamu data
    - Try to determine which are the failure cases of FAST-LIO2
- Brainstorm potential solutions to improve performance in these failure/degeneracy cases
- Begin implementation of these solutions


Notes:

- Try to plot raw imu data again with other plotting tools (maybe even with a python script) because they seem very discontinuous
- Also check what units we are plotting
- Overall try to plot things in python
- Condition number seems to be waaay to high, check if you are computing it correctly - could be when extracting the first 6 columns (that might be wrong)
- 