---

---
# Summary

### LOAM

- Feature-based (edge + planar extraction)
- Two-thread system: fast odometry + slow mapping
- Nonlinear optimization (LM), no tight IMU fusion
- Geometric ICP-style alignment

### FAST-LIO

- Tightly coupled LiDAR–IMU
- Iterated EKF (IEKF) framework
- Efficient Kalman gain (state-dimension inversion)
- Still uses edge + planar features
- Motion compensation via forward/backward propagation

### FAST-LIO2

- Direct raw point registration (no feature extraction)
- Incremental k-d tree (ikd-Tree)
- Still tightly coupled IEKF
- Better scalability + LiDAR generality
- Online extrinsic estimation

### Theory questions

LOAM and FAST-LIO1 both use feature extraction (of edges or planes) to compute residuals used for the state estimation/geometric transform. FAST-LIO2 replaces feature-based registration with direct raw point-to-map residuals. But in the project description, it is mentioned that we want to extend the IEKF measurement model by incorporating edge and planar LiDAR features to provide stronger geometric constraints in sparse or repetitive environments.

I want to clarify the architectural direction of this part of the project:

- Are we building on FAST-LIO2’s direct method and augmenting it with additional feature-based residuals (for strong edges/planes for example)?
- Or are we moving back toward a feature-based registration approach used in FAST-LIO1/LOAM but keeping the new idk-Tree framework?

(Would it make sense to first benchmark FAST-LIO1 vs FAST-LIO2 in such environments to quantify the trade-offs between direct and feature-based constraints?)

- ROS2 on WSL? He mentioned that docker could be an option, but if this works then it’s fine as well, as we are not concerned by computational resources.

### Administrative questions

- Would it be possible to change next week’s meeting to Wednesday morning?
- When registering to the semester project on mystudies?
    - Start date? 16th of Feb
    - Submission deadline? 
    - Supervisors? Zak and Efe
    - Will the research project/paper/thesis work be undertaken outside ETHZ? No

IFA administrator (Sabrina) - email here about the start date and submission deadline

---

# Meeting notes

For this week:

- Try to run the FAST-LIO2 code (try to run the examples)
- Start working with visualizations (plotting the states, inputs, gyro, IMU inputs, rotational state, position state) - can do this using ROS visualization tools or using the FAST-LIO tools
    - Crazy gyro inputs seem to be problem for degenerate cases. We want to use these visualizations to figure out what the problematic cases are, in order for us to see which solutions we should implement first.
- Try to understand the math behind the FAST-LIO IEKF (read FAST-LIO and FAST-LIO2 papers) - watching recursive estimation before will help

Future fix (step 1):

- Use attitude estimation (gravity vector for attitude estimation) - this is for later - this is what the links on the attitude estimation stuff is for

Message:

Here are some more resources for Andres which might be useful for you:

- Fastlio repo: [https://github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)
- Attitude estimation resources: [https://ahrs.readthedocs.io/en/latest/filters/ekf.html](https://ahrs.readthedocs.io/en/latest/filters/ekf.html)

[https://github.com/AIS-Bonn/attitude_estimator](https://github.com/AIS-Bonn/attitude_estimator)

[https://arxiv.org/pdf/1704.06053](https://arxiv.org/pdf/1704.06053)

- ros docker: [https://varhowto.com/install-ros-noetic-docker/](https://varhowto.com/install-ros-noetic-docker/) (howto guide to setup ros noetic in docker. I suppose you do not have ubuntu 20.04, so this is a fastest way to put you up to speed)

I'll give you more details in the meeting about these resources.

- is it better to run ROS with Docker or on WSL? He doesn’t know, but ChatGPT says WSL