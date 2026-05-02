
To do (for the next weeks):
1. Look into how to best quantify performance of our LIO pipeline, to see if the changes we will be making are effective (EPE, ATE, ...)
2. Implement outlier removal from [[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf|D²-LIO]]
3. Implement probabilistic degeneracy mitigation as derived on [[Week 6 - Plan To Implement Degeneracy-Awareness]]
    1. With heuristic probabilities
    2. With probabilities as derived on [[Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization.pdf|Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization]] 
4. Work on the intensities
	1. Read [[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf|Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping]]
	2. Look into how to use COIN-LIO style intensity handling with solid-state Livox Mid-360 LiDAR

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

%%
Branches:
  - feature/outlier-removal
  - feature/probabilistic-degeneracy
  - feature/intensity-features
  - feature/adaptive-params
%%
---
# 1. Outlier Removal

Relevant papers:
- [[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf]]
- [[FAST-LIO2 - Fast Direct LiDAR-inertial Odometry.pdf]]

## What FAST-LIO2 already does

Analyzing the FAST-LIO code reveals **two gates** applied to each point before it is passed to the IEKF:

**Gate 1: Fixed squared-distance cutoff on the kNN search** (`laserMapping.cpp:851`):
$$\text{accept iff } d_{k\text{-NN}}^2 \;=\; \lVert p_{\text{world}} - q_K \rVert^2 \;\le\; 5 \text{ m}^2$$
where $q_K$ is the $K$-th (= 5th) nearest map point. This is a constant, blind to point range and to platform motion.

→ Mentioned in the FAST-LIO2 paper: "only when the neighbor points are within a given threshold around the target point would be viewed as inliers and hence used in the state estimation" ([[FAST-LIO2 - Fast Direct LiDAR-inertial Odometry.pdf|FAST-LIO2]], Sec. V-E, p. 9).

**Gate 2: Heuristic plane-residual score** (`laserMapping.cpp:861-863`):
$$s \;=\; 1 \;-\; 0.9\,\frac{\lvert p_{d2}\rvert}{\sqrt{\lVert p_{\text{body}}\rVert}}, \qquad \text{accept iff } s > 0.9$$
Equivalent to $\lvert p_{d2}\rvert < \tfrac{1}{9}\sqrt{\lVert p_{\text{body}}\rVert}$, i.e. the tolerance on the point-to-plane residual $p_{d2} = \mathbf{n}^\top p_{\text{world}} + d$ grows as $\sqrt{r}$ with the point's range $r = \lVert p_{\text{body}}\rVert$. 

→ Range-aware, but empirical, as the exponent and constants have no geometric derivation. Not mentioned in the paper.

Anything that passes both gates and has a valid plane fit is accepted as a correspondence.

## Is it worth augmenting with D²-LIO's threshold?

[[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf|D²-LIO]] (Eq. 7) proposes a per-point, physics-derived threshold:
$$\epsilon_j \;=\; \bigl\lVert t^{I_k}_{I_{k-1}} \bigr\rVert_2 \;+\; 2\,\bigl\lVert p^{I_k}_j \bigr\rVert_2 \,\sin\!\Bigl(\tfrac{1}{2}\,\bigl\lVert \mathrm{Log}(R^{I_k}_{I_{k-1}})^\vee \bigr\rVert\Bigr)$$
- The first term is the inter-scan translation, the second is the chord length at radius $\lVert p_j \rVert$ for the inter-scan rotation 
- This is a bound on how far a correct correspondence could have moved between scans

Key weakness of the baseline is Gate 2's **lack of motion awareness**: 
- on fast/aggressive motion the true correspondence can lie further from the predicted plane than $\sqrt{r}/9$  permits, so valid far-range points get discarded (when constraints are most needed)
- when nearly static, $\sqrt{r}/9$ is looser than necessary and admits spurious matches
- D²-LIO's ablation shows improvement on AVE when this gate is replaced on FAST-LIO2, confirming that the effect is real even on a pipeline that already has Gate 2.

→ **Worth augmenting:** 
- the change is a drop-in replacement of Gate 2 inside `h_share_model`
- costs one $\mathrm{Log}$ and $\sin$ per scan plus one mul-add per point, and does not touch the IEKF
- Gate 1 (the fixed 5 m² kNN cutoff) is orthogonal (it guards against map sparsity, not motion) and can be kept.
- The main caveat is that the benefit is expected to be modest on benign motion and mainly show up on sequences with high angular/translational rate

---

## Implementation: D²-LIO Adaptive Outlier Filter
%%
**Branch:** `feature/outlier-removal`
%%
### What was changed

The only file modified inside the FAST-LIO2 source is `src/FAST_LIO/src/laserMapping.cpp`. The config files `config/mid360.yaml` and `config/avia.yaml` received three new parameters.

### Core idea

Gate 2 of `h_share_model` — the heuristic plane-residual score

$$s = 1 - 0.9\,\frac{|p_{d2}|}{\sqrt{r}} > 0.9 \quad \Leftrightarrow \quad |p_{d2}| < \tfrac{\sqrt{r}}{9}$$

is **replaced** by the D²-LIO per-point threshold (Eq. 7, [[D²-LIO - Enhanced Optimization for LiDAR-IMU Odometry Considering Directional Degeneracy.pdf|D²-LIO]]):

$$\varepsilon_j = \|\Delta t\|_2 + 2\,r\,\sin\!\Bigl(\tfrac{1}{2}\,\|\Delta\theta\|\Bigr)$$
### Design decisions

- **Δ-pose computed once per scan, not per IEKF iteration:** the threshold is therefore constant across iterations, so it cannot oscillate and is cheap (one `Log` + one `sin` per scan).
	- Drawback: computed with propagated pose, which may not be fully accurate
- **Gate 1 kept:** the kNN squared-distance cutoff (`> 5 m²`) is orthogonal (guards against map sparsity) and was not changed.
- **First-scan fallback:** when no prior pose exists, `d2_scan_motion_ready = false` and the original `s > 0.9` gate applies.
- A clamp $[\varepsilon_{\min}, \varepsilon_{\max}]$ is added to prevent collapse when the platform is near-static and runaway when the IMU estimate is noisy. Current params:
	- $\varepsilon_{\min} = 0.05\text{m}$
	- $\varepsilon_{\max} = 2\text{m}$
	- These are not originally for the paper (and neither are we forced to use them), but it could be something nice to try out through an ablation study.

### New YAML parameters

```yaml
common:
    use_d2_outlier_filter: true # set true to activate
    d2_eps_min: 0.05            # meters — lower clamp on eps_j (near-static case)
    d2_eps_max: 2.0             # meters — upper clamp on eps_j (aggressive motion)
```

### Diagnostic topic

`/fastlio/d2_outlier_stats` publishes a `Float64MultiArray` per scan:

| Index | Content |
|---|---|
| 0 | `lidar_end_time` |
| 1 | `use_d2_outlier_filter` (0/1) |
| 2 | `d2_scan_motion_ready` (0/1) |
| 3 | `d2_delta_t_norm` (m) |
| 4 | `d2_sin_half_dtheta` |
| 5 | `effct_feat_num` (effective correspondences after filtering) |

### Expected behaviour

- **Benign / slow motion**: $\varepsilon_j$ small → tighter than the original $\sqrt{r}/9$ → fewer but cleaner correspondences.
- **Fast / aggressive motion**: $\varepsilon_j$ grows with $\|\Delta t\|$ and $r\sin(\Delta\theta/2)$ → tolerates larger residuals at far range, which the original gate would incorrectly reject.
- However, will not fix degeneracy issues

### Testing 
#### Cameroon dataset

- Still drifts, but the drift itself is noticeably slower!
- **Why?** A few possibilities:
	- Rejecting more bad correspondences
	- Preserving more valid correspondences
- However, slower drift may also mean a more conservative gating mechanism (with more rejected points overall). The filter may thus rely more on IMU propagation and accept fewer LiDAR corrections. May look better in the short term, while not actually improving long-term accuracy

→ Already observe a positive effect of this change:

![[Pasted image 20260414143202.png]]
- Around the failure mode, the minimum eigenvalues stay higher → better conditioned system

![[Pasted image 20260414143052.png]]
- The effective number of features after outlier rejection is very similar.
	- Only ever so slightly lower during normal functioning
	- During poorly conditioned parts, manages to protect the system from too much drift, which allows for more features then!

![[Pasted image 20260414143141.png]]
- This is just a check, where we see
	- D2 on curves are indeed with the D$^2$-LIO outlier removal on, and D2 off are with the original FAST-LIO2 implementation
	- Translation and rotation upper limit estimates are shown here too → translation drastically increases around failure mode

**Notes during the meeting:
Try this on other datasets:**
- **Silo**
- **Hallway ones (Senegal)** 
- **Ones that already worked well before (none of the ones we have)**
#### Extra tests:
###### 1. Cameroon dataset without upper and lower clamps

Tested with:
  ```
    d2_eps_min: 0.0             # meters — lower clamp on eps_j (near-static case)
    d2_eps_max: 100.0              # meters — upper clamp on eps_j (aggressive motion)
  ```
Behavior remained the same. Will be interesting to test through an ablation study once we have a method and dataset to quantify pipeline performance

###### 2. Senegal dataset

Also tested on the Senegal dataset (hallway issue), and here as expected the drift is still present
![[Pasted image 20260414140613.png]]

---
# 2. Probabilistic Degeneracy-Awareness

Relevant papers:
- [[Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization.pdf]]

## a. Heuristic Probabilities

3 probability computations:

`DEGEN_HEUR_HARD`:
$$  
p(\lambda) =  
\begin{cases}  
1, & \text{if } \lambda > \lambda_{\min} \\  
0, & \text{otherwise}  
\end{cases}  
$$
Tunable parameters:  
- $\lambda_{\min}^{\text{trans}}$ (`hard_lambda_min_trans`)
- $\lambda_{\min}^{\text{rot}}$ (`hard_lambda_min_rot`)

`DEGEN_HEUR_SOFT`:  
$$  
p(\lambda) = \frac{\lambda}{\lambda + \alpha}  
$$  
Tunable parameters:  
- $\alpha^{\text{trans}}$ (`soft_alpha_trans`)  
- $\alpha^{\text{rot}}$ (`soft_alpha_rot`)

`DEGEN_HEUR_SIGMOID`:  
$$  
p(\lambda) = \frac{1}{1 + \exp\left(-\frac{\lambda - \tau}{s}\right)}  
$$  
Tunable parameters:  
- $\tau^{\text{trans}}$ (`sigmoid_tau_trans`)  
- $\tau^{\text{rot}}$ (`sigmoid_tau_rot`)  
- $s^{\text{trans}}$ (`sigmoid_s_trans`)  
- $s^{\text{rot}}$ (`sigmoid_s_rot`)

```yaml
degeneracy:

    # Probabilistic degeneracy mitigation (Week 7).  When enabled, attenuates the
    # pose update along eigen-directions judged degenerate. DRPM update rule: 
    # x = U P Lambda^-1 U^T J^T b, P = diag(p_1..p_6), p_k in [0,1].

    # With enable=false, the filter is byte-identical to baseline FAST-LIO2.
    enable: true
    force_all_probabilities_one: false  # true: force all p_k=1 while keeping mitigation path active for testing
    probability_mode: "heuristic"   # "heuristic" | "probabilistic"
    heuristic_mode:   "soft"        # "hard" | "soft" | "sigmoid"

    # Hard-threshold mode: p_k = 1 if lambda_k > lambda_min else 0
    hard_lambda_min_trans: 100.0
    hard_lambda_min_rot:   100.0

    # Soft-threshold mode: p_k = lambda_k / (lambda_k + alpha)
    soft_alpha_trans:      4000.0
    soft_alpha_rot:        2000.0
    
    # Sigmoid mode:        p_k = sigmoid((lambda_k - tau) / s)
    sigmoid_tau_trans:     100.0
    sigmoid_tau_rot:       100.0
    sigmoid_s_trans:        50.0
    sigmoid_s_rot:          50.0
```


%% After computing the probabilities (when calling `degen_build_block_A`):
```C
    A = U * sqrt_p.asDiagonal() * U.transpose();
    B = U * probs.asDiagonal() * U.transpose();
    return true;
```
 %%
 
## b. Full probabilities

The authors of [[Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization.pdf|Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization]] provide an implementation of the probability computation: [GitHub - DRPM: Degeneracy Resilient Point-to-Plane Error Minimization](https://github.com/ntnu-arl/drpm/tree/main)

Upon testing, the probabilities seem to be reasonable (around 1 most of the time, one or two our of the six probabilities fall to around 0.6-0.8 during degeneracy). 

This has been implemented in the current pipeline. However, when probabilities become low (for both heuristic and full probabilities), we drift more quickly. It's almost as if reducing the trust in one direction causes us to drift.

Could be because: 
- Implementation is not correct
- IMU is not good enough
- Mathematical derivation is not completely sound
→ Will look into all of these for next meeting

**Notes during the meeting:**
**To test if the problem is with the IMU, start the map, do if statement slightly before it starts drifting and do dead-reckoning. Ex: detect probabilities are not very good, fully ignore LiDAR from there.**
**Also try setting probabilities to 0 and see if it behaves the same.**

---

%% For next week:
1. Quantify the performance of our added features
	1. Test each new features individually, and quantify their performance. This is important to understand the effect of different features, and whether they are useful
	2. Also test the combination of both added features and see if performance improves further
2. Work on adding intensity features
	1. Ideate how to implement intensities for the Livox-Mid360
	2. Make an initial implementation 
%%

Now that we have the globally optimized maps and poses of our Tinamu datasets, we can use these as ground truth and compare our maps to them. 

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
3. Work on adding intensity features
	1. Read [[Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping.pdf|Intensity enhanced for solid-state-LiDAR in simultaneous localization and mapping]]
	2. Ideate how to implement intensities for the Livox-Mid360 (i.e. COIN-LIO style intensity handling with solid-state Livox Mid-360 LiDAR)
	3. Make an initial implementation
%% 4. Background
	4. Watch recursive estimation lectures
	5.  Read and understand [[FAST-LIO2 - Fast Direct LiDAR-inertial Odometry.pdf|FAST-LIO2]] and [[FAST-LIO - A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter.pdf|FAST-LIO]] papers %%



%% 
Next tasks (today/tomorrow)
- Test with heuristic probabilities as well, which are based on eigenvalues. Maybe here separate translation and rotation to avoid eigenvalue scale issue
	- Check how it is that Claude wrote the code for this
	- Ideally we want to just extract eigenvalues from separate info matrices (one translation, one rotation), compute the probabilities from here, but then use the normal probabilistic formulation as it is at the moment (which combines both)
	- If Claude did it differently, try this out
- Try to understand full probability implementation, see if it's correct, see if it's mathematically sound, etc.
	- Remember if all probs = 1, then it look like the behavior goes back to normal. so put this in the analysis
	- Try to understand the whole matrix derivation, see if it's correct in the code but also conceptually.

3. Read and understand [[FAST-LIO2 - Fast Direct LiDAR-inertial Odometry.pdf|FAST-LIO2 - Fast Direct LiDAR-inertial Odometry]] 
%%