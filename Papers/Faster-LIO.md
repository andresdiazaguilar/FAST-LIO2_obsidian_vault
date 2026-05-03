# Faster-LIO: Lightweight Tightly Coupled LiDAR-Inertial Odometry Using Parallel Sparse Incremental Voxels

**Category**: FAST-LIO family  
**Failure Modes Addressed**: FM8 (Map-History Effects)

---

## 1. Problem Statement

FAST-LIO2 uses an **ikd-Tree** as its map representation, which is effective but has limitations: insertion/deletion operations can be slow during tree rebalancing, and the kNN search doesn't naturally exploit spatial locality for nearby points. Faster-LIO proposes **incremental voxels (iVox)** as an alternative map structure that provides faster operations with comparable or better accuracy.

---

## 2. Core Methodology

### 2.1 Incremental Voxel (iVox) Map

Instead of a kd-tree, iVox uses a **hash-based voxel grid**:
- Space is divided into voxels of fixed size $s$
- Each voxel stores up to $N_{\max}$ points
- Voxel lookup is $O(1)$ via hash map

**Point insertion**:
$$
\text{voxel\_key}(p) = \lfloor p / s \rfloor
$$

New points are inserted into their corresponding voxel. If the voxel is full, the oldest point is replaced (FIFO queue).

**kNN search**:
For a query point $q$, search the voxel containing $q$ and its 26 neighbors (3×3×3 grid). This provides approximate kNN in $O(1)$ amortized time.

### 2.2 Two Variants

**iVox-PHC (Point Hash Cloud)**:
- Points are stored individually in a flat hash map
- kNN searches over the hash map directly
- Fastest for small point counts per voxel

**iVox-LPH (Linear Point Hashing)**:
- Each voxel stores a fixed-size array of points
- More cache-friendly for dense voxels
- Better for larger voxel sizes

### 2.3 Comparison with ikd-Tree

| Property | ikd-Tree | iVox |
|---|---|---|
| Insertion | $O(\log N)$, with periodic rebalancing | $O(1)$ amortized |
| kNN search | $O(\log N + K)$ exact | $O(1)$ approximate |
| Deletion | $O(\log N)$ | $O(1)$ (FIFO or explicit) |
| Memory | $O(N)$ | $O(V + N)$ where $V$ = voxels |
| Rebalancing | Required (causes latency spikes) | Not needed |

### 2.4 IEKF Integration

Faster-LIO uses the **same IEKF pipeline as FAST-LIO2**, replacing only the map structure. The IEKF's measurement model, Kalman gain computation, and state update are identical. Only the kNN search and point insertion/deletion interface changes.

---

## 3. Key Equations

### Voxel hash function
$$
h(v_x, v_y, v_z) = (v_x \cdot p_1) \oplus (v_y \cdot p_2) \oplus (v_z \cdot p_3)
$$

where $p_1, p_2, p_3$ are large primes and $\oplus$ is XOR.

### Approximate kNN in iVox
For query point $q$ in voxel $V_q$:
$$
\text{kNN}(q) = \text{top-}K\left(\bigcup_{V \in \mathcal{N}_{27}(V_q)} \text{points}(V)\right)
$$

where $\mathcal{N}_{27}$ is the 27-voxel neighborhood.

### FIFO point management
When voxel $V$ has $|\text{points}(V)| = N_{\max}$:
$$
\text{points}(V) \leftarrow \text{points}(V) \setminus \{p_{\text{oldest}}\} \cup \{p_{\text{new}}\}
$$

---

## 4. Assumptions

1. **Approximate kNN is sufficient**: iVox provides approximate, not exact, kNN. The approximation error depends on the voxel size relative to the query radius.
2. **Fixed voxel size**: The voxel size is fixed and determines the map resolution. Unlike the ikd-Tree (which adapts to point distribution), iVox has uniform resolution.
3. **FIFO is a reasonable eviction policy**: The oldest point is removed when a voxel is full. This implicitly implements a temporal sliding window at the voxel level.

---

## 5. Limitations

1. **Approximate kNN**: The 27-voxel neighborhood search can miss nearest neighbors that are in non-adjacent voxels (rare for well-chosen voxel size).
2. **Fixed voxel resolution**: Cannot adapt resolution to local geometry (unlike the ikd-Tree's adaptive tree structure). Regions with fine detail may need finer voxels, but the resolution is uniform.
3. **Memory overhead**: Hash map overhead can be significant for very sparse environments (many empty voxels allocated).
4. **FIFO eviction is naive**: Points are evicted based on insertion order, not on their quality or importance. A point that was inserted early but is geometrically important (e.g., on a rarely-observed surface) gets evicted first.
5. **No lazy deletion**: The ikd-Tree supports lazy deletion (marking points without restructuring). iVox uses explicit removal.

---

## 6. Applicability to FAST-LIO2 / IEKF

**Directly applicable as a drop-in replacement for the ikd-Tree**:

- Same IEKF pipeline, different map structure
- Eliminates the latency spikes from ikd-Tree rebalancing
- Faster kNN for correspondence search
- FIFO eviction provides a natural map aging mechanism (partially addresses FM8: Map-History Effects)

**Key benefit for FM8**: The FIFO eviction means old, potentially corrupted points are naturally replaced by newer points. This provides a form of **implicit map freshness** that the ikd-Tree lacks (ikd-Tree points persist indefinitely until explicitly deleted or the bounding box moves).

**Integration**: Replace the ikd-Tree API calls with iVox API calls. The IEKF doesn't directly depend on the map structure, so the change is localized.

---

## 7. Applicability to Livox Mid-360

- **Approximate kNN**: For the Livox's non-uniform density, the approximate kNN from iVox may be slightly less accurate than the ikd-Tree's exact kNN. However, the speedup is typically more valuable than the marginal accuracy loss.
- **FIFO map aging**: Particularly useful for the Livox, where the non-repetitive pattern means different parts of the scene are observed at different times. FIFO naturally keeps the most recently observed points.
- **Voxel size selection**: Should be set based on the Mid-360's point spacing. Too small → many empty voxels and poor neighborhood coverage; too large → coarse resolution and poor kNN approximation. Typical values: 0.3-0.5m.
- **Computational benefit**: The speedup from $O(\log N)$ to $O(1)$ kNN is valuable for maintaining FAST-LIO2's real-time performance with the additional computational overhead of degeneracy detection and other improvements.
