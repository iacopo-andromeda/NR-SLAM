# NR-SLAM: Paper-to-Code Traceability Document

> **Paper:** "NR-SLAM: Non-Rigid Monocular SLAM"
> **Authors:** Juan J. Gómez Rodríguez, José M.M. Montiel, Juan D. Tardós
> **Repository:** [endomapper/NR-SLAM](https://github.com/endomapper/NR-SLAM)
> **Date of analysis:** 2025-02-09
> **Revision:** 2 (enhanced with line numbers, confidence levels, config parameters, and gap analysis)

---

## Table of Contents

1. [Step 1: Paper Decomposition](#step-1-paper-decomposition)
2. [Step 2: Codebase Survey](#step-2-codebase-survey)
3. [Step 3: Paper → Code Section-by-Section Mapping](#step-3-paper--code-section-by-section-mapping)
4. [Step 4: Mismatch & Gaps Analysis](#step-4-mismatch--gaps-analysis)
5. [Step 6: How to Read the Code If You Read the Paper](#step-6-how-to-read-the-code-if-you-read-the-paper)

---

## Confidence Level Key

| Level | Symbol | Meaning |
|-------|--------|---------|
| **HIGH** | 🟢 | Direct 1:1 correspondence verified with line numbers |
| **MEDIUM** | 🟡 | Conceptual match but with simplifications or restructuring |
| **LOW** | 🟠 | Partially implemented or substantially different from paper |
| **ABSENT** | 🔴 | Paper component not found in code |

---

## Step 1: Paper Decomposition

### 1.1 Problem Formulation

**Paper reference:** Section I (Introduction), Section III (Deformation Model)

> *"We present NR-SLAM, a novel non-rigid monocular SLAM system founded on the combination of a Dynamic Deformation Graph with a Visco-Elastic deformation model."* (Abstract)

**Problem:** Given a monocular video stream from a deforming environment (e.g., endoscopy), simultaneously estimate:
- Camera pose $T_{C^tW}$ for each frame $t$
- 3D map point positions $\mathbf{x}_i^t$ that change over time due to deformations
- A deformable map that can be initialized, extended, and refined

**Key assumptions:**
1. **Camera-over-Deformation (CoD):** Most image innovation comes from camera motion rather than deformations (Section III-A)
2. **Local rigidity:** Small local surface areas tend to behave as almost rigid (Section III-A)
3. **No topological constraints:** No assumption of planar topology, isometric deformations, or continuous surface (Section I)

---

### 1.2 State Representation

**Paper reference:** Sections III, IV-B, V-A

| Symbol | Description | Dimension |
|--------|-------------|-----------|
| $T_{C^tW}$ | Camera pose (SE(3) rigid body transformation) | 6 DoF |
| $\delta_i^t$ | Deformation vector for map point $i$ at time $t$ | 3 per point |
| $\mathbf{x}_i^t = \mathbf{x}_i^{t-1} + \delta_i^t$ | Current 3D position of map point $i$ | 3 per point |
| DDG | Dynamic Deformation Graph encoding point relationships | Graph structure |

**Total degrees of freedom per frame:** $6 + 3|P|$ where $|P|$ is the number of tracked map points (Section IV-B).

---

### 1.3 Deformation Model (Visco-Elastic)

**Paper reference:** Section III-A, Equations (1)–(3)

#### Elastic Regularizer (Eq. 1) — "Spring"

$$E_{ij,elas}^t = k \cdot \frac{(d_{ij}^t - d_{ij}^0)^2}{d_{ij}^0}$$

#### Viscous Regularizer (Eq. 2) — "Damper"

$$E_{ij,visc}^t = b_{ij}^t \|\delta_i^t - \delta_j^t\|^2$$

#### Pairwise Weight (Eq. 3)

$$b_{ij}^t = e^{-d_{ij}^2 / (2\sigma^2)}$$

---

### 1.4 Dynamic Deformation Graph (DDG)

**Paper reference:** Section III-B, Equation (4)

**Edge pruning criterion (Eq. 4):**

$$\frac{|d_{ij}^{max} - d_{ij}^{min}|}{d_{ij}^{min}} > \text{threshold}$$

---

### 1.5 Measurement Model — Data Association

**Paper reference:** Section IV-A

- Short-term: KLT optical flow with gain/bias compensation and SSIM validation
- Mid-term: Projection-based point reuse (Section IV-C)

---

### 1.6 Deformable Tracking — Optimization

**Paper reference:** Section IV-B, Equations (5)–(9)

**Two-stage coarse-to-fine optimization:**
1. Rigid camera-only (Eq. 5): $T_{C^tW}^* = \arg\min \sum_i E_{i,reproj}^t$
2. Joint camera + deformation (Eqs. 6–7):

$$\{T_{C^tW}^*, \{\delta_i^t\}^*\} = \arg\min \sum_i E_{i,reproj}^t + \lambda_{visc} \sum_{(i,j)} E_{ij,visc}^t + \lambda_{elas} \sum_{(i,j)} E_{ij,elas}^t$$

**Reprojection error (Eq. 8):**

$$E_{i,reproj}^t = \|\mathbf{u}_i^t - \pi(T_{C^tW}, \mathbf{x}_i^t)\|^2$$

**Deformation composition (Eq. 9):**

$$\mathbf{x}_i^t = \mathbf{x}_i^{t-1} + \delta_i^t$$

---

### 1.7 Mid-Term Data Association

**Paper reference:** Section IV-C

Uses DDG to propagate deformations to lost points, then reprojects and re-tracks with KLT.

---

### 1.8 Deformable Mapping — Bundle Adjustment

**Paper reference:** Section V-A, Equations (10)–(12)

**Deformable BA cost function (Eq. 10):**

$$\{T_{C^kW}^*, \{\mathbf{x}_i^k\}^*\} = \arg\min \sum_{k,i} E_{i,reproj}^k + \lambda_{visc} \sum_{k,(i,j)} E_{ij,visc}^{k} + \lambda_{elas} \sum_{k,(i,j)} E_{ij,elas}^k$$

---

### 1.9 Map Initialization (Monocular)

**Paper reference:** Section V-B

- Essential matrix estimation with RANSAC
- Midpoint triangulation from [27]
- Scale fixed to 3 cm median depth

---

### 1.10 Map Point Triangulation

**Paper reference:** Section V-C, Equations (13)–(14)

**Model selection:** Compare rigid vs. deformable triangulation counts with 1.5× factor.

**Deformable triangulation (Eq. 14):** Optimizes per-frame landmark positions in camera coordinates with neighbor-based spatial regularization.

---

### 1.11 System Architecture

**Paper reference:** Figure 2

Three parallel threads: Tracking, Mapping, Visualization.

---

### 1.12 Experimental Setup

**Paper reference:** Section VI

Datasets: Hamlyn (rectified stereo), Endomapper (monocular endoscopy), simulation (synthetic deformation with ground truth).

---

### 1.13 Ablation / Variants

**Paper reference:** Section VI, Table III

Tested variants: only elastic, only viscous, only rigid, visco-elastic (full).

---

### 1.14 Limitations / Assumptions

**Paper reference:** Section VII

- No loop closure
- No surgical tool segmentation
- CPU-only (real-time on modern hardware)
- Monocular scale ambiguity (fixed at initialization)

---

## Step 2: Codebase Survey

### 2.1 Overall Architecture

```
NR-SLAM/
├── apps/            # Executables (hamlyn, endomapper, simulation)
├── modules/
│   ├── SLAM/        # System orchestration (system.h/cc, settings.h/cc)
│   ├── tracking/    # Tracking thread (tracking.cc, initializers)
│   ├── mapping/     # Mapping thread (mapping.cc)
│   ├── optimization/# g2o edge types + main optimization functions
│   ├── map/         # Map, Frame, KeyFrame, MapPoint, RegularizationGraph
│   ├── matching/    # LucasKanadeTracker
│   ├── features/    # ShiTomasi feature detector
│   ├── calibration/ # PinHole, KannalaBrandt8
│   ├── masking/     # Image filtering (border, bright, predefined)
│   ├── datasets/    # Dataset loaders
│   ├── stereo/      # Stereo pattern matching
│   ├── utilities/   # Geometry, statistics, DBSCAN, evaluation
│   └── visualization/ # Pangolin 3D viewer
├── data/            # Per-dataset YAML configs
├── third_party/     # abseil-cpp, g2o, Sophus
└── CMakeLists.txt
```

**Build:** C++17, CMake, produces `libNR-SLAM.so` + 3 executables.

### 2.2 Key Source Files (by importance)

| File | Lines | Purpose |
|------|-------|---------|
| `optimization/g2o_optimization.cc` | 1162 | All 4 main optimization functions |
| `tracking/tracking.cc` | 522 | Full tracking pipeline |
| `mapping/mapping.cc` | 271 | Mapping pipeline, triangulation |
| `map/regularization_graph.cc` | 237 | DDG implementation |
| `matching/lucas_kanade_tracker.cc` | ~632 | KLT with gain/bias + SSIM |
| `optimization/*.cc` (10 files) | ~60 each | g2o edge type implementations |
| `utilities/geometry_toolbox.cc` | ~100 | Weight computation, triangulation |
| `SLAM/system.cc` | ~130 | Thread orchestration |

### 2.3 Dependencies and External Libraries

| Library | Use |
|---------|-----|
| **g2o** (third_party) | Graph optimization / Levenberg-Marquardt |
| **Sophus** (third_party) | SE(3) Lie group operations |
| **Abseil** (third_party) | Containers (flat_hash_map, btree_set), logging |
| **OpenCV 4** | Vision: KLT, features, image I/O, CLAHE |
| **Eigen3** | Linear algebra |
| **Pangolin** | 3D visualization |
| **MLPACK** | DBSCAN clustering |
| **Boost** | Filesystem |
| **fmt** | String formatting |

### 2.4 Configuration Parameters (from `data/*/settings.yaml`)

| YAML Key | Default | Description |
|----------|---------|-------------|
| `Camera.type` | — | `PinHole` or `KannalaBrandt8` |
| `Camera.fx/fy/cx/cy` | — | Intrinsics |
| `klt_window_size` | 21 | KLT tracking window |
| `klt_max_level` | 3 | KLT pyramid levels |
| `klt_max_iters` | 50 | KLT max iterations |
| `klt_epsilon` | 0.01 | KLT convergence threshold |
| `klt_min_eig_th` | 1e-4 | KLT min eigenvalue |
| `klt_min_SSIM` | 0.7 | SSIM rejection threshold |
| `images_to_insert_keyframe` | 5 | Frames between keyframe insertions |
| `radians_per_pixel` | — | Camera resolution (for parallax thresholds) |

---

## Step 3: Paper → Code Section-by-Section Mapping

### 3.1 Elastic Regularizer — Eq. 1

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section III-A, Eq. 1: $E_{ij,elas}^t = k \cdot \frac{(d_{ij}^t - d_{ij}^0)^2}{d_{ij}^0}$ |
| **Code Ref (tracking)** | `modules/optimization/position_regularizer_with_deformation.cc`, class `PositionRegularizerWithDeformation`, `computeError()` at line 34 |
| **Code Ref (DBA)** | `modules/optimization/position_regularizer.cc`, class `PositionRegularizer`, `computeError()` at line 35 |
| **Integration** | `g2o_optimization.cc` lines 340–356 (tracking), lines 1003–1020 (DBA) |
| **Config Params** | `k_` = 1.1 (hardcoded at `g2o_optimization.cc:259` and `:937`); `sigma_position` = 0.1 (hardcoded at `:207` and `:871`) |
| **Explanation** | Two implementations exist: `PositionRegularizerWithDeformation` operates on deformation vectors (`rest_position + δ`), used in tracking. `PositionRegularizer` operates on absolute landmark positions, used in DBA. Both compute `k_ * (d_current - d_rest) / d_rest`. The error is scalar (1D). The measurement `_measurement` stores $d_{ij}^0$ = `regularization_edge->first_distance`. |
| **Confidence** | 🟢 **HIGH** — Direct implementation of Eq. 1. The code computes `k_ * (current_distance - _measurement) / _measurement` which is the signed square-root form of the paper's squared energy. g2o internally squares the error. |

---

### 3.2 Viscous Regularizer — Eq. 2

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section III-A, Eq. 2: $E_{ij,visc}^t = b_{ij}^t \|\delta_i^t - \delta_j^t\|^2$ |
| **Code Ref (tracking)** | `modules/optimization/spatial_regularizer_with_deformation.cc`, class `SpatialRegularizerWithDeformation`, `computeError()` at line 38 |
| **Code Ref (DBA)** | `modules/optimization/spatial_regularizer.cc`, class `SpatialRegularizer`, `computeError()` at line 35 |
| **Code Ref (lost points)** | `modules/optimization/spatial_regularizer_fixed.cc`, class `SpatialRegularizerFixed`, `computeError()` at line 34 |
| **Integration** | `g2o_optimization.cc` lines 285–318 (tracking), lines 1052–1093 (DBA temporal), lines 490–537 (lost points) |
| **Config Params** | `sigma_spatial` = 0.1 × `scale` (hardcoded at `g2o_optimization.cc:210` and `:874`); weight `b_{ij}` = `regularization_edge->weight` from DDG |
| **Explanation** | **Tracking:** `SpatialRegularizerWithDeformation` takes two deformation vertices. Error = `weight_ * (flow_1 - flow_2)` (3D vector). **DBA:** `SpatialRegularizer` uses 4 absolute landmark vertices across two keyframes: error = `weight_ * ((x_i^{t+1} - x_i^t) - (x_j^{t+1} - x_j^t))`. This encodes the temporal viscous constraint using absolute positions. **Lost points:** `SpatialRegularizerFixed` has 1 vertex + 1 fixed reference, propagating deformation to untracked points. |
| **Confidence** | 🟢 **HIGH** — Three faithful variants implementing Eq. 2 in different optimization contexts. |

---

### 3.3 Pairwise Weight — Eq. 3

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section III-A, Eq. 3: $b_{ij}^t = e^{-d_{ij}^2 / (2\sigma^2)}$ |
| **Code Ref** | `modules/utilities/geometry_toolbox.cc`, function `InterpolationWeight()` at line 28 |
| **Integration** | Called in `regularization_graph.cc:49` (`AddEdge`) and `:114` (`UpdateConnection`) |
| **Config Params** | `sigma` = `options_.weight_sigma` (default 10.5, set from `sigma_scaled * 3` at initialization: `tracking.cc:199`); configurable via `RegularizationGraph::Options` |
| **Explanation** | Exact implementation: `exp(-(distance * distance) / (2 * sigma * sigma))`. Used both at edge creation and during updates when max_distance changes. |
| **Confidence** | 🟢 **HIGH** — Exact match to Eq. 3. |

---

### 3.4 DDG Edge Pruning — Eq. 4

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section III-B, Eq. 4: $\frac{\|d_{ij}^{max} - d_{ij}^{min}\|}{d_{ij}^{min}} > \text{th}$ |
| **Code Ref** | `modules/map/regularization_graph.cc`, method `UpdateConnection()` at line 120 |
| **Integration** | Called from `UpdateVertex()` at line 134; `UpdateVertex()` called from `g2o_optimization.cc:432` after tracking optimization |
| **Config Params** | `options_.streching_th` = 1.1 (default in `map.h:30`); `weight_sigma` = 10.5 (default in `map.h:29`) |
| **Explanation** | Code: `fabs((edge->max_distance - edge->min_distance) / edge->min_distance) > options_.streching_th`. When exceeded, edge status → `BAD`. `max_distance` and `min_distance` tracked incrementally via lines 107–113. |
| **Confidence** | 🟢 **HIGH** — Exact match to Eq. 4. |

---

### 3.5 Camera Pose Optimization (Rigid) — Eq. 5

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section IV-B, Eq. 5: $T_{C^tW}^* = \arg\min \sum_i E_{i,reproj}^t$ |
| **Code Ref** | `modules/optimization/g2o_optimization.cc`, function `CameraPoseOptimization()` at lines 51–147 |
| **Edge type** | `ReprojectionErrorOnlyPose` (`reprojection_error_only_pose.cc:55`, `computeError()`) |
| **Integration** | Called from `tracking.cc:319` → `CameraPoseEstimation()` at line 311 |
| **Config Params** | `th_huber_2dof_squared` = 5.99 (hardcoded at `:64`); iterations = {10, 10, 10} (hardcoded at `:103`); robust kernel disabled after iteration 2 (`:139`) |
| **Explanation** | Single SE3 vertex, fixed landmarks. 3-pass Levenberg-Marquardt with iterative outlier rejection. Motion model seed applied before call (`tracking.cc:314`). Landmarks set as fixed constants in `edge->landmark_world_`. |
| **Confidence** | 🟢 **HIGH** — Standard pose-only BA matching Eq. 5. |

---

### 3.6 Joint Camera + Deformation Optimization — Eqs. 6–9

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section IV-B, Eqs. 6–9: Joint optimization of $T_{C^tW}$ and $\{\delta_i^t\}$ with reprojection + viscous + elastic terms |
| **Code Ref** | `modules/optimization/g2o_optimization.cc`, function `CameraPoseAndDeformationOptimization()` at lines 149–556 |
| **Edge types** | `ReprojectionErrorWithDeformation` (`:229–249`), `SpatialRegularizerWithDeformation` (`:285–313`), `PositionRegularizerWithDeformation` (`:317–356`) |
| **Integration** | Called from `tracking.cc:327` → `CameraPoseAndDeformationEstimation()` |
| **Config Params** | `regularizers_per_point` = 10 (`:199`), `sigma_reprojection` = 0.5 px (`:203`), `sigma_position` = 0.1 (`:207`), `sigma_spatial` = 0.1 × scale (`:210`), `k_` = 1.1 (`:259`), IQR factor = 1.5 (`:407`), iterations = {10, 10} (`:363`) |
| **Explanation** | Builds a g2o graph with: (a) 1 SE3 vertex (camera), (b) N deformation vertices initialized to zero, (c) reprojection edges per point, (d) up to 10 spatial regularizer edges per point pair from DDG, (e) position regularizer edges for same pairs. Two outer iterations with outlier rejection (chi² > 5.99 for reprojection, > 0.584 for spatial). Post-optimization: IQR-based deformation outlier filtering (lines 395–415), regularization graph update (lines 424–442), lost mappoint deformation propagation (lines 444–556). |
| **Confidence** | 🟢 **HIGH** — Comprehensive implementation of Eqs. 6–9 with additional robustification. |

---

### 3.7 Reprojection Error — Eq. 8

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section IV-B, Eq. 8: $\|\mathbf{u}_i^t - \pi(T_{C^tW}, \mathbf{x}_i^t)\|^2$ |
| **Code Ref (pose-only)** | `reprojection_error_only_pose.cc:55`: `_error = observation - calibration_->Project(Tcw.map(landmark_world_))` |
| **Code Ref (with deformation)** | `reprojection_error_with_deformation.cc:42`: `_error = observation - calibration_->Project(Tcw.map(deformation + landmark_world_))` |
| **Code Ref (absolute/DBA)** | `reprojection_error.cc:36`: `_error = observation - calibration_->Project(Tcw.map(landmark_vertex->estimate()))` |
| **Code Ref (triangulation)** | `reprojection_error_only_deformation.cc`: projects landmark vertex in camera frame |
| **Config Params** | `sigma_reprojection` = 0.5 px (used to set information matrix = 4.0 per axis) |
| **Explanation** | Four variants of the same equation for different optimization contexts. All compute 2D pixel error between observed keypoint and projected 3D landmark through camera model. |
| **Confidence** | 🟢 **HIGH** — Direct implementations of Eq. 8 across all contexts. |

---

### 3.8 Deformation Composition — Eq. 9

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section IV-B, Eq. 9: $\mathbf{x}_i^t = \mathbf{x}_i^{t-1} + \delta_i^t$ |
| **Code Ref** | `reprojection_error_with_deformation.cc:48`: `Tcw.map(deformation + landmark_world_)` |
| **Code Ref** | `g2o_optimization.cc:422`: `current_landmark_position = deformation + previous_landmark_position` |
| **Explanation** | In the tracking optimization, `landmark_world_` stores $\mathbf{x}_i^{t-1}$ and the vertex estimates $\delta_i^t$. The composition happens implicitly inside the reprojection edge and explicitly when recovering results. |
| **Confidence** | 🟢 **HIGH** — Exact implementation. |

---

### 3.9 Short-Term Data Association (KLT) — Section IV-A

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section IV-A: KLT optical flow with gain and bias illumination compensation, SSIM-based outlier rejection |
| **Code Ref** | `modules/matching/lucas_kanade_tracker.cc`, class `LucasKanadeTracker` |
| **Key methods** | `Track()` at lines ~100–465 (multi-scale KLT with gain/bias: `alpha = sqrt(meanI2/meanJ2)`, `beta = meanI - alpha*meanJ`); SSIM computation at lines ~470–586 |
| **Integration** | Called from `tracking.cc:309` (`DataAssociation`) and `:476` (`PointReuse`) |
| **Config Params** | `klt_window_size` (YAML, default 21), `klt_max_level` (YAML, default 3), `klt_max_iters` (YAML, default 50), `klt_epsilon` (YAML, default 0.01), `klt_min_eig_th` (YAML, default 1e-4), `klt_min_SSIM` (YAML, default 0.7) |
| **Explanation** | Custom KLT implementation (not OpenCV's). Computes gain `alpha` and bias `beta` from patch statistics. SSIM uses standard formula with C1=(0.01×255)², C2=(0.03×255)². Paper mentions "patches are updated every 5 images" → this corresponds to keyframe insertion interval where `SetReferenceImage()` is called. |
| **Confidence** | 🟢 **HIGH** — Faithful implementation including gain/bias and SSIM. |

---

### 3.10 Mid-Term Data Association — Section IV-C

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section IV-C: "we estimate the scene flow of lost map points as a weighted average of the scene flow of their neighbours in the DDG" |
| **Code Ref (propagation)** | `g2o_optimization.cc` lines 444–556: Lost mappoint deformation estimation via `SpatialRegularizerFixed` edges with fixed tracked-neighbor deformations |
| **Code Ref (reuse)** | `tracking.cc` lines 404–510: `PointReuse()` — projects all map points, KLT tracks candidates with SSIM ≥ 0.75, checks reprojection error < 5.99 |
| **Config Params** | KLT pyramid level = 1 for reuse (hardcoded at `:429`), SSIM threshold = 0.75 (hardcoded at `:476`), reprojection threshold = 5.99 (hardcoded at `:491`), max regularizers for lost points = 10 (`:500`), optimization iterations = 10 (`:537`) |
| **Explanation** | **Phase 1:** After tracking optimization, a second g2o optimization propagates deformation to lost map points using `SpatialRegularizerFixed` edges. **Phase 2:** All untracked map points (both newly lost and from map) are projected into the current frame, tracked via KLT with stored photometric templates, and validated against reprojection error. Paper describes this as "project them back into the image using a constant velocity motion model" — code uses the optimized pose + DDG-propagated deformation instead. |
| **Confidence** | 🟡 **MEDIUM** — Conceptually matches paper but implementation uses optimization-based propagation rather than weighted average. The "weighted average" from the paper is effectively solved via the `SpatialRegularizerFixed` optimization which achieves a similar result. |

---

### 3.11 Deformable Bundle Adjustment — Eqs. 10–12

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section V-A, Eqs. 10–12: Sliding-window BA over keyframes with reprojection + viscous + elastic terms |
| **Code Ref** | `modules/optimization/g2o_optimization.cc`, function `LocalDeformableBundleAdjustment()` at lines 884–1162 |
| **Edge types** | `ReprojectionError` (`:966–984`), `PositionRegularizer` (`:1003–1020`), `SpatialRegularizer` (`:1052–1093`) |
| **Helper classes** | `TemporalPoint` (lines 820–850) and `SpatialPoint` (lines 852–882) for edge deduplication |
| **Integration** | Called from `mapping.cc:60` → `KeyFrameMapping()` |
| **Config Params** | `max_keyframes` = 5 (`:893`), min keyframes = 3 (`:926`), `regularizers_per_point` = 10 (`:860`), `sigma_reprojection` = 0.5 px (`:868`), `sigma_position` = 0.1 (`:871`), `sigma_spatial` = 0.1 × scale (`:874`), `k_` = 1.1 (`:937`), optimizer iterations = 5 (`:1100`) |
| **Explanation** | **Key design choice:** DBA uses **absolute landmark positions per-keyframe** as vertices (not deformation vectors). Each map point has separate 3D position vertices for each keyframe it appears in. This means the viscous regularizer (`SpatialRegularizer`, 4 vertices) connects `(x_i^k, x_j^k, x_i^{k+1}, x_j^{k+1})` across consecutive keyframes. The elastic regularizer (`PositionRegularizer`, 2 vertices) connects `(x_i^k, x_j^k)` within a single keyframe. 5-keyframe sliding window. |
| **Confidence** | 🟡 **MEDIUM** — Faithfully implements Eqs. 10–12 but with absolute landmark parameterization instead of deformation-based. The mathematical effect is equivalent since the regularizers encode the same constraints, but the optimization variables differ from the paper's presentation. |

---

### 3.12 Map Initialization — Section V-B

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section V-B: Essential matrix from 5-point RANSAC, midpoint triangulation, scale from 3 cm median depth |
| **Code Ref** | `modules/tracking/monocular_map_initializer.cc/h`, `essential_matrix_initializer.cc/h`, `tracking.cc` lines 141–215 (`MonocularMapInitialization`) |
| **Key steps** | (1) Feature extraction with ShiTomasi (NMS window = 7, `:45`), (2) KLT tracking between reference and current (window=21, level=4, SSIM=0.5), (3) Essential matrix estimation (min 8 matches, RANSAC seed=4, p=0.95, ε=0.005), (4) Midpoint triangulation (`geometry_toolbox.cc:49–85`), (5) Scale to 3 cm median depth (`tracking.cc:159`, `scale = 3.f / median_depth`), (6) Initialize DDG with σ = `sigma_scaled * 3` (`:199`) |
| **Config Params** | `rigid_initializer_max_features` = 4000 (`:62`), `rigid_initializer_min_sample_set_size` = 8 (`:63`), `rigid_initializer_min_parallax` = 0.999 (`:64`), `rigid_initializer_epipolar_threshold` = 0.005 (`:65`), `klt_min_SSIM` = 0.5 (`:60` for initializer) |
| **Explanation** | Paper: "We use the midpoint method from [27] to compute the Essential matrix." The code uses an 8-point algorithm variant, not 5-point. Midpoint triangulation uses inverse-depth weighting (`geometry_toolbox.cc:80–84`). Scale is hardcoded to 3 cm median depth as stated in paper. |
| **Confidence** | 🟡 **MEDIUM** — Matches paper description but uses 8-point (not 5-point) Essential matrix estimation. |

---

### 3.13 Map Point Triangulation — Section V-C

| Field | Detail |
|-------|--------|
| **Paper Ref** | Section V-C: Rigid triangulation with rigidity check, deformable triangulation (Eq. 14), model selection with 1.5× factor |
| **Code Ref (orchestration)** | `modules/mapping/mapping.cc`, `LandmarkTriangulation()` at lines 67–271 |
| **Code Ref (deformable)** | `modules/optimization/g2o_optimization.cc`, `DeformableTriangulation()` at lines 558–818 |
| **Edge types** | `ReprojectionErrorOnlyDeformation` (`:686–698`), `SpatialRegularizerWithObservation` (`:710–731`) |
| **Config Params** | Rigidity threshold = 0.004 (`mapping.cc:131`), parallax range = 10–20 × `rad_per_pixel` (`:151`), reprojection threshold = 5.991 (`:165, :175`), model selection ratio = 1.5 (`:183, :189`), min track length for deformable = 5 (`:98`), depth seed from neighbor average (`g2o_optimization.cc:638–655`), bad-edge rejection ratio = 0.5 (`:762, :774`), `sigma_spatial` = 0.1 (`:693`), deformable triangulation chi² = 7.815 (`:689`), iterations = 10 (`:734`) |
| **Explanation** | Rigid and deformable triangulation run in parallel for each candidate. Rigid: midpoint + parallax + reprojection checks. Deformable: per-frame landmark vertices in camera frame, seeded with average neighbor depth, regularized with observed neighbor flow via `SpatialRegularizerWithObservation`. Model selection: if rigid > 1.5× deformable, use rigid for all; if deformable ≥ 1.5× rigid, use deformable for all; otherwise skip. New points added to DDG (`:234–259`). |
| **Confidence** | 🟢 **HIGH** — Comprehensive implementation matching Section V-C description. |

---

### 3.14 System Architecture — Fig. 2

| Field | Detail |
|-------|--------|
| **Paper Ref** | Figure 2: Three-thread architecture (Tracking, Mapping, Visualization) |
| **Code Ref** | `modules/SLAM/system.cc`: creates threads for tracking, mapping, visualization |
| **Integration** | `system.h` orchestrates `Tracking`, `Mapping`, `ImageVisualizer` objects sharing a `Map` |
| **Explanation** | Tracking runs at frame rate. Mapping processes keyframes or does frame-level triangulation when idle. Visualization runs independently with Pangolin. |
| **Confidence** | 🟢 **HIGH** — Direct match. |

---

### 3.15 g2o Edge Type Taxonomy

| g2o Edge Class | Vertices | Error Dim | Paper Equation | Used In | File:Line (`computeError`) |
|---------------|----------|-----------|----------------|---------|---------------------------|
| `ReprojectionErrorOnlyPose` | SE3 (1) | 2 | Eq. 8 (fixed landmarks) | `CameraPoseOptimization` | `reprojection_error_only_pose.cc:55` |
| `ReprojectionErrorWithDeformation` | SE3 + Deformation (2) | 2 | Eqs. 8–9 | `CameraPoseAndDeformation` | `reprojection_error_with_deformation.cc:42` |
| `ReprojectionErrorOnlyDeformation` | Landmark (1) | 2 | Eq. 14 | `DeformableTriangulation` | `reprojection_error_only_deformation.cc` |
| `ReprojectionError` | SE3 + Landmark (2) | 2 | Eq. 8 (absolute) | `LocalDeformableBA` | `reprojection_error.cc:36` |
| `SpatialRegularizerWithDeformation` | Deformation_i, Deformation_j (2) | 3 | Eq. 2 | `CameraPoseAndDeformation` | `spatial_regularizer_with_deformation.cc:38` |
| `SpatialRegularizer` | 4 Landmarks (4) | 3 | Eq. 2 (temporal) | `LocalDeformableBA` | `spatial_regularizer.cc:35` |
| `SpatialRegularizerFixed` | Deformation (1) + fixed ref | 3 | Eq. 2 (propagation) | Lost point estimation | `spatial_regularizer_fixed.cc:34` |
| `SpatialRegularizerWithObservation` | Landmark^t, Landmark^{t+1} (2) | 3 | Eq. 14 | `DeformableTriangulation` | `spatial_regularizer_with_observation.cc:35` |
| `PositionRegularizer` | Landmark_i, Landmark_j (2) | 1 | Eq. 1 (absolute) | `LocalDeformableBA` | `position_regularizer.cc:35` |
| `PositionRegularizerWithDeformation` | Deformation_i, Deformation_j (2) | 1 | Eq. 1 (deformation) | `CameraPoseAndDeformation` | `position_regularizer_with_deformation.cc:34` |

---

### 3.16 Paper Section → Module Cross-Reference

| Paper Section | Code Module(s) | Primary Files | Verified Lines |
|---------------|---------------|---------------|----------------|
| **III-A** Visco-Elastic Model | `optimization/` | `position_regularizer*.cc`, `spatial_regularizer*.cc`, `geometry_toolbox.cc` | See §3.1–3.3 |
| **III-B** DDG | `map/` | `regularization_graph.cc` (237 lines) | `:31–120` |
| **IV-A** Short-term Association | `matching/`, `features/` | `lucas_kanade_tracker.cc`, `shi_tomasi.cc` | See §3.9 |
| **IV-B** Deformable Tracking | `tracking/`, `optimization/` | `tracking.cc`, `g2o_optimization.cc:51–556` | See §3.5–3.8 |
| **IV-C** Mid-term Association | `tracking/`, `optimization/` | `tracking.cc:404–510`, `g2o_optimization.cc:444–556` | See §3.10 |
| **V-A** Deformable BA | `mapping/`, `optimization/` | `mapping.cc:59–60`, `g2o_optimization.cc:884–1162` | See §3.11 |
| **V-B** Initialization | `tracking/` | `monocular_map_initializer.cc`, `essential_matrix_initializer.cc`, `tracking.cc:141–215` | See §3.12 |
| **V-C** Triangulation | `mapping/`, `optimization/` | `mapping.cc:67–271`, `g2o_optimization.cc:558–818` | See §3.13 |
| **VI** Experiments | `apps/`, `datasets/`, `utilities/` | `hamlyn.cc`, `endomapper.cc`, `simulation.cc`, `evaluation.cc` | — |
| **Fig. 2** Architecture | `SLAM/` | `system.h/cc`, `settings.h/cc` | See §3.14 |

---

## Step 4: Mismatch & Gaps Analysis

### 4.1 Unimplemented Paper Components

| Paper Component | Paper Section | Status | Notes |
|----------------|---------------|--------|-------|
| **Loop closure** | VII (Future Work) | 🔴 **ABSENT** | Paper explicitly states "For now, the system works well without loop closure" and lists it as future work. Not implemented in code. |
| **5-point Essential matrix** | V-B | 🟠 **DIFFERENT** | Paper says "Essential matrix" generally. Code uses 8-point algorithm (`essential_matrix_initializer.cc:63`, `rigid_initializer_min_sample_set_size=8`). Functionally equivalent but different RANSAC min sample. |
| **Explicit deformation magnitude tracking per eq.** | III-A | 🟡 **IMPLICIT** | Paper discusses energy formulation. Code uses g2o's internal squared-error handling rather than explicit energy computation. Equivalent but the code never computes the total energy — g2o minimizes it implicitly. |
| **Simulation deformation model (Eq. 15)** | VI-A | 🔴 **EXTERNAL** | Eq. 15 describes the synthetic deformation model used to generate simulation data. This is not part of the NR-SLAM system itself — the simulation dataset is generated externally. The `simulation.cc` app only reads pre-generated data. |

---

### 4.2 Undocumented Code (Not Described in Paper)

| Code Feature | Location | Description | Impact |
|-------------|----------|-------------|--------|
| **IQR-based deformation outlier filtering** | `g2o_optimization.cc:395–415` | After tracking optimization, deformation magnitudes exceeding $Q_3 + 1.5 \times IQR$ are rejected and those points' status set to `TRACKED` (losing 3D). | **Critical for robustness.** Prevents large erroneous deformations from corrupting the map. Not mentioned in paper. |
| **Lost mappoint deformation propagation** | `g2o_optimization.cc:444–556` | Second optimization pass estimates deformations for untracked points using `SpatialRegularizerFixed` edges with neighbors' deformations frozen. | **Core mid-term data association enabler.** Paper mentions DDG-based propagation but not this specific optimization-based approach. |
| **CLAHE preprocessing** | `system.cc` (CLAHE call), all apps | Contrast-Limited Adaptive Histogram Equalization applied to all input images before tracking. `clipLimit=3.0`, `tileGridSize=(8,8)`. | **Essential for endoscopy.** Addresses low-contrast, non-uniform illumination in endoscopic images. Not discussed in paper. |
| **DBSCAN clustering** | `utilities/dbscan.cc` → `tracking.cc` (stereo init) | Used for 3D point outlier filtering during stereo initialization. 2D: ε=0.2, min_pts=3. 3D: ε=2.5, min_pts=5. | Robustification of initialization. Paper mentions "cluster feature tracks" but does not detail DBSCAN parameters. |
| **Edge deduplication classes** | `g2o_optimization.cc:820–882` | `TemporalPoint` and `SpatialPoint` classes with custom hash functions prevent duplicate regularizer edges in DBA. | Engineering detail not in paper, needed for correct optimization. |
| **Point status BAD from graph quality** | `g2o_optimization.cc:438–442` | Points with < 50% good DDG connections are marked `BAD`. | Map quality maintenance not detailed in paper. |
| **Photometric information storage** | `tracking.cc:203–208, 426–430` | KLT patches stored per map point for mid-term re-tracking. | Enables persistent appearance templates. Mentioned implicitly in paper's mid-term section but not detailed. |
| **Stereo initialization path** | `tracking.cc:220–290` | Alternative stereo initialization with hardcoded baseline `3886.37`, depth range 35.5–70.5. | Commented out in default path. Not in paper (paper is monocular). |
| **Minimum tracked points exit** | `tracking.cc:89` | System `exit(0)` when tracked 3D points < 10. | Crash-avoidance heuristic. |

---

### 4.3 Simplifications vs. Paper

| Paper Description | Code Implementation | Nature of Simplification |
|-------------------|-------------------|------------------------|
| **"Weighted average of scene flow"** (Section IV-C) for lost points | Optimization via `SpatialRegularizerFixed` with Levenberg-Marquardt (10 iterations) | Code uses optimization rather than closed-form average. More robust but different algorithm. |
| **Deformation variables $\delta_i^t$ in DBA** (Eq. 10) | Absolute landmark positions $\mathbf{x}_i^k$ per keyframe | Mathematically equivalent constraints but different parameterization. DBA doesn't explicitly represent deformations — the viscous regularizer operates on position differences across keyframes instead. |
| **Keyframe insertion policy** (Section IV-B) | Simple periodic insertion every N frames (`images_to_insert_keyframe`, default 5) | Paper doesn't detail the policy. Code uses the simplest possible approach. No quality-based, parallax-based, or covisibility-based triggers. |
| **Continuous operation** | Hard exit on tracking failure (< 10 points) | No relocalization or recovery mechanism. Paper mentions this as a limitation. |

---

### 4.4 Hardcoded Assumptions & Magic Numbers

#### Statistical Thresholds (used across multiple locations)

| Value | Meaning | Locations | Justification |
|-------|---------|-----------|---------------|
| `5.99` / `5.991` | χ²(2, 0.05) — 95% confidence for 2-DOF reprojection | `g2o_optimization.cc:64,175,862`; `tracking.cc:491`; `mapping.cc:165,175`; `essential_matrix_initializer.cc:269,285` | Standard statistical threshold for 2D reprojection errors |
| `0.584` | χ²(3) threshold for 3-DOF regularizers | `g2o_optimization.cc:200,865` | Very tight threshold for 3D spatial constraints (not standard 95% confidence) |
| `7.815` | χ²(3, 0.05) for deformable triangulation regularizers | `g2o_optimization.cc:689` | Standard 95% confidence for 3-DOF |

#### Optimization Parameters

| Value | Name | Location | Paper Reference |
|-------|------|----------|-----------------|
| `{10,10,10}` | Camera-only iterations | `g2o_optimization.cc:103` | Not specified in paper |
| `{10,10}` | Camera+deformation iterations | `g2o_optimization.cc:363` | Not specified |
| `10` | Lost point/triangulation iterations | `g2o_optimization.cc:537,734` | Not specified |
| `5` | DBA iterations | `g2o_optimization.cc:1100` | Not specified |
| `1.1` | Spring constant `k_` | `g2o_optimization.cc:259,937` | Paper says "k is a global hyperparameter" |
| `0.5` px | Reprojection sigma | `g2o_optimization.cc:203,580,868` | Not specified |
| `0.1` | Position regularizer sigma | `g2o_optimization.cc:207,871` | Not specified |
| `0.1 × scale` | Spatial regularizer sigma | `g2o_optimization.cc:210,874` | Not specified |

#### Structural Constants

| Value | Purpose | Location |
|-------|---------|----------|
| `10` | Max spatial regularizers per point | `g2o_optimization.cc:199,500,860` |
| `5` | Max keyframes in DBA window | `g2o_optimization.cc:893` |
| `3` | Min keyframes for DBA | `g2o_optimization.cc:926` |
| `0.5` | Bad-connection ratio threshold | `g2o_optimization.cc:438,762,774` |
| `1.5` | IQR multiplier for outlier detection | `g2o_optimization.cc:407` |
| `0.75`/`0.25` | IQR quartile boundaries | `g2o_optimization.cc:403–404` |
| `3.0` / `(8,8)` | CLAHE clip limit / tile size | `system.cc` |
| `7` | ShiTomasi NMS window | `tracking.cc:45` |

#### Initialization Constants

| Value | Purpose | Location |
|-------|---------|----------|
| `3.0` | Median depth scale (cm) | `tracking.cc:159` |
| `× 3` | DDG sigma multiplier | `tracking.cc:199` |
| `4000` | Max features for initialization | `tracking.cc:62` |
| `8` | Min RANSAC sample set | `tracking.cc:63` |
| `0.999` | Min parallax cosine | `tracking.cc:64` |
| `0.005` | Epipolar threshold | `tracking.cc:65` |

#### Triangulation Constants

| Value | Purpose | Location |
|-------|---------|----------|
| `0.004` | Rigidity check threshold | `mapping.cc:131` |
| `10 × rad_per_pixel` | Min parallax (rigid) | `mapping.cc:151` |
| `20 × rad_per_pixel` | Max parallax (rigid) | `mapping.cc:151` |
| `0.0025 × 5` | Parallax threshold (deformable) | `g2o_optimization.cc:624` |
| `1.5` | Model selection ratio | `mapping.cc:183,189` |
| `5` | Min track length for deformable | `mapping.cc:98` |

#### KLT Parameters (duplicated across contexts)

| Context | Window | Level | Iters | Epsilon | MinEig | SSIM |
|---------|--------|-------|-------|---------|--------|------|
| **Tracking** (YAML) | 21 | 3 | 50 | 0.01 | 1e-4 | 0.7 |
| **Init KLT** (hardcoded) | 21 | 4 | 10 | 0.0001 | 0.0001 | 0.5 |
| **Point reuse** (mixed) | YAML | 1 | YAML | YAML | YAML | 0.75 |
| **Stereo matcher** | 21 | 4 | 10 | 0.0001 | 0.0001 | 0.5 |

---

### 4.5 Potential Issues & Observations

1. **Inconsistent SSIM thresholds:** Three different SSIM thresholds are used — 0.5 (initialization), 0.7 (tracking, from YAML), 0.75 (point reuse, hardcoded). The paper does not distinguish between these contexts. The strictest threshold (0.75) for point reuse makes sense since stored templates may be stale.

2. **Hardcoded stereo baseline:** `tracking.cc:228` contains `3886.37` — a hardcoded `baseline × fx` for the Hamlyn dataset stereo camera. This value is dataset-specific and should be configurable.

3. **Missing DBA KeyFrame pose fixation:** In `LocalDeformableBundleAdjustment()`, no keyframe pose is explicitly fixed (`setFixed(true)` is never called on camera vertices). The system relies on g2o's gauge freedom handling, which may lead to gauge drift. Standard practice would fix the oldest keyframe.

4. **Duplicate code pattern:** KLT parameter defaults are defined in at least 6 separate locations (`tracking.cc`, `monocular_map_initializer.h`, `stereo_lucas_kanade.h`, `system.cc`, `matching/lucas_kanade_tracker.h`) with slightly different values. This creates maintenance risk.

5. **Hard exit on tracking failure:** `tracking.cc:89` calls `exit(0)` when tracked points drop below 10. A production system should implement relocalization or graceful degradation.

6. **3-DOF chi² threshold inconsistency:** The 3-DOF threshold for tracking spatial regularizers is 0.584 (very tight, roughly 10% confidence), while for deformable triangulation it's 7.815 (standard 95% confidence). The tracking threshold is unusually strict and may cause over-rejection of valid spatial constraints.

7. **Variable naming:** `dumper_edges` in DBA (line 988) appears to be a misspelling of "damper_edges" (viscous damper).

---

### 4.6 Summary: Coverage Assessment

| Paper Component | Equations | Implementation Status | Confidence |
|----------------|-----------|----------------------|------------|
| Elastic regularizer | Eq. 1 | ✅ Full (2 variants) | 🟢 HIGH |
| Viscous regularizer | Eq. 2 | ✅ Full (3 variants) | 🟢 HIGH |
| Pairwise weight | Eq. 3 | ✅ Exact | 🟢 HIGH |
| DDG edge pruning | Eq. 4 | ✅ Exact | 🟢 HIGH |
| Camera-only optimization | Eq. 5 | ✅ Full | 🟢 HIGH |
| Joint tracking optimization | Eqs. 6–7 | ✅ Full + extras | 🟢 HIGH |
| Reprojection error | Eq. 8 | ✅ Full (4 variants) | 🟢 HIGH |
| Deformation composition | Eq. 9 | ✅ Exact | 🟢 HIGH |
| DBA cost function | Eqs. 10–12 | ✅ Different parameterization | 🟡 MEDIUM |
| Model selection | Eq. 13 | ✅ Full | 🟢 HIGH |
| Deformable triangulation | Eq. 14 | ✅ Full | 🟢 HIGH |
| Simulation deformation | Eq. 15 | External to NR-SLAM | 🔴 N/A |
| KLT with gain/bias | Section IV-A | ✅ Full | 🟢 HIGH |
| Mid-term data association | Section IV-C | ✅ Different approach | 🟡 MEDIUM |
| Map initialization | Section V-B | ✅ 8-pt instead of 5-pt | 🟡 MEDIUM |
| Loop closure | Section VII | ❌ Not implemented | 🔴 ABSENT |
| Tool segmentation | Section VII | ❌ Not implemented | 🔴 ABSENT |

**Overall assessment:** 12/15 equations have HIGH-confidence direct implementations. 3 have MEDIUM-confidence implementations with functionally equivalent but structurally different approaches. The code contains significant additional robustification (IQR filtering, CLAHE, DBSCAN, lost-point propagation) not described in the paper. No critical paper component is missing from the code — the absent items (loop closure, tool segmentation) are explicitly listed as future work in the paper.

---

## Step 6: How to Read the Code If You Read the Paper

This guide is written for a roboticist who has read (or is reading) the NR-SLAM paper and wants to navigate the implementation. It maps the paper's narrative flow onto concrete files and functions.

---

### 6.1 Where to Start

Begin with these three files, in order:

| Order | File | Why |
|-------|------|-----|
| 1 | `apps/hamlyn.cc` (or `endomapper.cc`, `simulation.cc`) | The `main()` function. Shows how a dataset is loaded and fed frame-by-frame into the `System` object. Read this to understand the input contract: one image per call to `System::TrackImage()` or `System::TrackImageWithStereo()`. |
| 2 | `modules/SLAM/system.cc` | The orchestrator. The constructor wires everything together (lines 27–99): `Settings` → `Map` → `Tracking` → `Mapping` → `MapVisualizer`. The per-frame pipeline is `System::TrackImage()` (line 113): preprocess → mask → `tracker_->TrackImage()` → `mapper_->DoMapping()`. This is the only place where tracking and mapping are called sequentially — there is no separate mapping thread despite the paper's Fig. 2 suggesting parallel execution (see §4.3). |
| 3 | `modules/SLAM/settings.cc` / `settings.h` | Reads the YAML config under `data/*/settings.yaml`. All camera intrinsics, KLT parameters, and visualization settings are parsed here. Check this file to understand what is configurable vs. hardcoded (see §4.4). |

---

### 6.2 The Per-Frame Data Flow

Every frame follows the same path through the system. This diagram traces a single call to `System::TrackImage()`:

```
  Image
   │
   ▼
  System::ImageProcessing()          [system.cc:192–203]
  ├── cv::cvtColor (RGB → gray)
  └── CLAHE (contrast enhancement)     ← Not in paper
   │
   ▼
  Masker::GetAllMasks()               [masking/masker.cc]
  (border filter, bright filter, predefined mask)
   │
   ▼
  Tracking::TrackImage()              [tracking.cc:71–112]
   │
   ├─ IF map is empty ──────────────────────────────────────────┐
   │   MonocularMapInitialization()   [tracking.cc:141–215]     │
   │   ├── ShiTomasi feature extraction                        │
   │   ├── KLT tracking to reference frame                     │
   │   ├── Essential matrix (8-pt RANSAC)                      │
   │   │   └── essential_matrix_initializer.cc                 │
   │   ├── Midpoint triangulation                              │
   │   │   └── geometry_toolbox.cc:49 (TriangulateMidPoint)    │
   │   ├── Scale normalization (3 cm median depth)             │
   │   └── DDG initialization (RegularizationGraph)            │
   │       └── regularization_graph.cc:31                      │
   │                                                           │
   ├─ ELSE (normal tracking) ──────────────────────────────────┘
   │   │
   │   ▼
   │  UpdateTriangulatedPoints()      [tracking.cc:507–522]
   │  (promote JUST_TRIANGULATED → TRACKED_WITH_3D)
   │   │
   │   ▼
   │  TrackCameraAndDeformation()     [tracking.cc:298–304]
   │   ├── DataAssociation()                                 ← Paper §IV-A
   │   │   └── LucasKanadeTracker::Track()  [lucas_kanade_tracker.cc]
   │   │       (KLT with gain/bias, SSIM filtering)
   │   │
   │   ├── CameraPoseEstimation()                            ← Paper Eq. 5
   │   │   ├── Apply motion model
   │   │   └── CameraPoseOptimization()  [g2o_optimization.cc:51–147]
   │   │       └── ReprojectionErrorOnlyPose edges
   │   │
   │   └── CameraPoseAndDeformationEstimation()              ← Paper Eqs. 6–9
   │       ├── CameraPoseAndDeformationOptimization()
   │       │   [g2o_optimization.cc:149–556]
   │       │   ├── ReprojectionErrorWithDeformation edges
   │       │   ├── SpatialRegularizerWithDeformation edges   ← Eq. 2
   │       │   ├── PositionRegularizerWithDeformation edges   ← Eq. 1
   │       │   ├── IQR deformation outlier filtering          ← Not in paper
   │       │   ├── RegularizationGraph::UpdateVertex()        ← Eq. 4
   │       │   └── Lost mappoint propagation                  ← Paper §IV-C
   │       │       └── SpatialRegularizerFixed edges
   │       └── Update motion model
   │   │
   │   ▼
   │  PointReuse()                    [tracking.cc:404–510]  ← Paper §IV-C
   │  (project all map points, KLT re-track, validate)
   │   │
   │   ▼
   │  KeyFrameInsertion()             [tracking.cc:349–351]
   │  (periodic, every N frames)
   │   │
   │   ▼
   │  map_->SetLastFrame()
   │
   ▼
  Mapping::DoMapping()                [mapping.cc:38–55]
   │
   ├─ IF pending keyframe ──────────────────────────────────────┐
   │   KeyFrameMapping()              [mapping.cc:59–60]        │
   │   └── LocalDeformableBundleAdjustment()                   │
   │       [g2o_optimization.cc:884–1162]               ← Paper Eqs. 10–12
   │       ├── ReprojectionError edges (absolute)              │
   │       ├── PositionRegularizer edges (Eq. 1)               │
   │       ├── SpatialRegularizer edges (Eq. 2, 4-vertex)      │
   │       └── 5-keyframe sliding window                       │
   │                                                           │
   ├─ ELSE (no keyframe pending) ──────────────────────────────┘
   │   FrameMapping()                 [mapping.cc:63–64]
   │   └── LandmarkTriangulation()    [mapping.cc:67–271]  ← Paper §V-C
   │       ├── Rigid triangulation (midpoint + parallax checks)
   │       ├── DeformableTriangulation()                       │
   │       │   [g2o_optimization.cc:558–818]           ← Paper Eq. 14
   │       │   ├── ReprojectionErrorOnlyDeformation edges      │
   │       │   └── SpatialRegularizerWithObservation edges      │
   │       ├── Model selection (1.5× ratio)                    │
   │       └── Add new points to RegularizationGraph           │
   │
   ▼
  (frame complete)
```

---

### 6.3 Where the Core SLAM Logic Lives

The mathematical heart of NR-SLAM is concentrated in a small number of files. If you are reviewing or extending the system, these are the files to study:

#### Tier 1 — The equations live here

| File | Lines | Paper Coverage | What to look for |
|------|-------|----------------|------------------|
| `optimization/g2o_optimization.cc` | 1162 | Eqs. 1–2, 5–14 | **The single most important file.** Contains all four optimization functions: `CameraPoseOptimization` (lines 51–147), `CameraPoseAndDeformationOptimization` (149–556), `DeformableTriangulation` (558–818), `LocalDeformableBundleAdjustment` (884–1162). Every cost function graph is assembled here. |
| `optimization/g2o_optimization.h` | ~80 | — | Function signatures. Read this first to see the API contract for each optimizer. |
| `map/regularization_graph.cc` | 237 | Eqs. 3–4 | The Dynamic Deformation Graph. `AddEdge`, `GetEdges` (sorted by status/weight), `UpdateConnection` (stretching criterion), `GetOptimizationNeighbours` (0th/1st/2nd order connectivity). |

#### Tier 2 — The g2o edge types (one equation each)

Each file in `optimization/` implements a single g2o edge (error term). They are short (~50–70 lines) and self-contained. Read `computeError()` and `linearizeOplus()` in each:

| File | Equation | Error dimension |
|------|----------|-----------------|
| `position_regularizer.cc` | Eq. 1 (absolute landmarks) | 1D scalar |
| `position_regularizer_with_deformation.cc` | Eq. 1 (deformation form) | 1D scalar |
| `spatial_regularizer_with_deformation.cc` | Eq. 2 (tracking) | 3D vector |
| `spatial_regularizer.cc` | Eq. 2 (DBA, 4-vertex temporal) | 3D vector |
| `spatial_regularizer_fixed.cc` | Eq. 2 (lost point propagation) | 3D vector |
| `spatial_regularizer_with_observation.cc` | Eq. 14 (deformable triangulation) | 3D vector |
| `reprojection_error_only_pose.cc` | Eq. 8 (pose-only tracking) | 2D pixel |
| `reprojection_error_with_deformation.cc` | Eqs. 8–9 (joint tracking) | 2D pixel |
| `reprojection_error.cc` | Eq. 8 (DBA, absolute) | 2D pixel |
| `reprojection_error_only_deformation.cc` | Eq. 14 (triangulation) | 2D pixel |

#### Tier 3 — Pipeline orchestration

| File | Role |
|------|------|
| `tracking/tracking.cc` | Tracking pipeline: data association → pose estimation → deformation estimation → point reuse → keyframe insertion. The control flow for a tracked frame is `TrackCameraAndDeformation()` at line 298. |
| `mapping/mapping.cc` | Mapping pipeline: routes to DBA (keyframe present) or triangulation (no keyframe). Entry is `DoMapping()` at line 38. |
| `SLAM/system.cc` | Wires `Tracking` + `Mapping` + `Map` together. Per-frame entry is `TrackImage()` at line 113. |

#### Tier 4 — Data structures you will encounter everywhere

| File | Key class | Role |
|------|-----------|------|
| `map/frame.h/cc` | `Frame` | Per-frame state: keypoints, landmark positions, statuses (`TRACKED`, `TRACKED_WITH_3D`, `JUST_TRIANGULATED`, `BAD`), camera pose $T_{CW}$, mappoint ID ↔ index maps. |
| `map/keyframe.h/cc` | `KeyFrame` | Extends `Frame` with a unique ID. Stored in the map for DBA. |
| `map/mappoint.h/cc` | `MapPoint` | Persistent 3D point: world position, keypoint ID, photometric template for re-tracking. |
| `map/map.h/cc` | `Map` | Central shared state: `mappoints_` (flat_hash_map), `keyframes_` (btree_map), `RegularizationGraph`, `TemporalBuffer`, `last_frame_`. Thread-safe via `absl::Mutex`. |
| `map/regularization_graph.h/cc` | `RegularizationGraph` | The DDG. Adjacency map of `Edge` structs with `weight`, `status` (VERIFIED/NEIGHBOR/NEUTRAL/BAD), `max_distance`, `min_distance`, `first_distance`. |
| `map/temporal_buffer.h/cc` | `TemporalBuffer` | Sliding buffer of recent frames for deformable triangulation. Stores per-frame keypoint tracks, camera poses, and landmark positions. |

---

### 6.4 Paper Section → Reading Order

If you have just finished a specific section of the paper and want to see the code:

| You just read… | Start here | Then follow to… |
|----------------|------------|-----------------|
| **Section III-A** (Visco-elastic model) | `position_regularizer.cc:35` (Eq. 1 `computeError`), `spatial_regularizer_with_deformation.cc:38` (Eq. 2), `geometry_toolbox.cc:28` (Eq. 3) | `g2o_optimization.cc:199–360` to see how the edges are assembled into a graph |
| **Section III-B** (DDG) | `regularization_graph.h` (data structures), then `regularization_graph.cc:40–120` (add/update/prune) | `g2o_optimization.cc:424–442` to see how the DDG is updated after tracking |
| **Section IV-A** (KLT data association) | `lucas_kanade_tracker.cc` — `Track()` at line ~100, SSIM at line ~470 | `tracking.cc:309` (`DataAssociation`) to see how tracking calls it |
| **Section IV-B** (Deformable tracking) | `tracking.cc:298–340` (pipeline), then `g2o_optimization.cc:51–556` (both optimization stages) | Edge files: `reprojection_error_only_pose.cc`, `reprojection_error_with_deformation.cc` |
| **Section IV-C** (Mid-term association) | `g2o_optimization.cc:444–556` (lost-point propagation), then `tracking.cc:404–510` (`PointReuse`) | `spatial_regularizer_fixed.cc` for the propagation edge |
| **Section V-A** (DBA) | `g2o_optimization.cc:884–1162` (`LocalDeformableBundleAdjustment`) | `spatial_regularizer.cc` (4-vertex temporal viscous), `position_regularizer.cc` (absolute elastic), `reprojection_error.cc` (absolute reprojection) |
| **Section V-B** (Initialization) | `tracking.cc:141–215` (`MonocularMapInitialization`) | `monocular_map_initializer.cc`, `essential_matrix_initializer.cc`, `geometry_toolbox.cc:49` (`TriangulateMidPoint`) |
| **Section V-C** (Triangulation) | `mapping.cc:67–271` (`LandmarkTriangulation`) | `g2o_optimization.cc:558–818` (`DeformableTriangulation`), `spatial_regularizer_with_observation.cc`, `reprojection_error_only_deformation.cc` |
| **Figure 2** (Architecture) | `system.cc` (constructor + `TrackImage`) | `tracking.h` and `mapping.h` for the interface contracts |

---

### 6.5 Common Extension Points

If you are extending NR-SLAM, these are the typical places to intervene:

| Goal | Where to modify | Notes |
|------|-----------------|-------|
| **Change the deformation model** | Create a new g2o edge in `optimization/`, wire it in `g2o_optimization.cc` where `SpatialRegularizerWithDeformation` / `PositionRegularizerWithDeformation` edges are added (lines 285–356) | Each edge needs `computeError()` + `linearizeOplus()`. Register with `G2O_REGISTER_TYPE`. |
| **Add loop closure** | New module. Would need a place recognition system, a pose-graph edge type, and a map correction step after `Mapping::DoMapping()`. Currently absent (see §4.1). | The `Map` class already has mutex-protected access. |
| **Change keyframe insertion policy** | `tracking.cc:345–354` — `NeedNewKeyFrame()`. Currently a simple frame counter. | Replace with parallax-based, covisibility-based, or tracking-quality-based logic. |
| **Tune regularization weights** | `g2o_optimization.cc` lines 199–210 (tracking) and 860–874 (DBA). All sigmas and `k_` are local constants. | Consider moving these to `settings.yaml` and `Tracking::Options` / `Mapping::Options`. |
| **Support a new camera model** | Subclass `CameraModel` (see `calibration/pin_hole.h`, `calibration/kannala_brandt_8.h`). Implement `Project()`, `Unproject()`, `ProjectionJacobian()`. Register in `settings.cc`. | The system is camera-model-agnostic through the `CameraModel` interface. |
| **Add a new dataset** | Create a loader in `modules/datasets/` (see `hamlyn.h/cc` for the pattern), add an app in `apps/`, add a YAML config in `data/`. | The `System` class is dataset-agnostic — it just receives images. |

---

### 6.6 Files You Can Safely Ignore (on a first read)

| Directory / File | Why |
|------------------|-----|
| `third_party/` | Vendored dependencies (abseil, g2o, Sophus). Standard libraries — no NR-SLAM-specific modifications. |
| `modules/stereo/` | Stereo matching utilities. Used only in the stereo initialization path (commented out by default). Not part of the core monocular pipeline described in the paper. |
| `modules/masking/` | Image pre-filtering (border, brightness, predefined masks). Preprocessing convenience — not algorithmically interesting. |
| `modules/visualization/` | Pangolin 3D viewer and OpenCV image display. Pure visualization, no SLAM logic. |
| `modules/datasets/` | Dataset loaders. Thin I/O wrappers. |
| `modules/utilities/evaluation.cc`, `frame_evaluator.cc` | Offline evaluation against ground truth. Not part of the SLAM pipeline. |
| `cmake_modules/` | Build infrastructure. |

---

*End of document.*
